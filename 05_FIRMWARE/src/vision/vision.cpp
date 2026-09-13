/**
 * vision.cpp
 * vision.h 的实现，用到 esp32-camera、esp_heap_caps、config.h。
 *
 * 内存：工作缓冲和连通域栈优先放 PSRAM（8MB 版本足够），失败退回内部 RAM。
 * 工作缓冲只存目标强度（HSV 的 V），同时当掩码和质心权重，省掉一张灰度图。
 */

#include "vision.h"

#include <Arduino.h>
#include <driver/ledc.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <cmath>

#include "config.h"

namespace {

uint8_t* s_mask = nullptr;    // 工作缓冲：0=非目标，>0=目标强度(V)，同时充当访问标记
int32_t* s_stack = nullptr;   // 连通域 BFS 栈（存打包的 idx/strength）
bool s_initialized = false;
bool s_cam_ok = false;

int s_roi[4] = {0, 0, 0, 0};  // 源图像坐标下的 ROI：x,y,w,h；w/h<=0 表示全画幅

// HSV 阈值，默认取 config.h 的初值。
int s_h_min = VISION_HSV_H_MIN, s_h_max = VISION_HSV_H_MAX;
int s_s_min = VISION_HSV_S_MIN, s_s_max = VISION_HSV_S_MAX;
int s_v_min = VISION_HSV_V_MIN, s_v_max = VISION_HSV_V_MAX;

// 判定为有效目标的最小连通域面积（工作缓冲像素）。
constexpr int kMinAreaPx = 8;

// RAII 守卫：任何 return 路径都会归还帧缓冲，杜绝泄漏。
struct FbGuard {
    camera_fb_t* fb;
    explicit FbGuard(camera_fb_t* f) : fb(f) {}
    ~FbGuard() {
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }
};

// RGB888 → HSV。h ∈ [0,180), s ∈ [0,1], v ∈ [0,1]。
void rgbToHsv(float r, float g, float b, float& h, float& s, float& v) {
    float mx = fmaxf(r, fmaxf(g, b));
    float mn = fminf(r, fminf(g, b));
    float delta = mx - mn;
    v = mx;
    s = (mx > 0.0f) ? (delta / mx) : 0.0f;
    if (delta <= 1e-6f) {
        h = 0.0f;
        return;
    }
    float hh;
    if (mx == r) {
        hh = 60.0f * fmodf((g - b) / delta, 6.0f);
    } else if (mx == g) {
        hh = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        hh = 60.0f * (((r - g) / delta) + 4.0f);
    }
    if (hh < 0.0f) hh += 360.0f;
    h = hh * 0.5f; // 折到 [0,180) 以匹配 OpenCV 习惯
}

inline bool hInRange(float h) {
    if (s_h_min <= s_h_max) {
        return h >= (float)s_h_min && h <= (float)s_h_max;
    }
    // H_MIN > H_MAX：跨 0° 的区间（如红色）。
    return h >= (float)s_h_min || h <= (float)s_h_max;
}

// 把 ROI 夹到实际帧范围内，并解析出有效的 [x,y,w,h]。
void resolveRoi(int frame_w, int frame_h, int& out_x, int& out_y, int& out_w, int& out_h) {
    int x = s_roi[0], y = s_roi[1], w = s_roi[2], h = s_roi[3];
    if (w <= 0 || h <= 0) { // 全画幅
        x = 0; y = 0; w = frame_w; h = frame_h;
    }
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > frame_w) w = frame_w - x;
    if (y + h > frame_h) h = frame_h - y;
    if (w < 1) w = 1;
    if (h < 1) h = 1;
    out_x = x; out_y = y; out_w = w; out_h = h;
}

// 中心加权降采样 + HSV 阈值，结果写入 s_mask。
// 权重：对每个工作像素覆盖的源块，用帐篷（三角）权重 w=(1-|nx|)*(1-|ny|)（加小量），
// 块中心权重最高。这样可抑制块边缘混入的背景像素，降低阈值抖动。
void downsampleAndThreshold(camera_fb_t* fb, int roi_x, int roi_y, int roi_w, int roi_h) {
    const uint16_t* src = reinterpret_cast<const uint16_t*>(fb->buf);
    const int fw = fb->width;
    const float sx = (float)roi_w / (float)VISION_WORK_W;
    const float sy = (float)roi_h / (float)VISION_WORK_H;

    for (int wy = 0; wy < VISION_WORK_H; ++wy) {
        int y0 = roi_y + (int)(wy * sy);
        int y1 = roi_y + (int)((wy + 1) * sy);
        if (y1 <= y0) y1 = y0 + 1;
        if (y1 > roi_y + roi_h) y1 = roi_y + roi_h;

        for (int wx = 0; wx < VISION_WORK_W; ++wx) {
            int x0 = roi_x + (int)(wx * sx);
            int x1 = roi_x + (int)((wx + 1) * sx);
            if (x1 <= x0) x1 = x0 + 1;
            if (x1 > roi_x + roi_w) x1 = roi_x + roi_w;

            float cx = (x0 + x1 - 1) * 0.5f;
            float cy = (y0 + y1 - 1) * 0.5f;
            float hx = (x1 - x0) * 0.5f;
            float hy = (y1 - y0) * 0.5f;
            if (hx < 0.5f) hx = 0.5f;
            if (hy < 0.5f) hy = 0.5f;

            float sr = 0.0f, sg = 0.0f, sb = 0.0f, sw = 0.0f;
            for (int py = y0; py < y1; ++py) {
                float ny = fabsf((py - cy) / hy);
                float wy_ = 1.0f - ny;
                if (wy_ < 0.0f) wy_ = 0.0f;
                for (int px = x0; px < x1; ++px) {
                    float nx = fabsf((px - cx) / hx);
                    float wx_ = 1.0f - nx;
                    if (wx_ < 0.0f) wx_ = 0.0f;
                    float w = wx_ * wy_ + 0.02f; // 小量避免零权重
                    uint16_t p = src[py * fw + px];
                    // RGB565 拆成 8 位各通道
                    float r = (float)((p >> 11) & 0x1F) * (255.0f / 31.0f);
                    float g = (float)((p >> 5) & 0x3F) * (255.0f / 63.0f);
                    float b = (float)(p & 0x1F) * (255.0f / 31.0f);
                    sr += r * w;
                    sg += g * w;
                    sb += b * w;
                    sw += w;
                }
            }

            uint8_t strength = 0;
            if (sw > 0.0f) {
                float r = sr / sw / 255.0f;
                float g = sg / sw / 255.0f;
                float b = sb / sw / 255.0f;
                float h, s, v;
                rgbToHsv(r, g, b, h, s, v);
                bool match = hInRange(h) &&
                             (s * 255.0f) >= (float)s_s_min && (s * 255.0f) <= (float)s_s_max &&
                             (v * 255.0f) >= (float)s_v_min && (v * 255.0f) <= (float)s_v_max;
                // 强度加权质心的权重直接取 V（亮度）；越亮贡献越大。
                if (match) {
                    strength = (uint8_t)(v * 255.0f + 0.5f);
                    if (strength == 0) strength = 1; // 匹配但极暗时保底，避免被当作背景
                }
            }
            s_mask[wy * VISION_WORK_W + wx] = strength;
        }
    }
}

// 在掩码上做 8 连通 BFS，找最大连通域并算强度加权质心。
// 亚像素质心公式： cx = Σ(V_i · x_i) / Σ(V_i)，cy 同理（V_i 为像素强度权重）。
void findLargestComponent(TargetObservation& out, int roi_x, int roi_y, int roi_w, int roi_h) {
    const int W = VISION_WORK_W, H = VISION_WORK_H;
    int best_area = 0;
    double best_sx = 0, best_sy = 0, best_sw = 0;
    int best_minx = 0, best_maxx = 0, best_miny = 0, best_maxy = 0;

    for (int i = 0; i < W * H; ++i) {
        if (s_mask[i] == 0) continue;

        int top = 0;
        // 打包 idx(高 24 位) | strength(低 8 位)，idx < 2^15 足够。
        s_stack[top++] = (i << 8) | s_mask[i];
        s_mask[i] = 0; // 入栈即标记已访问

        int area = 0;
        double sumx = 0, sumy = 0, sumw = 0;
        int minx = W, maxx = 0, miny = H, maxy = 0;

        while (top > 0) {
            int32_t packed = s_stack[--top];
            int idx = packed >> 8;
            int strength = packed & 0xFF;
            int x = idx % W;
            int y = idx / W;

            area++;
            sumx += (double)strength * x;
            sumy += (double)strength * y;
            sumw += strength;
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;

            // 8 邻域
            for (int dy = -1; dy <= 1; ++dy) {
                int ny = y + dy;
                if (ny < 0 || ny >= H) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx;
                    if (nx < 0 || nx >= W) continue;
                    int nidx = ny * W + nx;
                    uint8_t st = s_mask[nidx];
                    if (st == 0) continue;
                    s_mask[nidx] = 0;
                    s_stack[top++] = (nidx << 8) | st;
                }
            }
        }

        if (area > best_area) {
            best_area = area;
            best_sx = sumx; best_sy = sumy; best_sw = sumw;
            best_minx = minx; best_maxx = maxx;
            best_miny = miny; best_maxy = maxy;
        }
    }

    out.valid = false;
    if (best_area < kMinAreaPx || best_sw <= 0.0) {
        return;
    }

    // 工作缓冲坐标 → 源图像像素：ROI 被线性拉伸到工作缓冲，反向映射即乘以缩放系数。
    const float scale_x = (float)roi_w / (float)W;
    const float scale_y = (float)roi_h / (float)H;
    float cx = (float)(best_sx / best_sw);
    float cy = (float)(best_sy / best_sw);

    out.valid = true;
    out.centroid.x = (float)roi_x + (cx + 0.5f) * scale_x;
    out.centroid.y = (float)roi_y + (cy + 0.5f) * scale_y;
    out.bbox_w = (float)(best_maxx - best_minx + 1) * scale_x;
    out.bbox_h = (float)(best_maxy - best_miny + 1) * scale_y;
    float conf = (float)best_area / VISION_CONF_AREA_REF;
    out.centroid.confidence = (conf > 1.0f) ? 1.0f : conf;
}

} // namespace

bool visionInit() {
    s_h_min = VISION_HSV_H_MIN; s_h_max = VISION_HSV_H_MAX;
    s_s_min = VISION_HSV_S_MIN; s_s_max = VISION_HSV_S_MAX;
    s_v_min = VISION_HSV_V_MIN; s_v_max = VISION_HSV_V_MAX;
    s_roi[0] = s_roi[1] = 0;
    s_roi[2] = s_roi[3] = 0;

    const size_t mask_bytes = (size_t)VISION_WORK_W * VISION_WORK_H * sizeof(uint8_t);
    const size_t stack_bytes = (size_t)VISION_WORK_W * VISION_WORK_H * sizeof(int32_t);

    // 优先 PSRAM；失败回退内部 RAM，保证没有 PSRAM 的板子也能跑（只是余量小）。
    s_mask = (uint8_t*)heap_caps_malloc(mask_bytes, MALLOC_CAP_SPIRAM);
    if (!s_mask) {
        s_mask = (uint8_t*)heap_caps_malloc(mask_bytes, MALLOC_CAP_8BIT);
    }
    s_stack = (int32_t*)heap_caps_malloc(stack_bytes, MALLOC_CAP_SPIRAM);
    if (!s_stack) {
        s_stack = (int32_t*)heap_caps_malloc(stack_bytes, MALLOC_CAP_8BIT);
    }
    if (!s_mask || !s_stack) {
        Serial.println("[vision] 工作缓冲分配失败");
        return false;
    }

    camera_config_t cfg = {};
    cfg.ledc_channel = (ledc_channel_t)CAM_LEDC_CHANNEL;
    cfg.ledc_timer = (ledc_timer_t)CAM_LEDC_TIMER;
    cfg.pin_pwdn = CAM_PIN_PWDN;
    cfg.pin_reset = CAM_PIN_RESET;
    cfg.pin_xclk = CAM_PIN_XCLK;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;
    cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_d0 = CAM_PIN_Y2;
    cfg.pin_d1 = CAM_PIN_Y3;
    cfg.pin_d2 = CAM_PIN_Y4;
    cfg.pin_d3 = CAM_PIN_Y5;
    cfg.pin_d4 = CAM_PIN_Y6;
    cfg.pin_d5 = CAM_PIN_Y7;
    cfg.pin_d6 = CAM_PIN_Y8;
    cfg.pin_d7 = CAM_PIN_Y9;
    cfg.pin_vsync = CAM_PIN_VSYNC;
    cfg.pin_href = CAM_PIN_HREF;
    cfg.pin_pclk = CAM_PIN_PCLK;
    cfg.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
    cfg.pixel_format = PIXFORMAT_RGB565; // 需要原始像素做 HSV，不能用 JPEG
    cfg.frame_size = VISION_FRAME_SIZE;
    cfg.jpeg_quality = 12;               // 对 RGB565 无影响，仅为结构体填默认值
    cfg.fb_count = 2;                    // 双缓冲：取帧与处理可重叠，降低掉帧
#ifdef CAMERA_FB_IN_PSRAM
    cfg.fb_location = CAMERA_FB_IN_PSRAM; // 帧缓冲放 PSRAM
#endif
#ifdef CAMERA_GRAB_LATEST
    cfg.grab_mode = CAMERA_GRAB_LATEST;   // 总取最新帧，避免处理积压旧帧
#endif

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[vision] esp_camera_init 失败: 0x%x\n", (unsigned)err);
        s_cam_ok = false;
        return false;
    }
    s_cam_ok = true;
    s_initialized = true;
    return true;
}

bool visionCapture(TargetObservation& out) {
    out = TargetObservation();
    if (!s_initialized || !s_cam_ok) {
        return false;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        return false;
    }
    FbGuard guard(fb); // 从这里开始，任何 return 都会归还帧缓冲

    if (fb->format != PIXFORMAT_RGB565) {
        return false;
    }

    int roi_x, roi_y, roi_w, roi_h;
    resolveRoi(fb->width, fb->height, roi_x, roi_y, roi_w, roi_h);

    downsampleAndThreshold(fb, roi_x, roi_y, roi_w, roi_h);
    findLargestComponent(out, roi_x, roi_y, roi_w, roi_h);
    out.t_ms = millis();
    return out.valid;
}

void visionSetRoi(int x, int y, int w, int h) {
    s_roi[0] = x;
    s_roi[1] = y;
    s_roi[2] = w;
    s_roi[3] = h;
}

void visionSetThresholdHSV(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max) {
    s_h_min = h_min; s_h_max = h_max;
    s_s_min = s_min; s_s_max = s_max;
    s_v_min = v_min; s_v_max = v_max;
}
