/**
 * encoder.cpp
 * PCNT 正交解码实现。依赖 config.h 的引脚、线数、减速比、极性。
 *
 * 计数换算（推导见 config.h）：
 *   输出轴每转计数 = ENCODER_PPR * 4 * GEAR_RATIO
 *   ×4 来自 A/B 双通道四倍频，A、B 的上升沿和下降沿都计数。
 *   角度(deg) = 累计计数 / COUNTS_PER_OUTPUT_REV * 360
 */

#include "encoder.h"

#include <Arduino.h>
#include <driver/pcnt.h>
#include <cmath>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "../math/angle_utils.h"

// PCNT 上限 ±32767；保留余量，读回接近边界前就清零硬件计数器，
// 这样无论硬件在边界处是"回绕"还是"饱和"，软件都不会算错。
static const int16_t kPcntSafeLimit = 30000;

struct EncoderState {
    pcnt_unit_t unit;
    int32_t accum;         // 跨拍累计计数（四倍频）
    int16_t last_raw;      // 上一拍硬件计数
    bool started;          // init 后是否已建立基准
    float last_angle_deg;  // 上次角度（速度差分用）
    float vel_filt_dps;    // 一阶低通后的角速度
    bool has_angle;
};

static EncoderState s_enc[AXIS_COUNT];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_initialized = false;

// 每轴静态配置。
struct EncoderConfig {
    int pin_a;
    int pin_b;
    pcnt_unit_t unit;
    float counts_per_rev;
    bool invert;
};

static const EncoderConfig kEncoders[AXIS_COUNT] = {
    { PAN_ENC_A_PIN,  PAN_ENC_B_PIN,  (pcnt_unit_t)PAN_PCNT_UNIT,
      (float)COUNTS_PER_OUTPUT_REV_PAN,  ENCODER_INVERT_PAN != 0 },
    { TILT_ENC_A_PIN, TILT_ENC_B_PIN, (pcnt_unit_t)TILT_PCNT_UNIT,
      (float)COUNTS_PER_OUTPUT_REV_TILT, ENCODER_INVERT_TILT != 0 },
};

// 配置 PCNT 单元：channel0 数 A 相、channel1 数 B 相，两者共享同一计数器，
// 合成标准四倍频正交解码。极性组合只影响计数的符号，符号统一由 ENCODER_INVERT 修正。
static bool configureUnit(const EncoderConfig& c) {
    // channel0：以 A 为脉冲、B 为方向
    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = c.pin_a;
    ch0.ctrl_gpio_num = c.pin_b;
    ch0.channel = PCNT_CHANNEL_0;
    ch0.unit = c.unit;
    ch0.pos_mode = PCNT_COUNT_INC;
    ch0.neg_mode = PCNT_COUNT_DEC;
    ch0.hctrl_mode = PCNT_MODE_KEEP;
    ch0.lctrl_mode = PCNT_MODE_REVERSE;
    ch0.counter_h_lim = 32767;
    ch0.counter_l_lim = -32768;
    if (pcnt_unit_config(&ch0) != ESP_OK) {
        Serial.printf("[encoder] PCNT channel0 配置失败 unit=%d\n", (int)c.unit);
        return false;
    }

    // channel1：以 B 为脉冲、A 为方向，极性与 channel0 相反，合起来即四倍频
    pcnt_config_t ch1 = {};
    ch1.pulse_gpio_num = c.pin_b;
    ch1.ctrl_gpio_num = c.pin_a;
    ch1.channel = PCNT_CHANNEL_1;
    ch1.unit = c.unit;
    ch1.pos_mode = PCNT_COUNT_DEC;
    ch1.neg_mode = PCNT_COUNT_INC;
    ch1.hctrl_mode = PCNT_MODE_REVERSE;
    ch1.lctrl_mode = PCNT_MODE_KEEP;
    ch1.counter_h_lim = 32767;
    ch1.counter_l_lim = -32768;
    if (pcnt_unit_config(&ch1) != ESP_OK) {
        Serial.printf("[encoder] PCNT channel1 配置失败 unit=%d\n", (int)c.unit);
        return false;
    }

    // 过滤极短毛刺：约 1us 以内的抖动视为噪声（值需按实际信号质量调整）。
    pcnt_filter_enable(c.unit);
    pcnt_set_filter_value(c.unit, 100);

    pcnt_counter_pause(c.unit);
    pcnt_counter_clear(c.unit);
    pcnt_counter_resume(c.unit);
    return true;
}

bool encoderInit() {
    for (int i = 0; i < AXIS_COUNT; ++i) {
        if (!configureUnit(kEncoders[i])) {
            return false;
        }
        s_enc[i].unit = kEncoders[i].unit;
        s_enc[i].accum = 0;
        s_enc[i].last_raw = 0;
        s_enc[i].started = true;
        s_enc[i].last_angle_deg = 0.0f;
        s_enc[i].vel_filt_dps = 0.0f;
        s_enc[i].has_angle = false;
    }
    s_initialized = true;
    return true;
}

// 读取硬件计数并把增量累加进 accum。临界区保证 accum 与 last_raw 一致更新。
static void readAndAccumulate(uint8_t axis) {
    if (!s_initialized || axis >= AXIS_COUNT) {
        return;
    }
    EncoderState& e = s_enc[axis];
    int16_t raw = 0;
    pcnt_get_counter_value(e.unit, &raw);

    portENTER_CRITICAL(&s_mux);
    // 采用"简单相减 + 提前清零"而不是依赖硬件回绕：
    // 只要每拍计数增量远小于 kPcntSafeLimit（控制环 100Hz 下每拍仅数十计数），
    // 两拍之间就不会越过 16 位边界，相减总是正确。
    int32_t delta = (int32_t)raw - (int32_t)e.last_raw;
    e.accum += delta;
    e.last_raw = raw;
    portEXIT_CRITICAL(&s_mux);

    // 接近计数边界前主动清零硬件计数器并重置基准，避免触发边界行为。
    if (raw > kPcntSafeLimit || raw < -kPcntSafeLimit) {
        pcnt_counter_clear(e.unit);
        portENTER_CRITICAL(&s_mux);
        e.last_raw = 0;
        portEXIT_CRITICAL(&s_mux);
    }
}

int32_t encoderGetCount(uint8_t axis) {
    readAndAccumulate(axis);
    int32_t out = 0;
    portENTER_CRITICAL(&s_mux);
    out = s_enc[axis].accum;
    portEXIT_CRITICAL(&s_mux);
    return out;
}

float encoderGetAngleDeg(uint8_t axis) {
    if (axis >= AXIS_COUNT) {
        return 0.0f;
    }
    int32_t count = encoderGetCount(axis);
    float counts_per_rev = kEncoders[axis].counts_per_rev;
    if (counts_per_rev <= 0.0f) {
        return 0.0f;
    }
    float deg = ((float)count / counts_per_rev) * 360.0f;
    // 极性：让"计数增大"对应"正角度"，与 MOTOR_INVERT_* 联合标定。
    if (kEncoders[axis].invert) {
        deg = -deg;
    }
    return deg;
}

float encoderGetVelocityDps(uint8_t axis, float dt_s) {
    if (axis >= AXIS_COUNT || !(dt_s > 1e-6f)) {
        return 0.0f;
    }
    float angle = encoderGetAngleDeg(axis);
    EncoderState& e = s_enc[axis];

    // 轴有限行程，速度差分用线性语义，不回绕。统一走 angle_utils，避免裸的角度相减。
    float raw_dps = 0.0f;
    if (e.has_angle) {
        raw_dps = angleDiffDegLinear(e.last_angle_deg, angle) / dt_s;
    }
    e.last_angle_deg = angle;
    e.has_angle = true;

    // 一阶低通：编码器差分速度量化噪声大，直接送内环会让电机啸叫、抖动。
    float alpha = ENCODER_VEL_LPF_ALPHA;
    if (alpha <= 0.0f) alpha = 1.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    e.vel_filt_dps += alpha * (raw_dps - e.vel_filt_dps);
    return e.vel_filt_dps;
}

void encoderReset(uint8_t axis) {
    if (!s_initialized || axis >= AXIS_COUNT) {
        return;
    }
    EncoderState& e = s_enc[axis];
    pcnt_counter_clear(e.unit);
    portENTER_CRITICAL(&s_mux);
    e.accum = 0;
    e.last_raw = 0;
    e.vel_filt_dps = 0.0f;
    e.last_angle_deg = 0.0f;
    e.has_angle = false;
    portEXIT_CRITICAL(&s_mux);
}
