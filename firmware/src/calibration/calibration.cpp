/**
 * calibration.cpp
 * calibration.h 的实现。使用 math/angle_utils、Preferences(NVS)、config.h。
 */

#include "calibration.h"

#include <Preferences.h>
#include <cmath>
#include <cstring>
#include <utility>

#include "../math/angle_utils.h"
#include "config.h"

namespace {
constexpr const char* kNamespace = "aim_calib";
constexpr double kDegPerRad = 57.29577951308232;
} // namespace

void calibrationSetDefaults(CalibrationData& out, int width, int height) {
    out = CalibrationData();
    out.width = width;
    out.height = height;
    // 占位默认：约 64° 水平视场、正方形像素、主点取图像中心、无畸变。
    // 待实测标定：真实参数必须由标定流程覆盖。
    out.fx = 0.8f * (float)width;
    out.fy = 0.8f * (float)width;
    out.cx = ((float)width - 1.0f) / 2.0f;
    out.cy = ((float)height - 1.0f) / 2.0f;
    for (int i = 0; i < 5; ++i) out.dist[i] = 0.0f;
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c) out.affine[r][c] = 0.0f;
    out.pan_offset_deg = 0.0f;
    out.tilt_offset_deg = 0.0f;
    out.valid = false;
}

bool calibrationLoad(CalibrationData& out) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/true)) {
        return false;
    }
    if (!prefs.isKey("w")) {
        prefs.end();
        return false;
    }
    // 逐字段读取：字段名稳定，便于未来增删而互不影响。
    out.width  = prefs.getInt("w", 0);
    out.height = prefs.getInt("h", 0);
    out.fx = prefs.getFloat("fx", 0.0f);
    out.fy = prefs.getFloat("fy", 0.0f);
    out.cx = prefs.getFloat("cx", 0.0f);
    out.cy = prefs.getFloat("cy", 0.0f);
    out.dist[0] = prefs.getFloat("k1", 0.0f);
    out.dist[1] = prefs.getFloat("k2", 0.0f);
    out.dist[2] = prefs.getFloat("p1", 0.0f);
    out.dist[3] = prefs.getFloat("p2", 0.0f);
    out.dist[4] = prefs.getFloat("k3", 0.0f);
    out.affine[0][0] = prefs.getFloat("a00", 0.0f);
    out.affine[0][1] = prefs.getFloat("a01", 0.0f);
    out.affine[0][2] = prefs.getFloat("a02", 0.0f);
    out.affine[1][0] = prefs.getFloat("a10", 0.0f);
    out.affine[1][1] = prefs.getFloat("a11", 0.0f);
    out.affine[1][2] = prefs.getFloat("a12", 0.0f);
    out.pan_offset_deg  = prefs.getFloat("pan_off", 0.0f);
    out.tilt_offset_deg = prefs.getFloat("tilt_off", 0.0f);
    out.valid = prefs.getBool("valid", false);
    prefs.end();
    return true;
}

bool calibrationSave(const CalibrationData& in) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
        return false;
    }
    bool ok = true;
    ok &= prefs.putInt("w", in.width) > 0;
    ok &= prefs.putInt("h", in.height) > 0;
    ok &= prefs.putFloat("fx", in.fx) > 0;
    ok &= prefs.putFloat("fy", in.fy) > 0;
    ok &= prefs.putFloat("cx", in.cx) > 0;
    ok &= prefs.putFloat("cy", in.cy) > 0;
    ok &= prefs.putFloat("k1", in.dist[0]) > 0;
    ok &= prefs.putFloat("k2", in.dist[1]) > 0;
    ok &= prefs.putFloat("p1", in.dist[2]) > 0;
    ok &= prefs.putFloat("p2", in.dist[3]) > 0;
    ok &= prefs.putFloat("k3", in.dist[4]) > 0;
    ok &= prefs.putFloat("a00", in.affine[0][0]) > 0;
    ok &= prefs.putFloat("a01", in.affine[0][1]) > 0;
    ok &= prefs.putFloat("a02", in.affine[0][2]) > 0;
    ok &= prefs.putFloat("a10", in.affine[1][0]) > 0;
    ok &= prefs.putFloat("a11", in.affine[1][1]) > 0;
    ok &= prefs.putFloat("a12", in.affine[1][2]) > 0;
    ok &= prefs.putFloat("pan_off", in.pan_offset_deg) > 0;
    ok &= prefs.putFloat("tilt_off", in.tilt_offset_deg) > 0;
    ok &= prefs.putBool("valid", in.valid) > 0;
    prefs.end();
    return ok;
}

// 定长迭代反畸变：畸变模型无解析逆，但其形式为"恒等 + 小扰动"，
// 用不动点迭代即可收敛；固定 8 次以保证 C++ 端确定性。
static void undistortNormalized(const CalibrationData& d, float xn, float yn,
                                float& xu, float& yu) {
    const float k1 = d.dist[0], k2 = d.dist[1], p1 = d.dist[2], p2 = d.dist[3], k3 = d.dist[4];
    xu = xn;
    yu = yn;
    for (int i = 0; i < 8; ++i) {
        float r2 = xu * xu + yu * yu;
        float radial = 1.0f + r2 * (k1 + r2 * (k2 + r2 * k3));
        if (std::fabs(radial) < 1e-6f) break; // 防止退化系数导致除零
        float tx = 2.0f * p1 * xu * yu + p2 * (r2 + 2.0f * xu * xu);
        float ty = p1 * (r2 + 2.0f * yu * yu) + 2.0f * p2 * xu * yu;
        xu = (xn - tx) / radial;
        yu = (yn - ty) / radial;
    }
}

bool calibrationPixelToAngles(const CalibrationData& data, float px, float py,
                              float& bearing_deg, float& elevation_deg) {
    if (!std::isfinite(px) || !std::isfinite(py)) {
        return false;
    }

    if (data.valid) {
        // 已标定：直接用最小二乘拟合的线性模型，涵盖安装偏差/光轴不对齐。
        bearing_deg   = data.affine[0][0] * px + data.affine[0][1] * py + data.affine[0][2];
        elevation_deg = data.affine[1][0] * px + data.affine[1][1] * py + data.affine[1][2];
        return true;
    }

    // 未标定：回退针孔模型 + 反畸变 + 射线→角度。
    if (data.fx == 0.0f || data.fy == 0.0f) {
        return false;
    }
    float xn = (px - data.cx) / data.fx;
    float yn = (py - data.cy) / data.fy;
    float xu = xn, yu = yn;
    undistortNormalized(data, xn, yn, xu, yu);

    // 相机系：X 右、Y 下、Z 前。射线 normalize([xu,yu,1]) 的 atan2 与缩放无关。
    float X = xu, Y = yu, Z = 1.0f;
    float horiz = std::sqrt(X * X + Z * Z);
    bearing_deg   = (float)(std::atan2(X, Z) * kDegPerRad);   // 向右为正
    elevation_deg = (float)(std::atan2(-Y, horiz) * kDegPerRad); // 向上为正（Y 向下取负）
    return true;
}

bool calibrationAnglesToAxisDeg(const CalibrationData& data,
                                float bearing_deg, float elevation_deg,
                                bool wrap_360,
                                float& pan_deg, float& tilt_deg) {
    if (!std::isfinite(bearing_deg) || !std::isfinite(elevation_deg)) {
        return false;
    }

    // 叠加零点偏移。pan 是否环绕由调用方按机械形态传入；tilt 恒为限位轴。
    float pan_raw = bearing_deg + data.pan_offset_deg;
    if (wrap_360) {
        pan_deg = angleNormalize180(pan_raw);
    } else {
        pan_deg = pan_raw;
        if (pan_deg > PAN_MAX_DEG) pan_deg = PAN_MAX_DEG;
        if (pan_deg < PAN_MIN_DEG) pan_deg = PAN_MIN_DEG;
    }
    tilt_deg = elevation_deg + data.tilt_offset_deg;
    if (tilt_deg > TILT_MAX_DEG) tilt_deg = TILT_MAX_DEG;
    if (tilt_deg < TILT_MIN_DEG) tilt_deg = TILT_MIN_DEG;
    return true;
}

// 3x3 线性方程组求解（列主元高斯消元）。A、b 都以值传递，允许内部破坏。
static bool solve3(double A[3][3], double b[3], double x[3]) {
    for (int col = 0; col < 3; ++col) {
        // 选主元
        int piv = col;
        double best = std::fabs(A[col][col]);
        for (int r = col + 1; r < 3; ++r) {
            double v = std::fabs(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-12) return false; // 奇异
        if (piv != col) {
            for (int c = 0; c < 3; ++c) std::swap(A[col][c], A[piv][c]);
            std::swap(b[col], b[piv]);
        }
        // 消元
        for (int r = col + 1; r < 3; ++r) {
            double f = A[r][col] / A[col][col];
            for (int c = col; c < 3; ++c) A[r][c] -= f * A[col][c];
            A[r][col] = 0.0;
        }
    }
    // 回代
    for (int r = 2; r >= 0; --r) {
        double s = b[r];
        for (int c = r + 1; c < 3; ++c) s -= A[r][c] * x[c];
        x[r] = s / A[r][r];
    }
    return true;
}

bool calibrationSolveAffine(const float* pixel_pts, const float* angle_pts, int n,
                            float affine_out[2][3], float* rmse_out) {
    if (!pixel_pts || !angle_pts || !affine_out || n < 3) {
        return false;
    }

    // 构造法方程 (A^T A) x = A^T b，A 每行 [px,py,1]。
    double AtA[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double Atb[2][3] = {{0,0,0},{0,0,0}};
    for (int i = 0; i < n; ++i) {
        double row[3] = { pixel_pts[2 * i], pixel_pts[2 * i + 1], 1.0 };
        double bl = angle_pts[2 * i];
        double be = angle_pts[2 * i + 1];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) AtA[r][c] += row[r] * row[c];
            Atb[0][r] += row[r] * bl;
            Atb[1][r] += row[r] * be;
        }
    }

    double coeff_bearing[3], coeff_elev[3];
    if (!solve3(AtA, Atb[0], coeff_bearing)) return false;
    if (!solve3(AtA, Atb[1], coeff_elev)) return false;

    affine_out[0][0] = (float)coeff_bearing[0];
    affine_out[0][1] = (float)coeff_bearing[1];
    affine_out[0][2] = (float)coeff_bearing[2];
    affine_out[1][0] = (float)coeff_elev[0];
    affine_out[1][1] = (float)coeff_elev[1];
    affine_out[1][2] = (float)coeff_elev[2];

    if (rmse_out) {
        double se_b = 0.0, se_e = 0.0;
        for (int i = 0; i < n; ++i) {
            double px = pixel_pts[2 * i], py = pixel_pts[2 * i + 1];
            double pb = affine_out[0][0] * px + affine_out[0][1] * py + affine_out[0][2];
            double pe = affine_out[1][0] * px + affine_out[1][1] * py + affine_out[1][2];
            double db = angleDiffDegLinear(angle_pts[2 * i], pb);
            double de = angleDiffDegLinear(angle_pts[2 * i + 1], pe);
            se_b += db * db;
            se_e += de * de;
        }
        rmse_out[0] = (float)std::sqrt(se_b / n);
        rmse_out[1] = (float)std::sqrt(se_e / n);
    }
    return true;
}
