/**
 * turntable.cpp
 * turntable.h 的实现，用到 control/axis、calibration、math/angle_utils、hal/encoder。
 *
 * AimSolver 移植前的闭环流程：
 *   像素质心 -> calibrationPixelToAngles() 得到相对光轴的 bearing/elevation（度）
 *   -> alpha-beta 滤波，平滑并估计角速度
 *   -> 相对修正量叠加到当前轴角，得到目标角
 *   -> Axis 位置环执行（内部再串速度环）
 * TODO: 解算器移植后，这里替换为 AimSolver 输出（含仿射、预测、限幅）。
 */

#include "turntable.h"

#include <cmath>

#include "config.h"
#include "../hal/encoder.h"
#include "../math/angle_utils.h"
#include "axis.h"

// alpha-beta 参数：alpha 修正位置、beta 修正速度。数值越小越平滑、滞后越大。
// 待实测标定：需按目标运动速度与噪声重新整定。
namespace {
constexpr float kAlphaBetaAlpha = 0.50f; // 待实测标定
constexpr float kAlphaBetaBeta  = 0.20f; // 待实测标定
}

static Axis s_pan;
static Axis s_tilt;
static CalibrationData s_cal;
static bool s_initialized = false;
static bool s_hold = false;
static float s_hold_pan_deg = 0.0f;
static float s_hold_tilt_deg = 0.0f;

// pan 的机械形态（是否连续旋转），在 turretInit 里从 pan 的 AxisConfig 取一次。
// 这样"pan 是否环绕"只有 makeAxisConfig 一个来源，其余地方都引用它。
static bool s_pan_wrap = false;

// alpha-beta 状态（作用于"相对光轴"的 bearing/elevation）。
static bool s_has_track = false;
static float s_bearing_deg = 0.0f;
static float s_elevation_deg = 0.0f;
static float s_bearing_rate_dps = 0.0f;
static float s_elevation_rate_dps = 0.0f;

static float s_pan_err_deg = 0.0f;
static float s_tilt_err_deg = 0.0f;

// 按 config.h 组装单轴配置。
static AxisConfig makeAxisConfig(uint8_t axis_id) {
    AxisConfig c;
    c.axis_id = axis_id;
    c.max_slew_dps = (axis_id == AXIS_PAN) ? PAN_MAX_SLEW_DPS : TILT_MAX_SLEW_DPS;
    c.deadband_deg = (axis_id == AXIS_PAN) ? PAN_DEADBAND_DEG : TILT_DEADBAND_DEG;
    c.max_duty = (axis_id == AXIS_PAN) ? PAN_MAX_DUTY : TILT_MAX_DUTY;
    c.duty_slew_per_s = (axis_id == AXIS_PAN) ? PAN_DUTY_SLEW_PER_S : TILT_DUTY_SLEW_PER_S;

    if (axis_id == AXIS_PAN) {
        c.min_deg = PAN_MIN_DEG;
        c.max_deg = PAN_MAX_DEG;
        c.pos_kp = PAN_POS_KP; c.pos_ki = PAN_POS_KI; c.pos_kd = PAN_POS_KD;
        c.pos_i_min = PAN_POS_I_MIN; c.pos_i_max = PAN_POS_I_MAX;
        c.vel_kp = PAN_VEL_KP; c.vel_ki = PAN_VEL_KI; c.vel_kd = PAN_VEL_KD;
        c.vel_i_min = PAN_VEL_I_MIN; c.vel_i_max = PAN_VEL_I_MAX;
        // 环绕模式下 pan 视为无硬止点的连续旋转轴。
        c.wrap_360 = (ANGLE_WRAP_360_ENABLED != 0);
    } else {
        c.min_deg = TILT_MIN_DEG;
        c.max_deg = TILT_MAX_DEG;
        c.pos_kp = TILT_POS_KP; c.pos_ki = TILT_POS_KI; c.pos_kd = TILT_POS_KD;
        c.pos_i_min = TILT_POS_I_MIN; c.pos_i_max = TILT_POS_I_MAX;
        c.vel_kp = TILT_VEL_KP; c.vel_ki = TILT_VEL_KI; c.vel_kd = TILT_VEL_KD;
        c.vel_i_min = TILT_VEL_I_MIN; c.vel_i_max = TILT_VEL_I_MAX;
        c.wrap_360 = false; // tilt 永远有限位
    }
    return c;
}

// 把"相对修正量"叠加到当前绝对轴角，并按轴类型处理边界。
// 环绕轴：环绕归一；限位轴：夹紧到机械限位。
static float applyCorrectionDeg(float current_deg, float correction_deg,
                                float min_deg, float max_deg, bool wrap360) {
    float desired = current_deg + correction_deg;
    if (wrap360) {
        return angleNormalize180(desired);
    }
    if (desired > max_deg) desired = max_deg;
    if (desired < min_deg) desired = min_deg;
    return desired;
}

bool turretInit() {
    AxisConfig pan_cfg = makeAxisConfig(AXIS_PAN);
    AxisConfig tilt_cfg = makeAxisConfig(AXIS_TILT);
    s_pan_wrap = pan_cfg.wrap_360; // 与 pan 轴控制器内部用的是同一个标志
    if (!s_pan.init(AXIS_PAN, pan_cfg)) return false;
    if (!s_tilt.init(AXIS_TILT, tilt_cfg)) return false;
    s_pan.setMode(AxisMode::POSITION);
    s_tilt.setMode(AxisMode::POSITION);
    s_has_track = false;
    s_hold = false;
    s_initialized = true;
    return true;
}

void turretSetCalibration(const CalibrationData& cal) {
    s_cal = cal;
}

// alpha-beta 更新：对（可能环绕的）角度观测做预测-校正。
static void updateAlphaBeta(float measured_bearing_deg, float measured_elevation_deg, float dt_s) {
    if (!s_has_track) {
        s_bearing_deg = measured_bearing_deg;
        s_elevation_deg = measured_elevation_deg;
        s_bearing_rate_dps = 0.0f;
        s_elevation_rate_dps = 0.0f;
        s_has_track = true;
        return;
    }
    // 预测
    float pred_b = s_bearing_deg + s_bearing_rate_dps * dt_s;
    float pred_e = s_elevation_deg + s_elevation_rate_dps * dt_s;
    // 残差：环绕感知角差（禁止裸相减）
    float res_b = angleDiffDeg(pred_b, measured_bearing_deg);
    float res_e = angleDiffDeg(pred_e, measured_elevation_deg);
    // 校正
    s_bearing_deg = pred_b + kAlphaBetaAlpha * res_b;
    s_elevation_deg = pred_e + kAlphaBetaAlpha * res_e;
    s_bearing_rate_dps += kAlphaBetaBeta * res_b / dt_s;
    s_elevation_rate_dps += kAlphaBetaBeta * res_e / dt_s;
}

TurretSolution turretUpdate(const TargetObservation& obs, uint32_t now_ms) {
    TurretSolution sol;
    sol.t_ms = now_ms;

    if (!s_initialized) {
        return sol;
    }

    // 固定节拍：由调用方保证按 CONTROL_LOOP_HZ 调用。这里用标称周期，
    // 避免因 now_ms 抖动把 PID 的微分/积分算坏。
    float dt_s = (float)CONTROL_LOOP_BUDGET_MS / 1000.0f;
    if (!(dt_s > 1e-6f)) dt_s = 0.01f;

    // 当前轴角（直接读编码器，保证新鲜）。
    float cur_pan = encoderGetAngleDeg(AXIS_PAN);
    float cur_tilt = encoderGetAngleDeg(AXIS_TILT);

    bool obs_ok = obs.valid && obs.centroid.confidence > 0.0f;

    if (s_hold) {
        // 归零/安全位保持：忽略视觉，只维持显式目标。
        s_pan.setTargetDeg(s_hold_pan_deg);
        s_tilt.setTargetDeg(s_hold_tilt_deg);
    } else if (obs_ok) {
        float bearing_deg = 0.0f, elevation_deg = 0.0f;
        if (calibrationPixelToAngles(s_cal, obs.centroid.x, obs.centroid.y,
                                     bearing_deg, elevation_deg)) {
            updateAlphaBeta(bearing_deg, elevation_deg, dt_s);

            // 相对修正量叠加到当前轴角（图像右/上 → pan 正/tilt 正）。
            float desired_pan = applyCorrectionDeg(cur_pan, s_bearing_deg,
                                                   PAN_MIN_DEG, PAN_MAX_DEG, s_pan_wrap);
            float desired_tilt = applyCorrectionDeg(cur_tilt, s_elevation_deg,
                                                    TILT_MIN_DEG, TILT_MAX_DEG, false);
            s_pan.setTargetDeg(desired_pan);
            s_tilt.setTargetDeg(desired_tilt);

            sol.pan_rate_dps = s_bearing_rate_dps;
            sol.tilt_rate_dps = s_elevation_rate_dps;
        } else {
            obs_ok = false;
        }
    }

    if (!obs_ok && !s_hold) {
        // 目标丢失：保持当前位置（不清 alpha-beta 状态，短时丢失恢复后可继续）。
        s_pan.setTargetDeg(cur_pan);
        s_tilt.setTargetDeg(cur_tilt);
    }

    AxisState pan_state = s_pan.update(dt_s);
    AxisState tilt_state = s_tilt.update(dt_s);

    // 目标角误差（供 LOCKED 判定与遥测）。每轴传自己的 wrap 标志：pan 用 s_pan_wrap，
    // tilt 恒为 false。
    s_pan_err_deg = angleDiffAxisDeg(pan_state.position_deg, pan_state.target_deg, s_pan_wrap);
    s_tilt_err_deg = angleDiffAxisDeg(tilt_state.position_deg, tilt_state.target_deg, false);

    sol.pan_deg = pan_state.target_deg;
    sol.tilt_deg = tilt_state.target_deg;
    if (sol.pan_rate_dps == 0.0f && sol.tilt_rate_dps == 0.0f) {
        // 保持/无效时不做前馈。
    }
    sol.valid = obs_ok && !s_hold;
    sol.confidence = obs.centroid.confidence;
    return sol;
}

void turretSetAxesDeg(float pan_deg, float tilt_deg) {
    s_hold_pan_deg = pan_deg;
    s_hold_tilt_deg = tilt_deg;
    s_hold = true;
    s_pan.setTargetDeg(pan_deg);
    s_tilt.setTargetDeg(tilt_deg);
}

void turretClearHold() {
    s_hold = false;
}

void turretGetState(AxisState& pan, AxisState& tilt) {
    pan = s_pan.state();
    tilt = s_tilt.state();
}

void turretEmergencyStop() {
    s_pan.emergencyStop();
    s_tilt.emergencyStop();
}

bool turretInDeadband() {
    float pan_db = PAN_DEADBAND_DEG;
    float tilt_db = TILT_DEADBAND_DEG;
    return std::fabs(s_pan_err_deg) <= pan_db && std::fabs(s_tilt_err_deg) <= tilt_db;
}

float turretPanErrorDeg() { return s_pan_err_deg; }
float turretTiltErrorDeg() { return s_tilt_err_deg; }
