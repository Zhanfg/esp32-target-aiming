#pragma once
/**
 * aim_types.h
 * 跨模块数据结构。统一坐标/角度约定（全工程必须遵守）：
 *   图像坐标：x 向右、y 向下，原点左上角。
 *   相机坐标系：X 右、Y 下、Z 前。
 *   bearing = atan2(X, Z)，向右为正（俯视顺时针）。
 *   elevation = atan2(-Y, sqrt(X^2+Z^2))，向上为正。
 *   pan 向右转为正，tilt 上抬为正。
 *   长度 mm；角度内部弧度、模块间接口用度带 _deg；时间 _ms；角速度 _dps。
 */

#include <cstdint>

struct PixelPoint {
    float x = 0.0f;
    float y = 0.0f;
    float confidence = 0.0f; // [0,1]，0 表示无效/被遮挡
};

struct TargetObservation {
    bool valid = false;
    PixelPoint centroid; // 源图像像素坐标，亚像素
    float bbox_w = 0.0f;
    float bbox_h = 0.0f;
    uint32_t t_ms = 0;
};

// 单轴状态。舵机化后没有编码器计数，position_deg 在无反馈实现里就是当前指令角，
// 有反馈实现里是总线回报的实际角，用 has_feedback 区分。
struct AxisState {
    float position_deg = 0.0f;
    float velocity_dps = 0.0f;
    float target_deg = 0.0f;
    float feedback_deg = 0.0f;
    bool has_feedback = false;
    bool at_limit = false;
};

struct TurretSolution {
    float pan_deg = 0.0f;
    float tilt_deg = 0.0f;
    float pan_rate_dps = 0.0f;  // 可作前馈
    float tilt_rate_dps = 0.0f; // 可作前馈
    bool valid = false;         // false 表示目标丢失/超时
    float confidence = 0.0f;    // [0,1]
    uint32_t t_ms = 0;
};

// 区域 A：发射周期状态，名称与顺序按规格书 §10.1。
enum class CycleState : uint8_t {
    BOOT = 0,
    SAFE = 1,
    HOME = 2,
    READY = 3,
    AIM_ALLOWED = 4,
    PRELOAD = 5,
    ARMED = 6,
    RELEASE = 7,
    RECOVER = 8,
    INDEX = 9,
    FAULT = 10,
};

// 区域 B：指向质量状态，保留第一期 IDLE..LOCKED 的原语义。
// 枚举值被遥测与 PC 端脚本按整数解析，新状态只能追加在末尾，不得插队重排。
enum class AimState : uint8_t {
    IDLE = 0,
    CALIBRATING = 1,
    SEARCHING = 2,
    TRACKING = 3,
    LOCKED = 4,
    // 整数 5 原为第一期的 FAULT，已移入区域 A 的 CycleState::FAULT。
    // 保留空洞不重排，避免 CALIB_MODE 及其后取值平移。
    CALIB_MODE = 6,
};

// 故障码。分类见 faultSeverity，遥测 §19 的 fault_code 直接输出整数值。
// 同上，新增故障码只能追加在末尾。
enum class FaultCode : uint16_t {
    NONE = 0,
    PERIPH_INIT = 1,
    HOME_TIMEOUT = 2,
    HOME_SWITCH_CONFLICT = 3,
    RELEASE_GATE_SELFTEST = 4,
    OBS_LINK = 5,
    INDEX_FAIL = 6,
    WATCHDOG = 7,
    MECH_STUCK = 8,
    ESTOP = 9,
    MEMBRANE_RUPTURE = 10,
    DOUBLE_FEED = 11,
    FIRE_INHIBIT_SHORT = 12,
    I2C_BUS_HANG = 13,
    STATE_ILLEGAL = 14,
};

enum class FaultSeverity : uint8_t { NONE, SOFT, HARD };

// 软故障允许 FAULT → SAFE → HOME → READY；硬故障锁存，软件清除无效。
inline FaultSeverity faultSeverity(FaultCode c) {
    switch (c) {
        case FaultCode::NONE:
            return FaultSeverity::NONE;
        case FaultCode::HOME_TIMEOUT:
        case FaultCode::OBS_LINK:
        case FaultCode::INDEX_FAIL:
        case FaultCode::WATCHDOG:
        case FaultCode::ESTOP:
        case FaultCode::DOUBLE_FEED:
            return FaultSeverity::SOFT;
        case FaultCode::PERIPH_INIT:
        case FaultCode::HOME_SWITCH_CONFLICT:
        case FaultCode::RELEASE_GATE_SELFTEST:
        case FaultCode::MECH_STUCK:
        case FaultCode::MEMBRANE_RUPTURE:
        case FaultCode::FIRE_INHIBIT_SHORT:
        case FaultCode::I2C_BUS_HANG:
        case FaultCode::STATE_ILLEGAL:
            return FaultSeverity::HARD;
    }
    return FaultSeverity::HARD;
}

enum class FireMode : uint8_t { AIR_ONLY = 0, SOFT_PAYLOAD = 1 };

enum class TransmitterState {
    SAFE,
    ARMED,
    FIRING,
    COOLDOWN,
    FAULT
};
