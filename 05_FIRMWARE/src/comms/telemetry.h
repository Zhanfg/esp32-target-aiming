#pragma once
/**
 * telemetry.h
 * 串口 CSV 遥测，两条线并存，前缀不同、用途不同：
 *
 * 1) MST,：每发一条实验记录，字段与顺序按规格书 §19 的 17 项：
 *   MST,timestamp,experiment_id,prototype_version,mechanism_version,mode,boundary_state,
 *       preload_state,membrane_id,payload_id,cycle_count,stage1_event,stage2_event,
 *       mechanism_recovered,magazine_position,magazine_index_ok,fault_code,operator_note
 * 2) DIAG,：每控制拍一条帧轨迹，供 PC 端算指向精度、控制周期与锁定建立时间：
 *   DIAG,timestamp_ms,loop_us,obs_valid,obs_dropped,px,py,confidence,
 *        pan_deg,tilt_deg,pan_target_deg,tilt_target_deg,
 *        err_pan_deg,err_tilt_deg,err_deg,aim_state,has_feedback
 *   DIAG 行只在 AIM_VERBOSE_TELEMETRY==1 时输出。
 *
 * 单位：时间毫秒；loop_us 微秒；px/py 像素；角度均为度；confidence 为 0..1。
 * 字符串字段不得含逗号。事件行 EVT,<t_ms>,<tag>,<msg>。
 * AIM_VERBOSE_TELEMETRY==0 时 MST 只留低频心跳，DIAG 不输出。
 */

#include <cstdint>

struct TelemetryRecord {
    uint32_t timestamp = 0;
    uint16_t experiment_id = 0;
    const char* prototype_version = "";
    const char* mechanism_version = "";
    uint8_t mode = 0;               // FireMode 整数值
    uint8_t boundary_state = 0;
    uint8_t preload_state = 0;      // 0=空闲 1=预载中 2=已武装
    uint16_t membrane_id = 0;
    uint16_t payload_id = 0;
    uint32_t cycle_count = 0;
    uint8_t stage1_event = 0;
    uint8_t stage2_event = 0;
    uint8_t mechanism_recovered = 0;
    uint8_t magazine_position = 0;
    uint8_t magazine_index_ok = 0;
    uint16_t fault_code = 0;        // FaultCode 整数值
    const char* operator_note = "-";
};

// 每控制拍的帧轨迹，用于指向类与时序类验收指标。字段口径：
//   px/py/confidence：当前帧目标质心；obs_valid=0 时这三个字段与 err_* 均写 0，统计时剔除。
//   pan_deg/tilt_deg：轴当前角（无反馈实现即指令角）；pan_target_deg/tilt_target_deg：本拍目标角。
//   err_pan_deg/err_tilt_deg：目标质心经标定映射后的 bearing/elevation，即光轴到目标的角偏差分量。
//   err_deg：sqrt(err_pan_deg^2 + err_tilt_deg^2)，静态指向 RMSE 就按它算。
//   aim_state：AimState 整数值，LOCKED=4 与 TRACKING=3 可直接区分。
//   obs_dropped：本拍尝试取帧但未得到有效观测时为 1，其余为 0。
struct DiagRecord {
    uint32_t timestamp_ms = 0;
    uint32_t loop_us = 0;
    uint8_t obs_valid = 0;
    uint8_t obs_dropped = 0;
    float px = 0.0f;
    float py = 0.0f;
    float confidence = 0.0f;
    float pan_deg = 0.0f;
    float tilt_deg = 0.0f;
    float pan_target_deg = 0.0f;
    float tilt_target_deg = 0.0f;
    float err_pan_deg = 0.0f;
    float err_tilt_deg = 0.0f;
    float err_deg = 0.0f;
    uint8_t aim_state = 0;   // AimState 整数值
    uint8_t has_feedback = 0;
};

void telemetryInit(uint32_t baud);
void telemetryEmit(const TelemetryRecord& rec);
void telemetryEmitDiag(const DiagRecord& rec);
void telemetryEmitEvent(const char* tag, const char* msg);
