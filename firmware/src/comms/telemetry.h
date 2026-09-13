#pragma once
/**
 * telemetry.h
 * 串口 CSV 遥测。数据行前缀 AIM,，字段固定（PC 端脚本按此解析）：
 *   AIM,t_ms,state,obs_valid,px,py,conf,pan_deg,tilt_deg,pan_target,tilt_target,pan_rate,tilt_rate,enc_pan,enc_tilt,loop_us
 * 单位：角度度、角速度度/秒、时间毫秒；state 为 AimState 整数值。事件行 EVT,<t_ms>,<tag>,<msg>。
 * AIM_VERBOSE_TELEMETRY==0 时只留低频心跳。
 */

#include <cstdint>

#include "../aim_types.h"

void telemetryInit(uint32_t baud);
void telemetryEmit(const TargetObservation& obs, const TurretSolution& sol,
                   const AxisState& pan, const AxisState& tilt, AimState state);
void telemetrySetLoopUs(uint32_t loop_us);
void telemetryEmitEvent(const char* tag, const char* msg);
