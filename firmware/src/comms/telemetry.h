#pragma once
/**
 * telemetry.h
 * 串口 CSV 遥测：每帧一行观测/解算/轴状态，另有事件行。被 main 调用，
 * 依赖 Arduino 串口与 aim_types.h。
 *
 * 数据行以 "AIM," 开头，字段顺序固定（PC 端脚本按此解析）：
 *   AIM,t_ms,state,obs_valid,px,py,conf,pan_deg,tilt_deg,pan_target,tilt_target,
 *   pan_rate,tilt_rate,enc_pan,enc_tilt,loop_us
 * 字段约定：
 *   state       AimState 的整数值（见 aim_types.h 枚举顺序）
 *   obs_valid   0/1
 *   角度用度，角速度用度/秒，时间用毫秒，loop_us 用微秒
 *   AIM_VERBOSE_TELEMETRY==0 时退化为低频心跳行，只确认固件存活。
 */

#include <cstdint>

#include "../aim_types.h"

// 初始化遥测串口。
void telemetryInit(uint32_t baud);

// 输出一帧遥测。loop_us 通过 telemetrySetLoopUs() 预先注入。
void telemetryEmit(const TargetObservation& obs, const TurretSolution& sol,
                   const AxisState& pan, const AxisState& tilt, AimState state);

// 注入本拍 loop() 耗时（微秒），供遥测行使用。
void telemetrySetLoopUs(uint32_t loop_us);

// 输出一行事件（带标签），用于状态机切换、故障、看门狗超预算等。
void telemetryEmitEvent(const char* tag, const char* msg);
