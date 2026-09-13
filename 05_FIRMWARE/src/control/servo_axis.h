#pragma once
// servo_axis.h：单轴指向控制。无反馈（PWM 舵机）只做指令下发、机械限幅与角速度变化率限制；
// 有反馈（总线舵机）读回实际角后叠加一次位置校正。指令下发统一走 servoDrive()，不直接碰 LEDC/UART。

#include <cstdint>

#include "../aim_types.h"

bool servoAxisInit();

void servoAxisSetTargetDeg(uint8_t axis, float deg);
void servoAxisSetAxesDeg(float pan_deg, float tilt_deg);

// 固定节拍调用（CONTROL_LOOP_HZ），内部对两轴各下发一次。
void servoAxisUpdate(float dt_s);

AxisState servoAxisState(uint8_t axis);
void servoAxisGetState(AxisState& pan, AxisState& tilt);

bool servoAxisHasFeedback(uint8_t axis);

// 故障安全位：驱到 config.h 的安全角并保持力矩，避免 tilt 因重力下坠。
void servoAxisFaultSafe();

// 放弃力矩（PWM 实现停脉冲）。只在确认机械有阻尼或已落到安全位后调用。
void servoAxisEmergencyRelease();

bool servoAxisInDeadband();
float servoAxisPanErrorDeg();
float servoAxisTiltErrorDeg();
