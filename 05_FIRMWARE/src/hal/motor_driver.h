#pragma once
// TB6612FNG 双路 H 桥底层：把 duty ∈ [-1,1] 转成 IN1/IN2 方向电平与 LEDC PWM，
// 处理极性、死区补偿、上限与 STBY。轴编号用 AXIS_PAN/AXIS_TILT。

#include <cstdint>

bool motorDriverInit();

void motorSetDuty(uint8_t axis, float duty);
void motorBrake(uint8_t axis);
void motorCoast(uint8_t axis);
void motorDriverSetStandby(bool enable); // false 时两路立即失效，硬件级切断
