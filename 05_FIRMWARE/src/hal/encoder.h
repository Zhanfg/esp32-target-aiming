#pragma once
// 霍尔增量编码器：PCNT 硬件四倍频计数，输出累计计数、角度、角速度。
// 用 PCNT 而非 GPIO 中断是因为逐边沿进 ISR 在高速时会漏脉冲、角度漂移。
// 角度符号与 MOTOR_INVERT_* 联合标定；轴编号用 AXIS_PAN/AXIS_TILT。

#include <cstdint>

bool encoderInit();

int32_t encoderGetCount(uint8_t axis);
float encoderGetAngleDeg(uint8_t axis); // 零点为上电或 encoderReset 时刻
float encoderGetVelocityDps(uint8_t axis, float dt_s);
void encoderReset(uint8_t axis);
