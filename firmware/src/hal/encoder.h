#pragma once
/**
 * encoder.h
 * 霍尔增量编码器读取。用 PCNT 对 A/B 相做硬件四倍频计数，对外给累计计数、
 * 角度和角速度。软件只把 16 位计数值跨拍累加并换算，不逐脉冲处理。
 *
 * 依赖 config.h 与 ESP-IDF PCNT 驱动，被 control 层调用。
 *
 * 不用 GPIO 中断计数的原因：每个边沿都要进 ISR，高速时占用大量 CPU，中断延迟
 * 下还容易漏脉冲，角度会慢慢漂；PCNT 在硬件里检测边沿并计数，CPU 只需低频读取。
 *
 * 轴编号用 config.h 的 AXIS_PAN / AXIS_TILT。
 */

#include <cstdint>

// 配置两个 PCNT 单元（pan/tilt，各用 channel0+channel1 做四倍频）并清零计数。
bool encoderInit();

// 读取累计计数（int32，已跨 16 位计数上限累加）。内部做了临界区保护，可原子读取。
int32_t encoderGetCount(uint8_t axis);

// 由累计计数换算输出轴角度（度）。符号与 MOTOR_INVERT_* 联合标定，
// 保证"正占空比 → 角度增大"。零点为上电/encoderReset 时刻。
float encoderGetAngleDeg(uint8_t axis);

// 由角度差分 + 一阶低通估计角速度（度/秒）。dt_s 为两次调用间隔。
float encoderGetVelocityDps(uint8_t axis, float dt_s);

// 清零累计计数、硬件计数器与速度滤波器状态（上电零位标定、重新寻零时调用）。
void encoderReset(uint8_t axis);
