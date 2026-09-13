#pragma once
/**
 * motor_driver.h
 * TB6612FNG 双路 H 桥底层驱动：把逻辑占空比 duty ∈ [-1,1] 转成 IN1/IN2 方向电平
 * 与 LEDC PWM，并处理极性、死区补偿、占空比上限和 STBY。
 *
 * 依赖 config.h 与 ESP-IDF LEDC 驱动，被 control 层调用。
 * 轴编号用 config.h 的 AXIS_PAN / AXIS_TILT。
 */

#include <cstdint>

// 初始化 LEDC 通道/频率/分辨率，配置方向脚，并把 STBY 拉高使能驱动。
// 返回 false 表示 LEDC 定时器配置失败（已在上层串口告警）。
bool motorDriverInit();

// 设置某轴逻辑占空比，范围 [-1.0, 1.0]。正负号表示方向（经 MOTOR_INVERT_* 修正）。
// 内部完成：上限保护、死区补偿、方向脚切换、PWM 写入。
void motorSetDuty(uint8_t axis, float duty);

// 短路制动（刹车）：IN1=IN2=1，PWM=0。停得快但会有制动电流与机械冲击。
void motorBrake(uint8_t axis);

// 惯性滑行（自由停车）：IN1=IN2=0，PWM=0。停得缓、无制动电流。
void motorCoast(uint8_t axis);

// 控制 TB6612 的 STBY：false 时两路输出立即失效，是硬件级安全切断。
void motorDriverSetStandby(bool enable);
