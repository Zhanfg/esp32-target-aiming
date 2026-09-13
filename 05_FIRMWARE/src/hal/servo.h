#pragma once
/**
 * servo.h
 * 云台舵机抽象。用 SERVO_DRIVE_TYPE 在编译期选实现，采购前不必冻结选型：
 *   ServoPwm：经典三线 PWM，LEDC 输出 50Hz、脉宽 SERVO_PULSE_MIN_US..MAX_US，无反馈。
 *   ServoBus：总线舵机占位，协议未定，只保留接口与骨架，hasFeedback() 为 true。
 * 两实现在下发前都把角度按 config.h 的机械限幅夹紧。
 */

#include <cstdint>

class ServoDrive {
public:
    virtual ~ServoDrive() {}

    virtual bool init() = 0;

    virtual void writePanDeg(float deg) = 0;
    virtual void writeTiltDeg(float deg) = 0;

    // 无反馈实现返回上次指令值（位置不可测，只能给出估计），hasFeedback() 区分。
    virtual float readPanDeg() = 0;
    virtual float readTiltDeg() = 0;

    virtual bool hasFeedback() const = 0;

    // 使能/失能力矩。失能时 PWM 实现停止脉冲，舵机自由。
    virtual void setTorqueEnabled(bool enable) = 0;

    // 紧急释能：无条件立即失能，不等待、不读状态。
    virtual void emergencyRelease() = 0;
};

// 返回编译期选定的实现（单例）。
ServoDrive& servoDrive();
