#pragma once
// servo.h：云台舵机抽象，SERVO_DRIVE_TYPE 编译期选实现。ServoPwm 出 50Hz 三线 PWM 无反馈，
// ServoBus 为总线占位（hasFeedback()=true）。两实现下发前按 config.h 机械限幅夹紧。

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

ServoDrive& servoDrive();
