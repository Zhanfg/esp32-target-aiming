#pragma once
/**
 * transmitter.h
 * 发射器抽象接口。换装置时另写一个 Transmitter 实现，control/main 只持基类指针。
 * 契约：未 arm(true) 时 fire() 拒绝并返回 false；emergencyStop() 无条件生效并回 SAFE；
 * init() 后处于 SAFE，不自动武装。
 */

#include <cstdint>

#include "../aim_types.h"

class Transmitter {
public:
    virtual ~Transmitter() {}

    virtual bool init() = 0;
    virtual TransmitterState state() const = 0;
    virtual bool arm(bool enable) = 0; // false 立即回 SAFE
    virtual bool fire() = 0;
    virtual void update(uint32_t now_ms) = 0;
    virtual void emergencyStop() = 0;
};

// 占位实现：一路 GPIO（继电器/电机）脉冲模拟发射，用于跑通整机闭环与安全流程。
class MotorTransmitter : public Transmitter {
public:
    bool init() override;
    TransmitterState state() const override { return state_; }
    bool arm(bool enable) override;
    bool fire() override;
    void update(uint32_t now_ms) override;
    void emergencyStop() override;

private:
    void setOutput(bool active);

    TransmitterState state_ = TransmitterState::SAFE;
    bool armed_ = false;
    bool output_active_ = false;
    uint32_t phase_start_ms_ = 0;
};
