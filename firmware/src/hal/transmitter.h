#pragma once
/**
 * transmitter.h
 * 发射器抽象接口。当前用一路继电器/电机脉冲占位，换真实发射装置时另写一个
 * Transmitter 实现即可。
 *
 * 依赖 config.h 与 aim_types.h（TransmitterState）。control/main 只持有
 * Transmitter 基类指针，不知道具体实现，新增装置不用改 main 的状态机。
 *
 * 接口契约（实现必须遵守）：
 *   未 arm(true) 时 fire() 拒绝并返回 false；
 *   emergencyStop() 无条件立即生效，关闭输出并回到 SAFE，需重新 arm；
 *   init() 完成后处于 SAFE，不自动武装。
 */

#include <cstdint>

#include "../aim_types.h"

class Transmitter {
public:
    virtual ~Transmitter() {}

    // 初始化硬件并进入 SAFE。返回 false 表示初始化失败。
    virtual bool init() = 0;

    // 当前状态。
    virtual TransmitterState state() const = 0;

    // 武装/解除武装。true 仅在非 FAULT 时生效；false 立即回到 SAFE。
    virtual bool arm(bool enable) = 0;

    // 触发发射。仅当已武装时返回 true；否则拒绝并返回 false。
    virtual bool fire() = 0;

    // 推进内部时序（发射脉冲结束、冷却计时）。需被周期性调用。
    virtual void update(uint32_t now_ms) = 0;

    // 紧急停止：无条件立即切断输出并回到 SAFE。
    virtual void emergencyStop() = 0;
};

/**
 * MotorTransmitter：占位实现，用一路 GPIO（继电器/电机）脉冲模拟发射。
 *
 * 目的是让整机闭环与安全流程先跑通。接入真实发射器（激光、电磁、气动等）时，
 * 实现同一个 Transmitter 接口即可替换。
 */
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
    uint32_t phase_start_ms_ = 0; // FIRING / COOLDOWN 阶段的起始时刻
};
