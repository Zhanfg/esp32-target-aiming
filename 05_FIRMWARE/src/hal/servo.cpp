/**
 * servo.cpp
 * PWM 实现用 LEDC 出 50Hz 舵机信号；总线实现是占位骨架。
 *
 * 脉宽到占空比的换算：
 *   duty = 脉宽(us) × 1e-6 × 频率(Hz) × (2^分辨率 - 1)
 * 50Hz 周期对应 (2^16 - 1) 个计数，1500us 中位约 4915。
 */

#include "servo.h"

#include <Arduino.h>
#include <driver/ledc.h>
#include <cmath>

#include "config.h"

namespace {

// 把角度按本轴机械限幅夹紧。无反馈舵机指令越界会撞止挡，必须在最底层拦住。
float clampAxisDeg(uint8_t axis, float deg) {
    const float lo = (axis == AXIS_PAN) ? PAN_MIN_DEG : TILT_MIN_DEG;
    const float hi = (axis == AXIS_PAN) ? PAN_MAX_DEG : TILT_MAX_DEG;
    if (!std::isfinite(deg)) return lo; // 非有限值一律收到最小角，避免野值
    if (deg < lo) return lo;
    if (deg > hi) return hi;
    return deg;
}

float axisInvert(uint8_t axis) {
    if (axis == AXIS_PAN) return (SERVO_INVERT_PAN != 0) ? -1.0f : 1.0f;
    return (SERVO_INVERT_TILT != 0) ? -1.0f : 1.0f;
}

float axisServoMinDeg(uint8_t axis) {
    return (axis == AXIS_PAN) ? SERVO_PAN_MIN_DEG : SERVO_TILT_MIN_DEG;
}

float axisServoMaxDeg(uint8_t axis) {
    return (axis == AXIS_PAN) ? SERVO_PAN_MAX_DEG : SERVO_TILT_MAX_DEG;
}

class ServoPwm : public ServoDrive {
public:
    bool init() override {
        ledc_timer_config_t tcfg = {};
        tcfg.speed_mode = LEDC_LOW_SPEED_MODE;
        tcfg.duty_resolution = (ledc_timer_bit_t)SERVO_PWM_RESOLUTION_BITS;
        tcfg.timer_num = (ledc_timer_t)SERVO_LEDC_TIMER;
        tcfg.freq_hz = SERVO_PWM_FREQ_HZ;
        tcfg.clk_cfg = LEDC_AUTO_CLK;
        if (ledc_timer_config(&tcfg) != ESP_OK) {
            Serial.println("[servo] LEDC 定时器配置失败");
            return false;
        }

        const int channels[AXIS_COUNT] = { SERVO_PAN_LEDC_CHANNEL, SERVO_TILT_LEDC_CHANNEL };
        const int pins[AXIS_COUNT] = { SERVO_PAN_PIN, SERVO_TILT_PIN };
        for (int i = 0; i < AXIS_COUNT; ++i) {
            ledc_channel_config_t ccfg = {};
            ccfg.gpio_num = pins[i];
            ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
            ccfg.channel = (ledc_channel_t)channels[i];
            ccfg.timer_sel = (ledc_timer_t)SERVO_LEDC_TIMER;
            ccfg.duty = 0; // 先无脉冲，等第一次写角度再输出
            ccfg.hpoint = 0;
            if (ledc_channel_config(&ccfg) != ESP_OK) {
                Serial.printf("[servo] LEDC 通道配置失败 channel=%d\n", channels[i]);
                return false;
            }
        }
        initialized_ = true;
        torque_ = false; // 上电默认失能，归零流程开始时再使能
        return true;
    }

    void writePanDeg(float deg) override { write(AXIS_PAN, deg); }
    void writeTiltDeg(float deg) override { write(AXIS_TILT, deg); }

    float readPanDeg() override { return last_deg_[AXIS_PAN]; }
    float readTiltDeg() override { return last_deg_[AXIS_TILT]; }

    bool hasFeedback() const override { return false; }

    void setTorqueEnabled(bool enable) override {
        if (!initialized_) return;
        torque_ = enable;
        if (enable) {
            // 重新使能时恢复上次指令角，避免中间态。
            output(AXIS_PAN, last_deg_[AXIS_PAN]);
            output(AXIS_TILT, last_deg_[AXIS_TILT]);
        } else {
            outputRaw(SERVO_PAN_LEDC_CHANNEL, 0);
            outputRaw(SERVO_TILT_LEDC_CHANNEL, 0);
        }
    }

    void emergencyRelease() override {
        torque_ = false;
        if (!initialized_) return;
        outputRaw(SERVO_PAN_LEDC_CHANNEL, 0);
        outputRaw(SERVO_TILT_LEDC_CHANNEL, 0);
    }

private:
    void write(uint8_t axis, float deg) {
        last_deg_[axis] = clampAxisDeg(axis, deg);
        if (initialized_ && torque_) {
            output(axis, last_deg_[axis]);
        }
    }

    void output(uint8_t axis, float clamped_deg) {
        const int channel = (axis == AXIS_PAN) ? SERVO_PAN_LEDC_CHANNEL : SERVO_TILT_LEDC_CHANNEL;
        outputRaw(channel, angleToPulseUs(axis, clamped_deg));
    }

    static uint32_t angleToPulseUs(uint8_t axis, float clamped_deg) {
        const float lo = axisServoMinDeg(axis);
        const float hi = axisServoMaxDeg(axis);
        float t = 0.0f;
        if (hi > lo) {
            // 先套方向极性，再按电气行程归一。方向只影响斜率符号。
            const float d = clamped_deg * axisInvert(axis);
            t = (d - lo) / (hi - lo);
        }
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        const float pulse = SERVO_PULSE_MIN_US + t * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
        const uint32_t max_duty = (1u << SERVO_PWM_RESOLUTION_BITS) - 1u;
        const double duty = (double)pulse * 1e-6 * (double)SERVO_PWM_FREQ_HZ * (double)max_duty;
        uint32_t v = (uint32_t)(duty + 0.5);
        if (v > max_duty) v = max_duty;
        return v;
    }

    static void outputRaw(int channel, uint32_t duty) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
    }

    bool initialized_ = false;
    bool torque_ = false;
    float last_deg_[AXIS_COUNT] = { 0.0f, 0.0f };
};

// 总线舵机占位。采购选型后需要补齐：
//   1) 帧格式：帧头、ID、指令字、参数长度、校验（和校验或 CRC）与半双工收发切换时序；
//   2) 波特率：与 SERVO_BUS_BAUD 对应，确认舵机支持；
//   3) 读位置与负载的命令字：用于 readPanDeg/readTiltDeg 与失速判定；
//   4) 使能与释能的命令字，映射到 setTorqueEnabled/emergencyRelease。
// 协议未定前 init() 直接返回 false，让上层进 FAULT，不臆造任何字节序列。
class ServoBus : public ServoDrive {
public:
    bool init() override {
        Serial.println("[servo] 总线舵机协议未实现，拒绝初始化（协议未定，见 servo.cpp 注释）");
        return false;
    }

    void writePanDeg(float deg) override { last_deg_[AXIS_PAN] = clampAxisDeg(AXIS_PAN, deg); }
    void writeTiltDeg(float deg) override { last_deg_[AXIS_TILT] = clampAxisDeg(AXIS_TILT, deg); }

    // 接入后应从总线回报位置；未接入前返回上次指令值。
    float readPanDeg() override { return last_deg_[AXIS_PAN]; }
    float readTiltDeg() override { return last_deg_[AXIS_TILT]; }

    bool hasFeedback() const override { return true; }

    void setTorqueEnabled(bool enable) override { torque_ = enable; }

    void emergencyRelease() override { torque_ = false; }

private:
    bool torque_ = false;
    float last_deg_[AXIS_COUNT] = { 0.0f, 0.0f };
};

#if SERVO_DRIVE_TYPE == SERVO_DRIVE_BUS
ServoBus s_drive;
#else
ServoPwm s_drive;
#endif

} // namespace

ServoDrive& servoDrive() { return s_drive; }
