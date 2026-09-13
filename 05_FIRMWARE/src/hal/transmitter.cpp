/**
 * transmitter.cpp
 * 占位实现（继电器/电机版本）。main 通过 Transmitter 基类指针调用。
 *
 * 时序：fire() 进入 FIRING 并输出脉冲；update() 到时关输出并进入 COOLDOWN；
 * 冷却结束后，仍武装则回到 ARMED，否则回到 SAFE。
 */

#include "transmitter.h"

#include <Arduino.h>

#include "config.h"

void MotorTransmitter::setOutput(bool active) {
    bool level = (TRANSMITTER_ACTIVE_LEVEL != 0) ? active : !active;
    digitalWrite(TRANSMITTER_PIN, level ? HIGH : LOW);
    output_active_ = active;
}

bool MotorTransmitter::init() {
    pinMode(TRANSMITTER_PIN, OUTPUT);
    // 初始化时强制输出无效电平，确保上电过程中发射器绝不动作。
    setOutput(false);
    armed_ = false;
    state_ = TransmitterState::SAFE;
    phase_start_ms_ = 0;
    return true;
}

bool MotorTransmitter::arm(bool enable) {
    if (state_ == TransmitterState::FAULT) {
        // 故障状态下必须先 emergencyStop()/重新 init() 才能武装。
        return false;
    }
    if (enable) {
        armed_ = true;
        if (state_ == TransmitterState::SAFE) {
            state_ = TransmitterState::ARMED;
        }
    } else {
        armed_ = false;
        setOutput(false);
        state_ = TransmitterState::SAFE;
    }
    return true;
}

bool MotorTransmitter::fire() {
    // 安全契约：未武装一律拒绝。
    if (state_ != TransmitterState::ARMED || !armed_) {
        return false;
    }
    setOutput(true);
    state_ = TransmitterState::FIRING;
    phase_start_ms_ = millis();
    return true;
}

void MotorTransmitter::update(uint32_t now_ms) {
    switch (state_) {
        case TransmitterState::FIRING:
            if ((uint32_t)(now_ms - phase_start_ms_) >= TRANSMITTER_FIRE_PULSE_MS) {
                setOutput(false);
                state_ = TransmitterState::COOLDOWN;
                phase_start_ms_ = now_ms;
            }
            break;
        case TransmitterState::COOLDOWN:
            if ((uint32_t)(now_ms - phase_start_ms_) >= TRANSMITTER_COOLDOWN_MS) {
                // 冷却结束：仍武装则回到待发，否则安全。
                state_ = armed_ ? TransmitterState::ARMED : TransmitterState::SAFE;
            }
            break;
        default:
            break;
    }
}

void MotorTransmitter::emergencyStop() {
    // 无条件立即切断：不判断当前状态、不等待时序。
    setOutput(false);
    armed_ = false;
    phase_start_ms_ = 0;
    state_ = TransmitterState::SAFE;
}
