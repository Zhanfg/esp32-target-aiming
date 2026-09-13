/**
 * axis.cpp
 * axis.h 的实现。下层用到 hal（motor、encoder）与 math（pid、angle_utils）。
 */

#include "axis.h"

#include <cmath>

#include "config.h"
#include "../hal/encoder.h"
#include "../hal/motor_driver.h"
#include "../math/angle_utils.h"

bool Axis::init(uint8_t axis_id, const AxisConfig& cfg) {
    cfg_ = cfg;
    cfg_.axis_id = axis_id;

    // 位置环输出即速度设定，因此输出限幅 = 角速度上限。
    pos_pid_.setGains(cfg_.pos_kp, cfg_.pos_ki, cfg_.pos_kd);
    pos_pid_.setOutputLimits(-cfg_.max_slew_dps, cfg_.max_slew_dps);
    pos_pid_.setIntegralLimits(cfg_.pos_i_min, cfg_.pos_i_max);
    pos_pid_.setDeadband(cfg_.deadband_deg);
    pos_pid_.setDerivativeLpfAlpha(0.2f);

    // 速度环输出即 duty，因此输出限幅 = 占空比上限。
    vel_pid_.setGains(cfg_.vel_kp, cfg_.vel_ki, cfg_.vel_kd);
    vel_pid_.setOutputLimits(-cfg_.max_duty, cfg_.max_duty);
    vel_pid_.setIntegralLimits(cfg_.vel_i_min, cfg_.vel_i_max);
    vel_pid_.setDeadband(0.0f); // 速度环不需要死区，否则低速无法建立控制量
    vel_pid_.setDerivativeLpfAlpha(0.2f);

    pos_pid_.reset();
    vel_pid_.reset();
    last_duty_ = 0.0f;
    initialized_ = true;
    return true;
}

void Axis::setTargetDeg(float deg) {
    target_deg_ = deg;
}

void Axis::setTargetRate(float dps) {
    target_rate_dps_ = dps;
}

void Axis::setMode(AxisMode mode) {
    if (mode != mode_) {
        mode_ = mode;
        // 模式切换时清 PID 状态，避免位置环的大积分在速度模式下突然释放。
        pos_pid_.reset();
        vel_pid_.reset();
    }
}

void Axis::emergencyStop() {
    motorCoast(cfg_.axis_id);
    pos_pid_.reset();
    vel_pid_.reset();
    last_duty_ = 0.0f;
}

AxisState Axis::update(float dt_s) {
    if (!initialized_ || !(dt_s > 1e-6f)) {
        return state_;
    }

    const float pos_deg = encoderGetAngleDeg(cfg_.axis_id);
    const float vel_dps = encoderGetVelocityDps(cfg_.axis_id, dt_s);

    float duty = 0.0f;

    if (mode_ == AxisMode::POSITION) {
        // 角差按本轴机械形态选语义：传 cfg_.wrap_360，与下面软限位判定同一个标志。
        float err_deg = angleDiffAxisDeg(pos_deg, target_deg_, cfg_.wrap_360);
        // 位置 PID 由"已算好的误差"驱动，输出速度设定（已按 max_slew_dps 限幅）。
        float desired_rate = pos_pid_.updateError(err_deg, dt_s);
        // 速度环的设定与反馈都是角速度（非角度），直接相减是正确的，无需环绕处理。
        duty = vel_pid_.update(desired_rate, vel_dps, dt_s);
    } else { // VELOCITY
        duty = vel_pid_.update(target_rate_dps_, vel_dps, dt_s);
    }

    // 占空比上限（本轴）。
    if (duty > cfg_.max_duty) duty = cfg_.max_duty;
    if (duty < -cfg_.max_duty) duty = -cfg_.max_duty;

    // 软限位：接近限位时只允许朝安全方向运动。
    // 环绕轴（wrap_360）没有硬止点，跳过该保护。
    bool at_limit = false;
    if (!cfg_.wrap_360) {
        if (pos_deg >= cfg_.max_deg - SOFT_LIMIT_MARGIN_DEG) {
            at_limit = true;
            if (duty > 0.0f) duty = 0.0f;
        } else if (pos_deg <= cfg_.min_deg + SOFT_LIMIT_MARGIN_DEG) {
            at_limit = true;
            if (duty < 0.0f) duty = 0.0f;
        }
    }

    // duty 变化率限制：抑制电流冲击与齿轮背隙冲击。
    float max_delta = cfg_.duty_slew_per_s * dt_s;
    float delta = duty - last_duty_;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    duty = last_duty_ + delta;
    last_duty_ = duty;

    motorSetDuty(cfg_.axis_id, duty);

    state_.position_deg = pos_deg;
    state_.velocity_dps = vel_dps;
    state_.target_deg = target_deg_;
    state_.encoder_count = encoderGetCount(cfg_.axis_id);
    state_.at_limit = at_limit;
    return state_;
}
