/**
 * pid.cpp
 * pid.h 的实现，只用标准库。
 */

#include "pid.h"

#include <cmath>

void Pid::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::setOutputLimits(float min, float max) {
    // 容错：若调用方传反，自动交换，避免后续限幅逻辑失效。
    if (min > max) {
        float t = min; min = max; max = t;
    }
    out_min_ = min;
    out_max_ = max;
}

void Pid::setIntegralLimits(float min, float max) {
    if (min > max) {
        float t = min; min = max; max = t;
    }
    i_min_ = min;
    i_max_ = max;
}

void Pid::setDeadband(float band) {
    deadband_ = (band < 0.0f) ? -band : band;
}

void Pid::setDerivativeLpfAlpha(float alpha) {
    // 夹到 (0,1]，避免非法 α 让低通发散或完全失效。
    if (alpha <= 0.0f) alpha = 1.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    d_alpha_ = alpha;
}

void Pid::reset() {
    integral_ = 0.0f;
    d_lpf_ = 0.0f;
    last_meas_ = 0.0f;
    has_last_ = false;
    last_error_ = 0.0f;
    has_error_ = false;
}

// 公共收尾：给定误差与"已低通"的微分原始值，做抗积分饱和 + 限幅。
// d_raw 的符号约定为 d(error)/dt。
float Pid::finalize(float error, float d_raw, float dt_s) {
    // 死区：设定点附近直接输出 0 并冻结积分，避免执行器在死区边缘来回抖动。
    if (std::fabs(error) <= deadband_) {
        return 0.0f;
    }

    // 微分一阶低通：抑制量化噪声被微分放大。低通在死区判断之后更新，
    // 保证离开死区时微分状态是连续的。
    d_lpf_ += d_alpha_ * (d_raw - d_lpf_);
    float p_term = kp_ * error;
    float d_term = kd_ * d_lpf_;

    // 抗积分饱和（条件积分）：
    // 先试算"若接受本拍积分"的输出；只有当输出未朝饱和方向继续增大时才真正累加。
    // 为什么：若输出已经到上限、误差仍同号，继续累加积分会让积分无界增长，
    // 一旦误差反向，控制器需要很长时间才能"卸掉"积分，表现为严重过冲与回摆。
    float tentative_integral = integral_ + ki_ * error * dt_s;
    float tentative_out = p_term + tentative_integral + d_term;

    bool saturate_high = (tentative_out > out_max_) && (error > 0.0f);
    bool saturate_low = (tentative_out < out_min_) && (error < 0.0f);
    if (!saturate_high && !saturate_low) {
        integral_ = tentative_integral;
    }

    // 积分独立限幅：作为条件积分之外的第二道保险。
    if (integral_ > i_max_) integral_ = i_max_;
    if (integral_ < i_min_) integral_ = i_min_;

    float out = p_term + integral_ + d_term;
    if (out > out_max_) out = out_max_;
    if (out < out_min_) out = out_min_;
    return out;
}

float Pid::update(float setpoint, float measured, float dt_s) {
    // 非法步长：返回积分保持的输出，不做任何状态推进，避免除零/发散。
    if (!(dt_s > 1e-6f)) {
        float held = integral_;
        if (held > out_max_) held = out_max_;
        if (held < out_min_) held = out_min_;
        return held;
    }

    float error = setpoint - measured;
    // 微分对测量值求导（derivative on measurement），避免设定值突变造成微分冲击；
    // d(error)/dt = -d(measured)/dt。
    float d_raw = 0.0f;
    if (has_last_) {
        d_raw = -(measured - last_meas_) / dt_s;
    }
    last_meas_ = measured;
    has_last_ = true;
    return finalize(error, d_raw, dt_s);
}

float Pid::updateError(float error, float dt_s) {
    if (!(dt_s > 1e-6f) || !std::isfinite(error)) {
        float held = integral_;
        if (held > out_max_) held = out_max_;
        if (held < out_min_) held = out_min_;
        return held;
    }
    // 误差已在外部算好（可能走环绕感知工具），这里直接对误差微分。
    float d_raw = 0.0f;
    if (has_error_) {
        d_raw = (error - last_error_) / dt_s;
    }
    last_error_ = error;
    has_error_ = true;
    return finalize(error, d_raw, dt_s);
}
