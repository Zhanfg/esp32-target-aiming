#pragma once
/**
 * pid.h
 * 通用单入单出 PID，位置环与速度环共用。只用 <cstdint>/<cmath>，可在主机端测试。
 *
 * 三处处理各针对一个问题：积分项单独限幅并在输出饱和时停积（抗饱和）；微分一阶低通
 * （编码器差分噪声大）；设定点附近死区输出 0（避免执行器来回蹭）。
 * 微分对测量值做（derivative on measurement），设定值阶跃不产生微分冲击。
 */
class Pid {
public:
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDeadband(float band);
    void setDerivativeLpfAlpha(float alpha); // α ∈ (0,1]，越小越平滑、相位滞后越大

    // setpoint=目标，measured=测量，dt_s 步长（秒）。返回限幅后的控制量。
    float update(float setpoint, float measured, float dt_s);

    // 由外部算好的误差驱动（可能经 angle_utils 环绕感知工具）。供位置环使用，
    // 误差 = angleDiffAxisDeg(当前角, 目标角, 本轴 wrap 标志)。
    float updateError(float error, float dt_s);

    void reset(); // 清零积分与微分状态

    float integral() const { return integral_; }

private:
    // 给定误差与 d(error)/dt 原始值，完成低通、抗积分饱和与限幅。
    float finalize(float error, float d_raw, float dt_s);

    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
    float out_min_ = -1.0f, out_max_ = 1.0f;
    float i_min_ = -1.0f, i_max_ = 1.0f;
    float deadband_ = 0.0f;
    float d_alpha_ = 0.2f;

    float integral_ = 0.0f;
    float d_lpf_ = 0.0f;
    float last_meas_ = 0.0f;
    bool has_last_ = false;
    float last_error_ = 0.0f;
    bool has_error_ = false;
};
