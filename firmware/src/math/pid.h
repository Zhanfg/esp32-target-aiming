#pragma once
/**
 * pid.h
 * 通用单入单出 PID，位置环与速度环共用。
 *
 * 只用 <cstdint>/<cmath>，不依赖 Arduino 与硬件，可在主机端测试。
 *
 * update() 里有三处处理，各针对一个具体问题：
 *   1. 抗积分饱和：积分项单独限幅，输出饱和时停止积分累加；
 *   2. 微分一阶低通：编码器差分速度噪声大，直接微分会放大成抖动；
 *   3. 死区：设定点附近输出 0，避免执行器在目标附近来回蹭。
 *
 * 微分对测量值做（derivative on measurement），设定值阶跃时不会产生微分冲击；
 * 设定值恒定时，它与对误差微分等价。
 */
class Pid {
public:
    // 设置比例/积分/微分增益。
    void setGains(float kp, float ki, float kd);
    // 设置输出限幅（控制器输出物理量的上下限，如 duty 或 dps）。
    void setOutputLimits(float min, float max);
    // 设置积分项独立限幅（防止积分在长时间误差下无界增长）。
    void setIntegralLimits(float min, float max);
    // 设置死区宽度（误差绝对值小于该值时按 0 处理）。
    void setDeadband(float band);
    // 微分一阶低通系数 α ∈ (0,1]：越小越平滑、相位滞后越大。
    void setDerivativeLpfAlpha(float alpha);

    // 执行一步：setpoint=目标，measured=测量，dt_s=步长（秒）。
    // 返回限幅后的控制量。内部按 setpoint-measured 求误差，微分对测量值做。
    float update(float setpoint, float measured, float dt_s);

    // 由外部已算好的误差驱动（误差可能经 angle_utils 的环绕感知工具计算，
    // 不能退化成裸相减）。微分对误差本身做一阶低通。
    // 供位置环使用：误差 = angleDiffAxisDeg(当前角, 目标角, 本轴 wrap 标志)。
    float updateError(float error, float dt_s);

    // 清零积分与微分状态（模式切换/重新使能时调用，避免旧状态冲激）。
    void reset();

    float integral() const { return integral_; }

private:
    // 公共收尾：给定误差与微分原始值 d(error)/dt，完成低通、抗积分饱和与限幅。
    float finalize(float error, float d_raw, float dt_s);

    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
    float out_min_ = -1.0f, out_max_ = 1.0f;
    float i_min_ = -1.0f, i_max_ = 1.0f;
    float deadband_ = 0.0f;
    float d_alpha_ = 0.2f;

    float integral_ = 0.0f;   // 积分累加器
    float d_lpf_ = 0.0f;      // 微分低通状态
    float last_meas_ = 0.0f;  // 上一拍测量值
    bool has_last_ = false;   // 是否已有上一拍（首拍无法求微分）
    float last_error_ = 0.0f; // 上一拍误差（updateError 用）
    bool has_error_ = false;
};
