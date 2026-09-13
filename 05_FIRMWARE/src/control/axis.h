#pragma once
/**
 * axis.h
 * 单轴串级 PID：位置外环 → 速度内环。pan/tilt 复用同一个类。
 *   位置误差 → 位置 PID → 速度设定（限幅到 max_slew_dps）→ 速度 PID → duty
 * 另负责软限位与 duty 变化率限制。增益等由调用方用 AxisConfig 注入。
 *
 * 两种模式：POSITION 定点指向/锁定保持；VELOCITY 连续跟踪运动目标（直接给角速度
 * 设定更平滑，也便于加前馈；定点仍用 POSITION）。
 */

#include <cstdint>

#include "../aim_types.h"
#include "../math/pid.h"

enum class AxisMode {
    POSITION, // 位置环 + 速度环
    VELOCITY  // 仅速度环（角度仍被读取用于限位与遥测）
};

// 单轴配置，由上层按 config.h 填充，便于复用到不同轴/机型。
struct AxisConfig {
    uint8_t axis_id = 0;
    float min_deg = -180.0f;
    float max_deg = 180.0f;
    float max_slew_dps = 180.0f;  // 位置环输出的速度设定上限
    float deadband_deg = 0.5f;
    float max_duty = 0.95f;       // 本轴占空比上限
    float duty_slew_per_s = 5.0f; // duty 每秒最大变化量
    bool wrap_360 = false;        // true=连续旋转轴，跳过硬软限位

    float pos_kp = 1.0f, pos_ki = 0.0f, pos_kd = 0.0f;
    float pos_i_min = -1.0f, pos_i_max = 1.0f;
    float vel_kp = 1.0f, vel_ki = 0.0f, vel_kd = 0.0f;
    float vel_i_min = -1.0f, vel_i_max = 1.0f;
};

class Axis {
public:
    bool init(uint8_t axis_id, const AxisConfig& cfg);

    void setTargetDeg(float deg);
    void setTargetRate(float dps); // 速度模式使用
    AxisState update(float dt_s);  // 以固定 dt_s 周期调用

    // 切换模式，切换时 reset PID，避免旧积分/微分造成冲激。
    void setMode(AxisMode mode);

    // 紧急停止：立即滑行停车并复位 PID，不改变目标，便于恢复后继续。
    void emergencyStop();

    AxisMode mode() const { return mode_; }
    const AxisState& state() const { return state_; }

private:
    AxisConfig cfg_;
    Pid pos_pid_;
    Pid vel_pid_;
    AxisMode mode_ = AxisMode::POSITION;
    float target_deg_ = 0.0f;
    float target_rate_dps_ = 0.0f;
    float last_duty_ = 0.0f;
    AxisState state_;
    bool initialized_ = false;
};
