#pragma once
/**
 * axis.h
 * 单轴串级 PID：位置环（外）+ 速度环（内）。pan 和 tilt 复用同一个类。
 *
 *   位置误差 -> 位置 PID -> 速度设定（限幅到 MAX_SLEW_DPS）-> 速度 PID -> duty
 *
 * 另外负责软限位保护和 duty 变化率限制。依赖 math/pid、hal/motor_driver、
 * hal/encoder、math/angle_utils、config.h。增益等参数由调用方用 AxisConfig
 * 注入，这里不直接读 config 里的具体常量。
 *
 * 两种模式：
 *   POSITION  定点指向、锁定保持。
 *   VELOCITY  连续跟踪运动目标。目标一直在动时，位置环会反复启停，速度模式直接
 *             给角速度设定更平滑，也便于加前馈；定点仍用 POSITION。
 */

#include <cstdint>

#include "../aim_types.h"
#include "../math/pid.h"

// 轴工作模式。
enum class AxisMode {
    POSITION, // 位置环 + 速度环
    VELOCITY  // 仅速度环（角度仍被读取用于限位与遥测）
};

// 单轴配置（由上层按 config.h 填充，便于复用到不同轴/不同机型）。
struct AxisConfig {
    uint8_t axis_id = 0;
    float min_deg = -180.0f;
    float max_deg = 180.0f;
    float max_slew_dps = 180.0f;  // 位置环输出的速度设定上限
    float deadband_deg = 0.5f;    // 位置环死区
    float max_duty = 0.95f;       // 本轴占空比上限
    float duty_slew_per_s = 5.0f; // duty 每秒最大变化量
    bool wrap_360 = false;        // true=连续旋转轴（跳过硬软限位）

    // 位置环增益与积分限幅
    float pos_kp = 1.0f, pos_ki = 0.0f, pos_kd = 0.0f;
    float pos_i_min = -1.0f, pos_i_max = 1.0f;
    // 速度环增益与积分限幅
    float vel_kp = 1.0f, vel_ki = 0.0f, vel_kd = 0.0f;
    float vel_i_min = -1.0f, vel_i_max = 1.0f;
};

class Axis {
public:
    // 初始化：保存配置并配置两个 PID。返回 false 表示配置非法。
    bool init(uint8_t axis_id, const AxisConfig& cfg);

    // 设置位置目标（度）。切到 POSITION 模式时使用。
    void setTargetDeg(float deg);

    // 设置速度目标（度/秒）。切到 VELOCITY 模式时使用。
    void setTargetRate(float dps);

    // 执行一拍控制，返回本轴最新状态。需以固定 dt_s 周期调用。
    AxisState update(float dt_s);

    // 切换模式。切换时会 reset PID，避免旧积分/微分状态造成冲激。
    void setMode(AxisMode mode);

    // 紧急停止：立即滑行停车并复位 PID。不改变目标，便于恢复后继续。
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
