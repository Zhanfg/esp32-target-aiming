#pragma once
/**
 * turntable.h
 * 把视觉观测（像素）转成双轴目标角，驱动 pan/tilt 闭环，输出 TurretSolution。
 *
 * 当前是"内联 alpha-beta 跟踪 + P 控制"的最小可用闭环，目的是让硬件先独立
 * 跑通；Python 解算器（AimSolver）移植后，这一段替换为 AimSolver 的输出。
 *
 * 被 main 调用，依赖 control/axis、calibration、math/angle_utils、hal/encoder、
 * config.h。
 *
 * 调用纪律：这一层的角度比较、相减、修正一律走 angle_utils，不写 a-b。
 * 切换 360° 环绕（ANGLE_WRAP_360_ENABLED）时只改 config，不用逐处排查裸减法。
 */

#include <cstdint>

#include "../aim_types.h"
#include "../calibration/calibration.h"

// 初始化两轴（读取 config.h 参数）。返回 false 表示轴初始化失败。
bool turretInit();

// 驱动一拍：输入最新观测（可无效）与当前时刻，返回解算结果。
// 该函数内部会调用两轴的 update()，需按固定节拍（CONTROL_LOOP_HZ）调用。
TurretSolution turretUpdate(const TargetObservation& obs, uint32_t now_ms);

// 直接设定两轴目标角（用于上电归零到安全位）。会打开"保持"标志，直到
// turretClearHold() 被调用前，turretUpdate 忽略视觉观测、只维持该目标。
void turretSetAxesDeg(float pan_deg, float tilt_deg);

// 清除保持标志，恢复视觉闭环控制。
void turretClearHold();

// 读取两轴最新状态。
void turretGetState(AxisState& pan, AxisState& tilt);

// 紧急停止：两轴立即滑行停车。
void turretEmergencyStop();

// 注入标定数据（main 加载后调用）。
void turretSetCalibration(const CalibrationData& cal);

// 两轴误差是否都在各自 deadband 内（供 main 统计 LOCKED 连续帧）。
bool turretInDeadband();

// 当前 pan/tilt 目标角误差（度），用于遥测/调试。
float turretPanErrorDeg();
float turretTiltErrorDeg();
