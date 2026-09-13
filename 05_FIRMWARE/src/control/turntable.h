#pragma once
/**
 * turntable.h
 * 视觉观测(像素) → 双轴目标角 → 驱动 pan/tilt 闭环，输出 TurretSolution。
 * 当前是内联 alpha-beta 跟踪 + P 控制的最小闭环，AimSolver 移植后替换这一段。
 * 角度比较、相减、修正一律走 angle_utils，不写裸减法。
 */

#include <cstdint>

#include "../aim_types.h"
#include "../calibration/calibration.h"

bool turretInit();

// 驱动一拍：输入最新观测（可无效）与当前时刻。内部调用两轴 update()，
// 需按 CONTROL_LOOP_HZ 固定节拍调用。
TurretSolution turretUpdate(const TargetObservation& obs, uint32_t now_ms);

// 直接设定两轴目标角（上电归零用）。会置保持标志，直到 turretClearHold()；
// 保持期间 turretUpdate 忽略视觉、只维持该目标。
void turretSetAxesDeg(float pan_deg, float tilt_deg);

void turretClearHold();
void turretGetState(AxisState& pan, AxisState& tilt);
void turretEmergencyStop();
void turretSetCalibration(const CalibrationData& cal);
bool turretInDeadband();
float turretPanErrorDeg();
float turretTiltErrorDeg();
