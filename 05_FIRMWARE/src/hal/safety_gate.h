#pragma once
// safety_gate.h：软件侧互锁判定，是硬件与门之外的第二层；输入按开路即安全侧读取，
// safetyGateUpdate() 控制环每拍调用，上电默认全部禁止直到读到实际电平。

#include <cstdint>

#include "../aim_types.h"

void safetyGateInit();

void safetyGateUpdate(uint32_t now_ms);

// 原始电平，高=有效：FIRE_INHIBIT=true 表示安全否决生效、禁止释放；EXTERNAL_ALLOW=true 为上层许可。
bool safetyFireInhibitAsserted();
bool safetyExternalAllow();
bool safetyMechRecovered();
bool safetyMagIndexOk();
bool safetyPanHome();
bool safetyTiltHome();
bool safetyMagHome();

void safetyLatchArmed(bool armed);
bool safetyArmedLatched();

// Interlock 1：MECH_RECOVERED 才准 PRELOAD。
bool safetyCanEnterPreload();

// Interlock 3 + §6.6 与门树：FIRE_INHIBIT 未断言、EXTERNAL_ALLOW 有效、MECH_RECOVERED、ARMED 锁存同时成立才允许释放。
bool safetyCanRelease();

// Interlock 2：MAG_INDEX_OK 且机构已复位，才允许执行带载荷动作（Mode B）。
bool safetyCanIndexPayload();

// 释放门控自检：确认扩展器健康且锁存未置位时 canRelease() 为假；硬件与门本身的验证见 §6.11。
bool safetyReleaseGateSelfTest();
