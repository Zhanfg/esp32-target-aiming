#pragma once
/**
 * safety_gate.h
 * 软件侧互锁判定。这是硬件与门之外的第二层，不替代释放路径上的物理与门。
 * 所有输入按开路即安全侧读取：读失败、总线超时、电平不确定一律返回禁止。
 * 上电默认全部禁止，直到 safetyGateUpdate() 读到实际电平。
 */

#include <cstdint>

#include "../aim_types.h"

void safetyGateInit();

// 采样一次扩展器输入，刷新缓存。控制环每拍调用。
void safetyGateUpdate(uint32_t now_ms);

// 原始电平：FIRE_INHIBIT / EXTERNAL_ALLOW 有效为高。
bool safetyFireInhibitAsserted(); // true=安全否决生效，禁止释放
bool safetyExternalAllow();       // true=上层许可
bool safetyMechRecovered();
bool safetyMagIndexOk();
bool safetyPanHome();
bool safetyTiltHome();
bool safetyMagHome();

// ARMED 锁存，参与 H1 与门树。
void safetyLatchArmed(bool armed);
bool safetyArmedLatched();

// Interlock 1：MECH_RECOVERED 才准 PRELOAD。
bool safetyCanEnterPreload();

// Interlock 3 + §6.6 与门树：FIRE_INHIBIT 未断言、EXTERNAL_ALLOW 有效、
// MECH_RECOVERED、ARMED 锁存同时成立才允许释放。
bool safetyCanRelease();

// Interlock 2：MAG_INDEX_OK 且机构已复位，才允许执行带载荷动作（Mode B）。
bool safetyCanIndexPayload();

// 释放门控自检：确认扩展器健康且锁存未置位时 canRelease() 为假。
// 硬件与门本身的验证要在台架上做，见架构文档 §6.11。
bool safetyReleaseGateSelfTest();
