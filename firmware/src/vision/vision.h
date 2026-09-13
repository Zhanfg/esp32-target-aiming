#pragma once
// 视觉感知接口。取帧→中心加权降采样→HSV 阈值→连通域→强度加权亚像素质心。
// 工作缓冲优先 PSRAM，失败回退内部 RAM。H_MIN>H_MAX 表示跨 0° 的红色区间。

#include <cstdint>

#include "../aim_types.h"

bool visionInit();

// 检测到目标返回 true；out 始终被写入（无效时 valid=false）。
bool visionCapture(TargetObservation& out);

void visionSetRoi(int x, int y, int w, int h); // w/h<=0 恢复全画幅
void visionSetThresholdHSV(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);
