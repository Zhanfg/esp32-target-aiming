#pragma once
/**
 * vision.h
 * 取一帧，找出最大目标的质心与包围框，输出 TargetObservation。
 * 处理链：RGB565 -> 中心加权降采样到 <=160x120 工作缓冲(PSRAM) -> HSV 阈值
 * -> 连通域 -> 最大连通域的强度加权亚像素质心。
 *
 * 被 control/main 调用，依赖 esp32-camera、config.h、aim_types.h 和 ESP-IDF
 * 堆分配（PSRAM 优先）。
 *
 * 每条退出路径都会归还帧缓冲（RAII 守卫），不会泄漏 esp_camera_fb_t。
 */

#include <cstdint>

#include "../aim_types.h"

// 初始化摄像头与工作缓冲。缓冲优先分配在 PSRAM，失败回退内部 RAM。
// 返回 false 表示分配或摄像头初始化失败。
bool visionInit();

// 抓取并处理一帧。返回 true 表示检测到有效目标；out 始终被写入（无效时 valid=false）。
bool visionCapture(TargetObservation& out);

// 设置搜索 ROI（源图像像素坐标）。w/h<=0 表示恢复全画幅。
void visionSetRoi(int x, int y, int w, int h);

// 设置 HSV 阈值（H:0~180, S/V:0~255）。允许 H_MIN>H_MAX 表示跨 0° 的红色区间。
void visionSetThresholdHSV(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);
