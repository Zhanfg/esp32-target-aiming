#pragma once
/**
 * aim_types.h
 * 跨模块数据结构。统一坐标/角度约定（全工程必须遵守）：
 *   图像坐标：x 向右、y 向下，原点左上角。
 *   相机坐标系：X 右、Y 下、Z 前。
 *   bearing = atan2(X, Z)，向右为正（俯视顺时针）。
 *   elevation = atan2(-Y, sqrt(X^2+Z^2))，向上为正。
 *   pan 向右转为正，tilt 上抬为正。
 *   长度 mm；角度内部弧度、模块间接口用度带 _deg；时间 _ms；角速度 _dps。
 */

#include <cstdint>

struct PixelPoint {
    float x = 0.0f;
    float y = 0.0f;
    float confidence = 0.0f; // [0,1]，0 表示无效/被遮挡
};

struct TargetObservation {
    bool valid = false;
    PixelPoint centroid; // 源图像像素坐标，亚像素
    float bbox_w = 0.0f;
    float bbox_h = 0.0f;
    uint32_t t_ms = 0;
};

struct AxisState {
    float position_deg = 0.0f;
    float velocity_dps = 0.0f;
    float target_deg = 0.0f;
    int32_t encoder_count = 0;
    bool at_limit = false;
};

struct TurretSolution {
    float pan_deg = 0.0f;
    float tilt_deg = 0.0f;
    float pan_rate_dps = 0.0f;  // 可作前馈
    float tilt_rate_dps = 0.0f; // 可作前馈
    bool valid = false;         // false 表示目标丢失/超时
    float confidence = 0.0f;    // [0,1]
    uint32_t t_ms = 0;
};

// 新增状态一律加在末尾：state 的整数值被遥测与 PC 端脚本直接使用，插值会平移后续取值。
enum class AimState {
    IDLE,
    CALIBRATING,
    SEARCHING,
    TRACKING,
    LOCKED,
    FAULT,
    CALIB_MODE
};

enum class TransmitterState {
    SAFE,
    ARMED,
    FIRING,
    COOLDOWN,
    FAULT
};
