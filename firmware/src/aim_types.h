#pragma once
/**
 * aim_types.h
 * 视觉、控制、通信之间传递的数据结构。不含算法与硬件依赖。
 *
 * 不依赖其它模块，也不 include <Arduino.h>，便于迁到 ESP-IDF 或做主机端单元测试。
 *
 * 统一坐标/角度约定（全工程必须遵守）：
 *   - 图像坐标：x 向右、y 向下，原点左上角。
 *   - 相机坐标系：X 右、Y 下、Z 前。
 *   - bearing（方位角）= atan2(X, Z)，向右为正（俯视顺时针）。
 *   - elevation（俯仰角）= atan2(-Y, sqrt(X^2+Z^2))，向上为正。
 *   - pan 角：正值 = 舵盘向右转；tilt 角：正值 = 摄像头上抬。
 *   - 长度 mm；角度内部用弧度、模块间接口用度并带 _deg；时间 _ms；角速度 _dps。
 */

#include <cstdint>

// 像素坐标点 + 置信度。x/y 单位为源图像像素，与标定内参同一分辨率。
struct PixelPoint {
    float x = 0.0f;          // 像素 x（向右为正）
    float y = 0.0f;          // 像素 y（向下为正）
    float confidence = 0.0f; // [0,1]，0 表示无效/被遮挡
};

// 一帧视觉观测结果。
struct TargetObservation {
    bool valid = false;      // 本帧是否检测到可用目标
    PixelPoint centroid;     // 目标质心（源图像像素坐标，亚像素）
    float bbox_w = 0.0f;     // 目标外接框宽（像素）
    float bbox_h = 0.0f;     // 目标外接框高（像素）
    uint32_t t_ms = 0;       // 观测时间戳（毫秒）
};

// 单轴实时状态。
struct AxisState {
    float position_deg = 0.0f;  // 当前角度（度）
    float velocity_dps = 0.0f;  // 当前角速度（度/秒）
    float target_deg = 0.0f;    // 当前目标角度（度）
    int32_t encoder_count = 0;  // 原始编码器累计计数（四倍频）
    bool at_limit = false;      // 是否触及软限位
};

// 舵盘解算输出：双轴目标角 + 指令角速度。valid=false 表示目标丢失/超时。
struct TurretSolution {
    float pan_deg = 0.0f;        // pan 目标角（度，向右为正）
    float tilt_deg = 0.0f;       // tilt 目标角（度，向上为正）
    float pan_rate_dps = 0.0f;   // pan 指令角速度（度/秒），可作前馈
    float tilt_rate_dps = 0.0f;  // tilt 指令角速度（度/秒），可作前馈
    bool valid = false;          // 解是否可用
    float confidence = 0.0f;     // [0,1] 置信度
    uint32_t t_ms = 0;           // 解算时间戳（毫秒）
};

// 固件顶层状态机的状态。
enum class AimState {
    IDLE,         // 空闲：已初始化，未搜索
    CALIBRATING,  // 标定中/标定无效
    SEARCHING,    // 搜索目标：无有效观测
    TRACKING,     // 跟踪中：有观测但未稳定锁定
    LOCKED,       // 已锁定：连续多帧误差在死区内
    FAULT         // 故障：摄像头失败/看门狗超时等，输出强制切断
};

// 发射器状态。
enum class TransmitterState {
    SAFE,      // 未武装，禁止发射
    ARMED,     // 已武装，可发射
    FIRING,    // 发射脉冲进行中
    COOLDOWN,  // 发射后冷却
    FAULT      // 故障，已强制切断
};
