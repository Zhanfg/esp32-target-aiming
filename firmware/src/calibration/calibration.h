#pragma once
/**
 * calibration.h
 * 标定数据的存取与换算：相机内参、畸变系数、像素到角度的仿射、零点偏移，
 * 以及对像素-角度对应点做最小二乘拟合。
 *
 * 被 vision/control/main 使用，依赖 math/angle_utils 与 config.h，持久化用
 * Arduino Preferences(NVS)。
 *
 * 标定步骤（人工）：
 *   1. 打点：把目标或标定板依次放到若干已知像素位置，记下每个位置对应的
 *      pan/tilt 角度（度），凑够 N 组 (px,py)<->(pan,tilt) 对应点，N>=6 更稳；
 *   2. 解算：calibrationSolveAffine() 拟合 2x3 仿射矩阵，即像素到角度的线性模型；
 *   3. 校验：看各分量 RMSE，超过约 1° 说明打点质量差或模型不合适，补点重打；
 *      需要时再叠加 pinhole 畸变模型。
 */

#include <cstdint>

struct CalibrationData {
    int width = 0;             // 标定时的图像宽（源分辨率，像素）
    int height = 0;            // 标定时的图像高
    float fx = 0.0f;           // 焦距 x（px）
    float fy = 0.0f;           // 焦距 y（px）
    float cx = 0.0f;           // 主点 x（px）
    float cy = 0.0f;           // 主点 y（px）
    float dist[5] = {0,0,0,0,0}; // 畸变系数 (k1,k2,p1,p2,k3)

    // 像素(齐次 [px,py,1]) → 角度(deg) 的 2x3 仿射矩阵：
    //   bearing_deg   = a[0][0]*px + a[0][1]*py + a[0][2]
    //   elevation_deg = a[1][0]*px + a[1][1]*py + a[1][2]
    float affine[2][3] = {{0,0,0},{0,0,0}};

    float pan_offset_deg = 0.0f;   // pan 零点偏移（度）
    float tilt_offset_deg = 0.0f;  // tilt 零点偏移（度）
    bool valid = false;            // 是否已通过标定/校验
};

// 用内置占位默认值填充：fx=fy=0.8*width（约 64° 水平视场）、主点取图像中心、
// 畸变系数为 0、valid=false。没有实测标定时应使用它，并进入 CALIBRATING。
void calibrationSetDefaults(CalibrationData& out, int width, int height);

// 从 NVS 逐字段读取标定数据（不使用整体 blob，便于后续字段增删与版本兼容）。
// 返回 false 表示无有效记录。
bool calibrationLoad(CalibrationData& out);

// 逐字段写入 NVS。返回 false 表示写入失败。
bool calibrationSave(const CalibrationData& in);

// 像素 → 角度（度）。
//   - 若 data.valid：使用已标定的 2x3 仿射线性模型；
//   - 否则回退到针孔模型：反畸变迭代 → 归一化射线 → bearing/elevation。
// 输出为相机系下的方位/俯仰角（度），向右/向上为正。
bool calibrationPixelToAngles(const CalibrationData& data, float px, float py,
                              float& bearing_deg, float& elevation_deg);

// 角度观测 -> 舵盘轴角度（度）：叠加 pan/tilt 零点偏移。wrap_360 决定 pan 是环绕
// 归一还是夹到机械限位；tilt 恒夹紧。返回 false 表示输入非有限。
bool calibrationAnglesToAxisDeg(const CalibrationData& data,
                                float bearing_deg, float elevation_deg,
                                bool wrap_360,
                                float& pan_deg, float& tilt_deg);

// 最小二乘拟合像素→角度 2x3 仿射。
//   pixel_pts：交错 [x0,y0,x1,y1,...] 共 n 组
//   angle_pts：交错 [bearing0,elevation0,...] 共 n 组
//   affine_out：输出 2x3 矩阵
//   rmse_out（可空）：输出 [bearing_rmse, elevation_rmse]（度）
// 返回 false 表示点数不足(n<3)或法方程奇异（打点退化，如所有点共线）。
bool calibrationSolveAffine(const float* pixel_pts, const float* angle_pts, int n,
                            float affine_out[2][3], float rmse_out[2]);
