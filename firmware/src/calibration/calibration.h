#pragma once
/**
 * calibration.h
 * 标定数据存取与换算：内参、畸变、像素→角度仿射、零点偏移，以及对应点最小二乘。
 * 持久化用 Preferences(NVS)。流程：打 N 组 (px,py)<->(pan,tilt) 点，N>=6 更稳；
 * 解算 2x3 仿射；RMSE 超约 1° 说明打点质量差或模型不合适，补点重打。
 */

#include <cstdint>

struct CalibrationData {
    int width = 0;
    int height = 0;
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float dist[5] = {0,0,0,0,0}; // (k1,k2,p1,p2,k3)

    // 像素齐次 [px,py,1] → 角度(deg) 的 2x3 仿射：
    //   bearing   = a[0][0]*px + a[0][1]*py + a[0][2]
    //   elevation = a[1][0]*px + a[1][1]*py + a[1][2]
    float affine[2][3] = {{0,0,0},{0,0,0}};

    float pan_offset_deg = 0.0f;
    float tilt_offset_deg = 0.0f;
    bool valid = false;
};

// 占位默认：fx=fy=0.8*width（约 64° 水平视场）、主点取中心、畸变 0、valid=false。
void calibrationSetDefaults(CalibrationData& out, int width, int height);

// NVS 逐字段读写（不用整体 blob，便于字段增删）。无有效记录返回 false。
bool calibrationLoad(CalibrationData& out);
bool calibrationSave(const CalibrationData& in);

// 像素 → 角度（度）。valid 时用仿射，否则回退针孔模型（反畸变迭代→归一化射线→角度）。
bool calibrationPixelToAngles(const CalibrationData& data, float px, float py,
                              float& bearing_deg, float& elevation_deg);

// 角度 → 舵盘轴角：叠加零点偏移。wrap_360 决定 pan 环绕归一还是夹到机械限位，tilt 恒夹紧。
bool calibrationAnglesToAxisDeg(const CalibrationData& data,
                                float bearing_deg, float elevation_deg,
                                bool wrap_360,
                                float& pan_deg, float& tilt_deg);

// 最小二乘拟合像素→角度 2x3 仿射。
//   pixel_pts/angle_pts：交错 [x0,y0,...] / [bearing0,elevation0,...]，各 n 组
//   rmse_out（可空）：输出 [bearing_rmse, elevation_rmse]（度）
// n<3 或法方程奇异（打点退化/共线）返回 false。
bool calibrationSolveAffine(const float* pixel_pts, const float* angle_pts, int n,
                            float affine_out[2][3], float rmse_out[2]);
