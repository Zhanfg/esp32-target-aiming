#pragma once
/**
 * angle_utils.h
 * 角度归一化、角差、限速趋近，控制、解算、标定共用。
 *
 * 只依赖 <cmath>，不碰硬件，也不读全局配置。
 *
 * 两种角差语义：
 *   线性：轴有硬限位，角度不跨 ±180° 边界，角差按 b-a 直接算，不回绕。tilt 用。
 *   环绕：轴可连续旋转（无硬止点，需配滑环），角差取最短路径，落在 (-180,180]；
 *         移动结果回绕到 [0,360)。pan 在启用 360° 时用。
 *
 * 环绕与否由调用方按轴的机械形态经 wrap_360 参数传入，本文件不做判断，与 Python
 * 侧 shortest_delta_deg(a, b, wrap_360) 一致。全局宏 ANGLE_WRAP_360_ENABLED 只在
 * turntable.cpp 组装 pan 的 AxisConfig 时读一次，决定该轴 wrap_360 的初值；
 * 本文件的函数不再依赖它。
 *
 * 调用纪律：角度相减一律走 angleDiffAxisDeg(a, b, wrap_360)，不写 a-b。
 */

// 把角度环绕归一到 [min_deg, max_deg)。
// 环绕轴用它把角度塞进任意连续区间；限位轴一般直接夹紧（见控制层）。
float angleNormalizeDeg(float deg, float min_deg, float max_deg);

// 把角度归一到半开区间 (-180, 180]（度）。环绕运算的基础。
// 约定 +180 保留为 +180，-180 也映射为 +180（两者物理等价）。
float angleNormalize180(float deg);

// 把角度归一到 [0, 360)。
float angleWrap360(float deg);

// 最短角差（环绕语义）：从 a 转到 b 的最短路径角差，结果在 (-180, 180]。
// 限位轴用它会绕远路，限位轴请用 angleDiffDegLinear()。
float angleDiffDeg(float a, float b);

// 线性角差（限位语义）：返回 b - a，不取最短路径、不回绕。
float angleDiffDegLinear(float a, float b);

// 按轴的机械形态选角差语义：wrap_360=true 走最短路径环绕分支，false 走线性分支。
// 调用方必须传自己那个轴的真实形态，不要拿全局开关代替。
inline float angleDiffAxisDeg(float a, float b, bool wrap_360) {
    return wrap_360 ? angleDiffDeg(a, b) : angleDiffDegLinear(a, b);
}

// 从 current 朝 target 趋近，单步位移不超过 max_step（度）。
// wrap_360=true：沿最短路径移动，结果回绕到 [0,360)，不绕远路；
// wrap_360=false：沿直线移动且不越过 target（|step|<=|delta| 且同号）。
// max_step 必须非负；为 0 时保持不动。
float angleMoveToward(float current, float target, float max_step, bool wrap_360);
