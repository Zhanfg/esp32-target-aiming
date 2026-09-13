#pragma once
// 角度工具。角差：限位轴线性 b-a，环绕轴最短路径 (-180,180]；相减一律走 angleDiffAxisDeg，
// 且必须传本轴真实 wrap 形态，不要拿全局开关代替。归一整：180 到 (-180,180]，360 到 [0,360)。

float angleNormalizeDeg(float deg, float min_deg, float max_deg);
float angleNormalize180(float deg);
float angleWrap360(float deg);
float angleDiffDeg(float a, float b);
float angleDiffDegLinear(float a, float b);
inline float angleDiffAxisDeg(float a, float b, bool wrap_360) {
    return wrap_360 ? angleDiffDeg(a, b) : angleDiffDegLinear(a, b);
}
// 朝 target 趋近，单步不超过 max_step；wrap_360 走最短路径并回绕到 [0,360)。
float angleMoveToward(float current, float target, float max_step, bool wrap_360);
