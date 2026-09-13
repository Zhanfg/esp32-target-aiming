/**
 * angle_utils.cpp
 *
 * 输入为 NaN/Inf 时行为未定义，调用方（视觉等）需先保证有限。
 */

#include "angle_utils.h"

#include <cmath>

// 内部：把 x 输入 fmodf 并保证结果非负，因为 C 的 fmodf 对手负数会返回负值。
static inline float positiveFmod(float x, float y) {
    float r = std::fmod(x, y);
    if (r < 0.0f) {
        r += y;
    }
    return r;
}

float angleWrap360(float deg) {
    return positiveFmod(deg, 360.0f); // [0, 360)
}

float angleNormalize180(float deg) {
    // 先折到 [0,360)，再把 (180,360) 的部分减 360 → (-180,180]。
    // 180 恰好在边界上保持不变，符合"(-180,180]"的半开约定。
    float n = angleWrap360(deg);
    if (n > 180.0f) {
        n -= 360.0f;
    }
    return n;
}

float angleNormalizeDeg(float deg, float min_deg, float max_deg) {
    // 环绕归一到 [min,max)。与 angleWrap360 的区别是允许任意区间而不仅是 [0,360)。
    // 若区间退化（max<=min）直接返回归一后的 min，避免除零。
    float span = max_deg - min_deg;
    if (span <= 0.0f) {
        return min_deg;
    }
    return min_deg + positiveFmod(deg - min_deg, span);
}

float angleDiffDeg(float a, float b) {
    // 环绕分支：最短路径角差，先相减再归一。裸减在 +179° 转 -179° 时得 -358°，
    // 实际只需 +2°。
    return angleNormalize180(b - a);
}

float angleDiffDegLinear(float a, float b) {
    // 线性分支：限位轴用，保留原始数值（可能超过 ±180），不做最短路径处理。
    return b - a;
}

float angleMoveToward(float current, float target, float max_step, bool wrap_360) {
    if (max_step < 0.0f) {
        max_step = 0.0f; // 非法输入按保持不动处理
    }
    if (wrap_360) {
        // 沿最短路径移动并回绕到 [0,360)，避免从 359° 到 1° 绕一整圈。
        float delta = angleDiffDeg(current, target);
        float step = delta;
        if (step > max_step) step = max_step;
        if (step < -max_step) step = -max_step;
        return angleWrap360(current + step);
    }
    // 直线趋近，不越过 target。
    float delta = target - current;
    float step = delta;
    if (step > max_step) step = max_step;
    if (step < -max_step) step = -max_step;
    return current + step;
}
