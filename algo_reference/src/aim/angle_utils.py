"""角度工具函数，单位统一为度，与 C++ `angle_utils` 逐函数对应。

带 wrap_360 参数的函数有线性与环绕两种模式：
- 线性模式（wrap_360=False）：轴有硬限位，角度不跨 ±180° 边界。角差直接按 b - a 计算，
  不取最短路径，也不回绕。tilt 这类行程有限的轴用这一支。
- 环绕模式（wrap_360=True）：轴可连续旋转，无硬止点，角度按 360° 周期处理。角差取最短路径
  （结果落在 (-180, 180]），移动后回绕到 [0, 360)。这是本项目预留 360° 能力的落点。
"""

from __future__ import annotations

import numpy as np


def normalize_180(deg: float) -> float:
    """把角度归一到半开区间 (-180, 180]，单位度。

    180 保留为 +180，-180 也映射为 +180，两者等价。环绕模式用它把任意角差压到最短路径区间，
    线性模式不使用。
    """
    scalar = np.isscalar(deg)
    a = np.asarray(deg, dtype=np.float64)
    n = np.mod(a, 360.0)          # [0, 360)
    n = np.where(n > 180.0, n - 360.0, n)  # ( -180, 180 ]
    return float(n) if scalar else n


def wrap_360(deg: float) -> float:
    """把角度归一到 [0, 360)，单位度。环绕模式的基础操作。"""
    scalar = np.isscalar(deg)
    n = np.mod(np.asarray(deg, dtype=np.float64), 360.0)
    return float(n) if scalar else n


# move_toward 的形参也叫 wrap_360，会遮蔽同名函数，这里先留一份函数引用。
_wrap_360_deg = wrap_360


def normalize_to_range(deg: float, lo: float, hi: float) -> float:
    """把角度环绕归一到 [lo, hi)，单位度。

    与 wrap_360 的区别是区间任意，不限于 [0, 360)。环绕模式下用来把角度塞进某个连续旋转轴
    的允许区间。
    """
    scalar = np.isscalar(deg)
    a = np.asarray(deg, dtype=np.float64)
    span = hi - lo
    n = np.mod(a - lo, span) + lo
    return float(n) if scalar else n


def shortest_delta_deg(a: float, b: float, wrap_360: bool = False) -> float:
    """返回从角度 a 转到角度 b 的角差，单位度。

    wrap_360=True 时取最短路径，结果落在 (-180, 180]；wrap_360=False 时直接返回 b - a，
    数值可能超过 ±180。两种模式的语义见模块顶部。
    """
    if wrap_360:
        return normalize_180(np.asarray(b, dtype=np.float64) - np.asarray(a, dtype=np.float64))
    scalar = np.isscalar(a) and np.isscalar(b)
    d = np.asarray(b, dtype=np.float64) - np.asarray(a, dtype=np.float64)
    return float(d) if scalar else d


def move_toward(current: float, target: float, max_step: float, wrap_360: bool = False) -> float:
    """从 current 朝 target 移动，单步位移不超过 max_step，单位度。

    环绕模式沿最短路径移动，结果回绕到 [0, 360)，不会绕远路；线性模式沿直线移动，保证不越过
    target（|step| <= |delta| 且同号）。max_step 必须非负，为 0 时保持不动。
    """
    if max_step < 0:
        raise ValueError("max_step 必须非负")
    if wrap_360:
        delta = shortest_delta_deg(current, target, wrap_360=True)
        step = float(np.clip(delta, -max_step, max_step))
        return _wrap_360_deg(float(current) + step)
    scalar = np.isscalar(current) and np.isscalar(target)
    delta = np.asarray(target, dtype=np.float64) - np.asarray(current, dtype=np.float64)
    step = np.clip(delta, -max_step, max_step)
    out = np.asarray(current, dtype=np.float64) + step
    return float(out) if scalar else out


def clamp(value: float, lo: float, hi: float) -> float:
    """把 value 夹紧到 [lo, hi]，线性，不环绕。"""
    scalar = np.isscalar(value)
    n = np.clip(np.asarray(value, dtype=np.float64), lo, hi)
    return float(n) if scalar else n
