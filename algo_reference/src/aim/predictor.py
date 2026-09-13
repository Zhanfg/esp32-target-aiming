"""预测与外推：常速和二阶外推、提前量预算、模型选择启发式。

单位约定：角度 rad，角速度 rad/s，加速度 rad/s^2，时间 lead 用 s、now_ms 用 ms。
"""

from __future__ import annotations

import numpy as np


def predict_linear(now_ms: float, lead_s: float, pos: float, vel: float) -> float:
    """常速外推，返回 pos + vel * lead_s。

    now_ms 只为保持 API 一致和后续扩展（按时间对齐输出）保留，当前不参与计算。
    """
    return float(pos) + float(vel) * float(lead_s)


def predict_quadratic(now_ms: float, lead_s: float, pos: float, vel: float, accel: float,
                      *, accel_floor_rad_s2: float = 0.05,
                      max_quad_correction_rad: float | None = None) -> float:
    """二阶外推，返回 pos + vel*lead_s + 0.5*accel*lead_s^2。

    加速度估计来自带噪的差分，真实值小时二阶项几乎全是噪声，因此 |accel| < accel_floor_rad_s2
    时强制退化为线性外推，不打乱稳态。二阶修正量 0.5*accel*lead_s^2 在目标剧烈机动时可能失控，
    可用 max_quad_correction_rad 限幅；默认不限幅，由调用方决定是否设防。
    """
    accel_eff = float(accel) if abs(float(accel)) >= accel_floor_rad_s2 else 0.0
    correction = 0.5 * accel_eff * float(lead_s) ** 2
    if max_quad_correction_rad is not None:
        correction = float(np.clip(correction, -max_quad_correction_rad, max_quad_correction_rad))
    return float(pos) + float(vel) * float(lead_s) + correction


def required_lead_time(angular_error_rad: float, max_rate_rad_s: float,
                       actuator_latency_s: float, settle_time_s: float) -> float:
    """返回提前量总前视时间，单位秒。

    激光直瞄没有飞行时间，滞后全部来自舵盘执行机构。模型假设：
    1. actuator_latency_s 是固定延迟，包括通信、编码器采样、驱动环路启动，这段时间电机不动。
    2. settle_time_s 是到位后的稳定和超调平息时间，这段时间不应认为已经锁住目标。
    3. 目标角速度不超过 max_rate_rad_s，这是最坏情况。
    4. angular_error_rad 是允许的最大指向误差，例如取死区角。

    在固定延迟内目标最多移动 max_rate_rad_s * actuator_latency_s，只能靠提前量补偿。
    angular_error_rad / max_rate_rad_s 是目标穿越整个误差盘所需时间，也是保证目标始终落在
    误差盘内的反应时间下界。两者相加再叠加稳定时间，得到保守的总前视时间：

        lead = angular_error_rad / max_rate_rad_s + actuator_latency_s + settle_time_s

    若编码器或陀螺能实测目标角速度，可用实测值替代 max_rate_rad_s，缩小过度前视带来的预测噪声。
    """
    if max_rate_rad_s <= 0.0:
        raise ValueError("max_rate_rad_s 必须为正")
    if angular_error_rad < 0.0:
        raise ValueError("angular_error_rad 必须非负")
    return angular_error_rad / max_rate_rad_s + actuator_latency_s + settle_time_s


def should_use_quadratic(samples, speed_threshold: float) -> bool:
    """模型选择启发式，samples 为最近若干个速度样本，单位 rad/s，可正可负。

    样本中最大绝对速度超过 speed_threshold 时返回 True，否则返回 False。低速或近静态目标的
    外推几乎不需要二阶项，二阶项只会放大带噪速度差分的噪声；速度超过阈值说明目标在快速运动，
    可能伴随机动，二阶项才有意义。
    """
    if len(samples) == 0:
        return False
    arr = np.asarray([abs(float(s)) for s in samples], dtype=np.float64)
    return float(np.max(arr)) > float(speed_threshold)
