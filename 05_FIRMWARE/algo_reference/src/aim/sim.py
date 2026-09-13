"""确定性合成目标生成器，供测试和 PC 端验证，不参与固件。

随机性全部来自 np.random.default_rng(seed)，给定 seed、参数和 numpy 版本即可复现。所有生成器
返回 list[PixelObservation]，像素坐标单位 px（x 向右、y 向下），时间戳单位 ms，下游 AimSolver
可直接消费。生成器工作在像素域，不依赖相机内参；intrinsics 参数为与求解器和标定 API 对称而
保留，不参与像素轨迹计算，角度真值由调用方用 geometry.pixel_to_angles 求。

时间约定：dt 为相邻采样的时间间隔，单位毫秒，第 i 帧时间戳为 i * dt。
"""

from __future__ import annotations

import numpy as np

from .types import CameraIntrinsics, PixelObservation

__all__ = [
    "make_static_target",
    "make_linear_target",
    "make_sinusoidal_target",
    "make_occluded_track",
]


def _as_pair(value, name: str) -> tuple[float, float]:
    """把标量或长度为 2 的序列统一成 (float, float)。"""
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size == 1:
        return float(arr[0]), float(arr[0])
    if arr.size == 2:
        return float(arr[0]), float(arr[1])
    raise ValueError(f"{name} 必须是标量或长度 2 的序列，实际长度 {arr.size}")


def make_static_target(
    n: int,
    dt: float,
    pixel: tuple[float, float],
    intrinsics: CameraIntrinsics,
    noise_px: float = 0.0,
    seed: int = 0,
) -> list[PixelObservation]:
    """静止目标：像素恒为 pixel，叠加零均值高斯噪声。

    - n：采样帧数，>= 1。
    - dt：采样间隔，毫秒，第 i 帧时间戳为 i*dt。
    - pixel：(x_px, y_px) 目标像素位置。
    - intrinsics：相机内参，本函数不使用，仅为 API 对称保留。
    - noise_px：像素噪声标准差，单位 px，0 表示无噪。
    - seed：随机种子。

    返回长度 n 的 list[PixelObservation]，confidence 恒为 1.0。
    """
    rng = np.random.default_rng(seed)
    x0, y0 = _as_pair(pixel, "pixel")
    obs: list[PixelObservation] = []
    for i in range(int(n)):
        obs.append(
            PixelObservation(
                t_ms=float(i) * float(dt),
                x=x0 + float(rng.normal(0.0, float(noise_px))),
                y=y0 + float(rng.normal(0.0, float(noise_px))),
                confidence=1.0,
            )
        )
    return obs


def make_linear_target(
    n: int,
    dt: float,
    pixel0: tuple[float, float],
    velocity_px_s: tuple[float, float],
    intrinsics: CameraIntrinsics,
    noise_px: float = 0.0,
    seed: int = 0,
) -> list[PixelObservation]:
    """匀速直线目标：p(t) = p0 + v * t，t 单位秒。

    - n：采样帧数。
    - dt：采样间隔，毫秒。
    - pixel0：(x_px, y_px) 初始像素位置。
    - velocity_px_s：(vx_px_s, vy_px_s) 像素速度，单位 px/s。
    - intrinsics：相机内参，不使用，仅为 API 对称保留。
    - noise_px：像素噪声标准差，单位 px。
    - seed：随机种子。

    返回 list[PixelObservation]，confidence 恒为 1.0。
    """
    rng = np.random.default_rng(seed)
    x0, y0 = _as_pair(pixel0, "pixel0")
    vx, vy = _as_pair(velocity_px_s, "velocity_px_s")
    obs: list[PixelObservation] = []
    for i in range(int(n)):
        t_s = float(i) * float(dt) / 1000.0
        obs.append(
            PixelObservation(
                t_ms=float(i) * float(dt),
                x=x0 + vx * t_s + float(rng.normal(0.0, float(noise_px))),
                y=y0 + vy * t_s + float(rng.normal(0.0, float(noise_px))),
                confidence=1.0,
            )
        )
    return obs


def make_sinusoidal_target(
    n: int,
    dt: float,
    pixel0: tuple[float, float],
    amplitude_px,
    freq_hz: float,
    intrinsics: CameraIntrinsics,
    noise_px: float = 0.0,
    seed: int = 0,
    phase_rad: float = 0.0,
) -> list[PixelObservation]:
    """正弦摆动目标：p(t) = p0 + A * sin(2*pi*f*t + phase)。

    - amplitude_px：振幅。标量表示 x/y 同振幅，长度 2 的序列 (Ax, Ay) 可给两轴不同振幅。
    - freq_hz：摆动频率，单位 Hz。
    - phase_rad：初相，单位 rad，默认 0，便于构造非零初始速度。
    - 其余参数含义同 make_linear_target。

    返回 list[PixelObservation]，confidence 恒为 1.0。
    """
    rng = np.random.default_rng(seed)
    x0, y0 = _as_pair(pixel0, "pixel0")
    ax, ay = _as_pair(amplitude_px, "amplitude_px")
    obs: list[PixelObservation] = []
    for i in range(int(n)):
        t_s = float(i) * float(dt) / 1000.0
        phase = 2.0 * np.pi * float(freq_hz) * t_s + float(phase_rad)
        obs.append(
            PixelObservation(
                t_ms=float(i) * float(dt),
                x=x0 + ax * float(np.sin(phase)) + float(rng.normal(0.0, float(noise_px))),
                y=y0 + ay * float(np.sin(phase)) + float(rng.normal(0.0, float(noise_px))),
                confidence=1.0,
            )
        )
    return obs


def make_occluded_track(
    n: int,
    dt: float,
    pixel0: tuple[float, float],
    velocity_px_s: tuple[float, float],
    intrinsics: CameraIntrinsics,
    noise_px: float = 0.0,
    seed: int = 0,
    gap_start: int | None = None,
    gap_len: int = 15,
) -> list[PixelObservation]:
    """带遮挡空档的匀速直线航迹，用于验证丢跟踪检测和重新捕获。

    空档区间 [gap_start, gap_start + gap_len) 内 confidence=0，坐标仍沿真值轨迹给出，便于
    CSV 人工检查；空档前后 confidence=1。目标在空档期间继续运动，空档结束后能否重新锁定
    取决于滑行预测是否足够准，匀速目标应能直接重锁。

    - gap_start：空档起始帧下标，None 时取 n//2 - gap_len//2，使空档居中。
    - gap_len：空档帧数，>= 0。
    - 其余参数同 make_linear_target。

    返回长度 n 的 list[PixelObservation]。
    """
    rng = np.random.default_rng(seed)
    x0, y0 = _as_pair(pixel0, "pixel0")
    vx, vy = _as_pair(velocity_px_s, "velocity_px_s")
    n = int(n)
    gap_len = int(gap_len)
    if gap_start is None:
        gap_start = max(0, n // 2 - gap_len // 2)
    gap_start = int(gap_start)
    gap_end = gap_start + gap_len
    noise = float(noise_px)

    obs: list[PixelObservation] = []
    for i in range(n):
        t_s = float(i) * float(dt) / 1000.0
        x = x0 + vx * t_s + float(rng.normal(0.0, noise))
        y = y0 + vy * t_s + float(rng.normal(0.0, noise))
        confidence = 0.0 if (gap_start <= i < gap_end) else 1.0
        obs.append(PixelObservation(t_ms=float(i) * float(dt), x=x, y=y, confidence=confidence))
    return obs
