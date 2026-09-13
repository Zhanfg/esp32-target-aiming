"""滤波与航迹管理：alpha-beta、常速 Kalman(CV)、双轴跟踪器、航迹丢失判定。

单位约定：角度 rad，角速度 rad/s，时间对外用 ms、内部动力学用 s。所有滤波器显式处理时间戳，
支持变 dt；非正 dt 一律抛 ValueError，不产生 NaN 或除零。C++ 端需要复刻这套防御性行为。
"""

from __future__ import annotations

from enum import Enum, auto

import numpy as np


class AlphaBetaFilter:
    """单轴常速 alpha-beta 滤波器，状态为位置和速度。

    dt 为相邻时间戳之差，单位 s：
        x_pred = x + v * dt
        r      = z - x_pred
        x      = x_pred + alpha * r
        v      = v + (beta / dt) * r
    beta/dt 把位置残差换算成速度修正，因此天然支持变 dt。

    - alpha ∈ (0,1]：位置修正增益，越大越信任观测。
    - beta ∈ (0,1]：速度修正增益，beta = alpha^2/(2-alpha) 为临界阻尼。
    - 构造参数 dt 只记录名义采样周期，实际 dt 由 update 的时间戳差求出。
    """

    def __init__(self, alpha: float, beta: float, dt: float):
        if not (0.0 < alpha <= 1.0):
            raise ValueError("alpha 必须位于 (0, 1]")
        if not (0.0 < beta <= 1.0):
            raise ValueError("beta 必须位于 (0, 1]")
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.nominal_dt = float(dt)
        self.position = 0.0
        self.velocity = 0.0
        self._t_ms: float | None = None

    def update(self, t_ms: float, z: float) -> float:
        """用时间戳 t_ms 的观测 z 更新，返回滤波后的位置估计。

        首次调用以 z 初始化位置、速度置 0。之后 dt 由时间戳差求得，dt <= 0（时间戳重复或
        倒退）抛 ValueError，状态保持不变。
        """
        if self._t_ms is None:
            self._t_ms = float(t_ms)
            self.position = float(z)
            self.velocity = 0.0
            return self.position
        dt_s = (float(t_ms) - self._t_ms) / 1000.0
        if dt_s <= 0.0:
            raise ValueError(f"AlphaBetaFilter: 非正 dt = {dt_s} s（时间戳 {t_ms}）")
        x_pred = self.position + self.velocity * dt_s
        r = float(z) - x_pred
        self.position = x_pred + self.alpha * r
        self.velocity = self.velocity + (self.beta / dt_s) * r
        self._t_ms = float(t_ms)
        return self.position

    def reset(self) -> None:
        """清空状态，回到未初始化。"""
        self.position = 0.0
        self.velocity = 0.0
        self._t_ms = None


class KalmanCV:
    """两状态（位置、速度）常速 Kalman 滤波器，numpy 实现。

    状态 x = [pos, vel]^T，观测 z = pos。过程噪声用白噪声加速度模型，离散化后
        Q = q * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]
    q 为过程噪声强度，单位 rad^2/s^3 或 px^2/s^3；测量噪声 r 为 z 的方差。predict(dt) 每次按
    实际 dt 重建 F、Q，支持变 dt。.covariance 暴露 2x2 协方差，供收敛性测试断言协方差迹下降。
    """

    def __init__(self, dt: float, process_noise: float, measurement_noise: float):
        self.nominal_dt = float(dt)
        self.q = float(process_noise)
        self.r = float(measurement_noise)
        self.reset()

    def reset(self) -> None:
        """状态归零，协方差重置为大的对角阵，表示完全不确定。"""
        self.x = np.zeros(2, dtype=np.float64)
        self.P = np.eye(2, dtype=np.float64) * 100.0

    @property
    def covariance(self) -> np.ndarray:
        """当前状态协方差矩阵 (2,2) 的副本。"""
        return self.P.copy()

    @property
    def state(self) -> tuple[float, float]:
        """返回 (pos, vel)。"""
        return float(self.x[0]), float(self.x[1])

    def predict(self, dt: float) -> np.ndarray:
        """按时间步 dt（秒）做先验预测，dt <= 0 抛 ValueError。"""
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError(f"KalmanCV: 非正 dt = {dt} s")
        F = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        Q = self.q * np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0],
                               [dt ** 3 / 2.0, dt ** 2]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        return self.x

    def update(self, z: float, r: float | None = None) -> np.ndarray:
        """用观测 z 做后验更新。r 可覆盖构造时的测量噪声，例如按置信度缩放。"""
        rr = self.r if r is None else float(r)
        H = np.array([1.0, 0.0], dtype=np.float64)
        P = self.P
        S = float(H @ P @ H + rr)
        K = (P @ H) / S
        y = float(z) - float(self.x[0])
        self.x = self.x + K * y
        # Joseph 形式，数值上保持对称正定
        I = np.eye(2, dtype=np.float64)
        A = I - np.outer(K, H)
        self.P = A @ P @ A.T + rr * np.outer(K, K)
        return self.x


class TwoAxisTracker:
    """组合两个 KalmanCV（bearing 和 elevation）并做残差门控的双轴跟踪器。

    任一轴残差绝对值超过 gate_rad（弧度）时整条观测被拒绝，防止离群点污染。置信度通过缩放
    测量噪声参与：R_eff = R / max(confidence, 0.05)，置信度越低，观测对状态的拉动越弱。
    """

    def __init__(self, dt_s: float = 0.05, process_noise: float = 1e-4,
                 measurement_noise: float = 1e-4, gate_rad: float = 0.5):
        self._bearing = KalmanCV(dt_s, process_noise, measurement_noise)
        self._elevation = KalmanCV(dt_s, process_noise, measurement_noise)
        self.gate_rad = float(gate_rad)
        self._t_ms: float | None = None

    @property
    def initialized(self) -> bool:
        """是否已经收到过至少一条观测。"""
        return self._t_ms is not None

    def update(self, t_ms: float, bearing_rad: float, elevation_rad: float,
               confidence: float = 1.0) -> tuple[float, float, float, float, bool]:
        """推进到 t_ms 并融合观测，返回 (bearing_hat, elevation_hat, bearing_rate, elevation_rate, valid)。

        首次调用初始化两个滤波器。之后先 predict(dt)，再按门控决定是否 update。被拒绝的观测
        不改状态，只保留预测结果。角度单位 rad，角速度 rad/s。
        """
        if self._t_ms is None:
            self._bearing.x = np.array([float(bearing_rad), 0.0])
            self._bearing.P = np.eye(2) * 0.01
            self._elevation.x = np.array([float(elevation_rad), 0.0])
            self._elevation.P = np.eye(2) * 0.01
            self._t_ms = float(t_ms)
            return (float(bearing_rad), float(elevation_rad), 0.0, 0.0, True)

        dt_s = (float(t_ms) - self._t_ms) / 1000.0
        if dt_s <= 0.0:
            raise ValueError(f"TwoAxisTracker: 非正 dt = {dt_s} s")
        self._bearing.predict(dt_s)
        self._elevation.predict(dt_s)

        res_b = float(bearing_rad) - float(self._bearing.x[0])
        res_e = float(elevation_rad) - float(self._elevation.x[0])
        if abs(res_b) <= self.gate_rad and abs(res_e) <= self.gate_rad:
            c = max(float(confidence), 0.05)
            self._bearing.update(bearing_rad, r=self._bearing.r / c)
            self._elevation.update(elevation_rad, r=self._elevation.r / c)

        self._t_ms = float(t_ms)
        return (float(self._bearing.x[0]), float(self._elevation.x[0]),
                float(self._bearing.x[1]), float(self._elevation.x[1]), True)

    def predict(self, t_ms: float) -> tuple[float, float, float, float, bool]:
        """无观测推进（滑行），只做 predict。未初始化时返回 valid=False。"""
        if self._t_ms is None:
            return 0.0, 0.0, 0.0, 0.0, False
        dt_s = (float(t_ms) - self._t_ms) / 1000.0
        if dt_s <= 0.0:
            raise ValueError(f"TwoAxisTracker: 非正 dt = {dt_s} s")
        self._bearing.predict(dt_s)
        self._elevation.predict(dt_s)
        self._t_ms = float(t_ms)
        return (float(self._bearing.x[0]), float(self._elevation.x[0]),
                float(self._bearing.x[1]), float(self._elevation.x[1]), True)

    def reset(self) -> None:
        """重置两个滤波器和时间戳。"""
        self._bearing.reset()
        self._elevation.reset()
        self._t_ms = None


class TrackStatus(Enum):
    """航迹状态。"""

    TRACKING = auto()
    SEARCHING = auto()


class TrackLossDetector:
    """航迹丢失判定，在 TRACKING 和 SEARCHING 之间切换。

    confidence >= min_confidence 时判定为 TRACKING 并刷新最近有效时间；否则在距最近有效时间
    超过 lost_after_ms 后判定为 SEARCHING。实现不做去抖，C++ 端可在此加滞回。
    """

    def __init__(self, lost_after_ms: float = 200.0, min_confidence: float = 0.3):
        self.lost_after_ms = float(lost_after_ms)
        self.min_confidence = float(min_confidence)
        self.status = TrackStatus.SEARCHING
        self.last_good_t_ms: float | None = None

    def update(self, t_ms: float, confidence: float) -> TrackStatus:
        """输入时间戳与置信度，返回更新后的状态。"""
        if confidence >= self.min_confidence:
            self.last_good_t_ms = float(t_ms)
            self.status = TrackStatus.TRACKING
        elif (self.last_good_t_ms is not None
              and (float(t_ms) - self.last_good_t_ms) >= self.lost_after_ms):
            self.status = TrackStatus.SEARCHING
        return self.status

    def reset(self) -> None:
        """回到 SEARCHING 初始状态。"""
        self.status = TrackStatus.SEARCHING
        self.last_good_t_ms = None
