"""filters 单元测试：alpha-beta、Kalman(CV)、变/重复时间戳防御、双轴门控。"""

from __future__ import annotations

import numpy as np
import pytest

from aim.filters import AlphaBetaFilter, KalmanCV, TwoAxisTracker


def test_alpha_beta_converges_to_constant_velocity():
    """alpha-beta 在带噪常速轨迹上收敛到真实位置与速度。"""
    rng = np.random.default_rng(1)
    dt_s = 0.02
    alpha, beta = 0.4, 0.1  # beta = alpha^2/(2-alpha) 临界阻尼
    filt = AlphaBetaFilter(alpha, beta, dt_s)
    p0, v = 0.1, 2.0
    n = 400
    for i in range(n):
        t_ms = i * dt_s * 1000.0
        z = p0 + v * (i * dt_s) + rng.normal(0.0, 5e-4)
        filt.update(t_ms, z)
    true_pos = p0 + v * ((n - 1) * dt_s)
    assert abs(filt.velocity - v) < 0.05
    assert abs(filt.position - true_pos) < 0.01


def test_kalman_converges_and_covariance_shrinks():
    """Kalman(CV) 对带噪常速轨迹收敛：位置误差小、速度接近真值、协方差迹下降。"""
    rng = np.random.default_rng(4)
    dt_s = 0.02
    kf = KalmanCV(dt_s, process_noise=1e-5, measurement_noise=1e-4)
    p0, v = 0.5, 1.5
    kf.x = np.array([p0, 0.0])
    kf.P = np.eye(2) * 0.01
    traces = []
    n = 500
    for i in range(1, n):
        kf.predict(dt_s)
        z = p0 + v * (i * dt_s) + rng.normal(0.0, 0.01)
        kf.update(z)
        traces.append(float(np.trace(kf.covariance)))
    assert abs(kf.state[0] - (p0 + v * ((n - 1) * dt_s))) < 0.01
    assert abs(kf.state[1] - v) < 0.05
    assert traces[-1] < traces[3]


def test_non_positive_dt_is_rejected_without_nan():
    """非正或重复时间戳抛 ValueError，内部状态保持有限，不产生 NaN。"""
    filt = AlphaBetaFilter(0.3, 0.05, 0.02)
    filt.update(0.0, 1.0)
    with pytest.raises(ValueError):
        filt.update(0.0, 2.0)  # 重复时间戳
    assert np.isfinite(filt.position) and np.isfinite(filt.velocity)

    kf = KalmanCV(0.02, 1e-4, 1e-4)
    with pytest.raises(ValueError):
        kf.predict(0.0)
    assert np.all(np.isfinite(kf.state)) and np.all(np.isfinite(kf.covariance))

    tracker = TwoAxisTracker(dt_s=0.02)
    tracker.update(0.0, 0.1, 0.1)
    with pytest.raises(ValueError):
        tracker.update(0.0, 0.2, 0.2)  # 重复时间戳
    # 之后用正 dt 仍能正常工作
    out = tracker.update(20.0, 0.11, 0.11)
    assert np.all(np.isfinite(out[:4]))


def test_variable_dt_produces_finite_output():
    """变 dt 序列下滤波器输出始终有限。"""
    filt = AlphaBetaFilter(0.3, 0.05, 0.02)
    kf = KalmanCV(0.02, 1e-4, 1e-4)
    t_ms = 0.0
    z = 0.0
    for dt_ms in [20.0, 33.0, 12.0, 41.0, 20.0, 25.0]:
        t_ms += dt_ms
        z += 0.1
        assert np.isfinite(filt.update(t_ms, z))
        kf.predict(dt_ms / 1000.0)
        assert np.all(np.isfinite(kf.update(z)))


def test_two_axis_tracker_gate_rejects_outlier():
    """双轴门控拒绝大幅离群点，离群点几乎不拉动估计。"""
    tracker = TwoAxisTracker(dt_s=0.02, process_noise=1e-6,
                             measurement_noise=1e-6, gate_rad=0.1)
    b_v, e_v = 0.2, 0.05          # 初值
    b_rate, e_rate = 0.5, 0.1     # rad/s 常速
    tracker.update(0.0, b_v, e_v)
    for i in range(1, 60):
        t_s = i * 0.02
        tracker.update(t_s * 1000.0, b_v + b_rate * t_s, e_v + e_rate * t_s)

    t_s = 60 * 0.02
    b_next = b_v + b_rate * t_s
    e_next = e_v + e_rate * t_s
    out = tracker.update(t_s * 1000.0, b_next + 3.0, e_next + 3.0)  # 巨大离群
    # 门控拒绝后估计停在预测值附近，不会被 3 rad 拉走
    assert abs(out[0] - b_next) < 0.05
    assert abs(out[1] - e_next) < 0.05
    assert np.all(np.isfinite(out[:4]))
