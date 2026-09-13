"""AimSolver 集成测试：静态稳定、提前量有效、限速、限位、滑行、遮挡重锁。"""

from __future__ import annotations

import numpy as np
import pytest

from aim import (
    AimSolver,
    AxisLimits,
    CalibrationModel,
    CameraIntrinsics,
    SolverConfig,
    TurretLimits,
    make_linear_target,
    make_occluded_track,
    make_static_target,
    pixel_to_angles,
)


def _intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics.default_for(640, 480)


def _identity_affine() -> np.ndarray:
    # [bearing_deg, elevation_deg, 1] -> (pan_deg, tilt_deg)，恒等映射，便于隔离几何误差。
    return np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def _make_solver(config: SolverConfig | None = None,
                 limits: TurretLimits | None = None,
                 affine: np.ndarray | None = None) -> AimSolver:
    intr = _intrinsics()
    calib = CalibrationModel(
        intrinsics=intr,
        affine_angle_to_pan_tilt=affine if affine is not None else _identity_affine(),
    )
    return AimSolver(intr, calib, limits if limits is not None else TurretLimits.default(),
                     config)


def _no_lead_config(**overrides) -> SolverConfig:
    base = dict(lead_actuator_latency_s=0.0, lead_settle_s=0.0,
                angular_error_budget_rad=0.0, nominal_dt_s=0.02)
    base.update(overrides)
    return SolverConfig(**base)


def test_static_target_is_stable_inside_deadband():
    """静态目标：预热后输出稳定，保持在配置 deadband 内。"""
    cfg = SolverConfig(lead_actuator_latency_s=0.03, lead_settle_s=0.01,
                       angular_error_budget_rad=0.0, nominal_dt_s=0.02)
    solver = _make_solver(cfg)
    intr = _intrinsics()
    obs = make_static_target(120, 20.0, (360.0, 220.0), intr, noise_px=0.2, seed=7)
    outs = [solver.update(o, o.t_ms) for o in obs]

    tail = outs[-40:]
    assert all(o.valid for o in tail)
    pan_vals = np.array([o.pan_deg for o in tail])
    tilt_vals = np.array([o.tilt_deg for o in tail])
    b_true, e_true = pixel_to_angles((360.0, 220.0), intr)

    deadband = TurretLimits.default().pan.deadband_deg
    assert pan_vals.max() - pan_vals.min() < deadband
    assert tilt_vals.max() - tilt_vals.min() < deadband
    assert abs(float(pan_vals.mean()) - np.degrees(b_true)) < deadband
    assert abs(float(tilt_vals.mean()) - np.degrees(e_true)) < deadband


def test_lead_compensation_reduces_steady_state_error():
    """核心断言：配置执行延迟并开启提前量后，稳态指向误差显著低于延迟为 0 的情况。

    舵盘指令链有固定延迟 L（物理属性，不随配置改变）。指令在 now 发出、到位在 now + L，
    此时目标已移动 v * L。延迟为 0 时稳态指向误差卡在 v * L；把指令外推到 now + L 后，
    舵盘到位时正对目标，误差降到滤波残差量级。
    """
    intr = _intrinsics()
    dt_ms = 20.0
    physical_latency_s = 0.04  # 2 帧
    p0 = (220.0, 240.0)
    v_px_s = (120.0, 0.0)

    noise_px = 0.2
    obs = make_linear_target(160, dt_ms, p0, v_px_s, intr, noise_px=noise_px, seed=11)
    # 测量噪声按本场景的像素噪声换算到角度方差 (px/fx)^2，让滤波增益与实际传感器匹配。
    # 角度轨迹有曲率，R 取大会让位置估计系统性超前或滞后，掩盖提前量的效果。
    angle_noise_var = (noise_px / intr.fx) ** 2

    def true_pan_deg_at(t_ms: float) -> float:
        # 用解析真值轨迹（无噪）在同一针孔模型下求目标真实 pan 角（度）。
        x = p0[0] + v_px_s[0] * (t_ms / 1000.0)
        y = p0[1] + v_px_s[1] * (t_ms / 1000.0)
        b, _ = pixel_to_angles((x, y), intr)
        return float(np.degrees(b))

    solver_lead = _make_solver(SolverConfig(
        lead_actuator_latency_s=physical_latency_s, lead_settle_s=0.0,
        angular_error_budget_rad=0.0, nominal_dt_s=0.02, max_rate_rad_s=5.0,
        tracker_measurement_noise=angle_noise_var,
    ))
    solver_no_lead = _make_solver(SolverConfig(
        lead_actuator_latency_s=0.0, lead_settle_s=0.0,
        angular_error_budget_rad=0.0, nominal_dt_s=0.02, max_rate_rad_s=5.0,
        tracker_measurement_noise=angle_noise_var,
    ))

    outs_lead = [solver_lead.update(o, o.t_ms) for o in obs]
    outs_no_lead = [solver_no_lead.update(o, o.t_ms) for o in obs]

    # 稳态窗口取后半段，避开滤波器初始收敛。
    window = range(60, len(obs))
    err_lead = np.array([
        abs(outs_lead[i].pan_deg - true_pan_deg_at(obs[i].t_ms + physical_latency_s * 1000.0))
        for i in window
    ])
    err_no_lead = np.array([
        abs(outs_no_lead[i].pan_deg - true_pan_deg_at(obs[i].t_ms + physical_latency_s * 1000.0))
        for i in window
    ])
    mean_lead = float(err_lead.mean())
    mean_no_lead = float(err_no_lead.mean())

    # 提前量应带来 3 倍以上的误差下降，达不到说明提前量逻辑没生效。
    assert mean_lead < 0.3 * mean_no_lead, (
        f"提前量未显著降低稳态误差：lead={mean_lead:.4f} deg, "
        f"no_lead={mean_no_lead:.4f} deg"
    )
    # 残余误差应接近滤波噪声量级，远小于一帧位移 v*L。
    assert mean_lead < 0.1


def test_rate_limit_step_bounded():
    """限速：阶跃输入下每一步角变化不超过 max_slew_dps * dt，留微小容差。"""
    limits = TurretLimits(
        pan=AxisLimits(-180.0, 180.0, 60.0, 0.5),
        tilt=AxisLimits(-90.0, 90.0, 60.0, 0.5),
    )
    cfg = _no_lead_config()
    solver = _make_solver(cfg, limits)
    intr = _intrinsics()
    obs = make_static_target(40, 20.0, (620.0, 60.0), intr, noise_px=0.0, seed=0)
    outs = [solver.update(o, o.t_ms) for o in obs]

    dt_s = 0.02
    pan_step_max = limits.pan.max_slew_dps * dt_s
    tilt_step_max = limits.tilt.max_slew_dps * dt_s
    for a, b in zip(outs, outs[1:]):
        assert abs(b.pan_deg - a.pan_deg) <= pan_step_max + 1e-9
        assert abs(b.tilt_deg - a.tilt_deg) <= tilt_step_max + 1e-9
        assert abs(b.pan_rate_dps) <= limits.pan.max_slew_dps + 1e-6
        assert abs(b.tilt_rate_dps) <= limits.tilt.max_slew_dps + 1e-6
    # 最终应到达目标角（限速只是拖慢，不阻止收敛）
    assert abs(outs[-1].pan_deg - np.degrees(pixel_to_angles((620.0, 60.0), intr)[0])) < 0.5


def test_limits_are_never_exceeded():
    """越界输入不产出超限的 pan/tilt。"""
    limits = TurretLimits(
        pan=AxisLimits(-10.0, 10.0, 720.0, 0.5),
        tilt=AxisLimits(-5.0, 5.0, 720.0, 0.5),
    )
    solver = _make_solver(_no_lead_config(), limits)
    intr = _intrinsics()
    # 图像角落，理论角度远超人为收紧的限位。
    obs = make_static_target(80, 20.0, (639.0, 0.0), intr, noise_px=0.0, seed=0)
    outs = [solver.update(o, o.t_ms) for o in obs]
    for o in outs:
        assert -10.0 - 1e-9 <= o.pan_deg <= 10.0 + 1e-9
        assert -5.0 - 1e-9 <= o.tilt_deg <= 5.0 + 1e-9
    assert abs(outs[-1].pan_deg - 10.0) < 1e-6
    assert abs(outs[-1].tilt_deg - 5.0) < 1e-6


def test_coast_then_timeout():
    """滑行：观测停止后在 coast 时间内仍有效，超时后 valid=False，全程无 NaN。"""
    intr = _intrinsics()
    cfg = _no_lead_config(coast_timeout_s=0.2)
    solver = _make_solver(cfg)
    obs = make_linear_target(30, 20.0, (320.0, 240.0), (50.0, 0.0), intr, noise_px=0.0, seed=0)
    outs = [solver.update(o, o.t_ms) for o in obs]
    t_last = obs[-1].t_ms

    within = solver.update(None, t_last + 100.0)   # 100ms < 200ms
    assert within.valid
    assert np.all(np.isfinite([within.pan_deg, within.tilt_deg,
                               within.pan_rate_dps, within.tilt_rate_dps]))

    beyond = solver.update(None, t_last + 250.0)   # 250ms > 200ms
    assert not beyond.valid
    assert not solver.is_tracking
    assert np.all(np.isfinite([beyond.pan_deg, beyond.tilt_deg,
                               beyond.pan_rate_dps, beyond.tilt_rate_dps]))

    for o in outs:
        assert np.all(np.isfinite([o.pan_deg, o.tilt_deg,
                                   o.pan_rate_dps, o.tilt_rate_dps]))


def test_update_before_any_observation_is_invalid_and_safe():
    solver = _make_solver(_no_lead_config())
    r = solver.update(None, 0.0)
    assert not r.valid
    assert r.pan_deg == 0.0 and r.tilt_deg == 0.0
    assert np.all(np.isfinite([r.pan_deg, r.tilt_deg, r.pan_rate_dps, r.tilt_rate_dps]))


def test_occlusion_recovers_tracking():
    """遮挡：空档内超时失效，空档结束后重新回到 tracking。"""
    intr = _intrinsics()
    cfg = _no_lead_config(coast_timeout_s=0.05, tracker_gate_rad=0.5)
    solver = _make_solver(cfg)
    obs = make_occluded_track(120, 20.0, (200.0, 240.0), (80.0, 0.0), intr,
                              noise_px=0.2, seed=3, gap_len=15)
    outs = [solver.update(o, o.t_ms) for o in obs]

    gap = [i for i, o in enumerate(obs) if not o.valid]
    assert len(gap) == 15
    # 空档内（滑行超时后）应报告丢失
    assert any(not outs[i].valid for i in gap)
    # 空档结束后应重新锁定
    post = outs[gap[-1] + 1:]
    assert any(o.valid for o in post)
    assert solver.is_tracking
    # 全程无 NaN
    for o in outs:
        assert np.all(np.isfinite([o.pan_deg, o.tilt_deg,
                                   o.pan_rate_dps, o.tilt_rate_dps]))


def test_reset_clears_state():
    """reset() 后回到未初始化状态。"""
    solver = _make_solver(_no_lead_config())
    intr = _intrinsics()
    obs = make_static_target(10, 20.0, (320.0, 240.0), intr, noise_px=0.0, seed=0)
    for o in obs:
        solver.update(o, o.t_ms)
    assert solver.is_tracking
    solver.reset()
    r = solver.update(None, obs[-1].t_ms + 20.0)
    assert not r.valid


def test_wrap_360_pan_uses_shortest_delta():
    """环绕 pan 轴：跨 ±180° 时指令角速度按最短路径计算，不出现巨大跳变。"""
    limits = TurretLimits(
        pan=AxisLimits(0.0, 360.0, 90.0, 0.5, wrap_360=True),
        tilt=AxisLimits(-45.0, 45.0, 90.0, 0.5),
    )
    # 恒等仿射，pan 截距取 175，使目标角落在 175+32=207° 附近；从 0 起步需走最短路径。
    affine = np.array([[1.0, 0.0, 175.0], [0.0, 1.0, 0.0]])
    solver = _make_solver(_no_lead_config(), limits, affine)
    intr = _intrinsics()
    obs = make_static_target(30, 20.0, (600.0, 240.0), intr, noise_px=0.0, seed=0)
    outs = [solver.update(o, o.t_ms) for o in obs]
    dt_s = 0.02
    for a, b in zip(outs, outs[1:]):
        # 环绕轴上，相邻指令的"最短角差"必须受限速约束
        from aim import shortest_delta_deg
        delta = shortest_delta_deg(a.pan_deg, b.pan_deg, wrap_360=True)
        assert abs(delta) <= limits.pan.max_slew_dps * dt_s + 1e-6
    for o in outs:
        assert 0.0 <= o.pan_deg < 360.0
