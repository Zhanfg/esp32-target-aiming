"""geometry 单元测试：角度往返、畸变往返、仿射解算、单点/批量形状。

锁定 C++ 移植依赖的坐标和角度约定：bearing = atan2(X, Z) 向右为正，
elevation = atan2(-Y, sqrt(X²+Z²)) 向上为正；畸变模型为 (k1,k2,p1,p2,k3)，反畸变用定长
迭代；标定仿射把 (bearing, elevation) 映射到 (pan, tilt)。
"""

from __future__ import annotations

import numpy as np
import pytest

from aim.geometry import (
    angles_to_ray,
    apply_affine,
    distort_points,
    pixel_to_angles,
    pixel_to_ray,
    ray_to_angles,
    solve_affine_angle_to_pan_tilt,
    undistort_points,
)
from aim.types import CameraIntrinsics


def _intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics.default_for(640, 480)


def test_angle_ray_roundtrip_grid():
    """角度 -> 射线 -> 角度，在含负值和边界值的网格上精度 1e-9。"""
    bearings = np.linspace(-np.pi + 1e-3, np.pi - 1e-3, 13)
    elevations = np.linspace(-np.pi / 2 + 1e-3, np.pi / 2 - 1e-3, 9)
    for b in bearings:
        for e in elevations:
            ray = angles_to_ray(b, e)
            b2, e2 = ray_to_angles(ray)
            assert abs(b2 - b) < 1e-9
            assert abs(e2 - e) < 1e-9


def test_angle_ray_roundtrip_batch():
    """批量角度 (N,3) 射线往返同样精确，且形状正确。"""
    b = np.array([-1.0, 0.0, 0.5, 2.0])
    e = np.array([-0.7, 0.0, 0.3, 0.9])
    rays = angles_to_ray(b, e)
    assert rays.shape == (4, 3)
    b2, e2 = ray_to_angles(rays)
    assert b2.shape == (4,) and e2.shape == (4,)
    assert np.max(np.abs(b2 - b)) < 1e-9
    assert np.max(np.abs(e2 - e)) < 1e-9


def test_zero_distortion_is_identity():
    """零畸变时正/反畸变都是恒等，1e-12 以内。"""
    intr = _intrinsics()
    rng = np.random.default_rng(0)
    pts = rng.uniform([0.0, 0.0], [640.0, 480.0], size=(50, 2))
    assert np.allclose(distort_points(pts, intr), pts, atol=1e-12)
    assert np.allclose(undistort_points(pts, intr), pts, atol=1e-12)


def test_distortion_roundtrip():
    """畸变后反畸变回到原像素，误差 1e-6 以内。"""
    intr = CameraIntrinsics(
        width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        dist_coeffs=(-0.05, 0.005, 0.0003, -0.0003, 0.0),
    )
    rng = np.random.default_rng(1)
    pts = rng.uniform([40.0, 40.0], [600.0, 440.0], size=(200, 2))
    distorted = distort_points(pts, intr)
    recovered = undistort_points(distorted, intr)
    err = np.max(np.abs(recovered - pts))
    assert err < 1e-6, f"畸变往返最大误差 {err} 超过 1e-6"


def test_affine_recovers_exact_mapping():
    """无噪对应点上，最小二乘精确恢复已知 2x3 仿射。"""
    rng = np.random.default_rng(2)
    angle_pts = rng.uniform([-20.0, -15.0], [20.0, 15.0], size=(60, 2))
    m_true = np.array([[0.05, 0.01, -16.0],
                       [0.004, 0.05, -12.0]])
    pan_tilt_pts = apply_affine(m_true, angle_pts)
    m_fit = solve_affine_angle_to_pan_tilt(angle_pts, pan_tilt_pts)
    assert np.allclose(m_fit, m_true, atol=1e-9)


def test_affine_with_noise_residual_within_tolerance():
    """角度输入叠加 0.5 度噪声后，对干净舵盘角真值的拟合残差仍在 0.05 度以内。

    0.5 度远大于 0.5 px 质心噪声对应的角度量级（fx=480 时约 0.06 度），这里刻意取更宽松
    的扰动，用来确认最小二乘对输入噪声的抑制。
    """
    rng = np.random.default_rng(3)
    angle_pts = rng.uniform([-20.0, -15.0], [20.0, 15.0], size=(200, 2))
    m_true = np.array([[0.05, 0.01, -16.0],
                       [0.004, 0.05, -12.0]])
    pan_tilt_clean = apply_affine(m_true, angle_pts)
    angle_noisy = angle_pts + rng.normal(0.0, 0.5, size=angle_pts.shape)
    m_fit = solve_affine_angle_to_pan_tilt(angle_noisy, pan_tilt_clean)
    residual = apply_affine(m_fit, angle_noisy) - pan_tilt_clean
    rms = float(np.sqrt(np.mean(residual ** 2)))
    assert rms < 0.05, f"含噪拟合 RMS 残差 {rms} 超过 0.05 度"


def test_single_and_batch_shapes():
    """单点输入返回单点形状，批量输入返回批量形状。"""
    intr = _intrinsics()
    ray1 = pixel_to_ray((320.0, 240.0), intr)
    assert ray1.shape == (3,)
    rays = pixel_to_ray(np.array([[320.0, 240.0], [400.0, 100.0]]), intr)
    assert rays.shape == (2, 3)

    b1, e1 = pixel_to_angles((320.0, 240.0), intr)
    assert np.isscalar(b1) and np.isscalar(e1)
    bN, eN = pixel_to_angles(np.array([[320.0, 240.0], [400.0, 100.0]]), intr)
    assert bN.shape == (2,) and eN.shape == (2,)

    d1 = distort_points((320.0, 240.0), intr)
    assert d1.shape == (2,)

    m = np.eye(2, 3)
    assert apply_affine(m, np.array([10.0, 20.0])).shape == (2,)
    assert apply_affine(m, np.array([[10.0, 20.0], [1.0, 2.0]])).shape == (2, 2)


def test_affine_shape_validation():
    with pytest.raises(ValueError):
        apply_affine(np.zeros((3, 3)), (1.0, 2.0))


def test_bearing_sign_convention():
    """右方正 bearing、上方正 elevation（锁定约定）。"""
    intr = _intrinsics()
    # 像素在中心右侧 -> bearing > 0
    b_right, _ = pixel_to_angles((500.0, 240.0), intr)
    b_left, _ = pixel_to_angles((140.0, 240.0), intr)
    assert b_right > 0.0 > b_left
    # 像素在中心上方（y 更小）-> elevation > 0
    _, e_up = pixel_to_angles((320.0, 100.0), intr)
    _, e_down = pixel_to_angles((320.0, 400.0), intr)
    assert e_up > 0.0 > e_down
