"""针孔相机几何与角度约定。C++ 移植以此为准，固件按同样约定编写。

坐标轴与角度正负号：
- 图像坐标：x 向右、y 向下，原点在左上角，单位 px。
- 相机坐标系：X 右、Y 下、Z 前。Y 向下与常见的"Y 上"相机系相反。
- bearing = atan2(X, Z)，单位 rad，向右为正（俯视顺时针）。
- elevation = atan2(-Y, sqrt(X^2 + Z^2))，单位 rad，向上为正。Y 向下，取负号后目标位于
  图像上方（Y<0）时俯仰角为正。
- pan 角正值表示舵盘向右转（俯视顺时针），tilt 角正值表示摄像头向上抬。
- 去畸变后的像素与射线满足针孔关系：
      X = (x - cx) / fx,  Y = (y - cy) / fy,  Z = 1，再归一化为单位向量。

所有函数同时接受单点 (2,) 与批量 (N, 2)，射线接受 (3,) / (N, 3)。内部用 np.atleast_2d
归一化，返回形状与输入对应，单点返回单点形状。
"""

from __future__ import annotations

import numpy as np

from .types import CameraIntrinsics

__all__ = [
    "distort_points",
    "undistort_points",
    "pixel_to_ray",
    "ray_to_angles",
    "angles_to_ray",
    "pixel_to_angles",
    "solve_affine_angle_to_pan_tilt",
    "apply_affine",
]


def _batch(pts) -> tuple[np.ndarray, bool]:
    """把输入归一化为 (N, 2)，并返回是否为单点的标志。"""
    arr = np.asarray(pts, dtype=np.float64)
    single = arr.ndim == 1
    return np.atleast_2d(arr), single


def _dist_coeffs5(intrinsics: CameraIntrinsics) -> tuple[float, float, float, float, float]:
    """取 (k1, k2, p1, p2, k3)；不足 5 个时补 0。"""
    c = tuple(float(v) for v in intrinsics.dist_coeffs) + (0.0,) * 5
    return c[0], c[1], c[2], c[3], c[4]


def distort_points(pts, intrinsics: CameraIntrinsics) -> np.ndarray:
    """正向畸变：无畸变像素 -> 畸变像素，标准径向-切向模型 (k1,k2,p1,p2,k3)。

    归一化坐标 (xn, yn)，r2 = xn^2 + yn^2：
        radial = 1 + k1 r2 + k2 r2^2 + k3 r2^3
        xd = xn * radial + 2 p1 xn yn + p2 (r2 + 2 xn^2)
        yd = yn * radial + p1 (r2 + 2 yn^2) + 2 p2 xn yn
    再乘焦距加主点回到像素。用于合成标定数据和往返测试，也是 undistort 的对照模型。

    输入 (2,) 或 (N,2)，返回同形状。
    """
    arr, single = _batch(pts)
    k1, k2, p1, p2, k3 = _dist_coeffs5(intrinsics)
    xn = (arr[:, 0] - intrinsics.cx) / intrinsics.fx
    yn = (arr[:, 1] - intrinsics.cy) / intrinsics.fy
    r2 = xn * xn + yn * yn
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    out = np.empty_like(arr)
    out[:, 0] = xd * intrinsics.fx + intrinsics.cx
    out[:, 1] = yd * intrinsics.fy + intrinsics.cy
    return out[0] if single else out


def undistort_points(pts, intrinsics: CameraIntrinsics) -> np.ndarray:
    """反畸变：畸变像素 -> 无畸变像素，模型与参数同 distort_points。

    畸变模型没有解析逆，用不动点迭代求解。固定观测的畸变坐标 xn，反复修正无畸变估计：
        xu = (xn - tangential(xu)) / radial(xu)
    定长 8 次即可对常规镜头收敛到亚微米像素精度。迭代次数固定，便于 C++ 端做确定性实现，
    也避免在 r 趋近 0 时解析求根带来的数值不稳定。

    输入 (2,) 或 (N,2)，返回同形状。
    """
    arr, single = _batch(pts)
    k1, k2, p1, p2, k3 = _dist_coeffs5(intrinsics)
    xn = (arr[:, 0] - intrinsics.cx) / intrinsics.fx
    yn = (arr[:, 1] - intrinsics.cy) / intrinsics.fy
    xu = xn.copy()
    yu = yn.copy()
    for _ in range(8):
        r2 = xu * xu + yu * yu
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        tx = 2.0 * p1 * xu * yu + p2 * (r2 + 2.0 * xu * xu)
        ty = p1 * (r2 + 2.0 * yu * yu) + 2.0 * p2 * xu * yu
        xu = (xn - tx) / radial
        yu = (yn - ty) / radial
    out = np.empty_like(arr)
    out[:, 0] = xu * intrinsics.fx + intrinsics.cx
    out[:, 1] = yu * intrinsics.fy + intrinsics.cy
    return out[0] if single else out


def pixel_to_ray(pts, intrinsics: CameraIntrinsics) -> np.ndarray:
    """去畸变后的像素坐标 -> 相机系单位方向向量。

    相机系约定 X 右、Y 下、Z 前，射线 = normalize([(x-cx)/fx, (y-cy)/fy, 1])。
    输入应为无畸变像素；有畸变时先调用 undistort_points，pixel_to_angles 已经把两步串好。

    输入 (2,) -> (3,)，输入 (N,2) -> (N,3)。
    """
    arr, single = _batch(pts)
    rays = np.empty((arr.shape[0], 3), dtype=np.float64)
    rays[:, 0] = (arr[:, 0] - intrinsics.cx) / intrinsics.fx
    rays[:, 1] = (arr[:, 1] - intrinsics.cy) / intrinsics.fy
    rays[:, 2] = 1.0
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    return rays[0] if single else rays


def ray_to_angles(rays) -> tuple:
    """相机系单位方向向量 -> (bearing_rad, elevation_rad)。

    bearing   = atan2(X, Z)，向右为正；
    elevation = atan2(-Y, sqrt(X^2 + Z^2))，向上为正。
    取值范围 bearing ∈ (-pi, pi]，elevation ∈ [-pi/2, pi/2]。

    输入 (3,) 返回两个 Python float；输入 (N,3) 返回两个 (N,) 数组。
    """
    arr = np.asarray(rays, dtype=np.float64)
    single = arr.ndim == 1
    a = np.atleast_2d(arr)
    bearing = np.arctan2(a[:, 0], a[:, 2])
    elevation = np.arctan2(-a[:, 1], np.sqrt(a[:, 0] ** 2 + a[:, 2] ** 2))
    if single:
        return float(bearing[0]), float(elevation[0])
    return bearing, elevation


def angles_to_ray(bearing_rad, elevation_rad) -> np.ndarray:
    """(bearing_rad, elevation_rad) -> 相机系单位方向向量，与 ray_to_angles 互逆。

    推导：设 e = elevation、b = bearing。Z 取前向，水平旋转 b 得到 (sin b, 0, cos b)；
    再绕水平轴抬升 e（向上为正，Y 向下取负）：
        X = cos(e) sin(b),  Y = -sin(e),  Z = cos(e) cos(b)。
    可验证 X^2+Y^2+Z^2 = 1，且 ray_to_angles(angles_to_ray(b,e)) == (b,e)（|e| < pi/2）。

    标量输入 -> (3,)，数组输入 -> (N,3)。
    """
    b = np.asarray(bearing_rad, dtype=np.float64)
    e = np.asarray(elevation_rad, dtype=np.float64)
    scalar = b.ndim == 0
    X = np.cos(e) * np.sin(b)
    Y = -np.sin(e)
    Z = np.cos(e) * np.cos(b)
    if scalar:
        return np.array([float(X), float(Y), float(Z)], dtype=np.float64)
    return np.stack([X, Y, Z], axis=-1)


def pixel_to_angles(pts, intrinsics: CameraIntrinsics) -> tuple:
    """像素 -> (bearing_rad, elevation_rad) 的便捷组合。

    依次执行 undistort_points、pixel_to_ray、ray_to_angles。
    输入 (2,) 返回两个标量；输入 (N,2) 返回两个 (N,) 数组。
    """
    undist = undistort_points(pts, intrinsics)
    rays = pixel_to_ray(undist, intrinsics)
    return ray_to_angles(rays)


def solve_affine_angle_to_pan_tilt(angle_pts, pan_tilt_pts) -> np.ndarray:
    """用最小二乘拟合 2x3 仿射矩阵 A，使 pan_tilt ≈ A @ [bearing, elevation, 1]^T。

    标定仿射的输入是角度，输出是舵盘角，定义见 docs/标定与几何推导.md 第 8 节：
    q = A · p̃，其中 p̃ = (bearing, elevation, 1)，q = (pan, tilt)。像素到角度的转换由
    pixel_to_angles 完成，在本函数之前。

    参数：
    - angle_pts (N,2)：标定采集的 (bearing, elevation)，单位度。
    - pan_tilt_pts (N,2)：对应的 (pan, tilt) 舵盘角，单位度。

    返回 A (2,3)。用 np.linalg.lstsq 求最小二乘解，可处理超定和有噪数据，不显式求逆。
    点数少于 3 时无唯一解，调用方需自行保证。
    """
    p = np.atleast_2d(np.asarray(angle_pts, dtype=np.float64))
    q = np.atleast_2d(np.asarray(pan_tilt_pts, dtype=np.float64))
    if p.shape[0] != q.shape[0]:
        raise ValueError(f"角度点与舵盘角点数量不一致: {p.shape[0]} vs {q.shape[0]}")
    design = np.column_stack([p, np.ones(p.shape[0], dtype=np.float64)])  # (N,3)
    m = np.empty((2, 3), dtype=np.float64)
    m[0] = np.linalg.lstsq(design, q[:, 0], rcond=None)[0]
    m[1] = np.linalg.lstsq(design, q[:, 1], rcond=None)[0]
    return m


def apply_affine(matrix, pts) -> np.ndarray:
    """应用 2x3 仿射：out = M @ [x, y, 1]^T。

    输入 (2,) -> (2,)，输入 (N,2) -> (N,2)。矩阵形状不是 (2,3) 时抛 ValueError。
    """
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape != (2, 3):
        raise ValueError(f"仿射矩阵必须是 2x3，实际形状 {m.shape}")
    arr, single = _batch(pts)
    hom = np.column_stack([arr, np.ones(arr.shape[0], dtype=np.float64)])  # (N,3)
    out = hom @ m.T
    return out[0] if single else out
