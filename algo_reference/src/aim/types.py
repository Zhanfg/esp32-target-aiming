"""核心数据类型定义，纯数据容器，不含算法逻辑。

坐标与角度约定与 geometry.py 一致，C++ 移植以此为准：
- 图像坐标：x 向右、y 向下，原点左上角。
- 相机坐标系：X 右、Y 下、Z 前。
- bearing = atan2(X, Z)，向右为正（俯视顺时针）。
- elevation = atan2(-Y, sqrt(X^2+Z^2))，向上为正。
- pan 角正值表示舵盘向右转（俯视顺时针），tilt 角正值表示摄像头向上抬。
- 单位：角度内部用弧度，对外用度并带 `_deg` 后缀；时间用毫秒 `_ms`；长度按字段名注明 mm 或 px。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .angle_utils import clamp as _clamp
from .angle_utils import shortest_delta_deg as _shortest_delta_deg


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔相机内参和径向-切向畸变系数。

    - fx/fy 单位 px，主点 (cx, cy) 单位 px。
    - dist_coeffs 约定为 (k1, k2, p1, p2, k3)，长度不足 5 时补 0。
    - 相机系 Y 向下（见模块顶部），与常见的"Y 上"相机系相反；焦距符号约定保证像素关系
      x = fx * X/Z + cx 仍然成立。
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist_coeffs: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def scale(self, f: float) -> "CameraIntrinsics":
        """按因子 f 缩放分辨率、焦距和主点，用于图像缩放或降采样。

        分辨率改变后同一物理视场要映射到同一组射线，所以 fx、fy、cx、cy 一起线性缩放。
        畸变系数不变：归一化坐标下的畸变模型与分辨率无关。
        """
        return CameraIntrinsics(
            width=int(round(self.width * f)),
            height=int(round(self.height * f)),
            fx=self.fx * f,
            fy=self.fy * f,
            cx=self.cx * f,
            cy=self.cy * f,
            dist_coeffs=self.dist_coeffs,
        )

    @classmethod
    def default_for(cls, width: int, height: int) -> "CameraIntrinsics":
        """生成占位用的针孔默认值：fx = fy = 0.8 * width（水平视场约 64°），正方形像素，
        主点取图像中心。没有实际标定数据时使用，真实参数应由标定流程覆盖。
        """
        fx = 0.8 * width
        fy = 0.8 * width  # 正方形像素假设：fy 与 fx 相同
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        return cls(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy,
                   dist_coeffs=(0.0, 0.0, 0.0, 0.0, 0.0))


@dataclass(frozen=True)
class PixelObservation:
    """像素观测：时间戳、目标像素坐标、置信度。

    - t_ms：观测时间戳，毫秒。
    - x/y：像素坐标，单位 px（x 向右、y 向下）。
    - confidence：取值 [0, 1]，0 表示无效或遮挡。
    """

    t_ms: float
    x: float
    y: float
    confidence: float = 1.0

    @property
    def valid(self) -> bool:
        """观测是否可用：置信度为正且坐标有限。"""
        return self.confidence > 0.0 and np.isfinite(self.x) and np.isfinite(self.y)


@dataclass(frozen=True)
class AngularObservation:
    """角度观测：时间戳、方位/俯仰角（弧度）、置信度。"""

    t_ms: float
    bearing_rad: float
    elevation_rad: float
    confidence: float = 1.0


@dataclass(frozen=True)
class AngularRate:
    """角速度，单位弧度/秒。"""

    bearing_rad_s: float
    elevation_rad_s: float


@dataclass(frozen=True)
class TurretSolution:
    """舵盘指令解：pan/tilt 目标角（度）、指令角速度（度/秒）、有效性。

    - pan_deg/tilt_deg：发给舵盘电机的目标角，已经过限位夹紧和角速度限幅。
    - pan_rate_dps/tilt_rate_dps：本帧相对上一帧的指令角速度，供固件做前馈。
    - valid=False 表示目标丢失或超时，调用方应转入搜索模式。
    """

    t_ms: float
    pan_deg: float
    tilt_deg: float
    pan_rate_dps: float
    tilt_rate_dps: float
    valid: bool
    confidence: float


@dataclass(frozen=True)
class AxisLimits:
    """单轴机械和指令限制。

    - min_deg/max_deg：机械行程，单位度。
    - max_slew_dps：指令角速度上限，单位度/秒。
    - deadband_deg：死区宽度，单位度，用于判断是否已对准。
    - wrap_360：该轴能否连续旋转（无硬止点，例如无挡圈的 pan 轴）。
    """

    min_deg: float
    max_deg: float
    max_slew_dps: float
    deadband_deg: float = 0.0
    wrap_360: bool = False

    def clamp(self, value: float) -> float:
        """把角度限制在允许范围内。

        wrap_360=False（有硬止点）直接夹紧到 [min_deg, max_deg]；wrap_360=True（连续旋转轴）
        把 value 环绕进 [min_deg, max_deg)，该轴没有止点，绕一整圈是合法动作。
        """
        if self.wrap_360:
            span = self.max_deg - self.min_deg
            return self.min_deg + float(np.mod(value - self.min_deg, span))
        return _clamp(value, self.min_deg, self.max_deg)

    def shortest_delta(self, a: float, b: float) -> float:
        """返回把 a 转到 b 所需的最短角差，单位度。

        wrap_360=True 走环绕分支，结果落在 (-180, 180]，允许跨过 ±180° 边界；wrap_360=False
        走线性分支，直接返回 b - a，不允许绕远路（轴有硬限位）。C++ 移植时这个分支结构原样保留。
        """
        if self.wrap_360:
            return _shortest_delta_deg(a, b, wrap_360=True)
        return b - a


@dataclass(frozen=True)
class TurretLimits:
    """双轴限制组合。"""

    pan: AxisLimits
    tilt: AxisLimits

    @classmethod
    def default(cls) -> "TurretLimits":
        """默认行程：pan ±90°，tilt -45°~+45°，角速度上限 360°/s，死区 1°。"""
        return cls(
            pan=AxisLimits(min_deg=-90.0, max_deg=90.0, max_slew_dps=360.0, deadband_deg=1.0),
            tilt=AxisLimits(min_deg=-45.0, max_deg=45.0, max_slew_dps=360.0, deadband_deg=1.0),
        )


@dataclass(frozen=True)
class CalibrationModel:
    """标定模型：相机内参、角度到舵盘角的仿射、零点偏移。

    - affine_angle_to_pan_tilt：2x3 矩阵 A，满足 q = A · p̃，其中
      p̃ = (bearing_deg, elevation_deg, 1)，q = (pan_deg, tilt_deg)，两者都不含零点偏移。
      A 由 solve_affine_angle_to_pan_tilt() 从标定对应点最小二乘拟合，涵盖安装偏差、
      光轴不对准等线性误差。像素到角度的转换由 pixel_to_angles 完成，不经过本矩阵。
    - pan_offset_deg/tilt_offset_deg：零点偏移（度），在仿射输出之后叠加。
    """

    intrinsics: CameraIntrinsics
    affine_angle_to_pan_tilt: np.ndarray
    pan_offset_deg: float = 0.0
    tilt_offset_deg: float = 0.0

    def __post_init__(self) -> None:
        arr = np.asarray(self.affine_angle_to_pan_tilt, dtype=np.float64)
        if arr.shape != (2, 3):
            raise ValueError(f"affine_angle_to_pan_tilt 必须是 2x3 矩阵，实际形状 {arr.shape}")
        object.__setattr__(self, "affine_angle_to_pan_tilt", arr)
