"""aim：ESP32-S3 双轴电机舵盘目标锁定项目的算法核心，Python 参考实现。

坐标轴方向、角度正负号、单位约定见 geometry.py 和 types.py 顶部注释，C++ 固件按同样约定手工
移植。顶层入口是 AimSolver，合成数据生成见 sim。

单位速查：角度内部用 rad，对外指令用 deg；时间 ms，提前量 s；长度按字段名注明 mm 或 px。
"""

from __future__ import annotations

from .angle_utils import (
    clamp,
    move_toward,
    normalize_180,
    normalize_to_range,
    shortest_delta_deg,
    wrap_360,
)
from .filters import (
    AlphaBetaFilter,
    KalmanCV,
    TrackLossDetector,
    TrackStatus,
    TwoAxisTracker,
)
from .geometry import (
    angles_to_ray,
    apply_affine,
    distort_points,
    pixel_to_angles,
    pixel_to_ray,
    ray_to_angles,
    solve_affine_angle_to_pan_tilt,
    undistort_points,
)
from .predictor import (
    predict_linear,
    predict_quadratic,
    required_lead_time,
    should_use_quadratic,
)
from .sim import (
    make_linear_target,
    make_occluded_track,
    make_sinusoidal_target,
    make_static_target,
)
from .solver import AimSolver, SolverConfig
from .types import (
    AngularObservation,
    AngularRate,
    AxisLimits,
    CalibrationModel,
    CameraIntrinsics,
    PixelObservation,
    TurretLimits,
    TurretSolution,
)

__version__ = "0.1.0"

__all__ = [
    "__version__",
    # solver
    "AimSolver",
    "SolverConfig",
    # types
    "AngularObservation",
    "AngularRate",
    "AxisLimits",
    "CalibrationModel",
    "CameraIntrinsics",
    "PixelObservation",
    "TurretLimits",
    "TurretSolution",
    # geometry
    "angles_to_ray",
    "apply_affine",
    "distort_points",
    "pixel_to_angles",
    "pixel_to_ray",
    "ray_to_angles",
    "solve_affine_angle_to_pan_tilt",
    "undistort_points",
    # angle utils
    "clamp",
    "move_toward",
    "normalize_180",
    "normalize_to_range",
    "shortest_delta_deg",
    "wrap_360",
    # filters
    "AlphaBetaFilter",
    "KalmanCV",
    "TrackLossDetector",
    "TrackStatus",
    "TwoAxisTracker",
    # predictor
    "predict_linear",
    "predict_quadratic",
    "required_lead_time",
    "should_use_quadratic",
    # sim
    "make_linear_target",
    "make_occluded_track",
    "make_sinusoidal_target",
    "make_static_target",
]
