"""AimSolver：从像素观测到 pan/tilt 指令的完整编排，顶层数学实现。

流水线，固件按同样顺序复刻：

    畸变校正 -> 像素转角度 -> 双轴跟踪更新 -> 计算所需提前量 -> 外推至 now + lead
    -> 应用标定仿射与零点偏移 -> 映射为 pan/tilt 度 -> 限位夹紧 -> 角速度限幅 -> TurretSolution

单位与坐标约定与 types.py、geometry.py 一致：
- 内部角度用弧度（rad），角速度用弧度/秒（rad/s）。
- 舵盘指令 pan/tilt 用度（deg），角速度指令用度/秒（dps）。
- 时间戳用毫秒（ms），提前量和延迟用秒（s）。
- 图像坐标 x 向右、y 向下；相机系 X 右、Y 下、Z 前。
- bearing = atan2(X, Z) 向右为正，elevation = atan2(-Y, sqrt(X²+Z²)) 向上为正。
- pan 正表示舵盘向右转，tilt 正表示摄像头向上抬。

提前量的物理模型见 predictor.required_lead_time。激光直瞄没有弹丸飞行时间，滞后来自舵盘
执行机构的固定延迟和到位后的稳定时间。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .angle_utils import move_toward
from .filters import TwoAxisTracker
from .geometry import apply_affine, pixel_to_angles
from .predictor import (
    predict_linear,
    predict_quadratic,
    required_lead_time,
    should_use_quadratic,
)
from .types import (
    CalibrationModel,
    CameraIntrinsics,
    PixelObservation,
    TurretLimits,
    TurretSolution,
)


# tracker_measurement_noise 默认值的来源。取质心噪声 0.5 px、标称焦距 fx = 480 px
# （对应 02_REQUIREMENTS/标定与几何推导.md 第 7 节的 640x480 示例）：角噪声 = atan(0.5/480)
# ≈ 1.042e-3 rad，方差 ≈ 1.085e-6 rad²。这只是量级参考，固件侧应当用实测质心噪声换算
# 角度方差，不要直接沿用。
_NOMINAL_CENTROID_NOISE_PX = 0.5
_NOMINAL_FOCAL_LENGTH_PX = 480.0
_NOMINAL_ANGLE_NOISE_RAD = math.atan(_NOMINAL_CENTROID_NOISE_PX / _NOMINAL_FOCAL_LENGTH_PX)
_DEFAULT_MEASUREMENT_NOISE_RAD2 = _NOMINAL_ANGLE_NOISE_RAD ** 2


@dataclass
class SolverConfig:
    """AimSolver 的可调参数。

    必需字段：
    - lead_actuator_latency_s：舵盘指令链路的固定延迟，单位秒，提前量的主要成分。
    - lead_settle_s：到位后的稳定和超调平息时间，单位秒，计入提前量。
    - max_rate_rad_s：假定目标可能达到的最坏角速度，单位 rad/s，用于把误差预算换算成反应时间。
    - coast_timeout_s：观测丢失后允许靠滤波预测滑行的时间上限，单位秒。
    - quadratic_speed_threshold：启用二阶外推的速度阈值，单位 rad/s。
    - use_quadratic：是否允许二阶外推，目标快速机动时才有意义。

    辅助字段，都有合理默认值，C++ 端可只保留必需字段：
    - nominal_dt_s：名义控制周期，单位秒，用于第一条指令的限速步长和跟踪器名义 dt。
    - angular_error_budget_rad：允许的最大指向误差，单位 rad；None 表示取两轴死区中较小者。
    - tracker_process_noise / tracker_measurement_noise / tracker_gate_rad：透传给 TwoAxisTracker。
    - min_confidence：低于该置信度的观测视为无效，转入滑行。

    tracker_measurement_noise 的默认值由标称质心噪声推导，推导过程见文件顶部常量处的注释。
    它只是量级参考，固件侧应当用实测质心噪声换算角度方差。
    """

    lead_actuator_latency_s: float = 0.05
    lead_settle_s: float = 0.03
    max_rate_rad_s: float = 3.0
    coast_timeout_s: float = 0.5
    quadratic_speed_threshold: float = 0.5
    use_quadratic: bool = False
    nominal_dt_s: float = 0.05
    angular_error_budget_rad: float | None = None
    tracker_process_noise: float = 1e-4
    tracker_measurement_noise: float = _DEFAULT_MEASUREMENT_NOISE_RAD2
    tracker_gate_rad: float = 0.5
    min_confidence: float = 0.3


class AimSolver:
    """把像素观测流转换为 pan/tilt 舵盘指令的编排器。

    每个控制周期调用一次 update()，传入当前帧的 PixelObservation（无观测时传 None）和当前
    时刻 now_ms，返回 TurretSolution。
    """

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        calibration: CalibrationModel,
        limits: TurretLimits,
        config: SolverConfig | None = None,
    ) -> None:
        # intrinsics 用于畸变校正和像素到角度的换算；calibration.intrinsics 只作标定元数据。
        # 运行时几何换算显式使用形参 intrinsics，不读标定模型内部字段。
        self._intrinsics = intrinsics
        self._calibration = calibration
        self._limits = limits
        self._config = config if config is not None else SolverConfig()

        self._tracker = TwoAxisTracker(
            dt_s=self._config.nominal_dt_s,
            process_noise=self._config.tracker_process_noise,
            measurement_noise=self._config.tracker_measurement_noise,
            gate_rad=self._config.tracker_gate_rad,
        )
        self.reset()

    # ------------------------------------------------------------------ 公共接口
    def reset(self) -> None:
        """清空全部跟踪与指令状态，回到上电初始态。上电时假定舵盘位于零点。"""
        self._tracker.reset()
        self._track_t_ms: float | None = None      # 滤波器状态对应的时刻
        self._last_good_t_ms: float | None = None  # 最近一次"被接受"观测的时刻
        self._last_confidence: float = 0.0
        self._filtered_bearing_rad: float = 0.0
        self._filtered_elevation_rad: float = 0.0
        self._bearing_rate_rad_s: float = 0.0
        self._elevation_rate_rad_s: float = 0.0
        self._bearing_accel_rad_s2: float = 0.0
        self._elevation_accel_rad_s2: float = 0.0
        self._prev_rate_t_ms: float | None = None
        self._prev_bearing_rate: float = 0.0
        self._prev_elevation_rate: float = 0.0
        self._last_pan_cmd_deg: float = 0.0
        self._last_tilt_cmd_deg: float = 0.0
        self._last_cmd_t_ms: float | None = None
        self._last_valid: bool = False

    @property
    def estimated_angles_rad(self) -> tuple[float, float]:
        """最近一次滤波后的 (bearing_rad, elevation_rad) 估计。"""
        return self._filtered_bearing_rad, self._filtered_elevation_rad

    @property
    def is_tracking(self) -> bool:
        """最近一次 update() 是否判定航迹有效，滑行超时后为 False。"""
        return self._last_valid

    def update(self, pixel_obs: PixelObservation | None, now_ms: float) -> TurretSolution:
        """推进到 now_ms 并返回本帧舵盘指令。

        参数：
        - pixel_obs：PixelObservation，字段单位见 types.py；None、坐标非有限或置信度低于
          min_confidence 时按无观测处理，进入滑行。
        - now_ms：当前控制时刻，单位毫秒，用于滤波外推和滑行超时判定。

        返回 TurretSolution，pan/tilt 单位度，角速度单位度/秒，valid 表示航迹是否仍可用。
        """
        now_ms = float(now_ms)
        config = self._config

        # ---- 1) 观测门控与跟踪更新 -------------------------------------------
        obs_ok = (
            pixel_obs is not None
            and pixel_obs.valid
            and float(pixel_obs.confidence) >= config.min_confidence
        )
        if obs_ok:
            assert pixel_obs is not None  # 供类型检查器收窄
            t_obs = float(pixel_obs.t_ms)
            bearing, elevation = pixel_to_angles(
                (float(pixel_obs.x), float(pixel_obs.y)), self._intrinsics
            )
            if not self._tracker.initialized:
                # 首帧直接初始化，不做门控（没有先验可比）。
                out = self._tracker.update(
                    t_obs, bearing, elevation, float(pixel_obs.confidence)
                )
                self._track_t_ms = t_obs
                self._apply_tracker_output(out)
                self._last_good_t_ms = t_obs
                self._last_confidence = float(pixel_obs.confidence)
            else:
                dt_obs_s = (t_obs - (self._track_t_ms or t_obs)) / 1000.0
                if dt_obs_s <= 0.0:
                    # 重复/倒退时间戳：不推进滤波，避免除零（与 filters 的防御语义一致）。
                    pass
                else:
                    accepted = self._within_gate(t_obs, bearing, elevation)
                    out = self._tracker.update(
                        t_obs, bearing, elevation, float(pixel_obs.confidence)
                    )
                    self._track_t_ms = t_obs
                    self._apply_tracker_output(out)
                    if accepted:
                        self._last_good_t_ms = t_obs
                        self._last_confidence = float(pixel_obs.confidence)

        # ---- 2) 把滤波器状态推进到当前时刻（无观测时即为滑行） ----------------
        if (
            self._tracker.initialized
            and self._track_t_ms is not None
            and now_ms > self._track_t_ms
        ):
            out = self._tracker.predict(now_ms)
            self._track_t_ms = now_ms
            self._apply_tracker_output(out)

        # ---- 3) 有效性判定（滑行超时 -> 丢失） --------------------------------
        valid = (
            self._tracker.initialized
            and self._last_good_t_ms is not None
            and (now_ms - self._last_good_t_ms) <= config.coast_timeout_s * 1000.0
        )
        self._last_valid = valid
        if not valid:
            # 丢失时保持上一条指令不动，速率置零，交给固件转搜索模式。
            return TurretSolution(
                t_ms=now_ms,
                pan_deg=self._last_pan_cmd_deg,
                tilt_deg=self._last_tilt_cmd_deg,
                pan_rate_dps=0.0,
                tilt_rate_dps=0.0,
                valid=False,
                confidence=0.0,
            )

        # ---- 4) 提前量预算 + 外推 --------------------------------------------
        error_budget_rad = config.angular_error_budget_rad
        if error_budget_rad is None:
            # 默认用两轴死区中较小者作为指向误差预算（最能约束提前量的那根轴）。
            error_budget_rad = math.radians(
                min(self._limits.pan.deadband_deg, self._limits.tilt.deadband_deg)
            )
        lead_s = required_lead_time(
            float(error_budget_rad),
            config.max_rate_rad_s,
            config.lead_actuator_latency_s,
            config.lead_settle_s,
        )

        rates = [self._bearing_rate_rad_s, self._elevation_rate_rad_s]
        if config.use_quadratic and should_use_quadratic(
            rates, config.quadratic_speed_threshold
        ):
            bearing_pred = predict_quadratic(
                now_ms, lead_s, self._filtered_bearing_rad,
                self._bearing_rate_rad_s, self._bearing_accel_rad_s2,
            )
            elevation_pred = predict_quadratic(
                now_ms, lead_s, self._filtered_elevation_rad,
                self._elevation_rate_rad_s, self._elevation_accel_rad_s2,
            )
        else:
            bearing_pred = predict_linear(
                now_ms, lead_s, self._filtered_bearing_rad, self._bearing_rate_rad_s
            )
            elevation_pred = predict_linear(
                now_ms, lead_s, self._filtered_elevation_rad, self._elevation_rate_rad_s
            )

        # ---- 5) 标定仿射 + 零点偏移 -> pan/tilt（度） -------------------------
        # 仿射的输入是预测后的方向角向量 [bearing_deg, elevation_deg, 1]，输出未加偏移的
        # (pan_deg, tilt_deg)。定义见 02_REQUIREMENTS/标定与几何推导.md 第 8 节和 CalibrationModel。
        bearing_deg = math.degrees(bearing_pred)
        elevation_deg = math.degrees(elevation_pred)
        mapped = apply_affine(
            self._calibration.affine_angle_to_pan_tilt, (bearing_deg, elevation_deg)
        )
        pan_target = float(mapped[0]) + self._calibration.pan_offset_deg
        tilt_target = float(mapped[1]) + self._calibration.tilt_offset_deg

        # ---- 6) 限位夹紧 ------------------------------------------------------
        pan_target = self._limits.pan.clamp(pan_target)
        tilt_target = self._limits.tilt.clamp(tilt_target)

        # ---- 7) 相对上一条指令做角速度限幅 ------------------------------------
        if self._last_cmd_t_ms is None:
            # 上电后首条指令：假定舵盘位于零点、以名义周期起步，因此首步也受限速约束。
            dt_s = float(config.nominal_dt_s)
        else:
            dt_s = (now_ms - self._last_cmd_t_ms) / 1000.0

        if dt_s <= 0.0:
            # 重复时间戳：保持上一条指令，速率置零，避免除零。
            pan_cmd = self._last_pan_cmd_deg
            tilt_cmd = self._last_tilt_cmd_deg
            pan_rate = 0.0
            tilt_rate = 0.0
        else:
            pan_cmd = move_toward(
                self._last_pan_cmd_deg, pan_target,
                self._limits.pan.max_slew_dps * dt_s,
                wrap_360=self._limits.pan.wrap_360,
            )
            tilt_cmd = move_toward(
                self._last_tilt_cmd_deg, tilt_target,
                self._limits.tilt.max_slew_dps * dt_s,
                wrap_360=self._limits.tilt.wrap_360,
            )
            # move_toward 已保证单步不超限；再夹紧一次以吸收标定/限位在运行中被改动的极端情况。
            pan_cmd = self._limits.pan.clamp(pan_cmd)
            tilt_cmd = self._limits.tilt.clamp(tilt_cmd)
            # 角速度必须用最短角差（环绕轴跨 ±180 时直接相减会得到错误的巨大值）。
            pan_delta = self._limits.pan.shortest_delta(self._last_pan_cmd_deg, pan_cmd)
            tilt_delta = self._limits.tilt.shortest_delta(self._last_tilt_cmd_deg, tilt_cmd)
            pan_rate = pan_delta / dt_s
            tilt_rate = tilt_delta / dt_s

        self._last_pan_cmd_deg = pan_cmd
        self._last_tilt_cmd_deg = tilt_cmd
        self._last_cmd_t_ms = now_ms

        return TurretSolution(
            t_ms=now_ms,
            pan_deg=pan_cmd,
            tilt_deg=tilt_cmd,
            pan_rate_dps=pan_rate,
            tilt_rate_dps=tilt_rate,
            valid=True,
            confidence=self._last_confidence,
        )

    # ------------------------------------------------------------------ 内部工具
    def _apply_tracker_output(self, out: tuple) -> None:
        """把 TwoAxisTracker 的 (b, e, brate, erate, valid) 写入内部状态，并估计加速度。"""
        bearing, elevation, bearing_rate, elevation_rate = out[0], out[1], out[2], out[3]
        if self._prev_rate_t_ms is not None and self._track_t_ms is not None:
            dt_s = (self._track_t_ms - self._prev_rate_t_ms) / 1000.0
            if dt_s > 0.0:
                # 相邻两帧速度差分估计加速度，供二阶外推使用；真实加速度小时会被 predictor
                # 的 accel_floor 自动忽略。
                self._bearing_accel_rad_s2 = (
                    bearing_rate - self._prev_bearing_rate
                ) / dt_s
                self._elevation_accel_rad_s2 = (
                    elevation_rate - self._prev_elevation_rate
                ) / dt_s
        self._prev_bearing_rate = float(bearing_rate)
        self._prev_elevation_rate = float(elevation_rate)
        self._prev_rate_t_ms = self._track_t_ms
        self._filtered_bearing_rad = float(bearing)
        self._filtered_elevation_rad = float(elevation)
        self._bearing_rate_rad_s = float(bearing_rate)
        self._elevation_rate_rad_s = float(elevation_rate)

    def _within_gate(self, t_obs_ms: float, bearing_rad: float, elevation_rad: float) -> bool:
        """相对上一状态做匀速外推，判断新观测是否落在门控半径内。

        跟踪器内部也做门控，这里再做一次是为了让编排器知道这条观测是否被采信，从而正确刷新
        last_good 时刻和滑行超时。判据与跟踪器一致：残差 <= gate_rad。
        """
        if self._track_t_ms is None:
            return True
        dt_s = (float(t_obs_ms) - self._track_t_ms) / 1000.0
        if dt_s <= 0.0:
            return False
        pred_b = self._filtered_bearing_rad + self._bearing_rate_rad_s * dt_s
        pred_e = self._filtered_elevation_rad + self._elevation_rate_rad_s * dt_s
        # 残差按 (-pi, pi] 环绕，避免目标恰好跨过 ±180° 时被误判成巨大离群。
        res_b = math.atan2(math.sin(float(bearing_rad) - pred_b),
                           math.cos(float(bearing_rad) - pred_b))
        res_e = math.atan2(math.sin(float(elevation_rad) - pred_e),
                           math.cos(float(elevation_rad) - pred_e))
        gate = self._config.tracker_gate_rad
        return abs(res_b) <= gate and abs(res_e) <= gate
