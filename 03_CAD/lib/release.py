"""RELEASE_MODULE：机械自锁卡榫与电磁铁拔销。

工作原理
--------
滑块把预载弹簧的力通过一个 6° 升角的斜面压在卡榫端部。载荷在斜面上产生的释放
分力是 N*tan(6°)≈0.105N，摩擦能提供的保持力是 mu*N=0.15N，所以卡榫在升角上自锁，
不需要通电就能保持啮合。电磁铁只拔出一根 1 mm 止推销，不承担主载荷。

止推销由自身复位弹簧压在卡榫的锁止孔里。卡榫要转开，必须先让销退出。线圈断电时
销保持插入，卡榫又被自锁面挡住，于是掉电不会释放，失效方向落在安全侧，与规格书
§10.4 禁止用软件时序替代机械互锁的要求一致。

几何只表达保持与拔销两个功能，不表达磁路。电磁铁选型力、销摩擦与斜面摩擦系数
都要实测标定，见 params.PARAM_SOURCES。
"""

from __future__ import annotations

import math

from . import params as P


def self_lock_margin() -> dict:
    """自锁余量。tan(升角) 要小于摩擦系数，比值越大越安全。"""
    angle = math.radians(P.REL_SEAR_ANGLE_DEG)
    drive = math.tan(angle)
    hold = P.REL_FRICTION_MU
    return {
        "angle_deg": P.REL_SEAR_ANGLE_DEG,
        "friction_angle_deg": P.SELF_LOCK_FRICTION_ANGLE,
        "drive_factor": drive,
        "hold_factor": hold,
        "margin": hold / drive,
    }


def pin_shear_area() -> float:
    """止推销受剪断面，mm^2。"""
    return math.pi * (P.REL_PIN_D / 2.0) ** 2


def build(z0: float, z1: float) -> list:
    parts = []
    sx, sy, sz = P.REL_SEAR_SIZE
    sear_z = z0 + 5.5
    # 卡榫本体，绕 Y 向枢轴转动
    parts.append(P.box("RELEASE_sear", "RELEASE_MODULE", "steel301",
                       (sx, sy, sz), (P.REL_SEAR_X, 0.0, sear_z),
                       note=f"自锁面 {P.REL_SEAR_ANGLE_DEG:.0f}°，tan 小于摩擦系数"))
    # 枢轴销
    parts.append(P.cyl("RELEASE_pivot_pin", "RELEASE_MODULE", "steel301",
                       P.REL_PIVOT_D / 2.0, P.REL_PIVOT_LEN, "y",
                       (P.REL_SEAR_X, 0.0, z0 + 5.0)))
    # 卡榫复位弹簧，使卡榫在拔销后退回让开
    parts.append(P.cyl("RELEASE_return_spring", "RELEASE_MODULE", "steel301",
                       P.REL_RETURN_SPRING_D / 2.0, P.REL_RETURN_SPRING_L, "z",
                       (P.REL_SEAR_X, 0.0, z0 + 1.5),
                       note="压缩弹簧，复位卡榫"))
    # 电磁铁本体
    ox, oy, oz = P.REL_SOLENOID_SIZE
    parts.append(P.box("RELEASE_solenoid_body", "RELEASE_MODULE", "petg",
                       (ox, oy, oz), (P.REL_SOLENOID_X, 0.0, z0 + 8.0),
                       note=f"微型推拉电磁铁，力约 {P.REL_SOLENOID_FORCE:.1f} N，待实测标定"))
    # 止推销，沿 Y 插入卡榫锁止孔
    parts.append(P.cyl("RELEASE_detent_pin", "RELEASE_MODULE", "steel301",
                       P.REL_PIN_D / 2.0, P.REL_PIN_LEN, "y",
                       (P.REL_SEAR_X, 0.0, z0 + 8.0),
                       note="掉电保持插入，销在则卡榫不能转"))
    # 销复位弹簧，压紧方向为插入
    parts.append(P.cyl("RELEASE_pin_spring", "RELEASE_MODULE", "steel301",
                       P.REL_PIN_SPRING_D / 2.0, P.REL_PIN_SPRING_L, "y",
                       (P.REL_SEAR_X, -6.0, z0 + 8.0),
                       note="销复位弹簧，断电时保持销插入"))
    # 滑块锁止块，卡榫扣在其台阶上
    lx, ly, lz = P.REL_LATCH_SIZE
    parts.append(P.box("RELEASE_latch_block", "RELEASE_MODULE", "petg",
                       (lx, ly, lz), (0.0, 0.0, z0 + 9.5),
                       note="滑块锁止台阶，卡榫扣合处"))
    return parts


def describe() -> str:
    m = self_lock_margin()
    return (
        f"自锁升角 {m['angle_deg']:.1f}°，摩擦角 {m['friction_angle_deg']:.2f}°，"
        f"保持比 {m['margin']:.2f}（>1 即自锁）；止推销断面 {pin_shear_area():.2f} mm^2；"
        f"掉电时销弹簧保持插入，失效方向安全"
    )
