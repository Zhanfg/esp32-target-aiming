"""MST-01 A/B/C 三级原型 CAD 参数集中处。

位置：03_CAD/lib/params.py

单位与坐标
----------
CAD 尺寸用 mm。仿真模型 04_SIMULATION/models/params.py 用 SI（m, N, kg），
几何量通过 MM_PER_M 换算对齐，不在 CAD 里另立一套数字。

+Z 为轴向，也就是预载与推进方向；X 是失稳单元拱跨方向；Y 是竖直方向。
§1.2 的「核心脉冲模块长度」对应 Z 向总长，「核心横向尺寸」对应 X/Y 中较大者。

尺寸来源
--------
关键尺寸在 PARAM_SOURCES 里登记，取值只有三类：
- 规格书章节，例如 ``§1.2``；
- 工程估计，附依据，例如标准件规格、材料手册量级、壁厚工艺限制；
- ``待实测标定``。

本文件不写没有出处的「实测值」。材料弹性常数与失稳等效刚度沿用仿真模型名义值，
并同时标 ``待实测标定``。

几何原语
--------
Solid 是只做尺寸、体积、包围盒与质量估算的轻量件，不依赖 FreeCAD。box 为长方体，
cylinder 为实心圆柱，tube 为空心圆筒（压环），leaf 为平面内倾斜薄片（失稳单元弹性片）。
FreeCAD 导出在 lib/freecad_export.py，另一步处理。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MM_PER_M = 1000.0

# ---------------------------------------------------------------------------
# §1.2 尺寸 envelope，单位 mm。这是工程目标带，不是冻结尺寸。
# ---------------------------------------------------------------------------
ENVELOPE = {
    "length_min": 30.0,
    "length_max": 50.0,
    "lateral_min": 15.0,
    "lateral_max": 25.0,
}

# ---------------------------------------------------------------------------
# 框架参数
# ---------------------------------------------------------------------------
FRAME_W = 20.0          # X 向外廓
FRAME_H = 18.0          # Y 向外廓
FRAME_WALL = 1.6        # 侧壁厚
FRAME_BASE_T = 1.6      # 底板厚
FRAME_REAR_T = 2.0      # 后盖板厚
FRAME_RIM_T = 2.0       # 前端膜片法兰压边厚
FRAME_BOSS_D = 3.2      # 后盖安装凸台直径（M3 沉台量级）
FRAME_BOSS_H = 2.0

# ---------------------------------------------------------------------------
# 失稳单元（von Mises truss）几何，与 04_SIMULATION/models/params.py 对齐
# ---------------------------------------------------------------------------
STAGE_HALF_SPAN = 8.0e-3 * MM_PER_M          # a = 8 mm
STAGE_BAR_LEN = 10.0e-3 * MM_PER_M           # L = 10 mm
STAGE_APEX_H = math.sqrt(STAGE_BAR_LEN**2 - STAGE_HALF_SPAN**2)  # h0 = 6 mm
STAGE_LEAF_T_PET = 0.5        # PET 弹性片厚度，早期几何验证
STAGE_LEAF_T_STEEL = 0.3      # 301/304 弹簧钢片厚度，寿命版
STAGE_LEAF_DEPTH = 10.0       # 弹性片宽度（Y 向）
STAGE_APEX_BLOCK = (8.0, 10.0, 4.0)   # 顶点滑块 x/y/z
STAGE_BASE_BLOCK = (4.0, 10.0, 3.0)   # 底部夹块
STAGE_PIN_D = 1.5
STAGE_PIN_LEN = 12.0
STAGE_CLAMP_T = 2.0
STAGE_K_BAR = 2000.0          # 等效轴向刚度 N/m，对齐仿真
STAGE_K_BASE = 1.0e4          # 边界刚度 N/m，对齐仿真

# ---------------------------------------------------------------------------
# 膜片与可拆压环
# ---------------------------------------------------------------------------
MEMB_CLAMP_RADIUS = 8.0e-3 * MM_PER_M        # 夹持半径 8 mm，对齐仿真
MEMB_APERTURE_D = 2.0 * MEMB_CLAMP_RADIUS    # 通径 16 mm
MEMB_RING_OD = 22.0
MEMB_RING_R_IN = MEMB_CLAMP_RADIUS
MEMB_DISC_R = MEMB_RING_OD / 2.0 - 1.0       # 膜片裁切半径 10 mm
MEMB_RING_T = 1.5                            # 单片压环厚
MEMB_SCREW_N = 4
MEMB_SCREW_D = 2.0
MEMB_SCREW_L = 6.0
MEMB_SCREW_PCD = 19.0                        # 螺钉分布圆直径
# 膜片材料厚度，对齐仿真 MEMBRANE 参数
MEMB_DISC_THICKNESS = {"silicone": 0.2, "pet": 0.05}

# ---------------------------------------------------------------------------
# 释放机构：机械自锁卡榫 + 电磁铁拔销
# ---------------------------------------------------------------------------
REL_SEAR_ANGLE_DEG = 6.0      # 自锁面升角
REL_FRICTION_MU = 0.15        # 钢-钢干摩擦系数量级
REL_SEAR_SIZE = (3.0, 8.0, 12.0)     # 卡榫体 x/y/z
REL_SEAR_X = 6.5              # 卡榫所在 x 位置
REL_PIVOT_D = 2.0
REL_PIVOT_LEN = 12.0
REL_RETURN_SPRING_D = 3.0
REL_RETURN_SPRING_L = 4.0
REL_SOLENOID_SIZE = (7.0, 9.0, 7.0)  # 微型推拉电磁铁本体
REL_SOLENOID_X = 4.0
REL_SOLENOID_FORCE = 2.5      # N，常见 5V 微型推拉电磁铁量级
REL_PIN_D = 1.0
REL_PIN_LEN = 9.0
REL_PIN_STROKE = 2.0
REL_PIN_SPRING_D = 2.5
REL_PIN_SPRING_L = 4.0
REL_LATCH_SIZE = (12.0, 8.0, 4.0)    # 滑块锁止块

# ---------------------------------------------------------------------------
# A 级低速执行器（无 snap，直接推膜片）
# ---------------------------------------------------------------------------
ACT_BODY_SIZE = (14.0, 14.0, 10.0)
ACT_ROD_D = 3.0
ACT_ROD_LEN = 12.0

# ---------------------------------------------------------------------------
# 材料密度，kg/m^3。带材批次差异，弹性相关常数见各模块说明。
# ---------------------------------------------------------------------------
MATERIALS = {
    "pla":      {"density": 1240.0, "role": "框架打印件"},
    "petg":     {"density": 1270.0, "role": "框架与压环打印件"},
    "tpu":      {"density": 1200.0, "role": "§6.2 候选 B 膜片，本轮未选"},
    "silicone": {"density": 1100.0, "role": "§6.2 候选 A 膜片"},
    "pet":      {"density": 1380.0, "role": "§6.2 候选 C 膜片与 §7.2 弹性片"},
    "steel301": {"density": 7900.0, "role": "§7.2 弹簧钢弹性片与销"},
    "brass":    {"density": 8500.0, "role": "铜套量级"},
}

# 弹性片材料到厚度的映射
LEAF_THICKNESS = {"pet": STAGE_LEAF_T_PET, "steel301": STAGE_LEAF_T_STEEL}


# ---------------------------------------------------------------------------
# 几何原语
# ---------------------------------------------------------------------------
@dataclass
class Solid:
    """只做尺寸与质量估算的轻量几何件，单位 mm。"""

    name: str
    module: str
    material: str
    kind: str = "box"                     # box | cylinder | tube | leaf
    size: tuple = (0.0, 0.0, 0.0)
    radius: float = 0.0
    inner_radius: float = 0.0
    length: float = 0.0
    axis: str = "z"
    pos: tuple = (0.0, 0.0, 0.0)          # 中心（leaf 只用 pos[1] 作 y）
    p0: tuple | None = None               # leaf 起点 (x, z)
    p1: tuple | None = None               # leaf 终点 (x, z)
    xsec: tuple = (0.0, 0.0)              # leaf 截面 (厚度, 宽度)
    count: int = 1
    note: str = ""

    def unit_volume(self) -> float:
        if self.kind == "box":
            return self.size[0] * self.size[1] * self.size[2]
        if self.kind == "cylinder":
            return math.pi * self.radius**2 * self.length
        if self.kind == "tube":
            return math.pi * (self.radius**2 - self.inner_radius**2) * self.length
        if self.kind == "leaf":
            length = math.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1])
            return self.xsec[0] * self.xsec[1] * length
        raise ValueError(f"未知 kind: {self.kind}")

    def volume(self) -> float:
        return self.unit_volume() * self.count

    def mass(self) -> float:
        """质量，g。体积 mm^3 乘密度 kg/m^3 再乘 1e-6。"""
        rho = MATERIALS[self.material]["density"]
        return self.volume() * rho * 1e-6

    def bbox(self):
        """返回 ((xmin, ymin, zmin), (xmax, ymax, zmax))，mm。"""
        cx, cy, cz = self.pos
        if self.kind == "box":
            hx, hy, hz = (s / 2.0 for s in self.size)
            return (cx - hx, cy - hy, cz - hz), (cx + hx, cy + hy, cz + hz)
        if self.kind in ("cylinder", "tube"):
            r = self.radius
            half = self.length / 2.0
            if self.axis == "z":
                h = (r, r, half)
            elif self.axis == "x":
                h = (half, r, r)
            else:
                h = (r, half, r)
            return (cx - h[0], cy - h[1], cz - h[2]), (cx + h[0], cy + h[1], cz + h[2])
        # leaf：在 XZ 平面内的斜置薄片，厚度垂直于杆轴，宽度沿 Y
        (x0, z0), (x1, z1) = self.p0, self.p1
        dx, dz = x1 - x0, z1 - z0
        length = math.hypot(dx, dz)
        if length == 0.0:
            raise ValueError("leaf 长度为零")
        nx, nz = -dz / length, dx / length
        ht, hb = self.xsec[0] / 2.0, self.xsec[1] / 2.0
        xs, ys, zs = [], [], []
        for (ex, ez) in ((x0, z0), (x1, z1)):
            for sx in (-1.0, 1.0):
                xs.append(ex + sx * ht * nx)
                zs.append(ez + sx * ht * nz)
        ys = [cy - hb, cy + hb]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def box(name, module, material, size, pos, count=1, note=""):
    return Solid(name=name, module=module, material=material, kind="box",
                 size=tuple(size), pos=tuple(pos), count=count, note=note)


def cyl(name, module, material, radius, length, axis, pos, count=1, note=""):
    return Solid(name=name, module=module, material=material, kind="cylinder",
                 radius=radius, length=length, axis=axis, pos=tuple(pos),
                 count=count, note=note)


def tube(name, module, material, radius, inner_radius, length, axis, pos,
         count=1, note=""):
    return Solid(name=name, module=module, material=material, kind="tube",
                 radius=radius, inner_radius=inner_radius, length=length,
                 axis=axis, pos=tuple(pos), count=count, note=note)


def leaf(name, module, material, p0, p1, thickness, width, y=0.0, count=1,
         note=""):
    return Solid(name=name, module=module, material=material, kind="leaf",
                 p0=tuple(p0), p1=tuple(p1), xsec=(thickness, width),
                 pos=(0.0, y, 0.0), count=count, note=note)


# ---------------------------------------------------------------------------
# 布局：三级沿 Z 的模块位置，由 build.py 调用
# ---------------------------------------------------------------------------
def layout(level: str) -> dict:
    """返回该级原型的 Z 向模块区间，单位 mm。

    membrane 区间是两片压环叠起来的总厚；frame_len 由膜片前端再加一个压边得到。
    stages 是各级失稳单元的底边 Z 位置。A 级不含失稳单元与释放卡榫。
    """
    level = level.upper()
    if level == "A":
        lay = dict(
            actuator=(4.0, 14.0),
            pushrod=(15.0, 27.0),
            membrane=(27.0, 30.0),
            stages=[],
            release=None,
        )
    elif level == "B":
        lay = dict(
            release=(3.0, 14.5),
            stages=[15.0],
            output=(25.0, 33.0),
            membrane=(33.0, 36.0),
            actuator=None,
        )
    elif level == "C":
        lay = dict(
            release=(3.0, 14.5),
            stages=[15.0, 27.0],
            output=(37.0, 39.0),
            membrane=(39.0, 42.0),
            actuator=None,
        )
    else:
        raise ValueError(f"A/B/C 之外的级别本轮不做: {level}")
    lay["level"] = level
    lay["rear"] = (0.0, FRAME_REAR_T)
    lay["wall_z0"] = FRAME_REAR_T
    lay["wall_z1"] = lay["membrane"][1]
    lay["frame_len"] = lay["membrane"][1] + FRAME_RIM_T
    return lay


# ---------------------------------------------------------------------------
# 参数来源登记
# ---------------------------------------------------------------------------
PARAM_SOURCES = {
    "FRAME_W": "§1.2 横向 15-25 mm，取 20",
    "FRAME_H": "§1.2 横向 15-25 mm，取 18",
    "FRAME_WALL": "工程估计：0.4 mm 喷嘴 4 圈壁厚 1.6 mm",
    "FRAME_REAR_T": "工程估计：打印后盖 2 mm，兼顾刚度与螺纹深度",
    "FRAME_BOSS_D": "工程估计：M3 螺钉沉台 3.2 mm",
    "STAGE_HALF_SPAN": "对齐 04_SIMULATION/models/params.py A_VMT = 8 mm，§1.2 横向量级",
    "STAGE_BAR_LEN": "对齐 04_SIMULATION/models/params.py L_VMT = 10 mm",
    "STAGE_APEX_H": "几何导出 h0 = sqrt(L^2 - a^2) = 6 mm",
    "STAGE_LEAF_T_PET": "§7.2 PET 弹性片做几何验证，厚度 0.5 mm 为工程估计；待实测标定",
    "STAGE_LEAF_T_STEEL": "§7.2 弹簧钢做寿命版，厚度 0.3 mm 为工程估计；待实测标定",
    "STAGE_LEAF_DEPTH": "工程估计：取 10 mm 保证面外刚度，避免出平面失稳",
    "STAGE_K_BAR": "对齐仿真 K_BAR = 2000 N/m，等效折算式参数；待实测标定",
    "STAGE_K_BASE": "对齐仿真 K_BASE = 1e4 N/m；待实测标定",
    "MEMB_CLAMP_RADIUS": "对齐仿真 radius = 8 mm，§1.2 横向量级",
    "MEMB_RING_OD": "工程估计：夹持半径 8 mm 加 3 mm 压环边，外径 22 mm，落在 §1.2 横向上限 25 mm 内",
    "MEMB_RING_T": "工程估计：FDM 单片压环 1.5 mm",
    "MEMB_DISC_THICKNESS": "§6.2 候选 A 硅胶 0.2 mm、候选 C PET 0.05 mm，对齐仿真；待实测标定",
    "MEMB_SCREW_D": "标准件 M2",
    "REL_SEAR_ANGLE_DEG": "工程估计：自锁条件 tan(alpha) < mu，mu=0.15 时摩擦角 8.5°，取 6° 留余量",
    "REL_FRICTION_MU": "工程估计：钢-钢干摩擦 0.15 量级；待实测标定",
    "REL_SOLENOID_FORCE": "工程估计：常见 5V 微型推拉电磁铁 2-3 N；待实测标定",
    "REL_PIN_D": "工程估计：1.0 mm 钢销，受剪断面约 0.79 mm^2",
    "REL_PIN_STROKE": "工程估计：2 mm 行程足以让卡榫转过自锁角",
    "ACT_BODY_SIZE": "工程估计：低速微型执行器本体量级，A 级只作基线推动",
}

# 自锁余量随角度变化，release.describe() 会打印
SELF_LOCK_FRICTION_ANGLE = math.degrees(math.atan(REL_FRICTION_MU))
