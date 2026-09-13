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
# 核心链布局常数：D / E 级的单级与双级共用同一套几何。
# 双级底边间距沿用 C 级的 12 mm，保证与 B/C 的对照是同一条轴向链。
# ---------------------------------------------------------------------------
STAGE_Z_SPACING = 12.0        # 两级底边 Z 向间距，与 C 级一致
OUTPUT_LEN = 2.0              # 顶点滑块到膜片的传力杆长度
MEMB_STACK_T = 2.0 * MEMB_RING_T   # 两片压环叠厚

# ---------------------------------------------------------------------------
# BOUNDARY_MODULE：可机械切换的边界状态（§5.4、§13 D 级、§16 Test D1）
# 底座托架沿 Z 浮动，两根 Y 向锁销插入托架即 locked，拔出即 released。
# 两种状态的边界刚度不同，切换可逆、有机械止挡与定位珠，拨杆露出框架顶部，
# 高速视频能看到状态与顶点运动。
# ---------------------------------------------------------------------------
BND_CARRIAGE = (16.0, 10.0, 4.0)      # 浮动托架 x/y/z
BND_GUIDE_POST_D = 2.0                # Y 向导柱直径
BND_GUIDE_POST_L = 12.0
BND_LOCK_PIN_D = 1.6                  # Y 向锁销直径
BND_LOCK_PIN_L = 6.0                  # 锁销短，抬起即脱离托架
BND_LEVER = (16.0, 4.0, 2.0)          # 拨杆板，两销连成一体
BND_DETENT_D = 3.0                    # 定位珠/柱塞量级
BND_DETENT_L = 4.0
BND_STOP_BLOCK = (4.0, 8.0, 3.0)      # released 行程机械止挡
BND_RETURN_SPRING_D = 4.0             # 边界回复弹簧（低刚度支路）
BND_RETURN_SPRING_L = 5.0
BND_STATES = ("locked", "released")
# released 状态放开托架的 Z 向行程上限，由止挡块确定，保证可复现
BND_TRAVEL = 1.0
# 两种边界的等效边界刚度 N/m：locked 走刚性支路，released 走弹簧支路
BND_K_BASE = {"locked": STAGE_K_BASE, "released": STAGE_K_BASE / 10.0}
BND_PIN_Y = {"locked": 0.0, "released": 8.0}     # 锁销中心 Y，抬起后脱开托架
BND_LEVER_Y = {"locked": 6.0, "released": 9.0}   # 拨杆中心 Y，状态指示

# ---------------------------------------------------------------------------
# PRELOAD_MODULE：多种预载状态（§5.4、§23.3 预载为优先扫描变量）
# 预载用可换垫片设定托架的初始 Z 向压入量，垫片总厚即预载值。
# ---------------------------------------------------------------------------
PRELOAD_STATES = {
    "P0": 0.0,     # 无预载，作为对照
    "P1": 0.5,
    "P2": 1.0,
    "P3": 1.5,
}
PRELOAD_HOLDER = (14.0, 10.0, 3.0)
PRELOAD_SHIM_W = 8.0
PRELOAD_SHIM_H = 10.0
PRELOAD_SHIM_T = 0.5          # 单片垫片厚，状态值按此厚度取整
PRELOAD_PUSHER = (8.0, 10.0, 2.0)

# ---------------------------------------------------------------------------
# MAGAZINE_MODULE：6 / 8 位旋转供给盘（§8、§9、§13 E 级）
# 载荷为超轻软圆片（§8.2），每工位一个 pocket。pocket 做成环形座加中心通孔，
# 硬珠会从中心孔漏下，薄片才能平铺跨住座圈；再配固定间隙的盖板做厚度防呆。
# ---------------------------------------------------------------------------
MAG_POSITIONS = (6, 8)
MAG_PAYLOAD_D = 14.0          # 软圆片直径，略小于 16 mm 通径
MAG_PAYLOAD_T = 2.0           # 软圆片厚度，工程估计
MAG_POCKET_HOLE_D = 8.0       # pocket 中心通孔，硬珠漏下
MAG_POCKET_SEAT_OD = 16.0     # 座圈外径
MAG_POCKET_GAP = 2.0          # 相邻座圈最小间隙
MAG_POCKET_LIP_H = 1.0        # 座圈凸出高度
MAG_PLATE_T = 3.0             # 盘体厚度
MAG_COVER_T = 2.0             # 索引盖板厚度
MAG_HUB_R = 5.0               # 轮毂外半径
MAG_AXLE_D = 6.0              # 中心轴直径
MAG_RIM = 3.0                 # 盘体到座圈外缘的边距
MAG_CLEAR = 0.5               # 盖板与软圆片顶面的固定间隙
MAG_T = MAG_PLATE_T + MAG_POCKET_LIP_H + MAG_PAYLOAD_T + MAG_COVER_T + MAG_CLEAR
MAG_INDEX_SPRING_D = 3.0
MAG_INDEX_SPRING_L = 6.0
MAG_PAWL = (4.0, 6.0, 3.0)    # 索引棘爪
MAG_STOP_POST = (3.0, 3.0, 5.0)
MAG_ZERO_FLAG = (3.0, 6.0, 2.0)   # 零位标记，随盘转动的遮光片

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
    "epp":      {"density": 40.0, "role": "§8.1 超轻软圆片载荷（泡沫量级）；待实测标定"},
}

# 弹性片材料到厚度的映射
LEAF_THICKNESS = {"pet": STAGE_LEAF_T_PET, "steel301": STAGE_LEAF_T_STEEL}


def mag_pitch_radius(positions: int) -> float:
    """供给盘工位分布圆半径，mm。

    相邻座圈的最小弦距要容下座圈外径加间隙：2*R*sin(pi/N) >= OD + gap，
    取等号即最小分布圆，位越多 R 越大，这是 8 位盘变大的几何来源（§31）。
    """
    return (MAG_POCKET_SEAT_OD + MAG_POCKET_GAP) / (
        2.0 * math.sin(math.pi / positions))


def mag_disc_outer_r(positions: int) -> float:
    """供给盘盘体半径，分布圆半径加座圈半径再加边距。"""
    return mag_pitch_radius(positions) + MAG_POCKET_SEAT_OD / 2.0 + MAG_RIM


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
def _core_chain(z0: float, core: str):
    """从第一级底边 z0 起算核心链，返回 stages / output / membrane 区间。

    single 对应 B 级那种一个失稳单元，dual 对应 C 级那种两个串联。D 级与 E 级
    都按 core 参数调用，边界研究与供给研究都不绑定在级数上（统一架构 §3.2 的
    模型预警：双级相对单级增益有限，可能回退单级）。
    """
    core = core.lower()
    if core == "single":
        stages = [z0]
    elif core == "dual":
        stages = [z0, z0 + STAGE_Z_SPACING]
    else:
        raise ValueError(f"core 只能是 single 或 dual: {core}")
    apex_top = stages[-1] + STAGE_APEX_H + STAGE_APEX_BLOCK[2]
    output = (apex_top, apex_top + OUTPUT_LEN)
    membrane = (output[1], output[1] + MEMB_STACK_T)
    return stages, output, membrane


def layout(level: str, core: str | None = None) -> dict:
    """返回该级原型的 Z 向模块区间，单位 mm。

    membrane 区间是两片压环叠起来的总厚；frame_len 由膜片前端再加一个压边得到。
    stages 是各级失稳单元的底边 Z 位置。A 级不含失稳单元与释放卡榫。
    D / E 级额外给出 boundary、preload、magazine 区间，并用 core 选单级或双级。
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
    elif level in ("D", "E"):
        core = (core or "single").lower()
        release = (3.0, 14.5)
        boundary = (15.0, 19.0)
        preload = (15.0, 19.0)
        stages, output, membrane = _core_chain(boundary[1], core)
        lay = dict(
            release=release,
            boundary=boundary,
            preload=preload,
            stages=stages,
            output=output,
            membrane=membrane,
            actuator=None,
            core=core,
        )
        frame_len = membrane[1] + FRAME_RIM_T
        if level == "E":
            lay["magazine"] = (frame_len, frame_len + MAG_T)
        else:
            lay["magazine"] = None
    else:
        raise ValueError(f"A/B/C/D/E 之外的级别本轮不做: {level}")
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
    "BND_CARRIAGE": "工程估计：容纳两级底夹块，x 取框架内宽减 1 mm 余量",
    "BND_LOCK_PIN_D": "工程估计：1.6 mm 钢销，受剪断面约 2.0 mm^2",
    "BND_TRAVEL": "工程估计：released 边界 Z 向行程 1 mm，由止挡块硬限位；待实测标定",
    "BND_PIN_Y": "工程估计：locked 时销插入托架中心，released 时抬出托架外",
    "STAGE_Z_SPACING": "沿用 C 级两级底边间距 12 mm，保证 D/E 与 C 的级序对照同链",
    "PRELOAD_STATES": "§23.3 预载为优先扫描变量；0.5 mm 步距为工程估计，待实测标定",
    "PRELOAD_SHIM_T": "工程估计：0.5 mm 可换垫片，PETG 打印或冲片",
    "MAG_POSITIONS": "§9.1 6-8 位旋转供给盘，两种布局都要能算；§31 8 位变大则先 6 位",
    "MAG_PAYLOAD_D": "§8.1 EVA/EPP 软圆片，直径取 14 mm，略小于 16 mm 通径；待实测标定",
    "MAG_PAYLOAD_T": "§8.1 软圆片厚度 2 mm 为工程估计；待实测标定",
    "MAG_POCKET_HOLE_D": "§8.4 中心通孔 8 mm，硬珠（BB 4.5-6 mm）会漏下，薄片才能跨住",
    "MAG_POCKET_SEAT_OD": "工程估计：座圈外径 16 mm，对应通径量级",
    "MAG_POCKET_GAP": "工程估计：相邻座圈留 2 mm，避免 8 位盘相邻干涉",
    "MAG_CLEAR": "§8.4 盖板与软圆片顶面留 0.5 mm 固定间隙，超厚硬载荷会顶住盖板卡索引",
    "MAG_ZERO_FLAG": "§9.5 零位标记，随盘转动的遮光片，回零由 MCU 读取",
}

# 自锁余量随角度变化，release.describe() 会打印
SELF_LOCK_FRICTION_ANGLE = math.degrees(math.atan(REL_FRICTION_MU))
