"""名义参数集、出处与原型装配。

所有数值要么引用规格书章节，要么给出物理估计依据。没有来源的用 待实测标定 标注。
单位：m, N, kg, s。

膜片
----
候选 A 薄硅胶膜与候选 C PET 膜，对应规格书 §6.2。两者的弹性模量是文献常见
量级估计，实际片材批次差异大，必须实测标定。夹持半径 8 mm 取自 §1.2 的
横向 15–25 mm 量级。

VMT 双稳单元
------------
取浅拱几何：半跨 8 mm、杆长 10 mm，初始顶点高度约 6 mm，落在 §1.2 的核心
模块 15–25 mm 横向与 30–50 mm 长度之内。第一峰值（snap 点）出现在压缩量约
2.5 mm 处，属于 mm 级作动行程可达的范围。

等效轴向刚度 k_bar 把弹性片弯曲柔度折算到杆轴向，是等效参数，不是材料拉伸
刚度。名义 2000 N/m 是让 snap 力落在与膜片回复力可比的 0.3–1 N 量级，
待实测标定。边界刚度 k_base 名义 500 N/m，作为扫描变量。

质量
----
顶点运动质量取 0.5 g，零件级量级估计；膜片质量由 membrane 模型给出。实际值
依赖 CAD，待实测标定。

阻尼
----
每元件线粘性系数名义 0.01 N·s/m，约为单元临界阻尼的 1%。机械阻尼扫描时按
比例放大。真实来源是材料滞后与接触摩擦，本模型用等效线粘性代替。

作动
----
把基座位移直接作用到第一个节点，等价于刚性作动器。作动器自身的柔度不在模型
内；若需要，可把 k_act 作为第一段串联弹簧加入，扫描边界刚度时另行处理。
"""

from __future__ import annotations

from .chain import BASE, GROUND, BistableUnit, Chain, MembraneUnit
from .membrane import Membrane
from .vmt import VonMisesTruss

# ---------------------------------------------------------------------------
# 膜片材料
# ---------------------------------------------------------------------------
# 薄硅胶膜：E 取 1 MPa，硬度计 30A 左右的常见量级估计；t=0.2 mm；prestrain=5%
# 是手工张紧安装的典型预应变。# 待实测标定
SILICONE = dict(
    E=1.0e6, thickness=0.2e-3, radius=8.0e-3, nu=0.49, prestrain=0.05,
    density=1100.0,
)
# PET/Mylar 膜：E 取 3 GPa，PET 常见范围 2–4 GPa 的量级估计；t=50 um；
# prestrain=0.2%，聚酯膜安装预应变小于硅胶。# 待实测标定
PET = dict(
    E=3.0e9, thickness=50e-6, radius=8.0e-3, nu=0.35, prestrain=0.002,
    density=1380.0,
)

MATERIALS = {"silicone": SILICONE, "pet": PET}

# ---------------------------------------------------------------------------
# VMT 几何与刚度
# ---------------------------------------------------------------------------
A_VMT = 8.0e-3          # 半跨，§1.2 量级
L_VMT = 10.0e-3         # 杆长
K_BAR = 2000.0          # 等效轴向刚度，# 待实测标定
K_BASE = 1.0e4          # 边界刚度，扫描变量。名义值接近锁定支撑，保证单级能 snap
GAP_C = 0.6e-3          # 两级串联的 engagement gap，量级估计

# 质量与阻尼
M_ACT = 5.0e-4          # 作动侧运动质量，量级估计
M_STAGE = 5.0e-4        # 单级顶点运动质量，量级估计
C_DAMP = 0.01           # 每元件线粘性，约为单元临界阻尼 1%


def make_membrane(material="silicone", stiffness_scale=1.0):
    return Membrane(**MATERIALS[material], stiffness_scale=stiffness_scale)


def make_vmt(k_bar=K_BAR, k_base=K_BASE):
    return VonMisesTruss(A_VMT, L_VMT, k_bar, k_base)


# ---------------------------------------------------------------------------
# 原型装配
# ---------------------------------------------------------------------------
# 作动方式统一为力控：外部缓慢增大的力作用在节点0（作动侧），基座固定。
# 这样作动器有明确的力上限，snap 后不会像刚性位移驱动那样持续注入能量。
# 位移输入由求解出的节点位移给出。


def build_A(membrane, damping=C_DAMP):
    """普通膜片基线：作动力直接作用在膜片中心，无 snap。"""
    m0 = membrane.mass
    elements = [(0, GROUND, MembraneUnit(membrane))]
    return Chain([m0], elements, damping=damping, name="A")


def build_B(membrane, vmt, preload=0.0, damping=C_DAMP, m_act=M_ACT,
            m_stage=M_STAGE):
    """单级 snap：作动侧经 VMT1 推膜片。"""
    m0 = m_act
    m1 = m_stage + membrane.mass
    elements = [
        (0, 1, BistableUnit(vmt, gap=0.0, preload=preload)),
        (1, GROUND, MembraneUnit(membrane)),
    ]
    return Chain([m0, m1], elements, damping=damping, name="B")


def build_C(membrane, vmt1, vmt2, gap=GAP_C, preload1=0.0, preload2=0.0,
            damping=C_DAMP, m_act=M_ACT, m_stage=M_STAGE):
    """双级串联 snap：作动侧经 VMT1、engagement gap、VMT2 推膜片。"""
    m0 = m_act
    m1 = m_stage
    m2 = m_stage + membrane.mass
    elements = [
        (0, 1, BistableUnit(vmt1, gap=0.0, preload=preload1)),
        (1, 2, BistableUnit(vmt2, gap=gap, preload=preload2)),
        (2, GROUND, MembraneUnit(membrane)),
    ]
    return Chain([m0, m1, m2], elements, damping=damping, name="C")


def build_B_static(membrane, vmt):
    """位移控准静态用：作动节点作为固定基座，输出节点接膜片。"""
    m = M_STAGE + membrane.mass
    elements = [
        (BASE, 0, BistableUnit(vmt)),
        (0, GROUND, MembraneUnit(membrane)),
    ]
    return Chain([m], elements, name="B_static")


def build_C_static(membrane, vmt1, vmt2, gap=GAP_C):
    """位移控准静态用：基座经两级双稳元件接膜片。"""
    m0 = M_STAGE
    m1 = M_STAGE + membrane.mass
    elements = [
        (BASE, 0, BistableUnit(vmt1)),
        (0, 1, BistableUnit(vmt2, gap=gap)),
        (1, GROUND, MembraneUnit(membrane)),
    ]
    return Chain([m0, m1], elements, name="C_static")
