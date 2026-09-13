"""膜片的非线性力—位移模型。

模型来源
--------
把膜片当成预先张紧的圆形薄膜，夹持边界固定，中心被刚性推杆顶出。用 Föppl
大挠度薄膜理论的单模态近似，取挠曲形状

    w(r) = w0 * (1 - (r / a)**2)

a 是夹持半径，w0 是中心挠度。在这个形状下解轴对称 Föppl 方程的面内径向位移，
得到膜内应力场，再积分应变能，对 w0 求导得到中心力：

    F(w) = 2 * pi * N0 * w  +  [ pi * E * t * (7 - nu) / (6 * (1 - nu) * a**2) ] * w**3

线性项来自预张力 N0（单位 N/m），三次项来自几何大挠度引起的膜内拉伸。
预张力按双轴等拉伸取 N0 = E * t * eps0 / (1 - nu)。

推导要点（供核对）：设无量纲 rho = r/a，把 w 代入面内平衡方程
r**2 * N_r'' + 3 * r * N_r' = -(E*t/2) * (dw/dr)**2，解出 N_r、N_theta，
代入 r=a 处径向位移为零，得常数 C1 = E*t*(3-nu)*w0**2 / (4*a**2*(1-nu))。
应变能 U_stretch = (1/2) * ∫ (N_r * eps_r + N_theta * eps_theta) dA
化简为 pi * E * t * (7 - nu) / (24 * (1 - nu) * a**2) * w0**4。预张力项
U_pre = pi * N0 * w0**2。

假设
----
- 轴对称，膜只做横向变形，忽略弯曲刚度（薄膜假设，挠度远大于厚度）。
- 挠曲形状用单个抛物线模态，给出正确的函数形式（线性加三次硬化），系数是
  量级正确而非精确解。中心力理想化要求推杆半径远小于 a。
- 预张力与几何拉伸解耦，忽略预应变对二阶面内场的修正。
- 线弹性、小应变。粘弹滞后、气压反作用、湿度与温度效应都不在模型里。
- 有效质量按同一模态取 m_eff = rho * t * pi * a**2 / 6，用于瞬态。

适用范围
--------
中心挠度与 a 同量级、且远大于厚度 t 时最可靠。w0 << t 时退化为线性膜。
超过材料屈服应变后不适用。

已知局限
--------
- 单模态近似在挠度接近 a 时低估峰值力，误差随 w0/a 增大。
- 硅胶类材料有显著滞后，本模型按线弹性处理，会高估回弹能量。
- 膜片边缘的实际夹持并非完全固支，压环会有微量滑移。
"""

from __future__ import annotations

import math


class Membrane:
    """预张紧圆膜的中心力—挠度模型。

    参数
    ----
    E : float
        弹性模量，Pa。
    thickness : float
        膜厚，m。
    radius : float
        夹持半径，m。
    nu : float
        泊松比。
    prestrain : float
        双轴安装预应变，无量纲。
    density : float
        密度，kg/m**3。
    stiffness_scale : float
        整体刚度缩放因子，用于柔度扫描。同时缩放 E 与预张力对应的等效模量。
        默认 1.0。
    """

    def __init__(
        self,
        E,
        thickness,
        radius,
        nu=0.49,
        prestrain=0.05,
        density=1100.0,
        stiffness_scale=1.0,
    ):
        if E <= 0 or thickness <= 0 or radius <= 0:
            raise ValueError("E、thickness、radius 必须为正")
        if not (0.0 <= nu < 0.5):
            raise ValueError("nu 需在 [0, 0.5) 内")
        if prestrain < 0:
            raise ValueError("prestrain 不能为负")
        self.E = E
        self.thickness = thickness
        self.radius = radius
        self.nu = nu
        self.prestrain = prestrain
        self.density = density
        self.stiffness_scale = stiffness_scale

        E_eff = E * stiffness_scale
        # 双轴等拉伸的膜内力，plane stress。
        self.N0 = E_eff * thickness * prestrain / (1.0 - nu)
        self.k1 = 2.0 * math.pi * self.N0
        self.k3 = math.pi * E_eff * thickness * (7.0 - nu) / (
            6.0 * (1.0 - nu) * radius**2
        )
        # 抛物线模态下的等效质量。
        self.mass = density * thickness * math.pi * radius**2 / 6.0
        # 抛物线模态下的体积位移系数 V = coeff * w0。
        self.volume_coeff = math.pi * radius**2 / 3.0

    def force(self, w):
        """中心力，N。w 以中心挠度计，符号约定沿推杆正方向。"""
        return self.k1 * w + self.k3 * w**3

    def energy(self, w):
        """弹性势能，J。对 w 为偶函数，膜对两个方向都有回复。"""
        return 0.5 * self.k1 * w**2 + 0.25 * self.k3 * w**4

    def stiffness(self, w):
        """切线刚度 dF/dw，N/m。"""
        return self.k1 + 3.0 * self.k3 * w**2

    def volume_displacement(self, w):
        """膜片顶出造成的空气体积位移，m**3。"""
        return self.volume_coeff * w

    def small_deflection_stiffness(self):
        """小挠度极限刚度，N/m。等于 k1。"""
        return self.k1

    def describe(self):
        return (
            f"Membrane(E={self.E:.3g} Pa, t={self.thickness*1e6:.1f} um, "
            f"a={self.radius*1e3:.1f} mm, nu={self.nu:.2f}, "
            f"prestrain={self.prestrain*1e3:.2f} me, scale={self.stiffness_scale:.3g}) "
            f"N0={self.N0:.4g} N/m, k1={self.k1:.4g} N/m, "
            f"k3={self.k3:.4g} N/m^3, m_eff={self.mass*1e6:.2f} mg"
        )
