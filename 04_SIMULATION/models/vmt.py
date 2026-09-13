"""von Mises truss（VMT）双稳单元的降阶模型。

几何与模型
----------
两根等长斜杆，自由长度 L，底端半跨 a，顶点初始高度 h0 = sqrt(L**2 - a**2)。
底端可以在水平方向滑动，用一个横向边界弹簧 k_base 约束；两根杆各用等效轴向
刚度 k_bar 的线性弹簧表示。顶点竖向位移 h，底端横向张开量 u。

把势能写成

    U(h, u) = k_bar * (ell - L)**2 + k_base * u**2
    ell = sqrt((a + u)**2 + h**2)

（两项分别来自两根杆和两个底端弹簧，系数已合并）。对给定的 h，先由
dU/du = 0 解出 u，得到平衡路径；顶点力用包络定理

    F(h) = dU/dh = 2 * k_bar * (ell - L) * h / ell

k_base 很大时底端锁死，退化为经典的固定底 VMT；k_base 有限时底端可滑动，
snap 力与 post-snap 行为随边界刚度变化。这就是规格书 §5.4 与 SRC-04 里
讨论的边界状态，在本模型里是 k_base 的取值。

链元件形式
----------
串联链里把 VMT 当成一个非线性弹簧，自变量取压缩量 c（c 为正表示两节点靠近）。
元件对外推力 element_force(c) = -F(h0 - c)，在 c=0 处为零，先升到峰值
snap_force，再降到 c=2*h0 处回到零，之后变负，对应另一个稳定构型。

数值实现
--------
U(h, u) 对 u 的驻点在预计算网格上用向量化二分求解。给出 c 网格上的
element_force / element_energy / element_stiffness 表，运行时用线性插值，
避免在每个积分步里重复解方程。插值引入的误差由网格密度控制，网格默认
4001 点，覆盖 c ∈ [-0.5*h0, 3*h0]。

假设与局限
----------
- 杆为线弹性，忽略杆的弯曲与屈曲、节点摩擦、间隙与塑性。
- 质量集中在顶点，杆自身分布质量忽略。
- 底端弹簧线性，忽略支撑的间隙、预压与非线性。
- 等效轴向刚度 k_bar 把真实弹性片的弯曲柔度折算成杆的轴向刚度，是等效参数，
  需要实验标定；名义值见 params.py 的说明。
- 二维对称运动假设，不考虑面外失稳与不对称分岔。
"""

from __future__ import annotations

import math

import numpy as np


class VonMisesTruss:
    """对称 von Mises truss 双稳单元的降阶模型。"""

    def __init__(self, half_span, bar_length, k_bar, k_base, grid=4001):
        if bar_length <= half_span:
            raise ValueError("bar_length 必须大于 half_span，否则不是浅拱")
        if k_bar <= 0 or k_base <= 0:
            raise ValueError("k_bar 与 k_base 必须为正")
        self.a = float(half_span)
        self.L = float(bar_length)
        self.k_bar = float(k_bar)
        self.k_base = float(k_base)
        self.h0 = math.sqrt(self.L**2 - self.a**2)
        self._build_tables(grid)

    # ------------------------------------------------------------------
    # 预计算
    # ------------------------------------------------------------------
    def _base_offset(self, h):
        """给定顶点高度 h，解 dU/du = 0 得到底端张开量 u。"""
        a, L, kb, ks = self.a, self.L, self.k_bar, self.k_base

        def residual(u):
            ell = math.hypot(a + u, h)
            return kb * (ell - L) * (a + u) / ell + ks * u

        lo = -a * 0.999
        hi = max(4.0 * L, 4.0 * a)
        f_lo = residual(lo)
        if f_lo > 0:
            # 极端情形，向外扩一点
            lo = -a
            f_lo = residual(lo)
        f_hi = residual(hi)
        expand = 0
        while f_hi < 0 and expand < 40:
            hi *= 1.5
            f_hi = residual(hi)
            expand += 1
        if f_lo * f_hi > 0:
            # 找不到符号变化，退化为网格扫描取最小势能点
            us = np.linspace(lo, hi, 2001)
            vals = [
                kb * (math.hypot(a + u, h) - L) ** 2 + ks * u**2 for u in us
            ]
            i = int(np.argmin(vals))
            return float(us[i])
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if residual(lo) * residual(mid) <= 0:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    def _energy_at(self, h, u):
        ell = math.hypot(self.a + u, h)
        return self.k_bar * (ell - self.L) ** 2 + self.k_base * u**2

    def _apex_force(self, h, u):
        ell = math.hypot(self.a + u, h)
        return 2.0 * self.k_bar * (ell - self.L) * h / ell

    def _build_tables(self, grid):
        c = np.linspace(-0.5 * self.h0, 3.0 * self.h0, grid)
        h = self.h0 - c
        u = np.empty_like(h)
        for i, hi in enumerate(h):
            u[i] = self._base_offset(float(hi))
        ell = np.hypot(self.a + u, h)
        force = -2.0 * self.k_bar * (ell - self.L) * h / ell
        # c=0 对应 h=h0、u=0，势能本来就为零，不需要再平移
        energy = self.k_bar * (ell - self.L) ** 2 + self.k_base * u**2
        self._c_grid = c
        self._force_grid = force
        self._energy_grid = energy
        self._stiffness_grid = np.gradient(force, c)
        self._force_slope_grid = self._stiffness_grid
        # 正压缩段的第一个峰值，即 snap 点。取 c ∈ [0, h0]，超过 h0 后
        # 进入另一个构型再被拉伸，力的上升不属于 snap 峰值。
        window = (c >= 0.0) & (c <= self.h0)
        idx = int(np.argmax(np.where(window, force, -np.inf)))
        self.snap_stroke = float(c[idx])
        self.snap_force = float(force[idx])
        # 峰值之后 force 再次过零，即第二个无应力构型
        self.second_well_stroke = 2.0 * self.h0
        for i in range(idx, len(c) - 1):
            if force[i] > 0.0 >= force[i + 1]:
                span = force[i] - force[i + 1]
                frac = force[i] / span if span != 0 else 0.0
                self.second_well_stroke = float(c[i] + frac * (c[i + 1] - c[i]))
                break

    # ------------------------------------------------------------------
    # 元件接口（自变量为压缩量 c，c = -扩展）
    # ------------------------------------------------------------------
    def _hermite(self, x, fp, fp1, derivative=False):
        """三次 Hermite 插值（可求导），端点外按端点值/斜率线性延伸。

        用斜率连续的三次插值代替 np.interp 的分段线性，避免力表的折点在高
        速运动时向系统注入能量。
        """
        xp = self._c_grid
        n = xp.size
        xc = np.clip(x, xp[0], xp[-1])
        i = np.searchsorted(xp, xc, side="right") - 1
        i = np.clip(i, 0, n - 2)
        dx = xp[i + 1] - xp[i]
        t = (xc - xp[i]) / dx
        if not derivative:
            h00 = 2 * t**3 - 3 * t**2 + 1
            h10 = t**3 - 2 * t**2 + t
            h01 = -2 * t**3 + 3 * t**2
            h11 = t**3 - t**2
            return (
                h00 * fp[i]
                + h10 * dx * fp1[i]
                + h01 * fp[i + 1]
                + h11 * dx * fp1[i + 1]
            )
        dh00 = 6 * t**2 - 6 * t
        dh10 = 3 * t**2 - 4 * t + 1
        dh01 = -6 * t**2 + 6 * t
        dh11 = 3 * t**2 - 2 * t
        return (
            dh00 * fp[i]
            + dh10 * dx * fp1[i]
            + dh01 * fp[i + 1]
            + dh11 * dx * fp1[i + 1]
        ) / dx

    def element_force(self, c):
        return self._hermite(c, self._force_grid, self._force_slope_grid)

    def element_energy(self, c):
        # 能量表的斜率是力，不是刚度
        return self._hermite(c, self._energy_grid, self._force_grid, derivative=False)

    def element_stiffness(self, c):
        return self._hermite(
            c, self._force_grid, self._force_slope_grid, derivative=True
        )

    def describe(self):
        return (
            f"VMT(a={self.a*1e3:.2f} mm, L={self.L*1e3:.2f} mm, "
            f"h0={self.h0*1e3:.2f} mm, k_bar={self.k_bar:.4g} N/m, "
            f"k_base={self.k_base:.4g} N/m) snap_force={self.snap_force:.4g} N "
            f"@ stroke={self.snap_stroke*1e3:.3f} mm"
        )
