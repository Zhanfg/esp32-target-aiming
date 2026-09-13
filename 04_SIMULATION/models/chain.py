"""串联链的元件与装配。

把原型 A/B/C 都写成同一条一维串联链，节点之间放非线性弹簧元件：

    基座 --[作动弹簧 k_act]-- 节点0 --[元件1]-- 节点1 --[元件2]-- ... --[膜片]-- 地

节点编号从 0 开始，地节点固定为零位移，基座位移由外部给定。每个元件只依赖
两端相对位移 s = x_j - x_i，提供势能 W(s) 和广义力 g(s)，约定 g(s) 是作用在
节点 j 上、沿 +x 方向的力。对线性弹簧 W = k*s**2/2，g = -k*s（拉伸时把 j 拉回）。

元件类型
--------
LinearSpring   线弹性，用于作动/边界弹簧。
BistableUnit   VMT 双稳元件，可带 engagement gap 与常值预载。
MembraneUnit   膜片负载，势能直接用 membrane.energy(s)。

数值约定
--------
g(s) = -dW/ds。静态残差 R_j += g，R_i -= g。牛顿法的雅可比由各元件切线
刚度拼装。装配规模很小（节点数不超过 3），直接稠密求解。
"""

from __future__ import annotations

import numpy as np

BASE = -1
GROUND = -2


class LinearSpring:
    def __init__(self, k):
        if k <= 0:
            raise ValueError("k 必须为正")
        self.k = float(k)

    def potential(self, s):
        return 0.5 * self.k * s * s

    def force(self, s):
        return -self.k * s

    def stiffness(self, s):
        return -self.k


class BistableUnit:
    """VMT 双稳元件。

    压缩量 c = -s - gap，只有 c > 0 才参与受力，用来表示单元之间的 engagement
    gap。preload 是一个常值压缩偏置力，preload > 0 会降低等效 snap 阈值。
    """

    def __init__(self, vmt, gap=0.0, preload=0.0):
        self.vmt = vmt
        self.gap = float(gap)
        self.preload = float(preload)

    def _compression(self, s):
        return -s - self.gap

    def _active(self, c):
        # gap==0 的单元始终参与受力（拉伸段走 VMT 的张力分支）；
        # gap>0 的单元在间隙闭合前完全不受力。
        return c > 0.0 or self.gap == 0.0

    def potential(self, s):
        c = self._compression(s)
        if not self._active(c):
            return 0.0
        return float(self.vmt.element_energy(c)) - self.preload * c

    def force(self, s):
        c = self._compression(s)
        if not self._active(c):
            return 0.0
        return float(self.vmt.element_force(c)) - self.preload

    def stiffness(self, s):
        c = self._compression(s)
        if not self._active(c):
            return 0.0
        return -float(self.vmt.element_stiffness(c))

    def engaged(self, s):
        return self._active(self._compression(s))


class MembraneUnit:
    def __init__(self, membrane):
        self.membrane = membrane

    def potential(self, s):
        return self.membrane.energy(s)

    def force(self, s):
        return -self.membrane.force(s)

    def stiffness(self, s):
        return -self.membrane.stiffness(s)


class Chain:
    """一维串联链。

    参数
    ----
    masses : sequence of float
        动态节点质量，长度即节点数 n。节点索引 0..n-1。
    elements : sequence of (int, int, element)
        每个三元组给出左右节点编号与元件。节点编号可以是 0..n-1、BASE(-1)
        或 GROUND(-2)。约定 s = x_j - x_i。
    damping : float or sequence
        每个元件的线粘性系数 c，力为 c*(v_j - v_i)。标量表示所有元件同值。
    name : str
        标识。
    """

    def __init__(self, masses, elements, damping=0.0, name=""):
        self.masses = np.asarray(masses, dtype=float)
        self.n = len(self.masses)
        self.elements = list(elements)
        if np.any(self.masses <= 0):
            raise ValueError("质量必须为正")
        if np.isscalar(damping):
            self.damping = [float(damping)] * len(self.elements)
        else:
            d = list(damping)
            if len(d) != len(self.elements):
                raise ValueError("damping 长度需与 elements 相同")
            self.damping = [float(x) for x in d]
        self.name = name

    # ------------------------------------------------------------------
    # 编号与位移查询
    # ------------------------------------------------------------------
    def _pos(self, node, x, base_pos):
        if node == BASE:
            return base_pos
        if node == GROUND:
            return 0.0
        return x[node]

    def _vel(self, node, v, base_vel):
        if node == BASE:
            return base_vel
        if node == GROUND:
            return 0.0
        return v[node]

    # ------------------------------------------------------------------
    # 动力学
    # ------------------------------------------------------------------
    def accelerations(self, x, v, base_pos, base_vel, ext_force=None):
        a = np.zeros(self.n)
        for (i, j, elem), c in zip(self.elements, self.damping):
            s = self._pos(j, x, base_pos) - self._pos(i, x, base_pos)
            f = elem.force(s)
            if c:
                # 阻尼力作用在 j 上，方向与相对速度相反
                f += c * (self._vel(i, v, base_vel) - self._vel(j, v, base_vel))
            if 0 <= j < self.n:
                a[j] += f / self.masses[j]
            if 0 <= i < self.n:
                a[i] -= f / self.masses[i]
        if ext_force is not None:
            a = a + np.asarray(ext_force, dtype=float) / self.masses
        return a

    def energy(self, x, v, base_pos, base_vel):
        total = 0.0
        for (i, j, elem) in self.elements:
            s = self._pos(j, x, base_pos) - self._pos(i, x, base_pos)
            total += elem.potential(s)
        total += 0.5 * float(np.sum(self.masses * v * v))
        return total

    # ------------------------------------------------------------------
    # 静态
    # ------------------------------------------------------------------
    def residual_jacobian(self, x, base_pos):
        r = np.zeros(self.n)
        jac = np.zeros((self.n, self.n))
        for (i, j, elem) in self.elements:
            xi = self._pos(i, x, base_pos)
            xj = self._pos(j, x, base_pos)
            s = xj - xi
            f = elem.force(s)
            k = elem.stiffness(s)
            if 0 <= j < self.n:
                r[j] += f
            if 0 <= i < self.n:
                r[i] -= f
            # dR_j/dx_j += k, dR_j/dx_i -= k; 对 i 类似
            if 0 <= j < self.n:
                jac[j, j] += k
                if 0 <= i < self.n:
                    jac[j, i] -= k
            if 0 <= i < self.n:
                jac[i, i] += k
                if 0 <= j < self.n:
                    jac[i, j] -= k
        return r, jac

    def input_force(self, x, base_pos):
        """基座施加在节点0上的力，N。取第一个与 BASE 相连的元件。

        这个量就是准静态实验里作动器需要提供的力。正号表示沿 +x。
        """
        for (i, j, elem) in self.elements:
            if i == BASE:
                s = self._pos(j, x, base_pos) - base_pos
                return float(elem.force(s))
        raise ValueError("链中没有与 BASE 相连的元件")
