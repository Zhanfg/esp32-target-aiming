"""MST-01 数值仿真模型包。

- membrane : 预张紧圆膜的中心力—挠度非线性模型
- vmt      : von Mises truss 双稳单元降阶模型
- chain    : 一维串联链与元件装配
- solver   : 准静态求解与速度 Verlet 瞬态积分
- params   : 名义参数、出处与 A/B/C 原型装配
"""

from .chain import BASE, GROUND, BistableUnit, Chain, LinearSpring, MembraneUnit
from .membrane import Membrane
from .params import (
    build_A,
    build_B,
    build_C,
    make_membrane,
    make_vmt,
)
from .vmt import VonMisesTruss

__all__ = [
    "Membrane",
    "VonMisesTruss",
    "Chain",
    "LinearSpring",
    "BistableUnit",
    "MembraneUnit",
    "BASE",
    "GROUND",
    "build_A",
    "build_B",
    "build_C",
    "make_membrane",
    "make_vmt",
]
