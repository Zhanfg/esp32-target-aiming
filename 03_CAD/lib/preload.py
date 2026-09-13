"""PRELOAD_MODULE：多种预载状态（§5.4、§23.3）。

预载用可换垫片堆设定，垫片总厚就是预载量，从 params.PRELOAD_STATES 取值。垫片
装在释放卡榫座与第一级滑块之间，释放前先由卡榫把滑块压到设定的预载位置。换不同
垫片堆即换一个预载状态，不碰其余模块，符合 §23.3 只扫少数关键变量的要求。

扫描变量列表见 params.PRELOAD_STATES。取值量级是工程估计，需要实测标定，特别
要标定垫片厚度与滑块实际预压位移的对应关系。

垫片是独立可换件，拆一颗 M2 螺钉即可整包抽出（§22.2）。
"""

from __future__ import annotations

from . import params as P


def shim_count(state: str) -> int:
    """给定预载状态需要的垫片片数。"""
    if state not in P.PRELOAD_STATES:
        raise ValueError(f"预载状态只能是 {list(P.PRELOAD_STATES)}: {state}")
    return int(round(P.PRELOAD_STATES[state] / P.PRELOAD_SHIM_T))


def describe(state: str) -> str:
    n = shim_count(state)
    total = P.PRELOAD_STATES[state]
    return (f"预载状态 {state}：垫片 {n} 片，预载 {total:.2f} mm，"
            f"单片刻度 {P.PRELOAD_SHIM_T:.2f} mm（待实测标定）")


def build(z0: float, z1: float, state: str = "P0") -> list:
    n = shim_count(state)
    parts = []
    # 垫片座，固定在框架上
    hx, hy, hz = P.PRELOAD_HOLDER
    parts.append(P.box("PRELOAD_holder", "PRELOAD_MODULE", "petg", (hx, hy, hz),
                       (0.0, 0.0, z0 - 1.0),
                       note="垫片座，固定框架上"))
    # 可换垫片堆，总厚等于预载值
    for i in range(n):
        parts.append(P.box(f"PRELOAD_shim_{i}", "PRELOAD_MODULE", "petg",
                           (P.PRELOAD_SHIM_W, P.PRELOAD_SHIM_H, P.PRELOAD_SHIM_T),
                           (0.0, 0.0, z0 + 0.5 + P.PRELOAD_SHIM_T * (i + 0.5)),
                           note=f"{state} 第 {i + 1}/{n} 片，{P.PRELOAD_SHIM_T:.2f} mm"))
    # 顶块，把垫片堆的位移传给滑块
    px, py, pz = P.PRELOAD_PUSHER
    parts.append(P.box("PRELOAD_pusher", "PRELOAD_MODULE", "petg", (px, py, pz),
                       (0.0, 0.0, z0 + 0.5 + P.PRELOAD_SHIM_T * n + pz / 2.0),
                       note="顶块，把垫片总厚转成滑块预压位移"))
    return parts
