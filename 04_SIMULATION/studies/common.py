"""三个 study 共用的运行、事件检测与结果输出工具。

作动统一为力控：外部力先按 smoothstep 在 tramp 内升到目标值，然后保持。
力上限明确，snap 之后作动器不会像刚性位移驱动那样无限做功。
"""

from __future__ import annotations

import csv
import math
import os

import numpy as np

from models.solver import integrate

RESULTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")


def results_path(name):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    return os.path.join(RESULTS_DIR, name)


def force_ramp(f_max, t_ramp):
    """smoothstep 力曲线：t_ramp 内升到 f_max，之后保持。"""

    def f(t):
        if t >= t_ramp:
            return f_max
        s = t / t_ramp
        return f_max * (3.0 * s * s - 2.0 * s**3)

    return f


def run_chain(chain, f_max, t_ramp, t_end, dt, record_every=1):
    """力控激励下积分一条链，返回时间历程。"""
    f = force_ramp(f_max, t_ramp)

    def ext(t):
        a = np.zeros(chain.n)
        a[0] = f(t)
        return a

    return integrate(
        chain, t_end, dt, lambda t: 0.0, lambda t: 0.0,
        record_every=record_every, ext_force_func=ext,
    )


def element_compressions(chain, x, gaps):
    """返回每个元件（按 elements 顺序）的压缩量时间序列列表。

    gaps 是每个与 BistableUnit 对应的间隙；非双稳元件返回 None。
    """
    from models.chain import BistableUnit, GROUND, BASE

    out = []
    for k, (i, j, elem) in enumerate(chain.elements):
        if not isinstance(elem, BistableUnit):
            out.append(None)
            continue
        def pos(node):
            if node == GROUND:
                return 0.0
            if node == BASE:
                return 0.0
            return x[:, node]
        c = pos(i) - pos(j) - elem.gap
        out.append(c)
    return out


def detect_crossing(t, series, threshold, rising=True):
    """检测时间序列第一次穿过阈值的时刻，返回索引或 None。"""
    if series is None:
        return None
    for k in range(len(t) - 1):
        a, b = series[k], series[k + 1]
        if rising and a < threshold <= b:
            return k
        if (not rising) and a > threshold >= b:
            return k
    return None


def peak_after(t, series, t0, window=None):
    """t0 之后（可选窗口内）的最大绝对值与对应时间。"""
    mask = t >= t0
    if window is not None:
        mask &= t <= t0 + window
    if not np.any(mask):
        return 0.0, t0
    sub = np.abs(series[mask])
    ts = t[mask]
    i = int(np.argmax(sub))
    return float(sub[i]), float(ts[i])


def ringing_metric(t, series, t0):
    """事件后的振铃指标：返回 1ms 窗口内的速度峰峰值与过零次数。"""
    mask = (t >= t0) & (t <= t0 + 3e-3)
    if np.count_nonzero(mask) < 3:
        return 0.0, 0
    s = series[mask]
    p2p = float(np.max(s) - np.min(s))
    sign = np.sign(s)
    zc = int(np.count_nonzero(np.diff(sign) != 0))
    return p2p, zc


def write_csv(filename, header, rows):
    path = results_path(filename)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)
    return path


def run_and_events(chain, f_max, t_ramp, t_end, dt, record_every=1):
    """跑一次力控瞬态，检测每个双稳元件的 snap 事件。

    返回 (res, events)。events 每项包含元件序号、事件时刻、事件处压缩量、
    触发力和当时的输出节点速度。
    """
    from models.chain import BistableUnit, GROUND, BASE

    res = run_chain(chain, f_max, t_ramp, t_end, dt, record_every)
    t = res["t"]
    x = res["x"]
    v = res["v"]
    f = force_ramp(f_max, t_ramp)
    out_node = chain.n - 1
    events = []
    for k, (i, j, elem) in enumerate(chain.elements):
        if not isinstance(elem, BistableUnit):
            continue

        def pos(node):
            if node == GROUND:
                return 0.0
            if node == BASE:
                return 0.0
            return x[:, node]

        c = pos(i) - pos(j) - elem.gap
        idx = detect_crossing(t, c, elem.vmt.snap_stroke)
        events.append(
            {
                "element": k,
                "event_index": idx,
                "time": float(t[idx]) if idx is not None else None,
                "compression": float(c[idx]) if idx is not None else None,
                "trigger_force": float(f(t[idx])) if idx is not None else None,
                "output_velocity": float(v[idx, out_node]) if idx is not None else None,
            }
        )
    return res, events


def maybe_plot(filename, plot_func):
    """matplotlib 可选。缺库时跳过并返回 None。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None
    fig = plt.figure(figsize=(7, 4.5))
    ax = fig.add_subplot(111)
    plot_func(ax)
    fig.tight_layout()
    out = results_path(filename)
    fig.savefig(out, dpi=140)
    plt.close(fig)
    return out


MM = 1e3
MS = 1e3
