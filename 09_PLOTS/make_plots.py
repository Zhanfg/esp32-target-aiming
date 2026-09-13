#!/usr/bin/env python3
"""从 04_SIMULATION/results 的仿真 CSV 生成论文与汇报用图。

读取的数据目录固定在仓库的 04_SIMULATION/results，只读，不写回。
输出 PNG 落在本脚本所在目录 09_PLOTS/ 下，文件名用英文。

运行：
    python make_plots.py

matplotlib 不可用时脚本不报错退出，改为在 09_PLOTS/ 写出 PLOT_SPECS.md，
把每张图的数据来源、坐标轴、单位与标注讲清楚，方便在装了 matplotlib 的
机器上复现。
"""

from __future__ import annotations

import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
RESULTS = os.path.join(PROJECT, "04_SIMULATION", "results")
OUTDIR = HERE

# 名义 VMT 由 models/vmt.py 给出，作为瞬态 snap 事件的判据。导入失败时用常量兜底。
SNAP_STROKE_FALLBACK = 2.712e-3
SNAP_FORCE_FALLBACK = 1.713025377145777
GAP_C = 0.6e-3  # params.GAP_C，C 版两级之间的 engagement gap

# A 版是普通膜片，B 版单级 snap，C 版双级串联。颜色贯穿所有图。
COL_A = "#4d4d4d"
COL_B = "#1f6fb2"
COL_C = "#c0392b"
COL_REF = "#2e8b57"

FIGSIZE_W = 13.0
DPI = 200  # 13 in * 200 = 2600 px，紧裁后仍满足宽度 >= 2000 px


# ---------------------------------------------------------------------------
# 数据读取
# ---------------------------------------------------------------------------
def load_csv(name):
    """读一条 results CSV，返回 (表头, 二维数组)。"""
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"缺少仿真结果 {path}")
    with open(path, encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        rows = [[float(v) for v in row] for row in reader]
    return header, np.asarray(rows, dtype=float)


def load_csv_raw(name):
    """读一条结果 CSV，返回 (表头, 字符串二维表)。列里有文本时用这个。"""
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"缺少仿真结果 {path}")
    with open(path, encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        rows = [row for row in reader]
    return header, rows


def nominal_snap():
    """取得名义 VMT 的 snap 压缩量与 snap 力。优先读模型，失败退回常量。"""
    try:
        sys.dont_write_bytecode = True  # 不在 04_SIMULATION 里留 .pyc
        if PROJECT not in sys.path:
            sys.path.insert(0, PROJECT)
        sim_root = os.path.join(PROJECT, "04_SIMULATION")
        if sim_root not in sys.path:
            sys.path.insert(0, sim_root)
        from models.vmt import VonMisesTruss

        v = VonMisesTruss(8.0e-3, 10.0e-3, 2000.0, 1.0e4)
        return float(v.snap_stroke), float(v.snap_force)
    except Exception:
        return SNAP_STROKE_FALLBACK, SNAP_FORCE_FALLBACK


def output_columns(header):
    """瞬态 CSV 的列位置。结构是 t + x0..x(n-1) + v0..v(n-1)。"""
    n = (len(header) - 1) // 2
    out = n - 1
    # 列布局：0 是 t，1..n 是 x0..x(n-1)，n+1..2n 是 v0..v(n-1)
    return n, out, 1 + out, 1 + n + out  # 节点数, 输出节点, 位移列, 速度列


def first_crossing(t, series, level):
    """时间序列第一次穿过 level 的时刻，线性插值。"""
    for k in range(len(series) - 1):
        if series[k] < level <= series[k + 1]:
            span = series[k + 1] - series[k]
            frac = (level - series[k]) / span if span != 0 else 0.0
            return float(t[k] + frac * (t[k + 1] - t[k]))
    return None


# ---------------------------------------------------------------------------
# 图 1 力—位移
# ---------------------------------------------------------------------------
def fig_force_displacement(snap_stroke, snap_force):
    import matplotlib.pyplot as plt

    _, A = load_csv("A_force_displacement.csv")
    _, B = load_csv("B_force_displacement.csv")
    _, C = load_csv("C_force_displacement.csv")

    fig, ax = plt.subplots(figsize=(FIGSIZE_W, 6.2))
    ax.plot(A[:, 0] * 1e3, A[:, 1], color=COL_A, lw=2.6, label="A 普通膜片")
    ax.plot(B[:, 0] * 1e3, B[:, 2], color=COL_B, lw=2.4, label="B 单级 snap")
    ax.plot(C[:, 0] * 1e3, C[:, 3], color=COL_C, lw=2.4, label="C 双级串联")

    # 准静态扫描在第一个局部极大处停止，最后一点就是 snap 阈值
    bF, bX = B[-1, 2], B[-1, 0] * 1e3
    cF, cX = C[-1, 3], C[-1, 0] * 1e3
    ax.plot([bX], [bF], "o", color=COL_B, ms=9, zorder=6)
    ax.plot([cX], [cF], "D", color=COL_C, ms=9, zorder=6)
    ax.annotate(
        f"B snap 阈值\n{bF:.3f} N @ {bX:.2f} mm",
        xy=(bX, bF), xytext=(bX + 0.15, bF + 0.52),
        fontsize=13, color=COL_B,
        arrowprops=dict(arrowstyle="->", color=COL_B, lw=1.4),
    )
    ax.annotate(
        f"C 极限点\n{cF:.3f} N @ {cX:.2f} mm",
        xy=(cX, cF), xytext=(cX - 0.35, cF + 0.72),
        fontsize=13, color=COL_C, ha="right",
        arrowprops=dict(arrowstyle="->", color=COL_C, lw=1.4),
    )
    ax.annotate(
        "C 极限点处一级、二级压缩量相同，均为 "
        f"{(C[-1, 0] - C[-1, 1]) * 1e3:.3f} mm，各自距 fold "
        f"{snap_stroke * 1e3:.3f} mm 差 "
        f"{(snap_stroke - (C[-1, 0] - C[-1, 1])) * 1e3:.3f} mm。\n"
        f"C 的作动行程约 {cX:.1f} mm，比 B 多 {cX - bX:.1f} mm。",
        xy=(cX, cF), xytext=(3.7, 0.55),
        fontsize=12, color=COL_C,
        bbox=dict(boxstyle="round,pad=0.35", fc="white", ec=COL_C, alpha=0.85),
        arrowprops=dict(arrowstyle="->", color=COL_C, lw=1.2),
    )

    ax.set_xlabel("作动端位移 / mm")
    ax.set_ylabel("力 / N")
    ax.set_title("A / B / C 准静态力—位移曲线（位移控，扫描止于第一个极限点）")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=13)
    ax.set_xlim(0, 10.4)
    ax.set_ylim(0, 3.5)
    return save(fig, "fig01_force_displacement.png")


# ---------------------------------------------------------------------------
# 图 2 瞬态位移与速度
# ---------------------------------------------------------------------------
def fig_transient(snap_stroke):
    import matplotlib.pyplot as plt

    hA, A = load_csv("transient_A.csv")
    hB, B = load_csv("transient_B.csv")
    hC, C = load_csv("transient_C.csv")

    nA, outA, xcA, vcA = output_columns(hA)
    nB, outB, xcB, vcB = output_columns(hB)
    nC, outC, xcC, vcC = output_columns(hC)

    series = {
        "A": (A, nA, xcA, vcA, COL_A),
        "B": (B, nB, xcB, vcB, COL_B),
        "C": (C, nC, xcC, vcC, COL_C),
    }

    # 从瞬态位移重建各级压缩量，用 snap_stroke 判事件，与 studies 里的判据一致
    events = {}
    tB = B[:, 0]
    events["B"] = [first_crossing(tB, B[:, 1] - B[:, 2], snap_stroke)]
    tC = C[:, 0]
    events["C"] = [
        first_crossing(tC, C[:, 1] - C[:, 2], snap_stroke),          # 一级
        first_crossing(tC, C[:, 2] - C[:, 3] - GAP_C, snap_stroke),  # 二级
    ]

    fig, (ax_x, ax_v) = plt.subplots(
        2, 1, figsize=(FIGSIZE_W, 8.4), sharex=True,
        gridspec_kw=dict(height_ratios=[1, 1.15], hspace=0.12),
    )

    labels = {}
    for name, (D, n, xc, vc, color) in series.items():
        t = D[:, 0] * 1e3
        x = D[:, xc] * 1e3
        v = D[:, vc]
        ax_x.plot(t, x, color=color, lw=2.2)
        ax_v.plot(t, v, color=color, lw=2.2)

        ix = int(np.argmax(np.abs(x)))
        iv = int(np.argmax(np.abs(v)))
        ax_x.plot([t[ix]], [x[ix]], "o", color=color, ms=8)
        ax_v.plot([t[iv]], [v[iv]], "o", color=color, ms=8)
        labels[name] = (
            f"{name} 位移峰值 {x[ix]:.2f} mm（{t[ix]:.2f} ms）",
            f"{name} 速度峰值 {v[iv]:.2f} m/s（{t[iv]:.2f} ms）",
        )

    # snap 事件时刻：竖线贯穿两幅，速度图上竖排文字标出时刻
    for name, evs in events.items():
        color = COL_B if name == "B" else COL_C
        for k, te in enumerate(evs):
            if te is None:
                continue
            tag = "B snap" if name == "B" else "C"
            if name == "C":
                tag += " 一级" if k == 0 else " 二级"
            for ax in (ax_x, ax_v):
                ax.axvline(te * 1e3, color=color, ls="--", lw=1.3, alpha=0.75)
            ax_v.text(te * 1e3, ax_v.get_ylim()[1],
                      f" {tag} {te * 1e3:.2f} ms",
                      fontsize=11, color=color, va="top", ha="left", rotation=90)

    ax_x.set_ylabel("膜片位移 / mm")
    ax_v.set_ylabel("膜片速度 / (m/s)")
    ax_v.set_xlabel("时间 / ms")
    ax_x.set_title("A / B / C 膜片瞬态响应（力控，2 N 在 20 ms 内升至平台）")
    ax_x.grid(True, alpha=0.3)
    ax_v.grid(True, alpha=0.3)
    handles = [plt.Line2D([], [], color=series[n][4], lw=2.4) for n in ("A", "B", "C")]
    ax_x.legend(handles, [labels[n][0] for n in ("A", "B", "C")],
                loc="upper left", fontsize=12)
    ax_v.legend(handles, [labels[n][1] for n in ("A", "B", "C")],
                loc="lower left", fontsize=12)
    ax_x.set_xlim(0, 40)

    # A 的速度比 B/C 小一个量级，单开小窗才看得清峰形
    axin = ax_v.inset_axes([0.60, 0.50, 0.36, 0.40])
    tA = A[:, 0] * 1e3
    vA = A[:, vcA]
    axin.plot(tA, vA, color=COL_A, lw=1.8)
    ivA = int(np.argmax(np.abs(vA)))
    axin.plot([tA[ivA]], [vA[ivA]], "o", color=COL_A, ms=6)
    axin.set_title("A 速度放大", fontsize=11)
    axin.tick_params(labelsize=9)
    axin.grid(True, alpha=0.3)

    return save(fig, "fig02_transient.png")


# ---------------------------------------------------------------------------
# 图 3 边界刚度扫描
# ---------------------------------------------------------------------------
def fig_boundary(snap_force):
    import matplotlib.pyplot as plt

    _, D = load_csv("boundary_sensitivity.csv")
    k = D[:, 0]
    vmt_f, b_f, c_f = D[:, 1], D[:, 2], D[:, 3]

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(FIGSIZE_W, 5.6),
        gridspec_kw=dict(width_ratios=[1.55, 1], wspace=0.28),
    )

    ax1.semilogx(k, vmt_f, color=COL_REF, lw=2.0, ls=":", marker="s", ms=7,
                 label="VMT 单元固有 snap 力")
    ax1.semilogx(k, c_f, color=COL_C, lw=3.2, marker="D", ms=8,
                 label="C 阈值")
    ax1.semilogx(k, b_f, color=COL_B, lw=1.8, ls="--", marker="o", ms=7,
                 label="B 阈值")
    ax1.set_xlabel("边界刚度 k_base / (N/m)")
    ax1.set_ylabel("snap 阈值 / N")
    ax1.set_title("阈值随边界刚度变化")
    ax1.grid(True, which="both", alpha=0.3)
    ax1.legend(fontsize=12, loc="upper left")
    ax1.annotate("B 与 C 两条线几乎完全重合，\n相对差在 ppm 量级",
                 xy=(k[5], b_f[5]), xytext=(k[5] * 1.6, 0.45),
                 fontsize=12, color=COL_C,
                 bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=COL_C, alpha=0.85),
                 arrowprops=dict(arrowstyle="->", color=COL_C, lw=1.2))
    ax1.text(0.02, 0.52, "A 为普通膜片，无 snap 阈值，不在此图",
             transform=ax1.transAxes, ha="left", fontsize=11, color=COL_A)

    rel_ppm = np.abs(b_f - c_f) / np.maximum(np.abs(vmt_f), 1e-12) * 1e6
    ax2.semilogx(k, rel_ppm, color="#6c3483", lw=2.2, marker="o", ms=7)
    ax2.set_xlabel("边界刚度 k_base / (N/m)")
    ax2.set_ylabel("|B - C| / VMT / ppm")
    ax2.set_title("B 与 C 阈值的相对差")
    ax2.grid(True, which="both", alpha=0.3)
    ax2.axhline(0, color="k", lw=0.8, alpha=0.4)
    ax2.annotate(f"最大 {rel_ppm.max():.2f} ppm，约百万分之几",
                 xy=(k[int(np.argmax(rel_ppm))], rel_ppm.max()),
                 xytext=(k[4], rel_ppm.max() * 0.92),
                 fontsize=12, color="#6c3483",
                 arrowprops=dict(arrowstyle="->", color="#6c3483", lw=1.2))

    fig.suptitle("边界刚度对 snap 阈值的影响（准静态）", fontsize=16)
    return save(fig, "fig03_boundary_threshold.png")


# ---------------------------------------------------------------------------
# 图 4 级间失配
# ---------------------------------------------------------------------------
def parse_order(s):
    return tuple(int(x) for x in s.strip().strip("()").split(","))


def fig_sweep_mismatch():
    import matplotlib.pyplot as plt

    header, rows = load_csv_raw("sweep_mismatch.csv")
    idx = {name: i for i, name in enumerate(header)}
    ratio = np.array([float(r[idx["k_bar_ratio"]]) for r in rows])
    vpeak = np.array([float(r[idx["C_v_peak_m_per_s"]]) for r in rows])
    interval_ms = np.array([float(r[idx["C_interval_s"]]) for r in rows]) * 1e3
    orders = [parse_order(r[idx["C_order"]]) for r in rows]

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(FIGSIZE_W, 5.4), gridspec_kw=dict(wspace=0.26)
    )

    ax1.semilogx(ratio, vpeak, color="#1a5276", lw=2.2, marker="o", ms=9)
    ax1.set_xlabel("二级与一级刚度比 k_bar2 / k_bar1")
    ax1.set_ylabel("C 膜片峰值速度 / (m/s)")
    ax1.set_title("失配下的输出速度")
    ax1.grid(True, which="both", alpha=0.3)
    ax1.axvspan(1.0, 1.5, color="#f39c12", alpha=0.12)
    i_dip = int(np.argmin(vpeak))
    ax1.annotate(f"刚度比 {ratio[i_dip]:g} 处速度最低",
                 xy=(ratio[i_dip], vpeak[i_dip]),
                 xytext=(0.34, vpeak[i_dip] + 1.3),
                 fontsize=12, color="#1a5276",
                 arrowprops=dict(arrowstyle="->", color="#1a5276", lw=1.2))

    for x, y, o in zip(ratio, interval_ms, orders):
        color = COL_C if o[0] == 1 else COL_B
        ax2.plot([x], [y], "o", ms=10, color=color)
        ax2.annotate(f"{'二级' if o[0] == 1 else '一级'}先", (x, y),
                     xytext=(0, 9), textcoords="offset points",
                     ha="center", fontsize=11, color=color)
    ax2.semilogx(ratio, interval_ms, color="#7d3c98", lw=1.6, alpha=0.6, zorder=0)
    ax2.set_xlabel("二级与一级刚度比 k_bar2 / k_bar1")
    ax2.set_ylabel("C 两级 snap 间隔 / ms")
    ax2.set_title("级序在刚度比 1.0 到 1.5 之间翻转")
    ax2.grid(True, which="both", alpha=0.3)
    ax2.axvspan(1.0, 1.5, color="#f39c12", alpha=0.12)
    ax2.legend(handles=[
        plt.Line2D([], [], marker="o", ls="", color=COL_C, label="二级先触发"),
        plt.Line2D([], [], marker="o", ls="", color=COL_B, label="一级先触发"),
    ], fontsize=12, loc="lower left")

    for ax in (ax1, ax2):
        ax.set_xticks(ratio)
        ax.set_xticklabels([f"{r:g}" for r in ratio])
        ax.minorticks_off()
        ax.set_xlim(0.24, 3.6)

    fig.suptitle("级间失配扫描（C 版双级串联）", fontsize=16)
    return save(fig, "fig04_sweep_mismatch.png")


# ---------------------------------------------------------------------------
# 图 5 膜片刚度
# ---------------------------------------------------------------------------
def fig_sweep_membrane():
    import matplotlib.pyplot as plt

    _, D = load_csv("sweep_membrane.csv")
    scale = D[:, 0]
    vpeak = D[:, 3]

    fig, ax = plt.subplots(figsize=(FIGSIZE_W, 5.8))
    ax.semilogx(scale, vpeak, color="#117a65", lw=2.4, marker="o", ms=9)

    # 找真正的内部局部极值，端点不算
    loc_min = [i for i in range(1, len(vpeak) - 1)
               if vpeak[i] < vpeak[i - 1] and vpeak[i] < vpeak[i + 1]]
    loc_max = [i for i in range(1, len(vpeak) - 1)
               if vpeak[i] > vpeak[i - 1] and vpeak[i] > vpeak[i + 1]]
    i_min = min(loc_min, key=lambda i: vpeak[i]) if loc_min else int(np.argmin(vpeak))
    i_max = max(loc_max, key=lambda i: vpeak[i]) if loc_max else int(np.argmax(vpeak))
    ax.plot([scale[i_min]], [vpeak[i_min]], "v", color=COL_C, ms=12)
    ax.plot([scale[i_max]], [vpeak[i_max]], "^", color=COL_B, ms=12)
    ax.annotate(
        f"局部低点 scale={scale[i_min]:g}\n{vpeak[i_min]:.2f} m/s",
        xy=(scale[i_min], vpeak[i_min]), xytext=(0.012, 5.1),
        fontsize=12, color=COL_C,
        arrowprops=dict(arrowstyle="->", color=COL_C, lw=1.2),
    )
    ax.annotate(
        f"局部高点 scale={scale[i_max]:g}\n{vpeak[i_max]:.2f} m/s",
        xy=(scale[i_max], vpeak[i_max]), xytext=(0.05, 8.35),
        fontsize=12, color=COL_B,
        arrowprops=dict(arrowstyle="->", color=COL_B, lw=1.2),
    )
    ax.annotate(
        f"刚度放大 100 倍\n速度降到 {vpeak[-1]:.2f} m/s",
        xy=(scale[-1], vpeak[-1]), xytext=(1.0, 4.1),
        fontsize=12, color="#7d3c98",
        arrowprops=dict(arrowstyle="->", color="#7d3c98", lw=1.2),
    )
    ax.axvspan(1.0, 3.0, color="#f39c12", alpha=0.12, label="最佳区")
    ax.set_xlabel("膜片刚度放大系数")
    ax.set_ylabel("B 膜片峰值速度 / (m/s)")
    ax.set_title("膜片柔度扫描：峰值输出对柔度非单调")
    ax.grid(True, which="both", alpha=0.3)
    ax.set_ylim(3.0, 9.8)
    ax.legend(fontsize=12, loc="lower left")
    return save(fig, "fig05_sweep_membrane.png")


# ---------------------------------------------------------------------------
# 图 6 阻尼
# ---------------------------------------------------------------------------
def fig_sweep_damping():
    import matplotlib.pyplot as plt

    _, D = load_csv("sweep_damping.csv")
    c = D[:, 0]
    vpeak = D[:, 1]
    tail = D[:, 2]

    fig, ax = plt.subplots(figsize=(FIGSIZE_W, 5.8))
    ax.semilogx(c, vpeak, color="#b9770e", lw=2.4, marker="o", ms=9,
                label="膜片峰值速度")
    ax.set_xlabel("阻尼系数 c / (N·s/m)")
    ax.set_ylabel("B 膜片峰值速度 / (m/s)", color="#b9770e")
    ax.tick_params(axis="y", labelcolor="#b9770e")
    ax.grid(True, which="both", alpha=0.3)

    ax2 = ax.twinx()
    ax2.semilogx(c, tail, color="#2471a3", lw=2.0, ls="--", marker="s", ms=7,
                 label="2–4 ms 尾部均方根速度比")
    ax2.set_ylabel("尾部均方根速度 / 峰值", color="#2471a3")
    ax2.tick_params(axis="y", labelcolor="#2471a3")

    ax.annotate("c >= 0.1 时速度明显下降",
                xy=(0.3, vpeak[5]), xytext=(0.0016, 5.2),
                fontsize=12, color="#b9770e",
                arrowprops=dict(arrowstyle="->", color="#b9770e", lw=1.2))
    ax.set_title("阻尼扫描：峰值速度与尾部衰减")

    lines = ax.get_lines() + ax2.get_lines()
    ax.legend(lines, [l.get_label() for l in lines], fontsize=12, loc="lower left")
    return save(fig, "fig06_sweep_damping.png")


# ---------------------------------------------------------------------------
# 图 7 预载
# ---------------------------------------------------------------------------
def fig_sweep_preload():
    import matplotlib.pyplot as plt

    _, D = load_csv("sweep_preload.csv")
    frac = D[:, 0]
    preload = D[:, 1]
    vpeak = D[:, 2]

    fig, ax = plt.subplots(figsize=(FIGSIZE_W, 5.8))
    ax.plot(frac, vpeak, color="#7d3c98", lw=2.4, marker="o", ms=9)
    i_max = int(np.argmax(vpeak))
    ax.plot([frac[i_max]], [vpeak[i_max]], "^", color=COL_B, ms=12)
    ax.annotate(f"峰值 {vpeak[i_max]:.3f} m/s\n在预载 {frac[i_max]:.1f}×snap_force",
                xy=(frac[i_max], vpeak[i_max]),
                xytext=(frac[i_max] + 0.03, vpeak[i_max] - 1.1),
                fontsize=12, color=COL_B,
                arrowprops=dict(arrowstyle="->", color=COL_B, lw=1.2))
    ax.annotate(f"名义值 {vpeak[0]:.3f} m/s",
                xy=(frac[0], vpeak[0]), xytext=(0.015, 9.30),
                fontsize=12, color=COL_A,
                arrowprops=dict(arrowstyle="->", color=COL_A, lw=1.0))
    ax.set_xlabel("预载 / snap_force")
    ax.set_ylabel("B 膜片峰值速度 / (m/s)")
    ax.set_title("预载扫描：先升后降，整体变化不到 15%")
    ax.grid(True, alpha=0.3)
    ax.set_xticks(frac)
    ax.set_ylim(8.35, 9.78)
    return save(fig, "fig07_sweep_preload.png")


# ---------------------------------------------------------------------------
# 保存
# ---------------------------------------------------------------------------
def save(fig, filename):
    import matplotlib.pyplot as plt

    out = os.path.join(OUTDIR, filename)
    fig.savefig(out, dpi=DPI, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# 降级路径：写画图规格说明
# ---------------------------------------------------------------------------
def write_plot_specs():
    text = """# PLOT_SPECS

本文件由 `make_plots.py` 在检测不到 matplotlib 时生成。它逐张说明图怎么画，
数据来自 `04_SIMULATION/results/`，只读。装了 matplotlib 后跑一次
`python make_plots.py` 即可得到同名的 PNG。

约定：长度 m 换 mm，时间 s 换 ms，力 N，速度 m/s。A 普通膜片、B 单级 snap、
C 双级串联在全部图里用同一种颜色区分。A 版没有双稳单元，凡涉及 snap 阈值的
图都不含 A。

## fig01_force_displacement.png

- 数据：`A_force_displacement.csv`（x_m, force_N）、`B_force_displacement.csv`
  （base_pos_m, x0_m, input_force_N）、`C_force_displacement.csv`
  （base_pos_m, x0_m, x1_m, input_force_N）。
- 横轴：作动端位移 mm。A 用 x_m，B / C 用 base_pos_m。
- 纵轴：力 N。A 用 force_N，B / C 用 input_force_N。
- 标注：B、C 曲线最后一点就是各自 snap 阈值，A 版为普通膜片无阈值。C 的
  极限点是两级串联的共同极限，此处一级与二级压缩量相同，都在各自 fold 之前。
  另标出 C 需要的作动行程比 B 多出的部分。
- 图注：准静态力—位移曲线，位移控，扫描止于第一个极限点。用于判断 STOP-01。

## fig02_transient.png

- 数据：`transient_A.csv`、`transient_B.csv`、`transient_C.csv`。列为
  t_s、x0..x(n-1)、v0..v(n-1)，输出节点是最后一个节点，即膜片。
- 上panel：膜片位移 mm 对时间 ms。下panel：膜片速度 m/s 对时间 ms。
- 标注：三条曲线各自的峰值与到达时刻。B 的 snap 事件与 C 的一级、二级 snap
  事件画竖线，时刻由各自双稳元件的压缩量穿过 snap 压缩量确定，C 的一级压缩
  为 x0 - x1，二级压缩为 x1 - x2 - 0.6 mm。
- 附注：A 的速度比 B / C 小一个量级，下panel加一小窗单独显示 A。
- 图注：力控激励，2 N 在 20 ms 内升到平台。用于判断 STOP-01 / 02。

## fig03_boundary_threshold.png

- 数据：`boundary_sensitivity.csv`（k_base_N_per_m, vmt_snap_force_N,
  B_snap_force_N, C_snap_force_N, first_stage）。
- 左图：横轴 k_base 对数坐标，纵轴 snap 阈值 N，画 VMT 单元固有 snap 力、
  B 阈值、C 阈值三条线。
- 右图：横轴 k_base 对数坐标，纵轴 B 与 C 阈值的相对差，单位 ppm，
  分母取 VMT snap 力。
- 标注：B 与 C 几乎重合，右图给出量级。A 无 snap 阈值，不画。
- 图注：边界刚度对 snap 阈值的影响。用于判断双级是否改变静态阈值。

## fig04_sweep_mismatch.png

- 数据：`sweep_mismatch.csv`（k_bar_ratio, vmt2_snap_force_N,
  C_v_peak_m_per_s, C_interval_s, C_order, events_fired）。
- 左图：横轴刚度比 k_bar2 / k_bar1 对数坐标，纵轴 C 膜片峰值速度 m/s。
- 右图：横轴同上，纵轴两级 snap 间隔 ms，点的颜色与文字标出哪一级先触发。
- 标注：级序在刚度比 1.0 与 1.5 之间翻转。
- 图注：级间失配扫描，C 版双级串联。

## fig05_sweep_membrane.png

- 数据：`sweep_membrane.csv`（stiffness_scale, k1_N_per_m, k3_N_per_m3,
  B_v_peak_m_per_s, B_interval_s）。
- 横轴：膜片刚度放大系数，对数坐标。纵轴：B 膜片峰值速度 m/s。
- 标注：局部低点与局部高点，以及刚度放大 100 倍时的下降；名义值到 3 倍
  之间标为最佳区。
- 图注：柔度非单调。

## fig06_sweep_damping.png

- 数据：`sweep_damping.csv`（damping_Ns_per_m, B_v_peak_m_per_s,
  tail_rms_ratio）。
- 横轴：阻尼系数 N·s/m，对数坐标。左纵轴：B 膜片峰值速度 m/s。
  右纵轴：2–4 ms 窗口尾部均方根速度与峰值之比。
- 图注：阻尼扫描。

## fig07_sweep_preload.png

- 数据：`sweep_preload.csv`（preload_fraction, preload_N, B_v_peak_m_per_s）。
- 横轴：预载与 snap_force 的比值。纵轴：B 膜片峰值速度 m/s。
- 标注：峰值位置与名义值。
- 图注：预载扫描，先升后降，整体变化不大。
"""
    out = os.path.join(OUTDIR, "PLOT_SPECS.md")
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return out


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def setup_fonts():
    from matplotlib import font_manager, rcParams

    candidates = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Zen Hei",
        "Arial Unicode MS",
    ]
    available = {f.name for f in font_manager.fontManager.ttflist}
    chosen = next((n for n in candidates if n in available), None)
    if chosen:
        rcParams["font.sans-serif"] = [chosen] + list(rcParams["font.sans-serif"])
    rcParams["axes.unicode_minus"] = False
    rcParams["font.size"] = 14
    rcParams["axes.titlesize"] = 16
    rcParams["axes.labelsize"] = 14
    rcParams["xtick.labelsize"] = 12
    rcParams["ytick.labelsize"] = 12
    return chosen


def main():
    if not os.path.isdir(RESULTS):
        print(f"找不到结果目录 {RESULTS}")
        return 1

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: F401
    except Exception as exc:
        print(f"matplotlib 不可用（{exc}），改出 PLOT_SPECS.md")
        out = write_plot_specs()
        print(f"已写出 {out}")
        return 0

    chosen = setup_fonts()
    snap_stroke, snap_force = nominal_snap()
    print(f"中文字体：{chosen or '未找到，可能出现方块'}")
    print(f"名义 snap：压缩 {snap_stroke * 1e3:.3f} mm，力 {snap_force:.4f} N")

    jobs = [
        ("力—位移", lambda: fig_force_displacement(snap_stroke, snap_force)),
        ("瞬态响应", lambda: fig_transient(snap_stroke)),
        ("边界刚度", lambda: fig_boundary(snap_force)),
        ("级间失配", fig_sweep_mismatch),
        ("膜片柔度", fig_sweep_membrane),
        ("阻尼", fig_sweep_damping),
        ("预载", fig_sweep_preload),
    ]
    for label, fn in jobs:
        out = fn()
        size_kb = os.path.getsize(out) / 1024.0
        print(f"[{label}] {os.path.basename(out)}  {size_kb:.0f} KB")
    print("全部完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
