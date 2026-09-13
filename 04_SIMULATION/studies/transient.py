"""§23.2 瞬态：snap 后速度、级间间隔、膜片运动、结构振铃。

作动为力控：外力在 t_ramp 内升到 f_max 后保持。A/B/C 用同一力曲线和同一膜片，
比较膜片速度峰值、事件时刻与间隔、振铃衰减。

运行：
    python studies/transient.py
输出：
    results/transient_A.csv, transient_B.csv, transient_C.csv
    results/transient_summary.csv
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from models import make_membrane, make_vmt
from models.params import build_A, build_B, build_C
from studies.common import MM, MS, maybe_plot, peak_after, ringing_metric, run_and_events, write_csv

F_MAX = 2.0
T_RAMP = 0.02
T_END = 0.04
DT = 5e-7
REC = 4


def run_one(name, chain):
    res, events = run_and_events(chain, F_MAX, T_RAMP, T_END, DT, record_every=REC)
    t = res["t"]
    out = chain.n - 1
    x_out = res["x"][:, out]
    v_out = res["v"][:, out]

    imax = int(np.argmax(np.abs(v_out)))
    v_peak = float(np.abs(v_out).max())
    t_peak = float(t[imax])
    x_max = float(x_out.max())

    # 事件时刻按时间排序；间隔用于判定两级是否可分辨
    times = [e["time"] for e in events if e["time"] is not None]
    times_sorted = sorted(times)
    interval = (times_sorted[1] - times_sorted[0]) if len(times_sorted) >= 2 else float("nan")

    if times_sorted:
        p2p, zc = ringing_metric(t, v_out, times_sorted[0])
    else:
        p2p, zc = ringing_metric(t, v_out, t_peak)

    # 输入功：外部力对节点0做的功
    x0 = res["x"][:, 0]
    f = np.array([min(1.0, ti / T_RAMP) for ti in t])
    ft = F_MAX * (3 * f**2 - 2 * f**3)
    work = float(np.trapezoid(ft, x0)) if hasattr(np, "trapezoid") else float(np.trapz(ft, x0))

    rows = [(t[k],) + tuple(res["x"][k]) + tuple(res["v"][k]) for k in range(len(t))]
    header = ["t_s"] + [f"x{k}_m" for k in range(chain.n)] + [f"v{k}_m_per_s" for k in range(chain.n)]
    write_csv(f"transient_{name}.csv", header, rows)

    rec = {
        "name": name,
        "v_peak": v_peak,
        "t_peak": t_peak,
        "x_max": x_max,
        "work": work,
        "interval": interval,
        "ring_p2p": p2p,
        "ring_zc": zc,
        "events": events,
        "t": t,
        "v_out": v_out,
    }
    print(
        f"[{name}] 膜片峰值速度 {v_peak:.3f} m/s（t={t_peak*MS:.2f} ms），"
        f"最大位移 {x_max*MM:.3f} mm，输入功 {work*1e3:.3f} mJ"
    )
    for e in events:
        if e["time"] is not None:
            print(
                f"     元件{e['element']} snap：t={e['time']*MS:.3f} ms，"
                f"触发力 {e['trigger_force']:.3f} N，输出速度 {e['output_velocity']:.3f} m/s"
            )
    if len(times_sorted) >= 2:
        order = [e["element"] for e in sorted(events, key=lambda e: (e["time"] is None, e["time"] or 1e9))]
        print(
            f"     两级间隔 {interval*MS:.3f} ms，顺序 {order}，"
            f"事件后 3 ms 振铃峰峰 {p2p:.3f} m/s，过零 {zc} 次"
        )
    return rec


def main():
    mem = make_membrane("silicone")
    results = []
    results.append(run_one("A", build_A(mem)))
    results.append(run_one("B", build_B(mem, make_vmt())))
    results.append(run_one("C", build_C(mem, make_vmt(), make_vmt())))

    row_a = next(r for r in results if r["name"] == "A")
    row_b = next(r for r in results if r["name"] == "B")
    print(
        f"[对比] B/A 峰值速度比 {row_b['v_peak']/row_a['v_peak']:.1f}，"
        f"C/A {next(r for r in results if r['name']=='C')['v_peak']/row_a['v_peak']:.1f}"
    )

    table = []
    for r in results:
        table.append(
            (
                r["name"],
                r["v_peak"],
                r["t_peak"],
                r["x_max"],
                r["work"],
                r["interval"],
                r["ring_p2p"],
                r["ring_zc"],
            )
        )
    write_csv(
        "transient_summary.csv",
        ["prototype", "membrane_v_peak_m_per_s", "t_peak_s", "membrane_x_max_m",
         "input_work_J", "stage_interval_s", "ring_p2p_m_per_s", "ring_zero_crossings"],
        table,
    )

    def plot(ax):
        for r in results:
            ax.plot(r["t"] * MS, r["v_out"], label=r["name"])
        ax.set_xlabel("t / ms")
        ax.set_ylabel("membrane velocity / (m/s)")
        ax.legend()

    maybe_plot("transient_velocity.png", plot)


if __name__ == "__main__":
    main()
