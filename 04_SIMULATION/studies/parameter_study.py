"""§23.3 参数扫描：边界刚度、预载、级间失配、膜片刚度、阻尼。

每个扫描只动一个参数，其余固定为 params.py 的名义值。作动为力控，力在 20 ms
内升到各自阈值的一定倍数，然后保持。输出峰值膜片速度、事件顺序、级间间隔、
输入功与振铃。

运行：
    python studies/parameter_study.py
输出：
    results/sweep_boundary.csv
    results/sweep_preload.csv
    results/sweep_mismatch.csv
    results/sweep_membrane.csv
    results/sweep_damping.csv
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from models import make_membrane, make_vmt
from models.params import build_B, build_C
from studies.common import run_and_events, write_csv

T_RAMP = 0.02
T_END = 0.035
DT = 1e-6
REC = 4


def metrics(chain, f_max):
    res, events = run_and_events(chain, f_max, T_RAMP, T_END, DT, record_every=REC)
    out = chain.n - 1
    v_peak = float(np.abs(res["v"][:, out]).max())
    times = sorted(e["time"] for e in events if e["time"] is not None)
    if len(times) >= 2:
        interval = times[1] - times[0]
    else:
        interval = float("nan")
    order = [e["element"] for e in sorted(
        [e for e in events if e["time"] is not None], key=lambda e: e["time"])]
    fired = sum(1 for e in events if e["time"] is not None)
    return v_peak, interval, tuple(order), fired


def sweep_boundary(mem):
    rows = []
    for k_base in [2e2, 5e2, 1e3, 2e3, 5e3, 1e4, 3e4, 1e5]:
        v1 = make_vmt(k_base=k_base)
        v2 = make_vmt(k_base=k_base)
        B = build_B(mem, v1)
        C = build_C(mem, v1, v2)
        fmax = 1.3 * v1.snap_force
        vb, ib, ob, fb = metrics(B, fmax)
        vc, ic, oc, fc = metrics(C, fmax)
        rows.append((k_base, v1.snap_force, vb, vc, ic, str(oc)))
        print(f"[边界] k_base={k_base:.0e}  B v={vb:.2f}  C v={vc:.2f}  间隔={ic*1e3:.2f} ms  级序={oc}")
    write_csv(
        "sweep_boundary.csv",
        ["k_base_N_per_m", "vmt_snap_force_N", "B_v_peak_m_per_s", "C_v_peak_m_per_s",
         "C_interval_s", "C_order"],
        rows,
    )


def sweep_preload(mem):
    rows = []
    v = make_vmt()
    for frac in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
        pre = frac * v.snap_force
        B = build_B(mem, v, preload=pre)
        vb, ib, ob, fb = metrics(B, 1.3 * v.snap_force)
        rows.append((frac, pre, vb))
        print(f"[预载] {frac:.1f}×snap_force ({pre:.3f} N)  B v={vb:.2f}")
    write_csv(
        "sweep_preload.csv",
        ["preload_fraction", "preload_N", "B_v_peak_m_per_s"],
        rows,
    )


def sweep_mismatch(mem):
    rows = []
    v1 = make_vmt()
    for ratio in [0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0]:
        v2 = make_vmt(k_bar=2000.0 * ratio)
        C = build_C(mem, v1, v2)
        fmax = 1.3 * max(v1.snap_force, v2.snap_force)
        vc, ic, oc, fc = metrics(C, fmax)
        rows.append((ratio, v2.snap_force, vc, ic, str(oc), fc))
        print(f"[失配] k_bar2/k_bar1={ratio:.1f}  C v={vc:.2f}  间隔={ic*1e3:.2f} ms  级序={oc}  触发数={fc}")
    write_csv(
        "sweep_mismatch.csv",
        ["k_bar_ratio", "vmt2_snap_force_N", "C_v_peak_m_per_s", "C_interval_s",
         "C_order", "events_fired"],
        rows,
    )


def sweep_membrane():
    rows = []
    for scale in [0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0, 100.0]:
        mem = make_membrane("silicone", stiffness_scale=scale)
        v = make_vmt()
        B = build_B(mem, v)
        vb, ib, ob, fb = metrics(B, 2.0)
        rows.append((scale, mem.k1, mem.k3, vb, ib))
        print(f"[膜片] scale={scale:.2f}  k1={mem.k1:.2f}  B v={vb:.3f}")
    write_csv(
        "sweep_membrane.csv",
        ["stiffness_scale", "k1_N_per_m", "k3_N_per_m3", "B_v_peak_m_per_s", "B_interval_s"],
        rows,
    )


def sweep_damping(mem):
    rows = []
    v = make_vmt()
    for c in [1e-4, 1e-3, 1e-2, 5e-2, 1e-1, 3e-1, 1.0]:
        B = build_B(mem, v, damping=c)
        res, events = run_and_events(B, 2.0, T_RAMP, T_END, DT, record_every=REC)
        out = B.n - 1
        vv = res["v"][:, out]
        v_peak = float(np.abs(vv).max())
        times = sorted(e["time"] for e in events if e["time"] is not None)
        if times:
            t0 = times[0]
            # 事件后 2 到 4 ms 窗口的均方根速度与峰值之比，越小说明衰减越快
            mask = (res["t"] >= t0 + 2e-3) & (res["t"] <= t0 + 4e-3)
            if np.count_nonzero(mask) and v_peak > 0:
                decay = float(np.sqrt(np.mean(vv[mask] ** 2)) / v_peak)
            else:
                decay = float("nan")
        else:
            decay = float("nan")
        rows.append((c, v_peak, decay))
        print(f"[阻尼] c={c:.0e}  B v={v_peak:.3f}  2–4 ms 速度比={decay:.3f}")
    write_csv("sweep_damping.csv", ["damping_Ns_per_m", "B_v_peak_m_per_s", "tail_rms_ratio"], rows)


def main():
    mem = make_membrane("silicone")
    sweep_boundary(mem)
    sweep_preload(mem)
    sweep_mismatch(mem)
    sweep_membrane()
    sweep_damping(mem)


if __name__ == "__main__":
    main()
