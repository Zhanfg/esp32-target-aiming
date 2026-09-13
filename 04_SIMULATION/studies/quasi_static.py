"""§23.1 准静态：力—位移、稳定分支、snap 点、级序、边界敏感性。

A 版没有失稳单元，力—位移直接由膜片模型给出。
B / C 版用位移控准静态：固定基座缓慢位移，牛顿法解内部节点平衡，沿稳定分支
前进，直到输入力出现第一个局部极大（极限点）。这个极大值就是 snap 阈值，
对应的输入位移是极限点位移。到达极限点后不再跟踪，避免陷入不稳定分支。

运行：
    python studies/quasi_static.py
输出：
    results/A_force_displacement.csv
    results/B_force_displacement.csv
    results/C_force_displacement.csv
    results/boundary_sensitivity.csv
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from models import make_membrane, make_vmt
from models.chain import BASE, GROUND
from models.params import GAP_C, build_B_static, build_C_static
from models.solver import solve_static
from studies.common import MM, maybe_plot, write_csv


def scan_until_limit(chain, x_max, n_steps=1200):
    """沿稳定分支扫描，返回极限点之前（含）的记录。"""
    bases = np.linspace(0.0, x_max, n_steps)
    x = np.zeros(chain.n)
    rec = []
    for Xb in bases:
        x_new, ok, hess = solve_static(chain, Xb, x)
        if (not ok) or (np.isfinite(hess) and hess <= 0.0):
            break
        rec.append((float(Xb), x_new.copy(), float(chain.input_force(x_new, Xb))))
        x = x_new
    return rec


def first_limit(rec):
    """输入力第一个局部极大的索引，没有则返回最后一点。"""
    forces = np.array([r[2] for r in rec])
    best = 0
    scale = max(abs(forces).max(), 1e-12)
    for k in range(1, len(forces)):
        if forces[k] > forces[best] + 1e-9 * scale:
            best = k
        elif forces[k] < forces[best] - 1e-6 * scale:
            return best
    return best


def compress(elem, x, base_pos):
    def pos(node):
        if node == GROUND:
            return 0.0
        if node == BASE:
            return base_pos
        return x[node]

    return pos(elem[0]) - pos(elem[1]) - elem[2].gap


def evaluate_prototype(chain, name, x_max):
    rec = scan_until_limit(chain, x_max)
    i = first_limit(rec)
    Xb, xv, F = rec[i]
    rows = [(r[0],) + tuple(r[1]) + (r[2],) for r in rec[: i + 1]]
    header = ["base_pos_m"] + [f"x{k}_m" for k in range(chain.n)] + ["input_force_N"]
    write_csv(f"{name}_force_displacement.csv", header, rows)
    return rec, i, Xb, xv, F


def main():
    mem = make_membrane("silicone")
    v_nom = make_vmt()

    xs = np.linspace(0.0, 5e-3, 251)
    fs = [mem.force(x) for x in xs]
    write_csv("A_force_displacement.csv", ["x_m", "force_N"], list(zip(xs, fs)))
    print(f"[A] 膜片 5 mm 处力 {fs[-1]:.4f} N，小挠度刚度 {mem.small_deflection_stiffness():.2f} N/m")

    B = build_B_static(mem, v_nom)
    rec_b, ib, Xb_b, xb, Fb = evaluate_prototype(B, "B", 14e-3)
    print(f"[B] snap 阈值 {Fb:.4f} N，输入位移 {Xb_b*MM:.3f} mm，输出位移 {xb[-1]*MM:.3f} mm")
    if ib + 1 < len(rec_b):
        print(f"[B] 极限点之后输入力回落，位移出现跳变")

    C = build_C_static(mem, v_nom, make_vmt(), gap=GAP_C)
    rec_c, ic, Xb_c, xc, Fc = evaluate_prototype(C, "C", 16e-3)
    print(f"[C] 第一极限点 {Fc:.4f} N，输入位移 {Xb_c*MM:.3f} mm，输出位移 {xc[-1]*MM:.3f} mm")
    c1 = compress(C.elements[0], xc, Xb_c)
    c2 = compress(C.elements[1], xc, Xb_c)
    fold = v_nom.snap_stroke
    print(
        f"[C] 极限点处一级压缩 {c1*MM:.3f} mm、二级压缩 {c2*MM:.3f} mm，"
        f"各自距 fold 余量 {(fold-c1)*MM:.3f} / {(fold-c2)*MM:.3f} mm"
    )
    if abs((fold - c1) - (fold - c2)) < 0.02e-3:
        print("[C] 准静态下两级几乎同时到达 fold，静态无法区分先后")
    else:
        first = 1 if (fold - c1) < (fold - c2) else 2
        print(f"[C] 准静态下先到达 fold 的是第 {first} 级")

    rows = []
    for k_base in [2e2, 5e2, 1e3, 2e3, 5e3, 1e4, 3e4, 1e5]:
        v = make_vmt(k_base=k_base)
        Bs = build_B_static(mem, v)
        rb = scan_until_limit(Bs, 14e-3)
        b_force = rb[first_limit(rb)][2]
        Cs = build_C_static(mem, make_vmt(k_base=k_base), make_vmt(k_base=k_base), gap=GAP_C)
        rc = scan_until_limit(Cs, 16e-3)
        ic = first_limit(rc)
        c_force = rc[ic][2]
        xv = rc[ic][1]
        cc1 = compress(Cs.elements[0], xv, rc[ic][0])
        cc2 = compress(Cs.elements[1], xv, rc[ic][0])
        m1, m2 = v.snap_stroke - cc1, v.snap_stroke - cc2
        if abs(m1 - m2) < 0.02e-3:
            first = 0
        else:
            first = 1 if m1 < m2 else 2
        rows.append((k_base, v.snap_force, b_force, c_force, first))
        print(
            f"[边界] k_base={k_base:.0e} N/m  VMT snap 力 {v.snap_force:.3f} N，"
            f"B 阈值 {b_force:.3f} N，C 阈值 {c_force:.3f} N，先到级 {first}"
        )
    write_csv(
        "boundary_sensitivity.csv",
        ["k_base_N_per_m", "vmt_snap_force_N", "B_snap_force_N", "C_snap_force_N", "first_stage"],
        rows,
    )

    maybe_plot(
        "quasi_static_A.png",
        lambda ax: ax.plot(np.array(xs) * MM, fs),
    )


if __name__ == "__main__":
    main()
