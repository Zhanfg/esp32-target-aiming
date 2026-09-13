"""MST-01D 可切换边界研究版：locked / released 边界加多种预载状态。

重建：
    python MST-01D/build.py
    python MST-01D/build.py --core dual
    python MST-01D/build.py --boundary released
    python MST-01D/build.py --preload P2
    python MST-01D/build.py --stl           # 纯 Python 导出 STL
    python MST-01D/build.py --freecad       # FreeCAD 可用时才落盘

D 级的目的是建立 boundary state -> mechanical response 的实验映射（§13、§16 Test D1）。
边界由 BOUNDARY_MODULE 的锁销切换，预载由 PRELOAD_MODULE 的垫片堆设定。核心链
支持单级与双级两种，理由是统一架构 §3.2 记录双级相对单级增益有限、可能回退单级，
边界研究不该绑死在级数上。默认跑单级加 locked 边界，另打印单/双级对照。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib import (boundary, checks, frame, membrane, params as P, preload,  # noqa: E402
                 release, stage)

LEVEL = "D"
CORES = ("single", "dual")
LEAF_MATERIAL = "pet"


def build_output(core: str) -> list:
    lay = P.layout(LEVEL, core)
    z0, z1 = lay["output"]
    return [P.cyl("OUTPUT_rod", "OUTPUT", "steel301", P.ACT_ROD_D / 2.0,
                  z1 - z0, "z", (0.0, 0.0, (z0 + z1) / 2.0),
                  note="末级顶点滑块到膜片的传力杆")]


def build_gap_shim(core: str) -> list:
    """双级时第二级底座与第一级滑块之间的垫片，单级为空。"""
    if core != "dual":
        return []
    lay = P.layout(LEVEL, core)
    z1 = lay["stages"][0] + P.STAGE_APEX_H + P.STAGE_APEX_BLOCK[2]
    z2 = lay["stages"][1]
    thickness = max(z2 - z1, 0.1)
    return [P.box("STAGE_GAP_shim", "STAGE_2", "petg",
                  (P.FRAME_W - 2.0, 4.0, thickness),
                  (0.0, 0.0, (z1 + z2) / 2.0),
                  note=f"级间垫片，厚度 {thickness:.2f} mm，可换以扫 engagement gap")]


def build_all(core: str = "single", boundary_state: str = "locked",
              preload_state: str = "P0") -> list:
    lay = P.layout(LEVEL, core)
    parts = []
    parts += frame.build(LEVEL)
    parts += release.build(*lay["release"])
    parts += boundary.build(*lay["boundary"], state=boundary_state)
    parts += preload.build(*lay["preload"], state=preload_state)
    for i, z in enumerate(lay["stages"], start=1):
        parts += stage.build(i, z, LEAF_MATERIAL)
    parts += build_gap_shim(core)
    parts += build_output(core)
    parts += membrane.build(lay["membrane"][0], lay["membrane"][1], "silicone")
    return parts


def print_core_comparison(boundary_state: str, preload_state: str) -> None:
    print("---- D 级单级 / 双级对照 ----")
    for core in CORES:
        parts = build_all(core, boundary_state, preload_state)
        dx, dy, dz = checks.extents(parts)
        vol = checks.total_volume(parts)
        mass = sum(checks.mass_by_material(parts).values())
        print(f"  {core:>6}: 件数 {checks.total_parts(parts):>3} "
              f"包围盒 X={dx:.2f} Y={dy:.2f} Z={dz:.2f} mm "
              f"体积 {vol:.1f} mm^3 质量 {mass:.3f} g")


def parse_choice(flag: str, default: str, allowed) -> str:
    if flag in sys.argv:
        return sys.argv[sys.argv.index(flag) + 1]
    return default


def main() -> None:
    core = parse_choice("--core", "single", CORES)
    if core not in CORES:
        raise SystemExit(f"--core 只能是 {CORES}")
    state = parse_choice("--boundary", "locked", P.BND_STATES)
    if state not in P.BND_STATES:
        raise SystemExit(f"--boundary 只能是 {P.BND_STATES}")
    pre = parse_choice("--preload", "P0", tuple(P.PRELOAD_STATES))
    if pre not in P.PRELOAD_STATES:
        raise SystemExit(f"--preload 只能是 {list(P.PRELOAD_STATES)}")

    parts = build_all(core, state, pre)
    checks.print_report(f"D-{core}", parts)
    print(f"  边界状态 {boundary.stiffness_note(state)}")
    print(f"  {preload.describe(pre)}")
    print_core_comparison(state, pre)

    if "--freecad" in sys.argv:
        from lib import freecad_export
        outdir = os.path.dirname(os.path.abspath(__file__))
        freecad_export.export(parts, outdir, f"MST-01D_{core}_{state}")
    if "--stl" in sys.argv:
        from lib import stl
        outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")
        per, merged = stl.export_parts(
            parts, outdir, f"MST-01D_{core}_{state}")
        stl.print_export_report(per, merged)


if __name__ == "__main__":
    main()
