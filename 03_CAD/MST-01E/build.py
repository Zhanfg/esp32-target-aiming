"""MST-01E 自动供给完整版：6 / 8 位供给盘加索引，支持 Mode B。

重建：
    python MST-01E/build.py
    python MST-01E/build.py --positions 8
    python MST-01E/build.py --core dual --positions 6
    python MST-01E/build.py --stl           # 纯 Python 导出 STL
    python MST-01E/build.py --freecad       # FreeCAD 可用时才落盘

E 级在 D 级的边界与预载之上再加 MAGAZINE_MODULE（§9、§13）：6 位与 8 位两种布局
都能生成，跑完给出两者包围盒对比，选用依据 §31（8 位若明显变大就先 6 位）。核心
链同样有单级与双级两种。默认 6 位加单级，另打印 6/8 位对照。

载荷是 §8.1 超轻软圆片，pocket 用环形座圈加中心通孔排除硬弹丸（§8.4），细节见
README 与 lib/magazine.py。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib import (boundary, checks, frame, magazine, membrane, params as P,  # noqa: E402
                 preload, release, stage)

LEVEL = "E"
CORES = ("single", "dual")
LEAF_MATERIAL = "pet"


def build_output(core: str) -> list:
    lay = P.layout(LEVEL, core)
    z0, z1 = lay["output"]
    return [P.cyl("OUTPUT_rod", "OUTPUT", "steel301", P.ACT_ROD_D / 2.0,
                  z1 - z0, "z", (0.0, 0.0, (z0 + z1) / 2.0),
                  note="末级顶点滑块到膜片的传力杆")]


def build_gap_shim(core: str) -> list:
    if core != "dual":
        return []
    lay = P.layout(LEVEL, core)
    z1 = lay["stages"][0] + P.STAGE_APEX_H + P.STAGE_APEX_BLOCK[2]
    z2 = lay["stages"][1]
    thickness = max(z2 - z1, 0.1)
    return [P.box("STAGE_GAP_shim", "STAGE_2", "petg",
                  (P.FRAME_W - 2.0, 4.0, thickness),
                  (0.0, 0.0, (z1 + z2) / 2.0),
                  note=f"级间垫片，厚度 {thickness:.2f} mm")]


def build_magazine(core: str, positions: int) -> list:
    lay = P.layout(LEVEL, core)
    z0, z1 = lay["magazine"]
    return magazine.build(z0, z1, positions)


def build_all(core: str = "single", positions: int = 6,
              boundary_state: str = "locked", preload_state: str = "P0") -> list:
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
    parts += build_magazine(core, positions)
    return parts


def _summary(parts) -> str:
    dx, dy, dz = checks.extents(parts)
    vol = checks.total_volume(parts)
    mass = sum(checks.mass_by_material(parts).values())
    return (f"件数 {checks.total_parts(parts):>3} 包围盒 "
            f"X={dx:.2f} Y={dy:.2f} Z={dz:.2f} mm 体积 {vol:.1f} mm^3 "
            f"质量 {mass:.3f} g")


def print_position_comparison(core: str) -> None:
    print(f"---- E 级供给盘 6 / 8 位对照（核心 {core}）----")
    for n in P.MAG_POSITIONS:
        mag = build_magazine(core, n)
        mdx, mdy, mdz = checks.extents(mag)
        full = build_all(core, n)
        dx, dy, dz = checks.extents(full)
        over = max(dx, dy) - P.ENVELOPE["lateral_max"]
        print(f"  {n} 位 盘体 X={mdx:.2f} Y={mdy:.2f} Z={mdz:.2f} mm")
        print(f"       整机 X={dx:.2f} Y={dy:.2f} Z={dz:.2f} mm，"
              f"横向超出上限 {over:+.2f} mm，{magazine.describe(n)}")


def parse_choice(flag: str, default: str, allowed) -> str:
    if flag in sys.argv:
        return sys.argv[sys.argv.index(flag) + 1]
    return default


def main() -> None:
    core = parse_choice("--core", "single", CORES)
    if core not in CORES:
        raise SystemExit(f"--core 只能是 {CORES}")
    pos = int(parse_choice("--positions", "6", ("6", "8")))
    if pos not in P.MAG_POSITIONS:
        raise SystemExit(f"--positions 只能是 {P.MAG_POSITIONS}")
    state = parse_choice("--boundary", "locked", P.BND_STATES)
    pre = parse_choice("--preload", "P0", tuple(P.PRELOAD_STATES))

    parts = build_all(core, pos, state, pre)
    checks.print_report(f"E-{core}-{pos}pos", parts)
    print(f"  边界状态 {boundary.stiffness_note(state)}")
    print(f"  {preload.describe(pre)}")
    print(f"  供给盘 {magazine.describe(pos)}")
    print_position_comparison(core)

    if "--freecad" in sys.argv:
        from lib import freecad_export
        outdir = os.path.dirname(os.path.abspath(__file__))
        freecad_export.export(parts, outdir, f"MST-01E_{core}_{pos}pos")
    if "--stl" in sys.argv:
        from lib import stl
        outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")
        per, merged = stl.export_parts(
            parts, outdir, f"MST-01E_{core}_{pos}pos")
        stl.print_export_report(per, merged)


if __name__ == "__main__":
    main()
