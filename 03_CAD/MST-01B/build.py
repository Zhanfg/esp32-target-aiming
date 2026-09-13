"""MST-01B 单级 snap：基座经一个 von Mises truss 单元推膜片。

重建：
    python MST-01B/build.py
    python MST-01B/build.py --freecad
    python MST-01B/build.py --stl         # 纯 Python 导出 STL，不依赖 FreeCAD

弹性片独立可换（§7.1），打印件只做夹块、滑块与支座。释放用自锁卡榫加电磁铁
拔销，掉电不释放。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib import checks, frame, membrane, params as P, release, stage  # noqa: E402

LEVEL = "B"
LEAF_MATERIAL = "pet"      # §7.2 先用 PET 做几何验证，寿命版切 steel301


def build_output() -> list:
    lay = P.layout(LEVEL)
    z0, z1 = lay["output"]
    return [P.cyl("OUTPUT_rod", "OUTPUT", "steel301", P.ACT_ROD_D / 2.0,
                  z1 - z0, "z", (0.0, 0.0, (z0 + z1) / 2.0),
                  note="顶点滑块到膜片的传力杆")]


def build_all() -> list:
    lay = P.layout(LEVEL)
    parts = []
    parts += frame.build(LEVEL)
    parts += release.build(*lay["release"])
    for i, z in enumerate(lay["stages"], start=1):
        parts += stage.build(i, z, LEAF_MATERIAL)
    parts += build_output()
    parts += membrane.build(lay["membrane"][0], lay["membrane"][1], "silicone")
    return parts


def main() -> None:
    parts = build_all()
    checks.print_report(LEVEL, parts)
    print("  释放机构 " + release.describe())
    if "--freecad" in sys.argv:
        from lib import freecad_export
        outdir = os.path.dirname(os.path.abspath(__file__))
        freecad_export.export(parts, outdir, "MST-01B")
    if "--stl" in sys.argv:
        from lib import stl
        outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")
        per, merged = stl.export_parts(parts, outdir, "MST-01B")
        stl.print_export_report(per, merged)


if __name__ == "__main__":
    main()
