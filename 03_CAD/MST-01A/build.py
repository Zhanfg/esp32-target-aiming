"""MST-01A 普通膜片基线：低速执行器直接推膜片，无 snap，无释放卡榫。

重建：
    python MST-01A/build.py
    python MST-01A/build.py --freecad      # 追加 FreeCAD 导出，能跑才落盘
    python MST-01A/build.py --stl          # 纯 Python 导出 STL，不依赖 FreeCAD

A 级没有储能元件，§13 定为「低速执行器直接推膜片」的基线，所以不装 RELEASE_MODULE。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib import checks, frame, membrane, params as P  # noqa: E402

LEVEL = "A"


def build_actuator() -> list:
    lay = P.layout(LEVEL)
    z0, z1 = lay["actuator"]
    r0, r1 = lay["pushrod"]
    bx, by, bz = P.ACT_BODY_SIZE
    parts = [P.box("ACTUATOR_body", "ACTUATOR", "petg", (bx, by, bz),
                   (0.0, 0.0, (z0 + z1) / 2.0),
                   note="低速执行器本体，直接推膜片，无储能")]
    parts.append(P.cyl("ACTUATOR_pushrod", "ACTUATOR", "steel301",
                       P.ACT_ROD_D / 2.0, r1 - r0, "z",
                       (0.0, 0.0, (r0 + r1) / 2.0),
                       note="推杆，行程由执行器本身限制"))
    return parts


def build_all() -> list:
    lay = P.layout(LEVEL)
    parts = []
    parts += frame.build(LEVEL)
    parts += build_actuator()
    parts += membrane.build(lay["membrane"][0], lay["membrane"][1], "silicone")
    return parts


def main() -> None:
    parts = build_all()
    checks.print_report(LEVEL, parts)
    if "--freecad" in sys.argv:
        from lib import freecad_export
        outdir = os.path.dirname(os.path.abspath(__file__))
        freecad_export.export(parts, outdir, "MST-01A")
    if "--stl" in sys.argv:
        from lib import stl
        outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")
        per, merged = stl.export_parts(parts, outdir, "MST-01A")
        stl.print_export_report(per, merged)


if __name__ == "__main__":
    main()
