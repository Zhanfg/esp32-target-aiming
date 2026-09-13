"""MST-01C 双级串联 snap：两个 von Mises truss 单元串联再推膜片。

重建：
    python MST-01C/build.py
    python MST-01C/build.py --freecad
    python MST-01C/build.py --stl         # 纯 Python 导出 STL，不依赖 FreeCAD

两级串联要支持参数扫描（§23.3）。级间 engagement gap 通过第二级底座的 Z 向垫片
调节，级间刚度失配通过两级弹性片厚度或材料不同来实现。两级弹性片都能单独拆换。
释放用自锁卡榫加电磁铁拔销。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib import checks, frame, membrane, params as P, release, stage  # noqa: E402

LEVEL = "C"
# 级间参数扫描的两个旋钮，改这里或从外部覆盖
STAGE1_LEAF = "pet"
STAGE2_LEAF = "pet"
ENGAGEMENT_GAP = 2.0      # 第二级底座相对第一级滑块末端的 Z 向间隙，mm，待实测标定


def build_gap_shim() -> list:
    """级间垫片，垫片厚度即 engagement gap 的粗调。"""
    lay = P.layout(LEVEL)
    z1 = lay["stages"][0] + P.STAGE_APEX_H + P.STAGE_APEX_BLOCK[2]
    z2 = lay["stages"][1]
    thickness = max(z2 - z1, 0.1)
    return [P.box("STAGE_GAP_shim", "STAGE_2", "petg",
                  (P.FRAME_W - 2.0, 4.0, thickness),
                  (0.0, 0.0, (z1 + z2) / 2.0),
                  note=f"级间垫片，厚度 {thickness:.2f} mm，可换不同垫片扫 gap")]


def build_output() -> list:
    lay = P.layout(LEVEL)
    z0, z1 = lay["output"]
    return [P.cyl("OUTPUT_rod", "OUTPUT", "steel301", P.ACT_ROD_D / 2.0,
                  z1 - z0, "z", (0.0, 0.0, (z0 + z1) / 2.0),
                  note="第二级顶点到膜片的传力杆")]


def build_all() -> list:
    lay = P.layout(LEVEL)
    parts = []
    parts += frame.build(LEVEL)
    parts += release.build(*lay["release"])
    parts += stage.build(1, lay["stages"][0], STAGE1_LEAF)
    parts += build_gap_shim()
    parts += stage.build(2, lay["stages"][1], STAGE2_LEAF)
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
        freecad_export.export(parts, outdir, "MST-01C")
    if "--stl" in sys.argv:
        from lib import stl
        outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")
        per, merged = stl.export_parts(parts, outdir, "MST-01C")
        stl.print_export_report(per, merged)


if __name__ == "__main__":
    main()
