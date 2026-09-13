"""FreeCAD 导出，best-effort。

把 params.Solid 列表转成 FreeCAD 实体并导出 STEP 与 FCStd。FreeCAD 没装或
MCP 不通时只打印提示，不影响前面几何自检的交付。这一步在 A/B/C 自检之后才跑。

坐标与 Solid 一致：Z 轴向。box 用 Part.makeBox，cylinder 用 Part.makeCylinder，
tube 用外圆柱差内圆柱，leaf 用长方体绕 Y 旋转到倾角并经 Placement 平移。
"""

from __future__ import annotations

import math
import os


def _place_axis(shape, axis):
    """把默认沿 Z 的圆柱转到位。"""
    if axis == "z":
        return shape
    if axis == "x":
        return shape.rotate((0, 0, 0), (0, 1, 0), 90.0)
    if axis == "y":
        return shape.rotate((0, 0, 0), (1, 0, 0), 90.0)
    raise ValueError(axis)


def _shape_for(s, Part):
    if s.kind == "box":
        shp = Part.makeBox(*s.size)
        shp.translate((-s.size[0] / 2.0, -s.size[1] / 2.0, -s.size[2] / 2.0))
    elif s.kind == "cylinder":
        shp = _place_axis(Part.makeCylinder(s.radius, s.length), s.axis)
        # 圆柱默认从原点沿 +轴，转到以中心为基准
        off = {"z": (0, 0, s.length / 2.0), "x": (s.length / 2.0, 0, 0),
               "y": (0, s.length / 2.0, 0)}[s.axis]
        shp.translate((-off[0], -off[1], -off[2]))
    elif s.kind == "tube":
        outer = Part.makeCylinder(s.radius, s.length)
        inner = Part.makeCylinder(s.inner_radius, s.length + 0.1)
        shp = _place_axis(outer.cut(inner), s.axis)
        off = {"z": (0, 0, s.length / 2.0), "x": (s.length / 2.0, 0, 0),
               "y": (0, s.length / 2.0, 0)}[s.axis]
        shp.translate((-off[0], -off[1], -off[2]))
    elif s.kind == "leaf":
        (x0, z0), (x1, z1) = s.p0, s.p1
        dx, dz = x1 - x0, z1 - z0
        length = math.hypot(dx, dz)
        shp = Part.makeBox(length, s.xsec[1], s.xsec[0])
        shp.translate((-length / 2.0, -s.xsec[1] / 2.0, -s.xsec[0] / 2.0))
        angle = math.degrees(math.atan2(dz, dx))
        shp.rotate((0, 0, 0), (0, 1, 0), -angle)
    else:
        raise ValueError(s.kind)
    shp.translate((s.pos[0], s.pos[1], s.pos[2]))
    return shp


def export(parts, outdir: str, name: str) -> dict:
    try:
        import FreeCAD
        import Part
    except Exception as exc:  # noqa: BLE001
        print(f"[FreeCAD] 不可用（{exc.__class__.__name__}: {exc}），跳过导出")
        return {"ok": False, "reason": str(exc)}

    os.makedirs(outdir, exist_ok=True)
    doc = FreeCAD.newDocument(name)
    made = 0
    for i, s in enumerate(parts):
        try:
            shape = _shape_for(s, Part)
            obj = doc.addObject("Part::Feature", f"{s.name}_{i}")
            obj.Shape = shape
            made += 1
        except Exception as exc:  # noqa: BLE001
            print(f"[FreeCAD] 跳过 {s.name}: {exc}")
    doc.recompute()
    step_path = os.path.join(outdir, f"{name}.step")
    fcstd_path = os.path.join(outdir, f"{name}.FCStd")
    Part.export(doc.Objects, step_path)
    doc.saveAs(fcstd_path)
    print(f"[FreeCAD] 导出 {made}/{len(parts)} 件")
    print(f"[FreeCAD] STEP  {step_path}  {os.path.getsize(step_path)} B")
    print(f"[FreeCAD] FCStd {fcstd_path}  {os.path.getsize(fcstd_path)} B")
    return {"ok": True, "step": step_path, "fcstd": fcstd_path, "made": made}
