"""几何自检：件数、包围盒、体积、质量估算与 envelope 对照。

这里全部是纯 Python 运算，不依赖 FreeCAD。build.py 每跑一级就调用一次，
把数字打到终端，用来判断几何是否落在 §1.2 的 envelope 内、各模块是否齐全。

注意：件数按 Solid.count 累加；重叠体积不扣除，质量估算是上界量级。弹性片的
等效刚度与材料常数来自仿真名义值，不是实测。
"""

from __future__ import annotations

from collections import defaultdict

from . import params as P


def bbox_all(parts):
    xmin = ymin = zmin = float("inf")
    xmax = ymax = zmax = float("-inf")
    for s in parts:
        (ax, ay, az), (bx, by, bz) = s.bbox()
        xmin, ymin, zmin = min(xmin, ax), min(ymin, ay), min(zmin, az)
        xmax, ymax, zmax = max(xmax, bx), max(ymax, by), max(zmax, bz)
    return (xmin, ymin, zmin), (xmax, ymax, zmax)


def extents(parts):
    lo, hi = bbox_all(parts)
    return hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]


def total_parts(parts) -> int:
    return sum(s.count for s in parts)


def total_volume(parts) -> float:
    return sum(s.volume() for s in parts)


def mass_by_material(parts) -> dict:
    out = defaultdict(float)
    for s in parts:
        out[s.material] += s.mass()
    return dict(out)


def count_by_module(parts) -> dict:
    out = defaultdict(int)
    for s in parts:
        out[s.module] += s.count
    return dict(out)


def module_names(parts) -> list:
    return sorted({s.module for s in parts})


def envelope_verdict(parts) -> dict:
    dx, dy, dz = extents(parts)
    axial = dz
    lateral = max(dx, dy)
    return {
        "axial_mm": axial,
        "lateral_mm": lateral,
        "axial_ok": P.ENVELOPE["length_min"] <= axial <= P.ENVELOPE["length_max"],
        "lateral_ok": P.ENVELOPE["lateral_min"] <= lateral <= P.ENVELOPE["lateral_max"],
    }


def assert_positive(parts) -> list:
    """返回问题列表，空表示通过。"""
    problems = []
    for s in parts:
        if s.kind == "box" and min(s.size) <= 0.0:
            problems.append(f"{s.name}: 尺寸非正 {s.size}")
        if s.kind in ("cylinder", "tube") and (s.radius <= 0 or s.length <= 0):
            problems.append(f"{s.name}: 半径或长度非正")
        if s.kind == "tube" and s.inner_radius >= s.radius:
            problems.append(f"{s.name}: 内径不小于外径")
        if s.kind == "leaf":
            if s.p0 is None or s.p1 is None or min(s.xsec) <= 0:
                problems.append(f"{s.name}: 弹性片定义不完整")
    return problems


def print_report(level: str, parts) -> dict:
    dx, dy, dz = extents(parts)
    vol = total_volume(parts)
    masses = mass_by_material(parts)
    mods = count_by_module(parts)
    env = envelope_verdict(parts)

    print(f"==== MST-01{level} 几何自检 ====")
    print(f"  零件条目 {len(parts)}，零件总数 {total_parts(parts)}")
    print(f"  包围盒 X={dx:.2f} Y={dy:.2f} Z={dz:.2f} mm")
    print(f"  轴向 {env['axial_mm']:.2f} mm，横向 max(X,Y)={env['lateral_mm']:.2f} mm")
    print(f"  envelope 轴向 30-50: {'通过' if env['axial_ok'] else '超出'}"
          f"；横向 15-25: {'通过' if env['lateral_ok'] else '超出'}")
    print(f"  体积 {vol:.1f} mm^3")
    total_g = sum(masses.values())
    detail = "，".join(f"{k} {v:.2f} g" for k, v in sorted(masses.items()))
    print(f"  质量估算合计 {total_g:.2f} g（{detail}）")
    print(f"  模块件数 {dict(sorted(mods.items()))}")
    problems = assert_positive(parts)
    print(f"  尺寸合法性 {'通过' if not problems else '有问题: ' + '; '.join(problems)}")

    # 与关键设计参数对照
    print(f"  失稳单元几何 a={P.STAGE_HALF_SPAN:.1f} L={P.STAGE_BAR_LEN:.1f} "
          f"h0={P.STAGE_APEX_H:.2f} mm；膜片通径 {P.MEMB_APERTURE_D:.1f} mm，"
          f"压环外径 {P.MEMB_RING_OD:.1f} mm")
    return {
        "level": level,
        "entries": len(parts),
        "parts": total_parts(parts),
        "bbox": (dx, dy, dz),
        "env": env,
        "volume_mm3": vol,
        "mass_g": total_g,
        "mass_by_material": masses,
        "modules": mods,
        "problems": problems,
    }
