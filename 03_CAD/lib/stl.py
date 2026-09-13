"""纯 Python STL 写出与读回。

params.Solid 是解析式原语（box / cylinder / tube / leaf），本模块按尺寸和位置
直接三角化，不依赖 FreeCAD，也不用第三方库（struct 就够）。STL 本身不带单位，
这里按毫米导出，并在 80 字节文件头注明。法向由三角形顶点用右手定则现算，不假定
输入已给出法向。

圆柱与圆筒按周向分段逼近，默认 64 段。多边形面积相对理论圆偏低约 0.17%，
落在校验阈值之内，且分段数可以通过 segments 参数调整。
"""

from __future__ import annotations

import math
import struct

SEGMENTS = 64

_BASIS = {
    "z": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    "x": ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
    "y": ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
}

_BOX_SIGNS = (
    (-1.0, -1.0, -1.0), (1.0, -1.0, -1.0), (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0),
    (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0), (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
)
# 每个面两个三角形，绕向使法向朝外
_BOX_FACES = (
    (0, 3, 2), (0, 2, 1),      # z-
    (4, 5, 6), (4, 6, 7),      # z+
    (0, 1, 5), (0, 5, 4),      # y-
    (3, 7, 6), (3, 6, 2),      # y+
    (0, 4, 7), (0, 7, 3),      # x-
    (1, 2, 6), (1, 6, 5),      # x+
)


def _local_to_world(axis, center, lx, ly, lz):
    e1, e2, e3 = _BASIS[axis]
    return (
        center[0] + lx * e1[0] + ly * e2[0] + lz * e3[0],
        center[1] + lx * e1[1] + ly * e2[1] + lz * e3[1],
        center[2] + lx * e1[2] + ly * e2[2] + lz * e3[2],
    )


def _box_vertices(center, half, basis=None):
    if basis is None:
        basis = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    ex, ey, ez = basis
    hx, hy, hz = half
    cx, cy, cz = center
    verts = []
    for sx, sy, sz in _BOX_SIGNS:
        verts.append((
            cx + sx * hx * ex[0] + sy * hy * ey[0] + sz * hz * ez[0],
            cy + sx * hx * ex[1] + sy * hy * ey[1] + sz * hz * ez[1],
            cz + sx * hx * ex[2] + sy * hy * ey[2] + sz * hz * ez[2],
        ))
    return verts


def _box_mesh(center, half, basis=None):
    return _box_vertices(center, half, basis), list(_BOX_FACES)


def _cylinder_mesh(center, radius, length, axis, segments):
    hz = length / 2.0
    verts = []
    for z in (-hz, hz):
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            verts.append(_local_to_world(axis, center, radius * math.cos(a),
                                         radius * math.sin(a), z))
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        bi, bj, ti, tj = i, j, segments + i, segments + j
        faces.append((bi, bj, tj))
        faces.append((bi, tj, ti))
    cb = len(verts)
    verts.append(_local_to_world(axis, center, 0.0, 0.0, -hz))
    ct = len(verts)
    verts.append(_local_to_world(axis, center, 0.0, 0.0, hz))
    for i in range(segments):
        j = (i + 1) % segments
        faces.append((cb, j, i))                          # 底盖，法向 -轴向
        faces.append((ct, segments + i, segments + j))    # 顶盖，法向 +轴向
    return verts, faces


def _tube_mesh(center, radius, inner_radius, length, axis, segments):
    hz = length / 2.0
    verts = []
    # 外下、外上、内下、内上，各 segments 个
    for r in (radius, inner_radius):
        for z in (-hz, hz):
            for i in range(segments):
                a = 2.0 * math.pi * i / segments
                verts.append(_local_to_world(axis, center, r * math.cos(a),
                                             r * math.sin(a), z))
    ob, ot, ib, it = 0, segments, 2 * segments, 3 * segments
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        # 外壁朝外
        faces.append((ob + i, ob + j, ot + j))
        faces.append((ob + i, ot + j, ot + i))
        # 内壁朝内
        faces.append((ib + i, it + i, it + j))
        faces.append((ib + i, it + j, ib + j))
        # 顶环朝 +轴向
        faces.append((ot + i, ot + j, it + j))
        faces.append((ot + i, it + j, it + i))
        # 底环朝 -轴向
        faces.append((ob + i, ib + i, ib + j))
        faces.append((ob + i, ib + j, ob + j))
    return verts, faces


def _leaf_mesh(s):
    (x0, z0), (x1, z1) = s.p0, s.p1
    dx, dz = x1 - x0, z1 - z0
    length = math.hypot(dx, dz)
    if length == 0.0:
        raise ValueError(f"{s.name}: leaf 长度为零")
    ux, uz = dx / length, dz / length
    # e1 沿杆轴，e2 沿宽度(Y)，e3 垂直片面向外，构成右手基
    basis = ((ux, 0.0, uz), (0.0, 1.0, 0.0), (-uz, 0.0, ux))
    center = ((x0 + x1) / 2.0, s.pos[1], (z0 + z1) / 2.0)
    half = (length / 2.0, s.xsec[1] / 2.0, s.xsec[0] / 2.0)
    return _box_mesh(center, half, basis)


def _mesh_for(s, segments):
    if s.kind == "box":
        return _box_mesh(s.pos, tuple(v / 2.0 for v in s.size))
    if s.kind == "cylinder":
        return _cylinder_mesh(s.pos, s.radius, s.length, s.axis, segments)
    if s.kind == "tube":
        return _tube_mesh(s.pos, s.radius, s.inner_radius, s.length, s.axis,
                          segments)
    if s.kind == "leaf":
        return _leaf_mesh(s)
    raise ValueError(f"{s.name}: 未知 kind {s.kind}")


def solid_triangles(s, segments=SEGMENTS):
    """返回一个 Solid 的三角形列表，每个元素是三个世界坐标顶点。"""
    verts, faces = _mesh_for(s, segments)
    return [(verts[a], verts[b], verts[c]) for a, b, c in faces]


def solids_triangles(solids, segments=SEGMENTS):
    tris = []
    for s in solids:
        tris.extend(solid_triangles(s, segments))
    return tris


def triangle_normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    mag = math.sqrt(nx * nx + ny * ny + nz * nz)
    if mag == 0.0:
        return (0.0, 0.0, 0.0)
    return (nx / mag, ny / mag, nz / mag)


def mesh_stats(tris):
    """三角形数、包围盒、表面积、体积（散度定理）。单位 mm / mm^2 / mm^3。"""
    if not tris:
        return {"triangles": 0, "bbox": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                "area": 0.0, "volume": 0.0}
    xmin = ymin = zmin = float("inf")
    xmax = ymax = zmax = float("-inf")
    area = 0.0
    volume = 0.0
    for a, b, c in tris:
        for p in (a, b, c):
            xmin, ymin, zmin = min(xmin, p[0]), min(ymin, p[1]), min(zmin, p[2])
            xmax, ymax, zmax = max(xmax, p[0]), max(ymax, p[1]), max(zmax, p[2])
        cx = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
        cy = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2])
        cz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        area += 0.5 * math.sqrt(cx * cx + cy * cy + cz * cz)
        volume += (a[0] * (b[1] * c[2] - b[2] * c[1])
                   + a[1] * (b[2] * c[0] - b[0] * c[2])
                   + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return {
        "triangles": len(tris),
        "bbox": ((xmin, ymin, zmin), (xmax, ymax, zmax)),
        "area": area,
        "volume": volume,
    }


def _write_binary(name, tris, path):
    header = f"MST-01 STL, unit=mm, right-hand normals, name={name}"
    raw = header.encode("ascii", "replace")[:80]
    raw = raw + b" " * (80 - len(raw))
    with open(path, "wb") as fh:
        fh.write(raw)
        fh.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            nx, ny, nz = triangle_normal(a, b, c)
            fh.write(struct.pack("<12fH", nx, ny, nz,
                                 a[0], a[1], a[2], b[0], b[1], b[2],
                                 c[0], c[1], c[2], 0))


def _write_ascii(name, tris, path):
    lines = [f"solid {name}"]
    for a, b, c in tris:
        nx, ny, nz = triangle_normal(a, b, c)
        lines.append(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}")
        lines.append("    outer loop")
        for p in (a, b, c):
            lines.append(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {name}")
    with open(path, "w", encoding="ascii") as fh:
        fh.write("\n".join(lines) + "\n")


def write_stl(solids, path, name=None, ascii=False, segments=SEGMENTS):
    """把一个或多个 Solid 写成 STL，返回统计信息字典。

    返回包含 triangles、bbox、area、volume、bytes、path、format。
    """
    tris = solids_triangles(solids, segments)
    stats = mesh_stats(tris)
    if name is None:
        name = "MST-01"
    if ascii:
        _write_ascii(name, tris, path)
        fmt = "ascii"
    else:
        _write_binary(name, tris, path)
        fmt = "binary"
    import os
    stats["path"] = path
    stats["name"] = name
    stats["format"] = fmt
    stats["bytes"] = os.path.getsize(path)
    return stats


def print_export_report(per_part, merged):
    """打印每个导出文件的字节数、三角形数与包围盒。"""
    print("  STL 导出（单位 mm，右手法向）")
    for r in per_part:
        lo, hi = r["bbox"]
        print(f"    {r['solid']}.stl  {r['bytes']} B  {r['triangles']} tri  "
              f"bbox [{lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f}].."
              f"[{hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f}]")
    lo, hi = merged["bbox"]
    print(f"    合并 {merged['name']}.stl  {merged['bytes']} B  "
          f"{merged['triangles']} tri  bbox "
          f"[{lo[0]:.2f},{lo[1]:.2f},{lo[2]:.2f}].."
          f"[{hi[0]:.2f},{hi[1]:.2f},{hi[2]:.2f}]")


def export_parts(solids, outdir, name, ascii=False, segments=SEGMENTS):
    """每个 Solid 写一个 STL，再写一个整装合并 STL。

    返回 (per_part_stats, merged_stats)。文件名取零件名。目录不存在就建。
    """
    import os
    os.makedirs(outdir, exist_ok=True)
    per_part = []
    for s in solids:
        path = os.path.join(outdir, f"{s.name}.stl")
        stats = write_stl([s], path, name=s.name, ascii=ascii,
                          segments=segments)
        stats["solid"] = s.name
        stats["analytic_volume"] = s.unit_volume() * s.count
        per_part.append(stats)
    merged_path = os.path.join(outdir, f"{name}_assembly.stl")
    mstats = write_stl(solids, merged_path, name=f"{name}_assembly",
                       ascii=ascii, segments=segments)
    return per_part, mstats


def read_stl(path):
    """读回 STL，返回 (三角形列表, 存储法向列表)。自动区分二进制与 ASCII。"""
    with open(path, "rb") as fh:
        head = fh.read(5)
        fh.seek(0)
        if head == b"solid":
            data = fh.read()
        else:
            data = None
    if data is not None and b"facet" in data[:4096]:
        return _read_ascii(data)
    return _read_binary(path)


def _read_binary(path):
    with open(path, "rb") as fh:
        fh.read(80)
        n = struct.unpack("<I", fh.read(4))[0]
        tris = []
        normals = []
        for _ in range(n):
            vals = struct.unpack("<12fH", fh.read(50))
            normals.append((vals[0], vals[1], vals[2]))
            tris.append((vals[3:6], vals[6:9], vals[9:12]))
    return tris, normals


def _read_ascii(data):
    text = data.decode("ascii", "replace")
    tris = []
    normals = []
    cur = []
    cur_n = None
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "facet":
            cur_n = (float(parts[2]), float(parts[3]), float(parts[4]))
        elif parts[0] == "vertex":
            cur.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif parts[0] == "endfacet":
            if len(cur) == 3:
                tris.append(tuple(cur))
                normals.append(cur_n)
            cur = []
    return tris, normals
