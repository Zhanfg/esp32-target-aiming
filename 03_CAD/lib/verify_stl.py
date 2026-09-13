"""STL 几何校验：读回导出文件，与解析体积对照。

对 A/B/C 三级各自读完 exports/ 下的 STL，做这些事：
- 统计三角形数、包围盒、总表面积、体积（散度定理按三角形求和）；
- 与 checks.py 的解析体积对比，偏差超过 5% 报出来；
- 检查零面积三角形与法向异常（存储法向与顶点右手定则算出的法向是否一致）；
- 检查流形：闭合可定向面每条边应恰好被两个三角形反向共用，出现边界边或
  同向重复边都报警；
- 打印一张对照表。

解析体积由对应 build.py 的 build_all() 重建，保证和导出时同一套几何。运行：
    python lib/verify_stl.py            # A/B/C 全跑
    python lib/verify_stl.py A C        # 指定级
"""

from __future__ import annotations

import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from lib import checks, stl  # noqa: E402

TOL = 0.05          # 体积偏差阈值 5%
ZERO_AREA = 1.0e-9  # mm^2
NORMAL_DOT = 0.9    # 存储法向与现算法向的点积下限


def _load_level(level):
    path = os.path.join(_ROOT, f"MST-01{level}", "build.py")
    spec = importlib.util.spec_from_file_location(f"mst_build_{level}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _manifold(tris, nd=4):
    edge = {}
    for a, b, c in tris:
        for p, q in ((a, b), (b, c), (c, a)):
            k1 = (round(p[0], nd), round(p[1], nd), round(p[2], nd))
            k2 = (round(q[0], nd), round(q[1], nd), round(q[2], nd))
            key = tuple(sorted((k1, k2)))
            edge.setdefault(key, []).append((k1, k2))
    boundary = nonmanifold = flipped = 0
    for dirs in edge.values():
        if len(dirs) == 1:
            boundary += 1
        elif len(dirs) > 2:
            nonmanifold += 1
        elif dirs[0] == dirs[1]:
            flipped += 1
    return boundary, nonmanifold, flipped


def _inspect(path):
    tris, normals = stl.read_stl(path)
    stats = stl.mesh_stats(tris)
    zero = 0
    bad_norm = 0
    for (a, b, c), n in zip(tris, normals):
        nx, ny, nz = stl.triangle_normal(a, b, c)
        if nx == 0.0 and ny == 0.0 and nz == 0.0:
            zero += 1
            continue
        if n is not None:
            ln = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5
            dot = nx * n[0] + ny * n[1] + nz * n[2]
            if abs(ln - 1.0) > 1.0e-3 or (ln > 0 and dot / ln < NORMAL_DOT):
                bad_norm += 1
    boundary, nonmanifold, flipped = _manifold(tris)
    stats.update(zero_area=zero, bad_normal=bad_norm, boundary=boundary,
                 nonmanifold=nonmanifold, flipped=flipped, path=path)
    return stats


def _bbox_str(bb):
    lo, hi = bb
    return "[%.2f,%.2f,%.2f]..[%.2f,%.2f,%.2f]" % (
        lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])


def _row(name, avol, stats, dev=None):
    if dev is None:
        sv = stats["volume"]
        dev = (sv - avol) / avol * 100.0 if avol else 0.0
    flag = "  <== 超 5%" if abs(dev) > TOL * 100.0 else ""
    print("  %-30s %11.2f %11.2f %+8.3f%% %7d  %s%s"
          % (name, avol, stats["volume"], dev, stats["triangles"],
             _bbox_str(stats["bbox"]), flag))


def verify_level(level):
    mod = _load_level(level)
    parts = mod.build_all()
    outdir = os.path.join(_ROOT, f"MST-01{level}", "exports")
    print("==== MST-01%s STL 校验（单位 mm）====" % level)
    header = "  %-30s %11s %11s %9s %7s  %s" % (
        "零件名", "解析体积", "STL体积", "偏差", "三角形", "包围盒")
    print(header)

    analytic = {}
    for s in parts:
        analytic[s.name] = analytic.get(s.name, 0.0) + s.unit_volume() * s.count

    total_analytic = checks.total_volume(parts)
    bad = []
    missing = []
    zero_total = badnorm_total = 0
    part_boundary = part_nonman = part_flip = 0
    tri_total = 0
    for s in parts:
        path = os.path.join(outdir, f"{s.name}.stl")
        if not os.path.exists(path):
            missing.append(s.name)
            continue
        st = _inspect(path)
        _row(s.name, analytic[s.name], st)
        tri_total += st["triangles"]
        zero_total += st["zero_area"]
        badnorm_total += st["bad_normal"]
        part_boundary += st["boundary"]
        part_nonman += st["nonmanifold"]
        part_flip += st["flipped"]
        if abs(st["volume"] - analytic[s.name]) > TOL * analytic[s.name]:
            bad.append(s.name)

    merged_path = os.path.join(outdir, f"MST-01{level}_assembly.stl")
    merged = None
    if os.path.exists(merged_path):
        merged = _inspect(merged_path)
        _row(f"<合并> MST-01{level}_assembly", total_analytic, merged)
        if abs(merged["volume"] - total_analytic) > TOL * total_analytic:
            bad.append("assembly")

    n_files = len(parts)
    print("  零件文件 %d 个（缺 %d），合并文件 %s"
          % (n_files, len(missing), "有" if merged else "无"))
    print("  逐件三角形合计 %d，零面积 %d，法向异常 %d"
          % (tri_total, zero_total, badnorm_total))
    print("  逐件流形：边界边 %d，非流形边 %d，同向重复边 %d"
          % (part_boundary, part_nonman, part_flip))
    if merged:
        print("  合并文件流形：边界边 %d，非流形边 %d，同向重复边 %d"
              % (merged["boundary"], merged["nonmanifold"], merged["flipped"]))
        if merged["nonmanifold"] or merged["boundary"]:
            print("  合并件只是看装配，件与件贴合面共面会产生非流形边，"
                  "打印请用逐件文件")
    if missing:
        print("  缺失文件：" + "，".join(missing))
    if bad:
        print("  体积偏差超 5%%：" + "，".join(bad))
    # 合并件的非流形边来自零件贴合，不作为失败项；逐件必须是闭合流形
    ok = (not bad and not missing and zero_total == 0
          and badnorm_total == 0 and part_boundary == 0
          and part_nonman == 0 and part_flip == 0)
    print("  结论：%s" % ("通过" if ok else "有问题"))
    print()
    return {
        "level": level, "ok": ok, "bad": bad, "missing": missing,
        "zero": zero_total, "bad_normal": badnorm_total,
        "part_boundary": part_boundary, "part_nonmanifold": part_nonman,
        "part_flipped": part_flip,
        "merged": merged,
        "analytic_total": total_analytic,
        "merged_volume": merged["volume"] if merged else None,
    }


def main():
    args = [a.upper() for a in sys.argv[1:] if a.upper() in ("A", "B", "C")]
    levels = args or ["A", "B", "C"]
    results = [verify_level(lv) for lv in levels]
    print("==== 汇总 ====")
    for r in results:
        mv = r["merged_volume"]
        dev = ((mv - r["analytic_total"]) / r["analytic_total"] * 100.0
               if mv else float("nan"))
        print("  MST-01%s  解析 %.1f mm^3  整装 STL %.1f mm^3  偏差 %+.3f%%  %s"
              % (r["level"], r["analytic_total"], mv, dev,
                 "通过" if r["ok"] else "有问题"))
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
