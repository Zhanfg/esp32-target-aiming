"""STAGE 模块几何：von Mises truss 失稳单元。

两根等长弹性片从底边 (x=±a, z=z_base) 斜向上汇到顶点 (0, z_base+h0)，形成浅拱。
a=8 mm、L=10 mm 时 h0=6 mm，与 04_SIMULATION/models/vmt.py 的几何一致。

弹性片独立于打印件：两端用夹块与销定位，松开夹块螺钉即可整片抽出更换（§7.1、
§22.2）。打印件只做夹块、滑块与销座。材料可在 PET 与弹簧钢之间切换，厚度取自
params.LEAF_THICKNESS。

输出推力沿 +Z 从顶点滑块传出。向上敞口的方向能看到顶点运动。
"""

from __future__ import annotations

from . import params as P


def build(index: int, z_base: float, leaf_material: str = "pet") -> list:
    a = P.STAGE_HALF_SPAN
    h0 = P.STAGE_APEX_H
    thick = P.LEAF_THICKNESS[leaf_material]
    mod = f"STAGE_{index}"
    parts = []

    # 两片斜置弹性片，独立件，可更换
    parts.append(P.leaf(f"{mod}_leaf_negx", mod, leaf_material,
                        (-a, z_base), (0.0, z_base + h0),
                        thick, P.STAGE_LEAF_DEPTH,
                        note="独立弹性元件，§7.1 不由打印件承担主变形"))
    parts.append(P.leaf(f"{mod}_leaf_posx", mod, leaf_material,
                        (a, z_base), (0.0, z_base + h0),
                        thick, P.STAGE_LEAF_DEPTH))
    # 顶点滑块，把两片合成一个自由度并输出到下一级
    bx, by, bz = P.STAGE_APEX_BLOCK
    parts.append(P.box(f"{mod}_apex_slider", mod, "petg", (bx, by, bz),
                       (0.0, 0.0, z_base + h0 + bz / 2.0),
                       note="顶点滑块，输出 +Z 位移"))
    # 底部夹块，压住弹性片端部
    cx, cy, cz = P.STAGE_BASE_BLOCK
    for sx, tag in ((-1.0, "negx"), (1.0, "posx")):
        parts.append(P.box(f"{mod}_base_clamp_{tag}", mod, "petg",
                           (cx, cy, cz), (sx * a, 0.0, z_base),
                           note="夹块，松开后可换弹性片"))
    # 铰销
    for sx, tag in ((-1.0, "negx"), (1.0, "posx")):
        parts.append(P.cyl(f"{mod}_hinge_pin_{tag}", mod, "steel301",
                           P.STAGE_PIN_D / 2.0, P.STAGE_PIN_LEN, "y",
                           (sx * a, 0.0, z_base),
                           note="定位销，承受面外反力"))
    # 压紧螺钉（每端一颗 M2）
    for sx, tag in ((-1.0, "negx"), (1.0, "posx")):
        parts.append(P.cyl(f"{mod}_clamp_screw_{tag}", mod, "steel301",
                           P.MEMB_SCREW_D / 2.0, P.STAGE_CLAMP_T + 3.0, "y",
                           (sx * a, 0.0, z_base + 0.5),
                           note="M2 夹紧螺钉，弹性件可单独拆换"))
    return parts


def equivalent_stiffness_note() -> str:
    return (f"k_bar={P.STAGE_K_BAR:.4g} N/m, k_base={P.STAGE_K_BASE:.4g} N/m "
            f"(对齐仿真名义值，待实测标定)")
