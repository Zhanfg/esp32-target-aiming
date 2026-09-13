"""MAGAZINE_MODULE：6 / 8 位旋转供给盘与索引机构（§8、§9、§13 E 级）。

布局
----
盘体绕 Z 轴，盘轴相对炮口轴线在 X 向偏置一个分布圆半径，使正对炮口那个工位的
pocket 落在 x=0。每工位一个 pocket，pocket 数量 N 取 6 或 8。分布圆半径由相邻
座圈不干涉反算：2*R*sin(pi/N) >= 座圈外径 + 间隙，位越多 R 越大，盘径随之变大。
6 位与 8 位的包围盒对比见 build.py 打印结果，选用建议依据 §31。

载荷与 pocket
------------
载荷是 §8.1 的超轻软圆片，一工位一片，参数见 params.MAG_PAYLOAD_*。pocket 做成
环形座圈加中心通孔：薄片平铺时可跨住座圈，硬珠会从中心孔漏下；座圈凸出高度与
上方盖板的固定间隙合起来再限一次厚度，软圆片刚好躺平，硬弹丸要么漏下要么顶住
盖板让索引卡住。这样机械接口不能被无修改地塞入 §8.4 列的 BB、钢珠、玻璃珠、
陶瓷珠、金属柱或飞镖。盖板正对炮口的开口让软圆片被气流带出，开口只比软圆片
外径大一点，示意上以整环表示，故本模块的体积按上界计。

索引与接口
----------
索引棘爪压在座圈之间的分度槽上，盘转一工位由棘爪落槽确认。零位标记是一片随盘
转动的遮光片，MCU 读取它回到 HOME（§9.5）。盘可从中心轴沿 Z 拔出，轮毂上的
不对称键使盘只能按一个方向装回（防呆），装回后由零位标记重新回零。止挡柱与盘
上凸台限定最大转角，防止过转。
"""

from __future__ import annotations

import math

from . import params as P


def pocket_centers(positions: int, z: float) -> list:
    """返回各工位中心 (x, y, z)。第 0 工位固定在炮口轴线 x=y=0。"""
    r = P.mag_pitch_radius(positions)
    axis_x = r
    out = []
    for i in range(positions):
        ang = math.pi + 2.0 * math.pi * i / positions
        out.append((axis_x + r * math.cos(ang), r * math.sin(ang), z))
    return out


def build(z0: float, z1: float, positions: int = 6) -> list:
    if positions not in P.MAG_POSITIONS:
        raise ValueError(f"盘位只能是 {P.MAG_POSITIONS}: {positions}")
    r_pitch = P.mag_pitch_radius(positions)
    r_disc = P.mag_disc_outer_r(positions)
    axis_x = r_pitch
    r_hole = P.MAG_POCKET_HOLE_D / 2.0
    r_seat = P.MAG_POCKET_SEAT_OD / 2.0
    z_plate = z0 + P.MAG_PLATE_T / 2.0
    z_seat = z0 + P.MAG_PLATE_T + P.MAG_POCKET_LIP_H / 2.0
    z_payload = z0 + P.MAG_PLATE_T + P.MAG_POCKET_LIP_H + P.MAG_PAYLOAD_T / 2.0
    z_cover = z0 + P.MAG_PLATE_T + P.MAG_POCKET_LIP_H + P.MAG_PAYLOAD_T \
        + P.MAG_CLEAR + P.MAG_COVER_T / 2.0
    zc = (z0 + z1) / 2.0
    parts = []

    # 盘体：环形盘，中间留轮毂孔
    parts.append(P.tube("MAGAZINE_disc", "MAGAZINE_MODULE", "petg",
                        r_disc, P.MAG_HUB_R, P.MAG_PLATE_T, "z",
                        (axis_x, 0.0, z_plate),
                        note=f"{positions} 位盘体，盘径 {2 * r_disc:.1f} mm"))
    # 工位 pocket：座圈 + 软圆片
    for i, (px, py, _) in enumerate(pocket_centers(positions, z_seat)):
        parts.append(P.tube(f"MAGAZINE_pocket_{i}", "MAGAZINE_MODULE", "petg",
                            r_seat, r_hole, P.MAG_POCKET_LIP_H, "z",
                            (px, py, z_seat),
                            note=f"第 {i} 位环形座圈，中心通孔 {P.MAG_POCKET_HOLE_D:.0f} mm"))
        parts.append(P.cyl(f"MAGAZINE_payload_{i}", "MAGAZINE_MODULE", "epp",
                           P.MAG_PAYLOAD_D / 2.0, P.MAG_PAYLOAD_T, "z",
                           (px, py, z_payload),
                           note="§8.1 超轻软圆片，平铺在座圈上"))
    # 索引盖板：与软圆片顶面留固定间隙，防厚载荷
    parts.append(P.tube("MAGAZINE_cover", "MAGAZINE_MODULE", "petg",
                        r_disc, P.MAG_HUB_R, P.MAG_COVER_T, "z",
                        (axis_x, 0.0, z_cover),
                        note=f"索引盖板，与软圆片顶面固定间隙 {P.MAG_CLEAR:.1f} mm"))
    # 中心轴，盘可沿 Z 拔出（§9.5）
    parts.append(P.cyl("MAGAZINE_axle", "MAGAZINE_MODULE", "steel301",
                       P.MAG_AXLE_D / 2.0, P.MAG_T + 3.0, "z",
                       (axis_x, 0.0, zc),
                       note="中心轴，盘沿 Z 拔出与装回"))
    # 防呆键：轮毂上的不对称凸键，盘只有一个方向能装到底
    parts.append(P.box("MAGAZINE_key_boss", "MAGAZINE_MODULE", "petg",
                       (2.0, 2.0, P.MAG_T - 1.0),
                       (axis_x + P.MAG_HUB_R - 0.5, 0.0, zc),
                       note="不对称防呆键，轮毂键槽缺它才装得进"))
    # 中心轴前端挡圈，限制盘轴向窜动又允许拔出
    parts.append(P.tube("MAGAZINE_retainer", "MAGAZINE_MODULE", "petg",
                        P.MAG_AXLE_D / 2.0 + 1.0, P.MAG_AXLE_D / 2.0, 1.5,
                        "z", (axis_x, 0.0, z1 + 1.0),
                        note="轴端挡圈，轴向定位，拆下即可拔盘"))
    # 索引棘爪，落在工位之间的分度槽上
    parts.append(P.box("MAGAZINE_index_pawl", "MAGAZINE_MODULE", "steel301",
                       P.MAG_PAWL, (axis_x, -r_pitch, z_cover),
                       note="索引棘爪，落槽即一个工位"))
    parts.append(P.cyl("MAGAZINE_index_spring", "MAGAZINE_MODULE", "steel301",
                       P.MAG_INDEX_SPRING_D / 2.0, P.MAG_INDEX_SPRING_L, "y",
                       (axis_x, -r_pitch - 3.0, z_cover),
                       note="棘爪压簧，靠弹力落槽"))
    # 机械止挡柱，配盘上凸台限定最大转角
    parts.append(P.box("MAGAZINE_stop_post", "MAGAZINE_MODULE", "petg",
                       P.MAG_STOP_POST, (axis_x, -(r_pitch + r_seat + 1.0), z_cover),
                       note="止挡柱，与盘上凸台配合防止过转"))
    # 零位标记：随盘转动的遮光片
    parts.append(P.box("MAGAZINE_zero_flag", "MAGAZINE_MODULE", "petg",
                       P.MAG_ZERO_FLAG,
                       (axis_x + r_disc - P.MAG_ZERO_FLAG[0] / 2.0, 0.0, z_plate),
                       note="零位遮光片，MCU 读它回 HOME（§9.5）"))
    return parts


def describe(positions: int) -> str:
    r_pitch = P.mag_pitch_radius(positions)
    r_disc = P.mag_disc_outer_r(positions)
    step = 360.0 / positions
    return (f"{positions} 位：分布圆 R={r_pitch:.2f} mm，盘径 {2 * r_disc:.2f} mm，"
            f"分度角 {step:.1f}°")
