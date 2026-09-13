"""BOUNDARY_MODULE：可机械切换的边界状态（§5.4、§13 D 级）。

原理
----
失稳单元的两端底夹块改成坐在一块 Z 向浮动的托架里，不再直接锁死在框架上。托架
两侧沿框架导轨滑动，顶面留两个销孔。两根 Y 向锁销从框架顶部插下，穿过销孔把
托架压死在框架上，此时边界接近固定（locked，高刚度）。把连成一体的拨杆板向上
提起，两销退出托架，托架只剩一根低刚度回复弹簧与一个硬止挡决定边界，此时边界
接近滑动（released，低刚度）。两种状态的等效边界刚度见 params.BND_K_BASE。

可逆与可复现
------------
切换只提放拨杆，没有一次性拆装。两个位置各由硬限位确定：locked 时拨杆板落座在
框架顶面，销的插入深度由销头台肩卡住；released 时拨杆板被一对定位柱塞顶在上
止点，托架行程由止挡块限定为 params.BND_TRAVEL。因此状态不靠摩擦力维持，反复
切换后位置一致，拨杆升起高度又是肉眼可见的状态指示，高速视频能同时拍到拨杆与
顶点运动（§22.2）。销孔中心到托架中心的相对尺寸固定，销只要插到底就是 locked。

本模块只表达保持与放开两个功能，不表达摩擦与弹簧刚度；边界刚度与阻尼都要实测。
"""

from __future__ import annotations

from . import params as P


def stiffness_note(state: str) -> str:
    """给定状态的等效边界刚度说明，N/m。"""
    k = P.BND_K_BASE[state]
    return (f"{state} 边界刚度名义 {k:.4g} N/m，"
            f"locked 为刚性支路、released 为弹簧支路（待实测标定）")


def build(z0: float, z1: float, state: str = "locked") -> list:
    state = state.lower()
    if state not in P.BND_STATES:
        raise ValueError(f"边界状态只能是 {P.BND_STATES}: {state}")
    zc = (z0 + z1) / 2.0
    pin_y = P.BND_PIN_Y[state]
    lever_y = P.BND_LEVER_Y[state]
    parts = []

    # 浮动托架：正常情况下失稳单元底夹块坐在它上面
    parts.append(P.box("BOUNDARY_carriage", "BOUNDARY_MODULE", "petg",
                       P.BND_CARRIAGE, (0.0, 0.0, zc),
                       note="Z 向浮动托架，承载第一级底夹块"))
    # 两根 Y 向导柱，固定在框架上，给托架做面外导向
    for sx, tag in ((-1.0, "negx"), (1.0, "posx")):
        parts.append(P.cyl(f"BOUNDARY_guide_post_{tag}", "BOUNDARY_MODULE",
                           "steel301", P.BND_GUIDE_POST_D / 2.0,
                           P.BND_GUIDE_POST_L, "y",
                           (sx * 6.0, 0.0, zc),
                           note="面外导向柱，托架只沿 Z 滑动"))
    # 两根 Y 向锁销，插到底即 locked，抬起即 released
    for sx, tag in ((-1.0, "negx"), (1.0, "posx")):
        parts.append(P.cyl(f"BOUNDARY_lock_pin_{tag}", "BOUNDARY_MODULE",
                           "steel301", P.BND_LOCK_PIN_D / 2.0,
                           P.BND_LOCK_PIN_L, "y",
                           (sx * 6.0, pin_y, zc),
                           note=f"{state} 状态锁销，销头台肩限定插入深度"))
    # 拨杆板把两销连成一体，升降高度就是状态指示
    parts.append(P.box("BOUNDARY_lock_lever", "BOUNDARY_MODULE", "petg",
                       P.BND_LEVER, (0.0, lever_y, zc),
                       note=f"{state} 状态指示拨杆，顶部敞开可见"))
    # 定位柱塞，两个位置各一个硬止点
    parts.append(P.cyl("BOUNDARY_detent_plunger", "BOUNDARY_MODULE", "steel301",
                       P.BND_DETENT_D / 2.0, P.BND_DETENT_L, "y",
                       (0.0, 4.0, zc),
                       note="弹簧柱塞，给 locked/released 两个确定位置"))
    # 边界回复弹簧，released 时决定低刚度支路
    parts.append(P.cyl("BOUNDARY_return_spring", "BOUNDARY_MODULE", "steel301",
                       P.BND_RETURN_SPRING_D / 2.0, P.BND_RETURN_SPRING_L, "z",
                       (0.0, 4.0, zc),
                       note="低刚度支路弹簧，与 locked 刚性支路二选一"))
    # 行程止挡块，限定 released 状态托架 Z 向行程
    parts.append(P.box("BOUNDARY_stop_block", "BOUNDARY_MODULE", "petg",
                       P.BND_STOP_BLOCK, (0.0, 0.0, z1 + 1.0),
                       note=f"released 行程硬止挡，行程 {P.BND_TRAVEL:.1f} mm"))
    return parts
