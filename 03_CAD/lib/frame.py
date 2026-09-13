"""FRAME 模块几何。

框架是一个敞口槽型：后盖板、底板、两片侧壁，前端由膜片法兰闭环。顶部敞开，
方便从上方拍照看 snap 过程（§22.2）。侧壁沿 Z 连续，X 向开口落在拱跨方向两端，
不影响观察顶点竖直运动。

坐标：Z 轴向，X 拱跨，Y 竖直。返回 params.Solid 列表，尺寸单位 mm。
"""

from __future__ import annotations

from . import params as P


def build(level: str) -> list:
    lay = P.layout(level)
    wall_z0 = lay["wall_z0"]
    wall_z1 = lay["wall_z1"]
    wall_len = wall_z1 - wall_z0
    wall_mid = (wall_z0 + wall_z1) / 2.0
    wall_x = P.FRAME_W / 2.0 - P.FRAME_WALL / 2.0
    base_y = -(P.FRAME_H / 2.0 - P.FRAME_BASE_T / 2.0)
    parts = []

    # 后盖板
    parts.append(P.box("FRAME_rear_plate", "FRAME", "petg",
                       (P.FRAME_W, P.FRAME_H, P.FRAME_REAR_T),
                       (0.0, 0.0, P.FRAME_REAR_T / 2.0),
                       note="承载后盖，A/B/C 通用"))
    # 侧壁两片
    parts.append(P.box("FRAME_wall_negx", "FRAME", "petg",
                       (P.FRAME_WALL, P.FRAME_H, wall_len),
                       (-wall_x, 0.0, wall_mid),
                       note="顶部敞开以便观察"))
    parts.append(P.box("FRAME_wall_posx", "FRAME", "petg",
                       (P.FRAME_WALL, P.FRAME_H, wall_len),
                       (wall_x, 0.0, wall_mid)))
    # 底板
    parts.append(P.box("FRAME_base_plate", "FRAME", "petg",
                       (P.FRAME_W, P.FRAME_BASE_T, wall_len),
                       (0.0, base_y, wall_mid),
                       note="框架底面，可贴测试台"))
    # 前端压边，把膜片法兰夹在中间
    parts.append(P.box("FRAME_front_rim", "FRAME", "petg",
                       (P.FRAME_W, P.FRAME_H, P.FRAME_RIM_T),
                       (0.0, 0.0, wall_z1 + P.FRAME_RIM_T / 2.0),
                       note="§6.3 可拆压环的外压边"))
    # 后盖安装凸台
    parts.append(P.cyl("FRAME_boss_neg", "FRAME", "petg",
                       P.FRAME_BOSS_D / 2.0, P.FRAME_BOSS_H, "z",
                       (-P.FRAME_W / 4.0, -P.FRAME_H / 4.0, P.FRAME_REAR_T + P.FRAME_BOSS_H / 2.0),
                       note="M3 量级安装点"))
    parts.append(P.cyl("FRAME_boss_pos", "FRAME", "petg",
                       P.FRAME_BOSS_D / 2.0, P.FRAME_BOSS_H, "z",
                       (P.FRAME_W / 4.0, -P.FRAME_H / 4.0, P.FRAME_REAR_T + P.FRAME_BOSS_H / 2.0)))
    return parts
