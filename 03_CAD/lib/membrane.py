"""DIAPHRAGM_MODULE：膜片与可拆压环。

§6.3 要求可拆压环，不用永久胶粘。这里用前后两片环形压环把膜片夹在中间，四颗 M2
螺钉压紧。松开螺钉即可换膜，不需要拆框架（§22.2）。膜片夹持半径 8 mm，通径 16 mm，
压环外径 22 mm 落在 §1.2 横向 15-25 mm 内。

膜片有两种候选（§6.2）：薄硅胶膜与 PET 膜，厚度见 params.MEMB_DISC_THICKNESS。
两种都能装进同一副压环，便于 A/B 对照。
"""

from __future__ import annotations

import math

from . import params as P


def build(z0: float, z1: float, material: str = "silicone") -> list:
    mid = (z0 + z1) / 2.0
    ring_half = P.MEMB_RING_T / 2.0
    thick = P.MEMB_DISC_THICKNESS[material]
    parts = []

    # 膜片本体，裁切半径略大于夹持半径，压在压环下面
    parts.append(P.cyl("DIAPHRAGM_disc", "DIAPHRAGM_MODULE", material,
                       P.MEMB_DISC_R, thick, "z", (0.0, 0.0, mid),
                       note="§6.2 候选膜片，可整体更换"))
    # 后压环
    parts.append(P.tube("DIAPHRAGM_ring_rear", "DIAPHRAGM_MODULE", "petg",
                        P.MEMB_RING_OD / 2.0, P.MEMB_RING_R_IN, P.MEMB_RING_T,
                        "z", (0.0, 0.0, mid - ring_half),
                        note="§6.3 可拆压环"))
    # 前压环
    parts.append(P.tube("DIAPHRAGM_ring_front", "DIAPHRAGM_MODULE", "petg",
                        P.MEMB_RING_OD / 2.0, P.MEMB_RING_R_IN, P.MEMB_RING_T,
                        "z", (0.0, 0.0, mid + ring_half)))
    # 四颗压紧螺钉，按分布圆布置
    for i in range(P.MEMB_SCREW_N):
        ang = math.radians(45.0 + 90.0 * i)
        parts.append(P.cyl(f"DIAPHRAGM_screw_{i}", "DIAPHRAGM_MODULE",
                           "steel301", P.MEMB_SCREW_D / 2.0, P.MEMB_SCREW_L,
                           "z",
                           (P.MEMB_SCREW_PCD / 2.0 * math.cos(ang),
                            P.MEMB_SCREW_PCD / 2.0 * math.sin(ang), mid),
                           note="M2，压环夹紧"))
    return parts


def membrane_plane(z0: float, z1: float) -> float:
    return (z0 + z1) / 2.0
