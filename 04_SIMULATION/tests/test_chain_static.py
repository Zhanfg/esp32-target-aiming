"""串联链准静态性质测试。"""

import pytest

from models import make_membrane, make_vmt
from models.chain import BASE, GROUND
from models.params import GAP_C, build_B_static, build_C_static
from studies.quasi_static import compress, first_limit, scan_until_limit


def snap_threshold(chain, x_max):
    rec = scan_until_limit(chain, x_max)
    i = first_limit(rec)
    return rec[i]


def test_B_threshold_equals_element_snap_force():
    v = make_vmt()
    rec = snap_threshold(build_B_static(make_membrane("silicone"), v), 14e-3)
    assert rec[2] == pytest.approx(v.snap_force, rel=2e-2)


def test_boundary_stiffness_raises_threshold():
    thresholds = []
    for k_base in [2e3, 1e4, 1e5]:
        v = make_vmt(k_base=k_base)
        thresholds.append(snap_threshold(build_B_static(make_membrane("silicone"), v), 14e-3)[2])
    assert all(b > a for a, b in zip(thresholds, thresholds[1:]))


def test_stiffer_membrane_shortens_displacement_to_snap():
    v = make_vmt()
    soft = snap_threshold(build_B_static(make_membrane("silicone", 0.3), v), 14e-3)
    stiff = snap_threshold(build_B_static(make_membrane("silicone", 3.0), v), 14e-3)
    assert stiff[0] < soft[0]
    # 阈值仍由失稳单元决定
    assert soft[2] == pytest.approx(stiff[2], rel=1e-2)


def test_equal_units_reach_fold_together_statically():
    v1 = make_vmt()
    v2 = make_vmt()
    chain = build_C_static(make_membrane("silicone"), v1, v2, gap=GAP_C)
    rec = snap_threshold(chain, 16e-3)
    c1 = compress(chain.elements[0], rec[1], rec[0])
    c2 = compress(chain.elements[1], rec[1], rec[0])
    assert abs(c1 - c2) < 0.05e-3
    assert c1 == pytest.approx(v1.snap_stroke, abs=0.05e-3)


def test_larger_gap_delays_first_limit():
    positions = []
    for gap in [0.2e-3, 0.6e-3, 1.5e-3]:
        chain = build_C_static(make_membrane("silicone"), make_vmt(), make_vmt(), gap=gap)
        rec = snap_threshold(chain, 22e-3)
        positions.append(rec[0])
    assert all(b > a for a, b in zip(positions, positions[1:]))
