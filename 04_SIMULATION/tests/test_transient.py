"""瞬态积分的性质测试。"""

import numpy as np
import pytest

from models import make_membrane, make_vmt
from models.params import build_A, build_B, build_C
from models.solver import integrate
from studies.common import run_and_events


def peak_output_velocity(chain, f_max=2.0, t_ramp=0.02, t_end=0.04, dt=1e-6):
    res, events = run_and_events(chain, f_max, t_ramp, t_end, dt, record_every=4)
    return float(np.abs(res["v"][:, chain.n - 1]).max()), events


def test_energy_conserved_without_damping():
    m = make_membrane("silicone")
    ch = build_B(m, make_vmt(), damping=0.0)
    res = integrate(ch, 0.03, 2e-6, lambda t: 0.0, lambda t: 0.0,
                    x0=[2.5e-3, 1.0e-3], v0=[3.0, 0.0])
    tot = res["pe"] + res["ke"]
    drift = (tot.max() - tot.min()) / max(abs(tot[0]), 1e-12)
    assert drift < 1e-3


def test_snap_output_faster_than_direct_push():
    m = make_membrane("silicone")
    v_a, _ = peak_output_velocity(build_A(m))
    v_b, events = peak_output_velocity(build_B(m, make_vmt()))
    assert v_b > 5.0 * v_a
    assert any(e["time"] is not None for e in events)


def test_event_compression_matches_fold_stroke():
    m = make_membrane("silicone")
    v = make_vmt()
    _, events = peak_output_velocity(build_B(m, v))
    ev = events[0]
    assert ev["time"] is not None
    assert abs(ev["compression"] - v.snap_stroke) < 0.3e-3


def test_snap_fires_even_with_very_soft_membrane():
    m = make_membrane("silicone", stiffness_scale=0.02)
    _, events = peak_output_velocity(build_B(m, make_vmt()))
    assert any(e["time"] is not None for e in events)


def test_damping_reduces_post_snap_tail():
    m = make_membrane("silicone")
    v = make_vmt()

    def tail_ratio(c):
        res, events = run_and_events(build_B(m, v, damping=c), 2.0, 0.02, 0.04, 1e-6,
                                     record_every=4)
        tt = res["t"]
        vv = np.abs(res["v"][:, -1])
        t0 = next(e["time"] for e in events if e["time"] is not None)
        mask = (tt >= t0 + 2e-3) & (tt <= t0 + 4e-3)
        return float(np.sqrt(np.mean(vv[mask] ** 2)) / vv.max())

    assert tail_ratio(0.3) < tail_ratio(0.001)


def test_two_stage_produces_two_events():
    m = make_membrane("silicone")
    _, events = peak_output_velocity(build_C(m, make_vmt(), make_vmt()))
    assert sum(1 for e in events if e["time"] is not None) == 2
