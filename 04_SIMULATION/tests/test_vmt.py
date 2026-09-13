"""von Mises truss 模型的性质测试。"""

import math

import pytest

from models.params import make_vmt


def test_initial_height_from_geometry():
    v = make_vmt()
    assert v.h0 == pytest.approx(math.sqrt(v.L**2 - v.a**2))
    assert v.h0 < v.L


def test_zero_force_at_both_wells():
    v = make_vmt()
    assert v.element_force(0.0) == pytest.approx(0.0, abs=1e-6)
    assert v.element_force(2 * v.h0) == pytest.approx(0.0, abs=1e-6)
    assert v.element_force(v.h0) == pytest.approx(0.0, abs=1e-6)


def test_energy_has_two_wells_and_barrier():
    v = make_vmt()
    e_well1 = v.element_energy(0.0)
    e_well2 = v.element_energy(2 * v.h0)
    e_barrier = v.element_energy(v.h0)
    assert e_well1 == pytest.approx(0.0, abs=1e-9)
    assert e_well2 == pytest.approx(0.0, abs=1e-9)
    assert e_barrier > e_well1
    assert e_barrier > e_well2


def test_stiffness_sign_at_well_and_barrier():
    v = make_vmt(k_base=1e4)
    assert v.element_stiffness(0.0) > 0
    assert v.element_stiffness(v.h0) < 0


def test_snap_stroke_between_well_and_barrier():
    v = make_vmt()
    assert 0.0 < v.snap_stroke < v.h0
    assert v.snap_force > 0.0
    # 峰值处力确实最大
    assert v.element_force(v.snap_stroke) >= v.element_force(0.5 * v.snap_stroke)


def test_snap_force_monotonic_in_bar_stiffness():
    forces = [make_vmt(k_bar=k, k_base=1e4).snap_force for k in [500, 1000, 2000, 4000]]
    assert all(b > a for a, b in zip(forces, forces[1:]))


def test_snap_force_monotonic_in_boundary_stiffness():
    forces = [make_vmt(k_base=k).snap_force for k in [2e2, 1e3, 1e4, 1e5]]
    assert all(b > a for a, b in zip(forces, forces[1:]))


def test_released_boundary_reduces_snap_force():
    soft = make_vmt(k_base=1e2).snap_force
    locked = make_vmt(k_base=1e5).snap_force
    assert soft < locked


def test_energy_derivative_matches_force():
    v = make_vmt(k_base=1e4)
    d = 1e-9
    for c in [1e-4, 1e-3, 8e-3]:
        fd = (v.element_energy(c + d) - v.element_energy(c - d)) / (2 * d)
        assert fd == pytest.approx(v.element_force(c), rel=1e-4, abs=1e-4)
