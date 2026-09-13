"""膜片模型的性质测试。"""

import math

import pytest

from models.membrane import Membrane
from models.params import make_membrane


def test_small_deflection_limit_is_linear():
    m = make_membrane("silicone")
    w = 1e-6
    ratio = m.force(w) / (m.k1 * w)
    assert ratio == pytest.approx(1.0, rel=1e-5)


def test_force_is_odd_and_monotonic():
    m = make_membrane("silicone")
    ws = [0.0, 0.5e-3, 1e-3, 2e-3, 4e-3, 6e-3]
    forces = [m.force(w) for w in ws]
    assert all(b > a for a, b in zip(forces, forces[1:]))
    assert m.force(-2e-3) == pytest.approx(-m.force(2e-3), rel=1e-12)


def test_energy_derivative_matches_force():
    m = make_membrane("pet")
    d = 1e-9
    for w in [1e-4, 5e-4, 1e-3]:
        fd = (m.energy(w + d) - m.energy(w - d)) / (2 * d)
        assert fd == pytest.approx(m.force(w), rel=1e-5)


def test_stiffness_increases_with_deflection():
    m = make_membrane("silicone")
    assert m.stiffness(3e-3) > m.stiffness(1e-3) > m.stiffness(0.0)
    assert m.stiffness(0.0) == pytest.approx(m.k1)


def test_volume_displacement_proportional_to_center_deflection():
    m = make_membrane("silicone")
    assert m.volume_displacement(2e-3) == pytest.approx(2 * m.volume_displacement(1e-3))


def test_stiffness_scale_scales_both_terms():
    base = make_membrane("silicone", stiffness_scale=1.0)
    soft = make_membrane("silicone", stiffness_scale=0.01)
    w = 1e-3
    assert soft.force(w) == pytest.approx(0.01 * base.force(w), rel=1e-9)
    assert soft.k1 == pytest.approx(0.01 * base.k1)
    assert soft.mass == pytest.approx(base.mass)


def test_pet_stiffer_than_silicone_at_same_deflection_and_thickness():
    pet = Membrane(**{**{"E": 3.0e9, "thickness": 0.2e-3, "radius": 8e-3,
                         "nu": 0.35, "prestrain": 0.05}}, stiffness_scale=1.0)
    sil = make_membrane("silicone")
    assert pet.force(1e-3) > sil.force(1e-3)
