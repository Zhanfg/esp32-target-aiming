"""angle_utils 单元测试：归一化、最短角差、移动、夹紧。

覆盖环绕模式（wrap_360=True）与线性模式（wrap_360=False）的语义差异：环绕模式取最短路径、
可跨 ±180°，线性模式不绕远路、不越过目标。
"""

from __future__ import annotations

import numpy as np
import pytest

from aim.angle_utils import (
    clamp,
    move_toward,
    normalize_180,
    normalize_to_range,
    shortest_delta_deg,
    wrap_360,
)


@pytest.mark.parametrize(
    "value, expected",
    [
        (0.0, 0.0),
        (45.0, 45.0),
        (180.0, 180.0),      # 约定 +180 保留
        (-180.0, 180.0),     # -180 与 +180 等价，映射到 +180
        (360.0, 0.0),
        (-360.0, 0.0),
        (359.0, -1.0),
        (-359.0, 1.0),
        (540.0, 180.0),
        (720.0, 0.0),
        (-540.0, 180.0),
        (12345.0, np.mod(12345.0, 360.0) if np.mod(12345.0, 360.0) <= 180 else np.mod(12345.0, 360.0) - 360.0),
    ],
)
def test_normalize_180(value, expected):
    assert normalize_180(value) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize(
    "value, expected",
    [
        (0.0, 0.0),
        (360.0, 0.0),
        (-90.0, 270.0),
        (450.0, 90.0),
        (-360.0, 0.0),
        (100000.0, float(np.mod(100000.0, 360.0))),
    ],
)
def test_wrap_360(value, expected):
    assert wrap_360(value) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize(
    "value, lo, hi, expected",
    [
        (370.0, 0.0, 360.0, 10.0),
        (-10.0, 0.0, 360.0, 350.0),
        (190.0, -180.0, 180.0, -170.0),
        (-190.0, -180.0, 180.0, 170.0),
        (180.0, -180.0, 180.0, -180.0),  # 半开区间 [lo, hi)
        (10.0, -10.0, 10.0, -10.0),
        (0.0, -90.0, 90.0, 0.0),
    ],
)
def test_normalize_to_range(value, lo, hi, expected):
    assert normalize_to_range(value, lo, hi) == pytest.approx(expected, abs=1e-12)


def test_shortest_delta_wrap_crosses_pm180():
    """环绕模式下跨 ±180 边界取最短路径。"""
    assert shortest_delta_deg(170.0, -170.0, wrap_360=True) == pytest.approx(20.0)
    assert shortest_delta_deg(-170.0, 170.0, wrap_360=True) == pytest.approx(-20.0)
    assert shortest_delta_deg(350.0, 10.0, wrap_360=True) == pytest.approx(20.0)
    assert shortest_delta_deg(10.0, 350.0, wrap_360=True) == pytest.approx(-20.0)
    assert shortest_delta_deg(0.0, 270.0, wrap_360=True) == pytest.approx(-90.0)
    assert shortest_delta_deg(0.0, 180.0, wrap_360=True) == pytest.approx(180.0)


def test_shortest_delta_linear_does_not_wrap():
    """线性模式下角差就是 b - a，不取最短路径。"""
    assert shortest_delta_deg(170.0, -170.0, wrap_360=False) == pytest.approx(-340.0)
    assert shortest_delta_deg(-170.0, 170.0, wrap_360=False) == pytest.approx(340.0)
    assert shortest_delta_deg(-10.0, 10.0, wrap_360=False) == pytest.approx(20.0)


def test_move_toward_wrap_takes_short_path():
    """环绕模式下 350 -> 10 沿最短路径朝 + 方向走，5 度到 355。"""
    assert move_toward(350.0, 10.0, 5.0, wrap_360=True) == pytest.approx(355.0)
    assert move_toward(350.0, 10.0, 15.0, wrap_360=True) == pytest.approx(5.0)
    # 一步跨过回绕点
    assert move_toward(0.0, 270.0, 100.0, wrap_360=True) == pytest.approx(270.0)
    # 结果始终回绕到 [0, 360)
    out = move_toward(359.0, 1.0, 5.0, wrap_360=True)
    assert 0.0 <= out < 360.0


def test_move_toward_linear_never_overshoots():
    """线性模式不越过目标，也不绕远路。"""
    assert move_toward(0.0, 10.0, 15.0, wrap_360=False) == pytest.approx(10.0)
    assert move_toward(0.0, -10.0, 3.0, wrap_360=False) == pytest.approx(-3.0)
    assert move_toward(-10.0, 0.0, 3.0, wrap_360=False) == pytest.approx(-7.0)
    # 线性模式下 170 -> -170 只能朝 -340 的方向走，每步 -5
    assert move_toward(170.0, -170.0, 5.0, wrap_360=False) == pytest.approx(165.0)


def test_move_toward_rejects_negative_step():
    with pytest.raises(ValueError):
        move_toward(0.0, 10.0, -1.0, wrap_360=False)


@pytest.mark.parametrize(
    "value, lo, hi, expected",
    [
        (5.0, 0.0, 10.0, 5.0),
        (-1.0, 0.0, 10.0, 0.0),
        (11.0, 0.0, 10.0, 10.0),
        (0.0, 0.0, 10.0, 0.0),
        (10.0, 0.0, 10.0, 10.0),
    ],
)
def test_clamp(value, lo, hi, expected):
    assert clamp(value, lo, hi) == pytest.approx(expected)
