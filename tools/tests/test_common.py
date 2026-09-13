"""不依赖硬件和 OpenCV 的单元测试：遥测解析、config.h 宏、精度统计、仿射解算。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import common
import analyze_telemetry as at
import calibrate_axis as ca

from aim.geometry import apply_affine
from aim.types import CameraIntrinsics

GOOD_LINE = ("AIM,1234,4,1,100.50,80.25,0.900,1.500,-2.000,1.400,-1.900,"
             "3.000,2.500,-1234,567,8123")
CONFIG_PATH = Path(__file__).resolve().parents[2] / "firmware" / "include" / "config.h"


def _telemetry_row(t_ms, state, obs_valid, px, py, loop_us=2000):
    return {
        "t_ms": float(t_ms), "state": state, "obs_valid": obs_valid,
        "px": float(px), "py": float(py), "conf": 1.0,
        "pan_deg": 0.0, "tilt_deg": 0.0, "pan_target": 0.0, "tilt_target": 0.0,
        "pan_rate": 0.0, "tilt_rate": 0.0, "enc_pan": 0, "enc_tilt": 0,
        "loop_us": loop_us,
    }


# --- 遥测行解析 -------------------------------------------------------------


def test_parse_aim_line_ok():
    row = common.parse_aim_line(GOOD_LINE + "\r\n")
    assert row is not None
    assert row["t_ms"] == pytest.approx(1234.0)
    assert row["state"] == 4 and isinstance(row["state"], int)
    assert row["obs_valid"] == 1
    assert row["px"] == pytest.approx(100.5)
    assert row["enc_pan"] == -1234
    assert row["loop_us"] == 8123
    assert set(row) == set(common.TELEMETRY_FIELDS)


def test_parse_aim_line_rejects_bad_input():
    assert common.parse_aim_line("SYS, boot ok") is None
    assert common.parse_aim_line("AIM,1,2,3") is None
    assert common.parse_aim_line("AIM,1,2,3,4,5,6,7,8,9,10,11,12,13,abc") is None
    assert common.parse_aim_line("AIM,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16") is None
    assert common.parse_aim_line("") is None
    assert common.parse_aim_line("AIM," + ",".join(["x"] * 15)) is None


def test_read_aim_telemetry_counts_skipped(tmp_path):
    path = tmp_path / "run.csv"
    path.write_text(
        "AIM," + ",".join(common.TELEMETRY_FIELDS) + "\n"
        + GOOD_LINE + "\n"
        + "SYS, boot ok\n"
        + "AIM,1,2,3\n",
        encoding="utf-8",
    )
    rows, counts = common.read_aim_telemetry(path)
    assert len(rows) == 1
    assert counts == {"total": 4, "kept": 1, "skipped": 3}


def test_state_name_mapping():
    assert common.state_name(0) == "IDLE"
    assert common.state_name(4) == "LOCKED"
    assert common.state_name(99).startswith("UNKNOWN")


# --- config.h 宏解析 --------------------------------------------------------


def test_parse_real_config_macros():
    macros = common.parse_config_macros(CONFIG_PATH)
    assert macros["VISION_SRC_W"] == pytest.approx(320.0)
    assert macros["VISION_SRC_H"] == pytest.approx(240.0)
    assert macros["PAN_MIN_DEG"] == pytest.approx(-180.0)
    assert macros["TELEMETRY_BAUD"] == pytest.approx(115200.0)
    assert macros["CAM_PIN_PWDN"] == pytest.approx(-1.0)
    assert macros["COUNTS_PER_OUTPUT_REV_PAN"] == pytest.approx(1320.0)
    assert macros["CONTROL_LOOP_BUDGET_MS"] == pytest.approx(10.0)
    assert isinstance(macros["AIM_BUILD_VERSION"], str)
    assert "0.1.0" in macros["AIM_BUILD_VERSION"]
    assert macros["VISION_FRAME_SIZE"] == "FRAMESIZE_QVGA"


def test_parse_config_expression_and_skip_function_like(tmp_path):
    cfg = tmp_path / "mini.h"
    cfg.write_text(
        "#define A 100\n"
        "#define B (A / 4)   // 行尾注释要忽略\n"
        "#define C (B * 2 + 1)\n"
        "#define NAME \"x\"\n"
        "#define F(x) ((x) + 1)\n",
        encoding="utf-8",
    )
    macros = common.parse_config_macros(cfg)
    assert macros["B"] == pytest.approx(25.0)
    assert macros["C"] == pytest.approx(51.0)
    assert macros["NAME"] == '"x"'
    assert "F" not in macros


def test_default_intrinsics_from_config_uses_vision_size():
    intr, source = common.default_intrinsics_from_config(CONFIG_PATH)
    assert intr.width == 320 and intr.height == 240
    assert intr.fx == pytest.approx(256.0)
    assert "占位内参" in source


def test_grade_rmse_bands():
    assert "优秀" in common.grade_rmse(0.1)
    assert "可用" in common.grade_rmse(0.2)
    assert "可用" in common.grade_rmse(0.49)
    assert "偏大" in common.grade_rmse(0.5)
    assert "偏大" in common.grade_rmse(1.5)
    assert "不可用" in common.grade_rmse(1.51)


# --- analyze_telemetry 统计 -------------------------------------------------


def test_steady_error_stats_matches_hand_computed_values():
    intr = CameraIntrinsics.default_for(320, 240)
    offsets = [(1.0, 1.0), (0.0, 0.0), (2.0, 0.0), (-1.0, 2.0)]
    rows = [_telemetry_row(i * 10, 4, 1, intr.cx + dx, intr.cy + dy)
            for i, (dx, dy) in enumerate(offsets)]
    stats = at.steady_error_stats(rows, intr)
    assert stats["n"] == 4
    assert stats["mean_px"] == pytest.approx(1.41257, abs=1e-4)
    assert stats["rms_px"] == pytest.approx(1.65831, abs=1e-4)
    assert stats["p95_px"] == pytest.approx(2.20066, abs=1e-4)
    assert stats["rms_deg"] == pytest.approx(0.37118, abs=2e-3)
    assert stats["p95_deg"] == pytest.approx(0.49262, abs=3e-3)


def test_steady_error_ignores_non_locked_and_invalid():
    intr = CameraIntrinsics.default_for(320, 240)
    rows = [
        _telemetry_row(0, 3, 1, intr.cx + 5, intr.cy),
        _telemetry_row(10, 4, 0, intr.cx + 5, intr.cy),
        _telemetry_row(20, 4, 1, intr.cx, intr.cy),
    ]
    stats = at.steady_error_stats(rows, intr)
    assert stats["n"] == 1
    assert stats["rms_px"] == pytest.approx(0.0)


def test_loop_period_stats():
    rows = [_telemetry_row(0, 4, 1, 160, 120, loop_us=v)
            for v in (2000, 2010, 1990, 5000)]
    stats = at.loop_period_stats(rows)
    assert stats["n"] == 4
    assert stats["mean_us"] == pytest.approx(2750.0)
    assert stats["max_us"] == pytest.approx(5000.0)
    assert stats["min_us"] == pytest.approx(1990.0)


def test_loop_period_stats_skips_zero():
    rows = [_telemetry_row(0, 4, 1, 160, 120, loop_us=0)]
    assert at.loop_period_stats(rows)["n"] == 0


def test_lock_acquisition_and_observation_ratio():
    intr = CameraIntrinsics.default_for(320, 240)
    rows = [
        _telemetry_row(0, 2, 0, intr.cx, intr.cy),
        _telemetry_row(50, 3, 1, intr.cx + 3, intr.cy),
        _telemetry_row(120, 3, 1, intr.cx + 1, intr.cy),
        _telemetry_row(220, 4, 1, intr.cx, intr.cy),
    ]
    assert at.lock_acquisition_ms(rows) == pytest.approx(170.0)
    assert at.lost_observation_ratio(rows) == pytest.approx(0.25)


def test_lock_acquisition_returns_none_without_lock():
    rows = [_telemetry_row(0, 2, 1, 160, 120), _telemetry_row(20, 3, 1, 160, 120)]
    assert at.lock_acquisition_ms(rows) is None


def test_state_distribution_and_tracking_error():
    intr = CameraIntrinsics.default_for(320, 240)
    rows = [
        _telemetry_row(0, 3, 1, intr.cx + 2, intr.cy),
        _telemetry_row(10, 4, 1, intr.cx, intr.cy),
        _telemetry_row(20, 2, 0, intr.cx, intr.cy),
    ]
    dist = at.state_distribution(rows)
    by_name = {item["name"]: item for item in dist}
    assert by_name["TRACKING"]["count"] == 1
    assert by_name["LOCKED"]["ratio"] == pytest.approx(1 / 3)

    tracking = at.tracking_error_stats(rows, intr)
    assert tracking["n"] == 2
    assert tracking["los_rms_deg"] > 0.0
    assert tracking["axis_rms_deg"] == pytest.approx(0.0)


# --- calibrate_axis 解算 ----------------------------------------------------


def test_fit_affine_recovers_known_matrix():
    rng = np.random.default_rng(1)
    angle = rng.uniform([-15.0, -10.0], [15.0, 10.0], size=(40, 2))
    truth = np.array([[1.02, 0.03, -0.5],
                      [0.01, 1.01, 1.2]])
    target = apply_affine(truth, angle)
    fitted = ca.fit_affine(angle, target)
    assert np.allclose(fitted, truth, atol=1e-8)
    stats = ca.affine_rmse(fitted, angle, target)
    assert stats["rmse_deg"] < 1e-9
    assert stats["rmse_pan_deg"] < 1e-9
    assert stats["rmse_tilt_deg"] < 1e-9


def test_pixels_to_angles_deg_signs():
    intr = CameraIntrinsics.default_for(320, 240)
    angles = ca.pixels_to_angles_deg(
        np.array([[320.0, 120.0], [159.5, 120.0], [160.0, 20.0]]), intr)
    assert angles.shape == (3, 2)
    assert angles[0, 0] > 0.0
    assert angles[1, 0] == pytest.approx(0.0, abs=1e-9)
    assert angles[2, 1] > 0.0


def test_split_indices_is_disjoint_and_reproducible():
    train, hold = ca.split_indices(10, 0.3, seed=0)
    assert len(train) == 7 and len(hold) == 3
    assert set(train).isdisjoint(set(hold))
    assert len(set(train) | set(hold)) == 10

    _, hold_again = ca.split_indices(10, 0.3, seed=0)
    assert np.array_equal(hold, hold_again)

    train_small, hold_small = ca.split_indices(3, 0.5, seed=0)
    assert len(hold_small) == 0 and len(train_small) == 3


def test_coverage_metrics_flags_center_cluster():
    corners = np.array([[0.0, 0.0], [319.0, 0.0], [0.0, 239.0], [319.0, 239.0]])
    wide = ca.coverage_metrics(corners, 320, 240)
    assert wide["bbox_area_ratio"] > 0.99
    assert wide["outside_image"] == 0

    center = np.array([[150.0, 110.0], [170.0, 130.0]])
    tight = ca.coverage_metrics(center, 320, 240)
    assert tight["bbox_area_ratio"] < 0.5


def test_load_sample_axis_points():
    data = ca.load_axis_points(common.DEFAULT_SAMPLE_AXIS_CSV)
    assert data.shape == (13, 4)
    assert np.all(np.isfinite(data))


def test_load_axis_points_rejects_missing_columns(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("px,py\n1,2\n", encoding="utf-8")
    with pytest.raises(common.ToolError):
        ca.load_axis_points(path)


def test_sample_axis_points_end_to_end_low_rmse():
    data = ca.load_axis_points(common.DEFAULT_SAMPLE_AXIS_CSV)
    intr = CameraIntrinsics.default_for(320, 240)
    angles = ca.pixels_to_angles_deg(data[:, :2], intr)
    matrix = ca.fit_affine(angles, data[:, 2:4])
    stats = ca.affine_rmse(matrix, angles, data[:, 2:4])
    assert stats["rmse_deg"] < 0.2
    assert "优秀" in common.grade_rmse(stats["rmse_deg"])
