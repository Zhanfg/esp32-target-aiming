"""不依赖硬件和 OpenCV 的单元测试：遥测解析、config.h 宏、机制统计、仿射解算。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import common
import analyze_telemetry as at
import calibrate_axis as ca

from aim.geometry import apply_affine
from aim.types import CameraIntrinsics

CONFIG_PATH = Path(__file__).resolve().parents[3] / "05_FIRMWARE" / "include" / "config.h"

# 一行合法的 MST 遥测：1234ms，实验号 7，MST-01C，17 个字段齐全。
MST_SAMPLE = "MST,1234,7,MST-01C,MST-01C,0,1,2,3,4,12,5,6,1,3,1,0,-"


def _mst_line(**overrides) -> str:
    row = {name: 0 for name in common.TELEMETRY_FIELDS}
    row.update({"prototype_version": "MST-01C", "mechanism_version": "MST-01C",
                "operator_note": "-"})
    row.update(overrides)
    return "MST," + ",".join(str(row[name]) for name in common.TELEMETRY_FIELDS)


def _mst_row(**overrides):
    return common.parse_mst_line(_mst_line(**overrides))


# --- 遥测行解析 -------------------------------------------------------------


def test_parse_mst_line_ok():
    row = common.parse_mst_line(MST_SAMPLE + "\r\n")
    assert row is not None
    assert row["timestamp"] == 1234 and isinstance(row["timestamp"], int)
    assert row["experiment_id"] == 7
    assert row["prototype_version"] == "MST-01C"
    assert row["mechanism_version"] == "MST-01C"
    assert row["mode"] == 0
    assert row["cycle_count"] == 12
    assert row["magazine_position"] == 3
    assert row["operator_note"] == "-"
    assert set(row) == set(common.TELEMETRY_FIELDS)


def test_parse_mst_line_types():
    row = common.parse_mst_line(MST_SAMPLE)
    string_fields = {"prototype_version", "mechanism_version", "operator_note"}
    for name in common.TELEMETRY_FIELDS:
        if name in string_fields:
            assert isinstance(row[name], str)
        else:
            assert isinstance(row[name], int)


def test_parse_mst_line_rejects_bad_input():
    assert common.parse_mst_line("AIM,1,2,3") is None
    assert common.parse_mst_line("SYS, boot ok") is None
    assert common.parse_mst_line("EVT,100,STAGE1,t=200") is None
    assert common.parse_mst_line("MST,1,2,3") is None
    assert common.parse_mst_line("MST," + ",".join(["x"] * 17)) is None
    assert common.parse_mst_line(MST_SAMPLE.rsplit(",", 1)[0]) is None
    assert common.parse_mst_line("") is None


def test_parse_mst_line_rejects_header():
    header = "MST," + ",".join(common.TELEMETRY_FIELDS)
    assert common.parse_mst_line(header) is None


def test_read_mst_telemetry_counts_skipped(tmp_path):
    path = tmp_path / "run.csv"
    path.write_text(
        "# MST telemetry\n"
        + "MST," + ",".join(common.TELEMETRY_FIELDS) + "\n"
        + MST_SAMPLE + "\n"
        + "EVT,100,STAGE1,t=200\n"
        + "MST,1,2\n",
        encoding="utf-8",
    )
    rows, counts = common.read_mst_telemetry(path)
    assert len(rows) == 1
    assert counts == {"total": 5, "kept": 1, "skipped": 4}


def test_numeric_field_rejecting_empty_and_dash():
    assert common.parse_mst_line(_mst_line(timestamp="")) is None
    assert common.parse_mst_line(_mst_line(fault_code="-")) is None
    assert common.parse_mst_line(_mst_line(operator_note=""))["operator_note"] == ""


# --- DIAG 帧轨迹解析 --------------------------------------------------------


# 一行合法的 DIAG 帧轨迹：t=1234ms，本拍 1.2ms，有观测，处于 LOCKED。
DIAG_SAMPLE = ("DIAG,1234,1200,1,0,160.50,119.25,0.900,1.250,-0.500,"
               "1.300,-0.480,0.120,0.060,0.134,4,0")


def _diag_line(**overrides) -> str:
    row = {name: 0 for name in common.DIAG_FIELDS}
    row.update(overrides)
    return "DIAG," + ",".join(str(row[name]) for name in common.DIAG_FIELDS)


def _diag_row(**overrides):
    return common.parse_diag_line(_diag_line(**overrides))


def test_parse_diag_line_ok():
    row = common.parse_diag_line(DIAG_SAMPLE + "\r\n")
    assert row is not None
    assert row["timestamp_ms"] == 1234 and isinstance(row["timestamp_ms"], int)
    assert row["loop_us"] == 1200
    assert row["obs_valid"] == 1
    assert row["px"] == pytest.approx(160.50) and isinstance(row["px"], float)
    assert row["err_deg"] == pytest.approx(0.134)
    assert row["aim_state"] == 4
    assert set(row) == set(common.DIAG_FIELDS)


def test_parse_diag_line_types():
    row = common.parse_diag_line(DIAG_SAMPLE)
    for name in common.DIAG_FIELDS:
        if name in {"timestamp_ms", "loop_us", "obs_valid", "obs_dropped",
                    "aim_state", "has_feedback"}:
            assert isinstance(row[name], int)
        else:
            assert isinstance(row[name], float)


def test_parse_diag_line_rejects_bad_input():
    assert common.parse_diag_line("MST,1,2,3") is None
    assert common.parse_diag_line("DIAG,1,2,3") is None
    assert common.parse_diag_line("DIAG," + ",".join(["x"] * len(common.DIAG_FIELDS))) is None
    assert common.parse_diag_line(DIAG_SAMPLE.rsplit(",", 1)[0]) is None
    assert common.parse_diag_line("DIAG," + ",".join(common.DIAG_FIELDS)) is None
    assert common.parse_diag_line("") is None


def test_read_diag_telemetry_counts(tmp_path):
    path = tmp_path / "run.csv"
    path.write_text(
        "# DIAG frame telemetry\n"
        + "DIAG," + ",".join(common.DIAG_FIELDS) + "\n"
        + DIAG_SAMPLE + "\n"
        + "EVT,100,STAGE1,t=200\n",
        encoding="utf-8",
    )
    rows, counts = common.read_diag_telemetry(path)
    assert len(rows) == 1
    assert counts == {"total": 4, "kept": 1, "skipped": 3}


def test_read_telemetry_splits_two_lines(tmp_path):
    path = tmp_path / "run.csv"
    path.write_text(
        "# mixed\n"
        + "MST," + ",".join(common.TELEMETRY_FIELDS) + "\n"
        + "DIAG," + ",".join(common.DIAG_FIELDS) + "\n"
        + MST_SAMPLE + "\n"
        + DIAG_SAMPLE + "\n"
        + "garbage\n",
        encoding="utf-8",
    )
    mst_rows, diag_rows, counts = common.read_telemetry(path)
    assert len(mst_rows) == 1 and len(diag_rows) == 1
    assert counts == {"total": 6, "mst": 1, "diag": 1, "skipped": 3}


# --- 三项验收指标 -----------------------------------------------------------


def test_static_pointing_rmse_only_locked_valid():
    rows = [
        _diag_row(obs_valid=1, aim_state=4, err_pan_deg=0.3, err_tilt_deg=0.4,
                  err_deg=0.5),
        _diag_row(obs_valid=1, aim_state=4, err_pan_deg=-0.3, err_tilt_deg=0.4,
                  err_deg=0.5),
        _diag_row(obs_valid=0, aim_state=4, err_deg=9.9),   # 无效观测，剔除
        _diag_row(obs_valid=1, aim_state=3, err_deg=9.9),   # 仅 TRACKING，剔除
    ]
    stats = at.static_pointing_rmse(rows)
    assert stats["n"] == 2
    assert stats["rmse_deg"] == pytest.approx(0.5)
    assert stats["rmse_pan_deg"] == pytest.approx(0.3)
    assert stats["rmse_tilt_deg"] == pytest.approx(0.4)


def test_static_pointing_rmse_no_samples():
    assert at.static_pointing_rmse([_diag_row(obs_valid=0)]) == {"n": 0}


def test_control_period_stats_from_loop_us():
    rows = [_diag_row(loop_us=us) for us in (1000, 2000, 6000)]
    stats = at.control_period_stats(rows)
    assert stats["n"] == 3
    assert stats["mean_ms"] == pytest.approx(3.0)
    assert stats["max_ms"] == pytest.approx(6.0)
    assert stats["min_ms"] == pytest.approx(1.0)


def test_lock_establish_ms_from_first_obs_to_locked():
    rows = [
        _diag_row(timestamp_ms=100, obs_valid=0, aim_state=3),
        _diag_row(timestamp_ms=200, obs_valid=1, aim_state=3),
        _diag_row(timestamp_ms=450, obs_valid=1, aim_state=3),
        _diag_row(timestamp_ms=600, obs_valid=1, aim_state=4),
    ]
    lock = at.lock_establish_ms(rows)
    assert lock["found"] is True
    assert lock["ms"] == 400
    assert lock["start_ms"] == 200


def test_lock_establish_ms_no_locked():
    rows = [_diag_row(timestamp_ms=10, obs_valid=1, aim_state=3),
            _diag_row(timestamp_ms=20, obs_valid=1, aim_state=2)]
    lock = at.lock_establish_ms(rows)
    assert lock["found"] is False
    assert lock["reason"] == "未进入 LOCKED"


def test_lock_establish_ms_no_observation():
    assert at.lock_establish_ms([_diag_row(obs_valid=0)])["found"] is False


def test_analyze_main_runs_on_mixed_csv(tmp_path, capsys):
    path = tmp_path / "run.csv"
    lines = [
        "# mixed telemetry",
        "MST," + ",".join(common.TELEMETRY_FIELDS),
        "DIAG," + ",".join(common.DIAG_FIELDS),
        _mst_line(timestamp=0, mechanism_recovered=1, magazine_index_ok=1),
        _mst_line(timestamp=100, mechanism_recovered=1, magazine_index_ok=1),
        _diag_line(timestamp_ms=200, loop_us=1500, obs_valid=0, aim_state=2),
        _diag_line(timestamp_ms=300, loop_us=1800, obs_valid=1, aim_state=3,
                   err_pan_deg=0.1, err_tilt_deg=0.2, err_deg=0.2236),
        _diag_line(timestamp_ms=500, loop_us=1600, obs_valid=1, aim_state=4,
                   err_pan_deg=0.0, err_tilt_deg=0.0, err_deg=0.0),
        _diag_line(timestamp_ms=600, loop_us=1700, obs_valid=1, aim_state=4,
                   err_pan_deg=0.0, err_tilt_deg=0.0, err_deg=0.0),
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert at.main([str(path)]) == 0
    out = capsys.readouterr().out
    assert "机制遥测评估" in out
    assert "指向与验收指标" in out
    assert "静态指向 RMSE" in out and "0.000°" in out
    assert "控制周期" in out and "均值 1.650 ms" in out
    assert "锁定建立时间" in out and "200 ms" in out


def test_analyze_main_mst_only_prints_diag_notice(tmp_path, capsys):
    path = tmp_path / "run.csv"
    path.write_text(
        "# MST only\n"
        + "MST," + ",".join(common.TELEMETRY_FIELDS) + "\n"
        + _mst_line(timestamp=0) + "\n"
        + _mst_line(timestamp=100) + "\n",
        encoding="utf-8",
    )
    assert at.main([str(path)]) == 0
    out = capsys.readouterr().out
    assert "机制遥测评估" in out
    assert "未发现 DIAG, 行" in out
    assert "≤0.5°" not in out  # 没有 DIAG 时不打印指标表行


# --- 枚举名 -----------------------------------------------------------------


def test_cycle_state_name_mapping():
    assert common.cycle_state_name(0) == "BOOT"
    assert common.cycle_state_name(10) == "FAULT"
    assert common.cycle_state_name(99).startswith("UNKNOWN")


def test_aim_state_and_fault_names():
    assert common.aim_state_name(3) == "TRACKING"
    assert common.aim_state_name(4) == "LOCKED"
    assert common.aim_state_name(5).startswith("UNKNOWN")  # 第一期遗留空洞
    assert common.fault_code_name(0) == "NONE"
    assert common.fault_code_name(14) == "STATE_ILLEGAL"
    assert common.fault_severity_name(2) == "SOFT"
    assert common.fault_severity_name(8) == "HARD"
    assert common.preload_state_name(2) == "ARMED"
    assert common.boundary_state_name(1) == "BOUNDARY_1"
    assert common.fire_mode_name(1) == "SOFT_PAYLOAD"


# --- config.h 宏解析 --------------------------------------------------------


def test_parse_real_config_macros():
    macros = common.parse_config_macros(CONFIG_PATH)
    assert macros["VISION_SRC_W"] == pytest.approx(320.0)
    assert macros["VISION_SRC_H"] == pytest.approx(240.0)
    assert macros["PAN_MIN_DEG"] == pytest.approx(-180.0)
    assert macros["TELEMETRY_BAUD"] == pytest.approx(115200.0)
    assert macros["CAM_PIN_PWDN"] == pytest.approx(-1.0)
    assert macros["SERVO_PAN_MIN_DEG"] == pytest.approx(-180.0)
    assert macros["CONTROL_LOOP_BUDGET_MS"] == pytest.approx(10.0)
    assert isinstance(macros["AIM_BUILD_VERSION"], str)
    assert "0.2.0" in macros["AIM_BUILD_VERSION"]
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


def test_fault_distribution_names_and_severity():
    rows = [_mst_row(fault_code=0), _mst_row(fault_code=0), _mst_row(fault_code=2)]
    dist = at.fault_distribution(rows)
    by_code = {item["code"]: item for item in dist}
    assert by_code[0]["name"] == "NONE"
    assert by_code[2]["name"] == "HOME_TIMEOUT"
    assert by_code[2]["severity"] == "SOFT"
    assert by_code[2]["ratio"] == pytest.approx(1 / 3)


def test_state_distributions():
    rows = [
        _mst_row(preload_state=0, boundary_state=0, mode=0),
        _mst_row(preload_state=1, boundary_state=1, mode=1),
        _mst_row(preload_state=2, boundary_state=1, mode=1),
    ]
    preload = {item["name"]: item["count"] for item in at.preload_distribution(rows)}
    assert preload == {"IDLE": 1, "PRELOADING": 1, "ARMED": 1}
    boundary = {item["name"]: item["count"] for item in at.boundary_distribution(rows)}
    assert boundary == {"BOUNDARY_0": 1, "BOUNDARY_1": 2}
    modes = {item["name"]: item["count"] for item in at.mode_distribution(rows)}
    assert modes == {"AIR_ONLY": 1, "SOFT_PAYLOAD": 2}


def test_mechanism_summary():
    rows = [
        _mst_row(cycle_count=3, stage1_event=1, stage2_event=1, mechanism_recovered=1,
                 magazine_position=0, magazine_index_ok=1),
        _mst_row(cycle_count=4, stage1_event=2, stage2_event=2, mechanism_recovered=1,
                 magazine_position=1, magazine_index_ok=1),
        _mst_row(cycle_count=5, stage1_event=3, stage2_event=3, mechanism_recovered=0,
                 magazine_position=2, magazine_index_ok=0),
    ]
    summary = at.mechanism_summary(rows)
    assert summary["n"] == 3
    assert (summary["cycle_min"], summary["cycle_max"], summary["cycle_span"]) == (3, 5, 2)
    assert summary["recover_ratio"] == pytest.approx(2 / 3)
    assert summary["index_ok_ratio"] == pytest.approx(2 / 3)
    assert summary["stage2_max"] == 3
    assert summary["magazine_positions"] == [0, 1, 2]


def test_sample_interval_stats():
    rows = [_mst_row(timestamp=t) for t in (0, 100, 210, 290)]
    stats = at.sample_interval_stats(rows)
    assert stats["n"] == 3
    assert stats["mean_ms"] == pytest.approx((100 + 110 + 80) / 3)
    assert stats["max_ms"] == pytest.approx(110.0)
    assert stats["min_ms"] == pytest.approx(80.0)


def test_sample_interval_skips_nonpositive():
    rows = [_mst_row(timestamp=1000), _mst_row(timestamp=1000),
            _mst_row(timestamp=999)]
    assert at.sample_interval_stats(rows)["n"] == 0


def test_analyze_main_runs_on_mst_csv(tmp_path, capsys):
    path = tmp_path / "run.csv"
    lines = ["# MST telemetry",
             "MST," + ",".join(common.TELEMETRY_FIELDS)]
    for i in range(3):
        lines.append(_mst_line(timestamp=i * 100, cycle_count=i,
                               mechanism_recovered=1, magazine_index_ok=1,
                               fault_code=(2 if i == 2 else 0)))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert at.main([str(path)]) == 0
    out = capsys.readouterr().out
    assert "机制遥测评估" in out
    assert "MST 行 3" in out
    assert "HOME_TIMEOUT" in out


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
