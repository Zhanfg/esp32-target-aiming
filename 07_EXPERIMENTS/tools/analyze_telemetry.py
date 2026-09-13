"""遥测精度评估：读采集到的 AIM CSV，算误差、锁定时间、控制周期并给达标结论。

跑法：
    python analyze_telemetry.py run1.csv
    python analyze_telemetry.py run1.csv --json report.json

指标目标取自 02_REQUIREMENTS/方案设计.md 第 11.1 节，误差预算见 02_REQUIREMENTS/标定与几何推导.md 第 11 节。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import common
from common import DEFAULT_CONFIG_PATH, die, state_name, warn

from aim.geometry import pixel_to_angles
from aim.types import CameraIntrinsics

TRACKING_STATES = (3, 4)
LOCKED_STATE = 4
CONTROL_BUDGET_US = 5000.0
STATIC_RMSE_TARGET_DEG = 0.5
TRACKING_RMSE_TARGET_DEG = 1.5
LOCK_TARGET_MS = 500.0


def _los_angles_deg(rows, intrinsics: CameraIntrinsics) -> np.ndarray:
    """每行像素相对光轴的角度偏移，单位为度。"""
    pixels = np.column_stack([[r["px"] for r in rows], [r["py"] for r in rows]])
    bearing_rad, elevation_rad = pixel_to_angles(pixels, intrinsics)
    return np.degrees(np.hypot(bearing_rad, elevation_rad))


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values ** 2)))


def steady_error_stats(rows, intrinsics: CameraIntrinsics) -> dict:
    """LOCKED 且观测有效时的像素指向误差与对应角度误差。"""
    sel = [r for r in rows if r["state"] == LOCKED_STATE and r["obs_valid"] == 1]
    if not sel:
        return {"n": 0}
    dx = np.array([r["px"] - intrinsics.cx for r in sel])
    dy = np.array([r["py"] - intrinsics.cy for r in sel])
    pixel = np.hypot(dx, dy)
    deg = _los_angles_deg(sel, intrinsics)
    return {
        "n": len(sel),
        "mean_px": float(pixel.mean()),
        "rms_px": _rms(pixel),
        "p95_px": float(np.percentile(pixel, 95)),
        "rms_deg": _rms(deg),
        "p95_deg": float(np.percentile(deg, 95)),
    }


def tracking_error_stats(rows, intrinsics: CameraIntrinsics) -> dict:
    """TRACKING/LOCKED 时的视线角误差，以及轴位置对指令的跟踪误差。"""
    sel = [r for r in rows if r["state"] in TRACKING_STATES and r["obs_valid"] == 1]
    if not sel:
        return {"n": 0}
    deg = _los_angles_deg(sel, intrinsics)
    axis = np.hypot(
        np.array([r["pan_deg"] - r["pan_target"] for r in sel]),
        np.array([r["tilt_deg"] - r["tilt_target"] for r in sel]),
    )
    return {
        "n": len(sel),
        "los_mean_deg": float(deg.mean()),
        "los_rms_deg": _rms(deg),
        "los_p95_deg": float(np.percentile(deg, 95)),
        "axis_rms_deg": _rms(axis),
        "axis_max_deg": float(axis.max()),
    }


def lock_acquisition_ms(rows) -> float | None:
    first_obs = next((i for i, r in enumerate(rows) if r["obs_valid"] == 1), None)
    if first_obs is None:
        return None
    for r in rows[first_obs:]:
        if r["state"] == LOCKED_STATE:
            return float(r["t_ms"] - rows[first_obs]["t_ms"])
    return None


def loop_period_stats(rows) -> dict:
    values = np.array([r["loop_us"] for r in rows if r["loop_us"] > 0], dtype=float)
    if values.size == 0:
        return {"n": 0}
    return {
        "n": int(values.size),
        "mean_us": float(values.mean()),
        "max_us": float(values.max()),
        "min_us": float(values.min()),
        "std_us": float(values.std()),
        "p95_us": float(np.percentile(values, 95)),
    }


def state_distribution(rows) -> list[dict]:
    counts: dict[int, int] = {}
    for r in rows:
        counts[r["state"]] = counts.get(r["state"], 0) + 1
    total = len(rows)
    return [
        {"code": code, "name": state_name(code), "count": count,
         "ratio": count / total if total else 0.0}
        for code, count in sorted(counts.items())
    ]


def lost_observation_ratio(rows) -> float:
    if not rows:
        return 0.0
    return sum(1 for r in rows if r["obs_valid"] == 0) / len(rows)


def _verdict(value, target, lower_is_better=True, available=True):
    if not available:
        return "无数据"
    if lower_is_better:
        return "达标" if value <= target else "超标"
    return "达标" if value >= target else "超标"


def _table_row(label, value, target, verdict):
    return f"  {label:<16}{value:<30}{target:<12}{verdict}"


def render_report(path, counts, intr, source, steady, tracking, lock_ms, loop, states,
                  lost_ratio) -> tuple[str, list[str]]:
    lines = []
    lines.append("遥测精度评估")
    lines.append(f"  输入：{path}，AIM 行 {counts['kept']}，"
                 f"丢弃非 AIM/畸形行 {counts['skipped']}")
    lines.append(f"  内参：fx={intr.fx:.2f} fy={intr.fy:.2f} "
                 f"cx={intr.cx:.2f} cy={intr.cy:.2f}，来源 {source}")
    lines.append("")
    lines.append("  指标                  数值                        目标        结论")
    lines.append(_table_row(
        "稳态指向 RMS",
        f"{steady['rms_px']:.3f} px / {steady['rms_deg']:.3f} 度" if steady["n"] else "无",
        f"≤{STATIC_RMSE_TARGET_DEG} 度",
        _verdict(steady.get("rms_deg", 0.0), STATIC_RMSE_TARGET_DEG, True, steady["n"] > 0)))
    lines.append(_table_row(
        "稳态指向 95 分位",
        f"{steady['p95_px']:.3f} px / {steady['p95_deg']:.3f} 度" if steady["n"] else "无",
        "-", "-"))
    lines.append(_table_row(
        "跟踪视线 RMS",
        f"{tracking['los_rms_deg']:.3f} 度" if tracking["n"] else "无",
        f"≤{TRACKING_RMSE_TARGET_DEG} 度",
        _verdict(tracking.get("los_rms_deg", 0.0), TRACKING_RMSE_TARGET_DEG,
                 True, tracking["n"] > 0)))
    lines.append(_table_row(
        "轴跟踪 RMS",
        f"{tracking['axis_rms_deg']:.3f} 度（峰值 {tracking['axis_max_deg']:.3f}）"
        if tracking["n"] else "无", "-", "-"))
    lines.append(_table_row(
        "锁定建立时间",
        f"{lock_ms:.1f} ms" if lock_ms is not None else "未进入 LOCKED",
        f"≤{LOCK_TARGET_MS:.0f} ms",
        _verdict(lock_ms if lock_ms is not None else 0.0, LOCK_TARGET_MS,
                 True, lock_ms is not None)))
    lines.append(_table_row(
        "控制周期均值",
        f"{loop['mean_us']:.1f} us（最大 {loop['max_us']:.1f}）" if loop["n"] else "无",
        f"≤{CONTROL_BUDGET_US:.0f} us",
        _verdict(loop.get("mean_us", 0.0), CONTROL_BUDGET_US, True, loop["n"] > 0)))
    lines.append(_table_row(
        "控制周期抖动",
        f"std {loop['std_us']:.1f} us / p95 {loop['p95_us']:.1f} us" if loop["n"] else "无",
        "-", "-"))
    lines.append(_table_row(
        "丢观测比例", f"{lost_ratio * 100:.1f}%", "-", "-"))
    lines.append("")
    lines.append("  状态分布：")
    for item in states:
        lines.append(f"    {item['name']:<11} {item['count']:>6}  {item['ratio'] * 100:5.1f}%")
    lines.append("")

    notes = []
    if steady["n"] == 0:
        notes.append("没有 LOCKED 段，先确认目标能进死区且 LOCK_FRAMES_N 不会一直不满足。")
    elif steady["rms_deg"] > STATIC_RMSE_TARGET_DEG:
        notes.append("稳态指向超标。按第 11 节误差预算，先查分割质心稳定性与减速箱反向空程，"
                     "这两项是主导；再看内参畸变残余是否在视野边缘放大。")
    if tracking["n"] and tracking["los_rms_deg"] > TRACKING_RMSE_TARGET_DEG:
        notes.append("跟踪误差超标。目标中低速时若误差仍大，多数是机械间隙或控制带宽不足。")
    if lock_ms is not None and lock_ms > LOCK_TARGET_MS:
        notes.append("锁定偏慢。检查 SEARCHING 到 TRACKING 的判定，以及 PAN/TILT_DEADBAND_DEG 是否过紧。")
    if loop["n"] and loop["mean_us"] > CONTROL_BUDGET_US:
        notes.append("控制周期超标。确认 CONTROL_LOOP_HZ 配置，并排查单帧里是否混入了视觉或串口阻塞。")
    elif loop["n"] and loop["max_us"] > CONTROL_BUDGET_US:
        notes.append(f"控制周期均值达标，但最大值 {loop['max_us']:.0f} us 超预算，"
                     "查偶发阻塞：串口打印、NVS 写入或视觉处理抖动都可能是来源。")
    if lost_ratio > 0.2:
        notes.append(f"丢观测 {lost_ratio * 100:.1f}%，偏高。查 HSV 阈值与曝光，"
                     "必要时让预测滑行顶过短暂遮挡。")
    for note in notes:
        lines.append("  " + note)
    return "\n".join(lines), notes


def _resolve_intrinsics(args):
    if args.intrinsics_json:
        data = json.loads(Path(args.intrinsics_json).read_text(encoding="utf-8"))
        dist = data.get("dist_coeffs") or [0.0] * 5
        intr = CameraIntrinsics(
            width=int(data["width"]), height=int(data["height"]),
            fx=float(data["fx"]), fy=float(data["fy"]),
            cx=float(data["cx"]), cy=float(data["cy"]),
            dist_coeffs=tuple(float(v) for v in dist),
        )
        return intr, Path(args.intrinsics_json).name

    intr, source = common.default_intrinsics_from_config(args.config)
    intr = common.apply_intrinsics_overrides(
        intr, width=args.width, height=args.height, fx=args.fx, fy=args.fy,
        cx=args.cx, cy=args.cy)
    if any(v is not None for v in (args.width, args.height, args.fx, args.fy,
                                   args.cx, args.cy)):
        source += " + 命令行覆盖"
    return intr, source


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="评估采集到的遥测：指向误差、跟踪误差、锁定时间、控制周期、状态分布。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("csv", help="capture_telemetry.py 采集的 CSV")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH),
                        help="config.h 路径，用于取图像尺寸和内参宏（若有）")
    parser.add_argument("--intrinsics-json", default=None,
                        help="内参 JSON（calibrate_intrinsics.py 的输出），优先于 config.h")
    parser.add_argument("--width", type=int, default=None, help="源图像宽，覆盖内参")
    parser.add_argument("--height", type=int, default=None, help="源图像高，覆盖内参")
    parser.add_argument("--fx", type=float, default=None, help="焦距 fx（px）")
    parser.add_argument("--fy", type=float, default=None, help="焦距 fy（px）")
    parser.add_argument("--cx", type=float, default=None, help="主点 cx（px）")
    parser.add_argument("--cy", type=float, default=None, help="主点 cy（px）")
    parser.add_argument("--json", default=None, help="机器可读报告的输出路径")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    source_path = Path(args.csv)
    if not source_path.is_file():
        die(f"文件不存在：{source_path}", hint="先用 capture_telemetry.py 采集，或传入已有 CSV")

    rows, counts = common.read_aim_telemetry(source_path)
    if not rows:
        die("没有解析到 AIM 行",
            hint="确认文件来自 capture_telemetry.py，且采集时波特率与 TELEMETRY 输出正确")
    if counts["skipped"]:
        warn(f"跳过了 {counts['skipped']} 行非 AIM/畸形数据")

    intr, source = _resolve_intrinsics(args)
    if "占位内参" in source:
        warn("内参未标定，像素与角度的换算会有系统偏差，先跑 calibrate_intrinsics.py")

    steady = steady_error_stats(rows, intr)
    tracking = tracking_error_stats(rows, intr)
    lock_ms = lock_acquisition_ms(rows)
    loop = loop_period_stats(rows)
    states = state_distribution(rows)
    lost_ratio = lost_observation_ratio(rows)

    report, notes = render_report(source_path, counts, intr, source, steady, tracking,
                                  lock_ms, loop, states, lost_ratio)
    print(report)

    if args.json:
        record = {
            "input": str(source_path),
            "intrinsics": {
                "width": intr.width, "height": intr.height,
                "fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy,
                "source": source,
            },
            "counts": counts,
            "steady_error": steady,
            "tracking_error": tracking,
            "lock_acquisition_ms": lock_ms,
            "loop_period": loop,
            "state_distribution": states,
            "lost_observation_ratio": lost_ratio,
            "targets": {
                "steady_rms_deg": STATIC_RMSE_TARGET_DEG,
                "tracking_rms_deg": TRACKING_RMSE_TARGET_DEG,
                "lock_ms": LOCK_TARGET_MS,
                "control_period_us": CONTROL_BUDGET_US,
            },
            "notes": notes,
        }
        Path(args.json).write_text(json.dumps(record, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
        print(f"  已写出 JSON 报告：{args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
