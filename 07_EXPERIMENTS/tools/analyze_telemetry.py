"""遥测评估：同读 MST, 与 DIAG, 两条线。

跑法：
    python analyze_telemetry.py run1.csv
    python analyze_telemetry.py run1.csv --json report.json

MST, 行对应规格书 §19 的每发记录，用来汇总循环、事件、恢复、索引和故障。
DIAG, 行是开发期 AIM_VERBOSE_TELEMETRY=1 才有的每控制拍帧轨迹，用来恢复三项验收指标：
静态指向 RMSE（对照 02_REQUIREMENTS/方案设计.md §11.1 的 ≤0.5°）、控制周期（≤5 ms）、
锁定建立时间（≤500 ms）。两条线的解析互不影响。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import common
from common import die, warn

RECOVER_TARGET = 1.0
INDEX_OK_TARGET = 1.0
# 任一非零 fault_code 都算超标。
FAULT_FREE_TARGET = 0

# 三项验收口径，数值来自 02_REQUIREMENTS/方案设计.md §11.1。
STATIC_RMSE_TARGET_DEG = 0.5
CONTROL_PERIOD_TARGET_MS = 5.0
LOCK_TIME_TARGET_MS = 500.0

# AimState::LOCKED 的整数值，与 05_FIRMWARE/src/aim_types.h 一致。
LOCKED_AIM_STATE = 4


def distribution(rows, field: str) -> list[dict]:
    counts: dict[int, int] = {}
    for row in rows:
        code = int(row[field])
        counts[code] = counts.get(code, 0) + 1
    total = len(rows)
    return [
        {"code": code, "count": count, "ratio": count / total if total else 0.0}
        for code, count in sorted(counts.items())
    ]


def preload_distribution(rows) -> list[dict]:
    return [dict(item, name=common.preload_state_name(item["code"]))
            for item in distribution(rows, "preload_state")]


def boundary_distribution(rows) -> list[dict]:
    return [dict(item, name=common.boundary_state_name(item["code"]))
            for item in distribution(rows, "boundary_state")]


def mode_distribution(rows) -> list[dict]:
    return [dict(item, name=common.fire_mode_name(item["code"]))
            for item in distribution(rows, "mode")]


def fault_distribution(rows) -> list[dict]:
    return [
        dict(item,
             name=common.fault_code_name(item["code"]),
             severity=common.fault_severity_name(item["code"]))
        for item in distribution(rows, "fault_code")
    ]


def mechanism_summary(rows) -> dict:
    """循环计数跨度、级间事件累计、恢复与索引到位比例、工位覆盖。"""
    if not rows:
        return {"n": 0}
    cycles = [int(r["cycle_count"]) for r in rows]
    recovered = [int(r["mechanism_recovered"]) for r in rows]
    index_ok = [int(r["magazine_index_ok"]) for r in rows]
    return {
        "n": len(rows),
        "cycle_min": min(cycles),
        "cycle_max": max(cycles),
        "cycle_span": max(cycles) - min(cycles),
        "recover_ratio": sum(recovered) / len(rows),
        "index_ok_ratio": sum(index_ok) / len(rows),
        "stage1_max": max(int(r["stage1_event"]) for r in rows),
        "stage2_max": max(int(r["stage2_event"]) for r in rows),
        "magazine_positions": sorted({int(r["magazine_position"]) for r in rows}),
    }


def sample_interval_stats(rows) -> dict:
    """相邻两行 timestamp 的差值，单位 ms。回绕或倒序产生的非正值直接丢掉。"""
    values = [
        int(cur["timestamp"]) - int(prev["timestamp"])
        for prev, cur in zip(rows, rows[1:])
        if int(cur["timestamp"]) - int(prev["timestamp"]) > 0
    ]
    if not values:
        return {"n": 0}
    array = np.array(values, dtype=float)
    return {
        "n": int(array.size),
        "mean_ms": float(array.mean()),
        "max_ms": float(array.max()),
        "min_ms": float(array.min()),
        "std_ms": float(array.std()),
        "p95_ms": float(np.percentile(array, 95)),
    }


def static_pointing_rmse(diag_rows) -> dict:
    """静态指向 RMSE（度）。口径：DIAG 行里 obs_valid=1 且处于 LOCKED 的样本，
    err_deg 即目标质心经标定映射后相对光轴的角偏差，按均方根汇总。

    同时给出 err_pan_deg / err_tilt_deg 两轴分量，便于区分是哪一轴偏。
    """
    values = [
        (float(r["err_pan_deg"]), float(r["err_tilt_deg"]), float(r["err_deg"]))
        for r in diag_rows
        if int(r["obs_valid"]) == 1 and int(r["aim_state"]) == LOCKED_AIM_STATE
    ]
    if not values:
        return {"n": 0}
    array = np.array(values, dtype=float)
    return {
        "n": int(array.shape[0]),
        "rmse_deg": float(np.sqrt(np.mean(array[:, 2] ** 2))),
        "rmse_pan_deg": float(np.sqrt(np.mean(array[:, 0] ** 2))),
        "rmse_tilt_deg": float(np.sqrt(np.mean(array[:, 1] ** 2))),
    }


def control_period_stats(diag_rows) -> dict:
    """控制周期（ms）。用 DIAG 的 loop_us 换算，给出均值、最大与 95 分位。"""
    values = [float(r["loop_us"]) / 1000.0 for r in diag_rows]
    if not values:
        return {"n": 0}
    array = np.array(values, dtype=float)
    return {
        "n": int(array.size),
        "mean_ms": float(array.mean()),
        "max_ms": float(array.max()),
        "min_ms": float(array.min()),
        "p95_ms": float(np.percentile(array, 95)),
    }


def lock_establish_ms(diag_rows) -> dict:
    """锁定建立时间（ms）。从首个有效观测到首次进入 LOCKED 的时间差。

    按文件顺序扫描，时间戳回绕或倒序只让该样本作废，不会算成负值。
    """
    t0 = None
    for row in diag_rows:
        if int(row["obs_valid"]) != 1:
            continue
        t0 = int(row["timestamp_ms"])
        break
    if t0 is None:
        return {"found": False, "reason": "无有效观测"}
    for row in diag_rows:
        ts = int(row["timestamp_ms"])
        if ts < t0:
            continue
        if int(row["aim_state"]) == LOCKED_AIM_STATE:
            return {"found": True, "ms": ts - t0, "start_ms": t0, "locked_ms": ts}
    return {"found": False, "reason": "未进入 LOCKED", "start_ms": t0}


def acceptance_metrics(diag_rows) -> dict:
    """三项验收指标汇总，供渲染与 JSON 共用。"""
    return {
        "available": bool(diag_rows),
        "pointing_rmse": static_pointing_rmse(diag_rows),
        "control_period": control_period_stats(diag_rows),
        "lock_time": lock_establish_ms(diag_rows),
    }


def run_metadata(rows) -> dict:
    if not rows:
        return {}
    notes = sorted({r["operator_note"] for r in rows
                    if r["operator_note"] not in ("-", "")})
    return {
        "experiment_id": sorted({int(r["experiment_id"]) for r in rows}),
        "prototype_version": sorted({r["prototype_version"] for r in rows}),
        "mechanism_version": sorted({r["mechanism_version"] for r in rows}),
        "operator_notes": notes,
    }


def _verdict(value: float, target: float, available: bool = True) -> str:
    if not available:
        return "无数据"
    return "达标" if value <= target else "超标"


def _table_row(label, value, target, verdict) -> str:
    return f"  {label:<14}{value:<32}{target:<10}{verdict}"


def render_report(path, counts, meta, summary, intervals, faults, modes, preloads,
                  boundaries) -> tuple[str, list[str]]:
    lines = []
    lines.append("机制遥测评估")
    lines.append(f"  输入：{path}，MST 行 {counts['kept']}，"
                 f"丢弃非 MST/畸形行 {counts['skipped']}")
    if meta:
        lines.append(f"  实验号：{meta['experiment_id']}  "
                     f"原型：{','.join(meta['prototype_version']) or '-'}  "
                     f"机构：{','.join(meta['mechanism_version']) or '-'}")
    if intervals["n"]:
        lines.append(f"  采样间隔：均值 {intervals['mean_ms']:.1f} ms，"
                     f"最大 {intervals['max_ms']:.1f} ms，样本 {intervals['n']}")
    lines.append("")
    lines.append("  指标                数值                            目标      结论")
    recover_ratio = summary.get("recover_ratio", 0.0)
    index_ratio = summary.get("index_ok_ratio", 0.0)
    fault_rows = sum(item["count"] for item in faults if item["code"] != 0)
    lines.append(_table_row(
        "机构恢复率", f"{recover_ratio * 100:.1f}%", f"=100%",
        _verdict(1.0 - recover_ratio, 0.0, summary["n"] > 0)))
    lines.append(_table_row(
        "索引到位率", f"{index_ratio * 100:.1f}%", f"=100%",
        _verdict(1.0 - index_ratio, 0.0, summary["n"] > 0)))
    lines.append(_table_row(
        "非零故障码", f"{fault_rows} 行", f"={FAULT_FREE_TARGET} 行",
        _verdict(float(fault_rows), float(FAULT_FREE_TARGET), bool(summary["n"]))))
    lines.append(_table_row(
        "循环计数",
        f"{summary.get('cycle_min', 0)}..{summary.get('cycle_max', 0)}"
        f"（跨度 {summary.get('cycle_span', 0)}）" if summary["n"] else "无",
        "-", "-"))
    lines.append(_table_row(
        "级间事件累计",
        f"stage1 最大 {summary.get('stage1_max', 0)} / "
        f"stage2 最大 {summary.get('stage2_max', 0)}" if summary["n"] else "无",
        "-", "-"))
    lines.append(_table_row(
        "工位覆盖",
        ",".join(str(p) for p in summary.get("magazine_positions", []))
        if summary["n"] else "无", "-", "-"))
    lines.append("")
    lines.append("  预载状态分布：")
    for item in preloads:
        lines.append(f"    {item['name']:<16} {item['count']:>6}  {item['ratio'] * 100:5.1f}%")
    lines.append("  边界状态分布：")
    for item in boundaries:
        lines.append(f"    {item['name']:<16} {item['count']:>6}  {item['ratio'] * 100:5.1f}%")
    lines.append("  模式分布：")
    for item in modes:
        lines.append(f"    {item['name']:<16} {item['count']:>6}  {item['ratio'] * 100:5.1f}%")
    lines.append("  故障码分布：")
    for item in faults:
        lines.append(
            f"    {item['name']:<16} {item['count']:>6}  {item['ratio'] * 100:5.1f}%"
            f"  {item['severity']}")
    lines.append("")

    notes = []
    bad_faults = [item for item in faults if item["code"] != 0]
    if bad_faults:
        detail = "，".join(f"{item['name']}({item['severity']})" for item in bad_faults)
        notes.append(f"出现故障码：{detail}。软故障可从 FAULT 回 SAFE 再回 READY，"
                     "硬故障锁存，先按 05_FIRMWARE/fault-codes.md 定位再复现。")
    if summary["n"] and summary["recover_ratio"] < RECOVER_TARGET:
        notes.append("有行 mechanism_recovered=0，机构未回到位。查弹簧回复力、导轨阻滞"
                     "与 MECH_RECOVER_MS 是否够长。")
    if summary["n"] and summary["index_ok_ratio"] < INDEX_OK_TARGET:
        notes.append("有行 magazine_index_ok=0，供给盘未到位或卡滞。查 INDEX_MAX_RETRY、"
                     "工位传感器与 MAG_POSITIONS。")
    if summary["n"] and summary["cycle_max"] == summary["cycle_min"]:
        notes.append("cycle_count 全程没变，本段可能停在 READY/FAULT 未走完一个发射周期。")
    for note in notes:
        lines.append("  " + note)
    return "\n".join(lines), notes


def render_acceptance(path, acc, counts) -> tuple[str, list[str]]:
    """渲染指向与时序三项验收指标。无 DIAG 行时给明确提示，不报错也不出空表。"""
    lines = ["", "指向与验收指标（数据源 DIAG,）"]
    if not acc["available"]:
        lines.append("  未发现 DIAG, 行。静态指向 RMSE、控制周期、锁定建立时间需要开发期")
        lines.append("  AIM_VERBOSE_TELEMETRY=1 采集的帧轨迹；本文件只有 MST, 行，")
        lines.append("  机制类指标照常输出，这三项留待补采。")
        return "\n".join(lines), ["输入缺少 DIAG, 行，三项验收指标未计算。"]

    lines.append(f"  输入：{path}，DIAG 行 {counts['diag']}，"
                 f"丢弃无法识别行 {counts['skipped']}")
    lines.append("")
    lines.append("  指标                数值                            目标      结论")
    notes: list[str] = []

    rmse = acc["pointing_rmse"]
    if rmse["n"]:
        value = (f"{rmse['rmse_deg']:.3f}°"
                 f"（pan {rmse['rmse_pan_deg']:.3f} / tilt {rmse['rmse_tilt_deg']:.3f}，"
                 f"样本 {rmse['n']}）")
        verdict = _verdict(rmse["rmse_deg"], STATIC_RMSE_TARGET_DEG)
        if verdict == "超标":
            notes.append(f"静态指向 RMSE {rmse['rmse_deg']:.3f}° 超过 "
                         f"{STATIC_RMSE_TARGET_DEG}°。查像素到角度标定、光轴与轴系对齐。")
    else:
        value = "无 LOCKED 且有效的观测"
        verdict = "无数据"
    lines.append(_table_row("静态指向 RMSE", value,
                            f"≤{STATIC_RMSE_TARGET_DEG:.1f}°", verdict))

    period = acc["control_period"]
    if period["n"]:
        value = f"均值 {period['mean_ms']:.3f} ms / 最大 {period['max_ms']:.3f} ms"
        verdict = _verdict(period["max_ms"], CONTROL_PERIOD_TARGET_MS)
        if verdict == "超标":
            notes.append(f"控制周期最大 {period['max_ms']:.3f} ms 超过 "
                         f"{CONTROL_PERIOD_TARGET_MS} ms。查视觉与遥测是否拖长单拍。")
    else:
        value = "无 DIAG 行"
        verdict = "无数据"
    lines.append(_table_row("控制周期", value,
                            f"≤{CONTROL_PERIOD_TARGET_MS:.0f} ms", verdict))

    lock = acc["lock_time"]
    if lock["found"]:
        value = f"{lock['ms']} ms（t={lock['start_ms']}→{lock['locked_ms']}）"
        verdict = _verdict(lock["ms"], LOCK_TIME_TARGET_MS)
        if verdict == "超标":
            notes.append(f"锁定建立时间 {lock['ms']} ms 超过 {LOCK_TIME_TARGET_MS} ms。"
                         "查 LOCK_WINDOW_MS、LOCK_MIN_VISION_FRAMES 与视觉帧率。")
    else:
        value = f"无（{lock.get('reason', '未建立')}）"
        verdict = "无数据"
        notes.append("本次 DIAG 段未建立 LOCKED，锁定建立时间无法计算。")
    lines.append(_table_row("锁定建立时间", value,
                            f"≤{LOCK_TIME_TARGET_MS:.0f} ms", verdict))
    return "\n".join(lines), notes


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="评估采集到的遥测：MST, 行的机制指标与 DIAG, 行的指向、时序验收指标。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("csv", help="capture_telemetry.py 采集的 CSV")
    parser.add_argument("--json", default=None, help="机器可读报告的输出路径")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    source_path = Path(args.csv)
    if not source_path.is_file():
        die(f"文件不存在：{source_path}", hint="先用 capture_telemetry.py 采集，或传入已有 CSV")

    mst_rows, diag_rows, counts = common.read_telemetry(source_path)
    if not mst_rows:
        die("没有解析到 MST 行",
            hint="确认文件来自 capture_telemetry.py，且采集时波特率与 TELEMETRY 输出正确")
    if counts["skipped"]:
        warn(f"跳过了 {counts['skipped']} 行无法识别的数据")

    meta = run_metadata(mst_rows)
    summary = mechanism_summary(mst_rows)
    intervals = sample_interval_stats(mst_rows)
    faults = fault_distribution(mst_rows)
    modes = mode_distribution(mst_rows)
    preloads = preload_distribution(mst_rows)
    boundaries = boundary_distribution(mst_rows)

    # render_report 沿用原统计口径，把混合计数折算成它认识的 total/kept/skipped。
    mech_counts = {"total": counts["total"], "kept": counts["mst"],
                   "skipped": counts["skipped"]}
    report, notes = render_report(source_path, mech_counts, meta, summary, intervals,
                                  faults, modes, preloads, boundaries)
    print(report)

    acc = acceptance_metrics(diag_rows)
    acc_report, acc_notes = render_acceptance(source_path, acc, counts)
    print(acc_report)
    notes = notes + acc_notes

    if args.json:
        record = {
            "input": str(source_path),
            "counts": counts,
            "run_metadata": meta,
            "mechanism_summary": summary,
            "sample_interval_ms": intervals,
            "fault_distribution": faults,
            "mode_distribution": modes,
            "preload_state_distribution": preloads,
            "boundary_state_distribution": boundaries,
            "acceptance": {
                "pointing_rmse": acc["pointing_rmse"],
                "control_period": acc["control_period"],
                "lock_time": acc["lock_time"],
            },
            "targets": {
                "recover_ratio": RECOVER_TARGET,
                "index_ok_ratio": INDEX_OK_TARGET,
                "fault_rows": FAULT_FREE_TARGET,
                "static_rmse_deg": STATIC_RMSE_TARGET_DEG,
                "control_period_ms": CONTROL_PERIOD_TARGET_MS,
                "lock_time_ms": LOCK_TIME_TARGET_MS,
            },
            "notes": notes,
        }
        Path(args.json).write_text(json.dumps(record, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
        print(f"  已写出 JSON 报告：{args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
