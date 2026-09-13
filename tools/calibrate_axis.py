"""轴角标定：由像素与舵盘角对应点解出 2x3 仿射矩阵。

链路是 像素 -> (bearing, elevation) -> 仿射 -> (pan, tilt)，角度换算和最小二乘都调
algo_reference，脚本只负责读数据、留出验证和报告。

输入 CSV 表头：px,py,pan_deg,tilt_deg
    px, py      目标在源图像里的像素坐标（与内参同一分辨率）
    pan_deg     该点在画面中央时舵盘的实际水平角
    tilt_deg    同上的俯仰角
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

import common
from common import (
    DEFAULT_CONFIG_PATH,
    DEFAULT_SAMPLE_AXIS_CSV,
    ToolError,
    c_float,
    die,
    grade_rmse,
    warn,
)

from aim.geometry import apply_affine, pixel_to_angles, solve_affine_angle_to_pan_tilt
from aim.types import CameraIntrinsics

AXIS_COLUMNS = ("px", "py", "pan_deg", "tilt_deg")


def load_axis_points(path) -> np.ndarray:
    """读标定点，返回 (N,4)：px, py, pan_deg, tilt_deg。"""
    rows = []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ToolError(f"{path} 是空文件，需要表头 {','.join(AXIS_COLUMNS)}")
        missing = [name for name in AXIS_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ToolError(
                f"{path} 缺少列 {', '.join(missing)}",
                hint=f"表头应为 {','.join(AXIS_COLUMNS)}",
            )
        for line_no, raw in enumerate(reader, start=2):
            try:
                values = [float(raw[name]) for name in AXIS_COLUMNS]
            except (TypeError, ValueError):
                raise ToolError(f"{path} 第 {line_no} 行有非数值字段")
            if not all(np.isfinite(values)):
                raise ToolError(f"{path} 第 {line_no} 行含 inf/nan")
            rows.append(values)
    if len(rows) < 3:
        raise ToolError(
            f"{path} 只有 {len(rows)} 个有效点，仿射至少要 3 个",
            hint="把标定点补到 9 个以上，并覆盖画面四角",
        )
    return np.asarray(rows, dtype=np.float64)


def pixels_to_angles_deg(pixels: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    pts = np.atleast_2d(np.asarray(pixels, dtype=np.float64))
    bearing_rad, elevation_rad = pixel_to_angles(pts, intrinsics)
    return np.column_stack([np.degrees(bearing_rad), np.degrees(elevation_rad)])


def split_indices(n: int, holdout_frac: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """固定 seed 随机留出验证点，训练集至少保留 3 个。"""
    indices = np.arange(n)
    n_hold = max(0, min(int(round(n * holdout_frac)), n - 3))
    if n_hold == 0:
        return indices, np.empty(0, dtype=int)
    rng = np.random.default_rng(seed)
    shuffled = rng.permutation(indices)
    return np.sort(shuffled[n_hold:]), np.sort(shuffled[:n_hold])


def fit_affine(angle_pts: np.ndarray, pan_tilt_pts: np.ndarray) -> np.ndarray:
    return solve_affine_angle_to_pan_tilt(angle_pts, pan_tilt_pts)


def affine_rmse(matrix, angle_pts: np.ndarray, pan_tilt_pts: np.ndarray) -> dict:
    """在给定点对上算 RMSE，单位为度。"""
    angle = np.atleast_2d(np.asarray(angle_pts, dtype=np.float64))
    target = np.atleast_2d(np.asarray(pan_tilt_pts, dtype=np.float64))
    residual = apply_affine(matrix, angle) - target
    return {
        "n": int(angle.shape[0]),
        "rmse_deg": float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))),
        "rmse_pan_deg": float(np.sqrt(np.mean(residual[:, 0] ** 2))),
        "rmse_tilt_deg": float(np.sqrt(np.mean(residual[:, 1] ** 2))),
    }


def coverage_metrics(pixels: np.ndarray, width: int, height: int) -> dict:
    """标定点铺得够不够开，返回视野跨度与包围盒占比。"""
    pts = np.atleast_2d(np.asarray(pixels, dtype=np.float64))
    span_x = float(pts[:, 0].max() - pts[:, 0].min()) / max(width, 1)
    span_y = float(pts[:, 1].max() - pts[:, 1].min()) / max(height, 1)
    return {
        "span_x_ratio": span_x,
        "span_y_ratio": span_y,
        "bbox_area_ratio": span_x * span_y,
        "outside_image": int(np.sum(
            (pts[:, 0] < 0) | (pts[:, 0] > width - 1)
            | (pts[:, 1] < 0) | (pts[:, 1] > height - 1)
        )),
    }


def format_affine_c(matrix: np.ndarray, rmse_deg: float, source_note: str) -> str:
    """生成对应 CalibrationData::affine 的 C 初始化片段。"""
    lines = [
        "// 轴角标定结果：q = A * (bearing_deg, elevation_deg, 1)^T，单位度。",
        f"// 留出点 RMSE {rmse_deg:.4f} 度，来自 {source_note}。",
        "// 对应 CalibrationData::affine，可作 NVS 初始值或编译期兜底。",
        "static const float kCalibAffineAngleToPanTilt[2][3] = {",
    ]
    for row in matrix:
        cells = ", ".join(c_float(v) for v in row)
        lines.append(f"    {{ {cells} }},")
    lines.append("};")
    return "\n".join(lines) + "\n"


def _resolve_intrinsics(args) -> tuple[CameraIntrinsics, str]:
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
        cx=args.cx, cy=args.cy, dist=(args.k1, args.k2, args.p1, args.p2, args.k3))
    if any(v is not None for v in (args.width, args.height, args.fx, args.fy,
                                   args.cx, args.cy, args.k1, args.k2,
                                   args.p1, args.p2, args.k3)):
        source += " + 命令行覆盖"
    return intr, source


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="由像素-舵盘角对应点最小二乘解出 2x3 仿射矩阵，并用留出点评估。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("csv", nargs="?", default=str(DEFAULT_SAMPLE_AXIS_CSV),
                        help="标定点 CSV，表头 px,py,pan_deg,tilt_deg")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH),
                        help="config.h 路径，用它的图像尺寸和内参宏（若有）")
    parser.add_argument("--intrinsics-json", default=None,
                        help="内参 JSON（calibrate_intrinsics.py 的输出），优先于 config.h")
    parser.add_argument("--width", type=int, default=None, help="源图像宽，覆盖内参")
    parser.add_argument("--height", type=int, default=None, help="源图像高，覆盖内参")
    parser.add_argument("--fx", type=float, default=None, help="焦距 fx（px）")
    parser.add_argument("--fy", type=float, default=None, help="焦距 fy（px）")
    parser.add_argument("--cx", type=float, default=None, help="主点 cx（px）")
    parser.add_argument("--cy", type=float, default=None, help="主点 cy（px）")
    parser.add_argument("--k1", type=float, default=None, help="径向畸变 k1")
    parser.add_argument("--k2", type=float, default=None, help="径向畸变 k2")
    parser.add_argument("--p1", type=float, default=None, help="切向畸变 p1")
    parser.add_argument("--p2", type=float, default=None, help="切向畸变 p2")
    parser.add_argument("--k3", type=float, default=None, help="径向畸变 k3")
    parser.add_argument("--holdout", type=float, default=0.3,
                        help="留出验证比例，0 表示全部点参与拟合")
    parser.add_argument("--seed", type=int, default=0, help="留出划分的随机种子")
    parser.add_argument("--json", default="calibrate_axis.json", help="JSON 存档输出路径")
    parser.add_argument("--c-code", default="calibrate_axis_config.h.txt",
                        help="C 代码片段输出路径")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not 0.0 <= args.holdout < 1.0:
        die(f"--holdout 应在 [0, 1) 内，收到 {args.holdout}")

    try:
        data = load_axis_points(args.csv)
        intr, source = _resolve_intrinsics(args)
    except ToolError as exc:
        die(str(exc), exc.hint)

    pixels = data[:, :2]
    pan_tilt = data[:, 2:4]
    if intr.width <= 0 or intr.height <= 0:
        die(f"内参尺寸不合法：{intr.width}x{intr.height}")

    angle_pts = pixels_to_angles_deg(pixels, intr)
    train_idx, hold_idx = split_indices(len(data), args.holdout, args.seed)
    matrix = fit_affine(angle_pts[train_idx], pan_tilt[train_idx])
    train_rmse = affine_rmse(matrix, angle_pts[train_idx], pan_tilt[train_idx])
    if len(hold_idx) > 0:
        eval_rmse = affine_rmse(matrix, angle_pts[hold_idx], pan_tilt[hold_idx])
    else:
        warn("没有留出点，RMSE 用训练集自身，数值会偏乐观")
        eval_rmse = train_rmse

    cover = coverage_metrics(pixels, intr.width, intr.height)

    print("轴角标定")
    print(f"  输入：{args.csv}，共 {len(data)} 点")
    print(f"  内参：fx={intr.fx:.3f} fy={intr.fy:.3f} "
          f"cx={intr.cx:.3f} cy={intr.cy:.3f}，来源 {source}")
    print(f"  划分：训练 {len(train_idx)} 点，留出 {len(hold_idx)} 点，seed={args.seed}")
    print("  仿射矩阵 A（输入 bearing/elevation 度，输出 pan/tilt 度）：")
    for row in matrix:
        print("      [ " + "  ".join(f"{v: .6f}" for v in row) + " ]")
    print(f"  留出 RMSE：{eval_rmse['rmse_deg']:.4f} 度 "
          f"（pan {eval_rmse['rmse_pan_deg']:.4f}，tilt {eval_rmse['rmse_tilt_deg']:.4f}）")
    print(f"  训练 RMSE：{train_rmse['rmse_deg']:.4f} 度")
    print(f"  第 10 节评价：{grade_rmse(eval_rmse['rmse_deg'])}")
    print(f"  点分布：x 跨 {cover['span_x_ratio'] * 100:.1f}% 视野，"
          f"y 跨 {cover['span_y_ratio'] * 100:.1f}%，包围盒占比 "
          f"{cover['bbox_area_ratio'] * 100:.1f}%")
    if cover["outside_image"]:
        warn(f"有 {cover['outside_image']} 个点落在图像范围外，检查像素坐标")
    if cover["bbox_area_ratio"] < 0.5:
        warn("标定点偏集中，画面边缘以外要靠外推，那里误差会被放大")
        print("  建议：补采画面四角与边缘的点后重跑")

    c_code = format_affine_c(matrix, eval_rmse["rmse_deg"], Path(args.csv).name)
    Path(args.c_code).write_text(c_code, encoding="utf-8")
    record = {
        "input": str(args.csv),
        "intrinsics": {
            "width": intr.width, "height": intr.height,
            "fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy,
            "dist_coeffs": list(intr.dist_coeffs), "source": source,
        },
        "seed": args.seed,
        "holdout_fraction": args.holdout,
        "train_count": int(len(train_idx)),
        "holdout_count": int(len(hold_idx)),
        "affine_angle_to_pan_tilt": matrix.tolist(),
        "rmse_deg": eval_rmse["rmse_deg"],
        "rmse_pan_deg": eval_rmse["rmse_pan_deg"],
        "rmse_tilt_deg": eval_rmse["rmse_tilt_deg"],
        "train_rmse_deg": train_rmse["rmse_deg"],
        "grade": grade_rmse(eval_rmse["rmse_deg"]),
        "coverage": cover,
    }
    Path(args.json).write_text(json.dumps(record, ensure_ascii=False, indent=2),
                               encoding="utf-8")
    print(f"  已写出：{args.json}，{args.c_code}")
    print("  下一步：RPE 与 RMSE 都达标后，把 C 片段里的矩阵写入 NVS 初始值，"
          "再跑 analyze_telemetry.py 看整机精度")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
