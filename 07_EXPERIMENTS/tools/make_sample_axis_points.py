"""生成 sample_axis_points.csv 的脚本，不要手改 CSV。

样例由虚拟内参和虚拟仿射反算而来，只为端到端跑通 calibrate_axis.py，不代表任何真实设备。
实际标定要在自己机器上打点。

    python make_sample_axis_points.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from common import DEFAULT_SAMPLE_AXIS_CSV

from aim.geometry import apply_affine, pixel_to_angles
from aim.types import CameraIntrinsics

# 与 config.h 的 VISION_SRC_W/H 一致，calibrate_axis.py 不传内参时也用这套。
INTRINSICS = CameraIntrinsics.default_for(320, 240)

AFFINE = np.array([
    [1.03, 0.02, -0.60],
    [0.01, 1.02, 0.90],
])

# 四角、四边中点、中心加四个内圈点，铺开比全挤在中央更能暴露外推误差。
PIXELS = np.array([
    [20.0, 20.0], [160.0, 18.0], [300.0, 22.0],
    [18.0, 120.0], [160.0, 120.0], [302.0, 120.0],
    [22.0, 220.0], [160.0, 222.0], [298.0, 218.0],
    [90.0, 60.0], [230.0, 60.0], [90.0, 180.0], [230.0, 180.0],
])

NOISE_DEG = 0.03
SEED = 7


def generate() -> np.ndarray:
    """返回 (N,4)：px, py, pan_deg, tilt_deg。"""
    bearing, elevation = pixel_to_angles(PIXELS, INTRINSICS)
    angles = np.column_stack([np.degrees(bearing), np.degrees(elevation)])
    pan_tilt = apply_affine(AFFINE, angles)
    rng = np.random.default_rng(SEED)
    pan_tilt = pan_tilt + rng.normal(0.0, NOISE_DEG, size=pan_tilt.shape)
    return np.column_stack([PIXELS, pan_tilt])


def main() -> Path:
    data = generate()
    lines = ["px,py,pan_deg,tilt_deg"]
    for row in data:
        lines.append(f"{row[0]:.1f},{row[1]:.1f},{row[2]:.5f},{row[3]:.5f}")
    DEFAULT_SAMPLE_AXIS_CSV.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return DEFAULT_SAMPLE_AXIS_CSV


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="重新生成 sample_axis_points.csv。")
    parser.add_argument("--stdout", action="store_true", help="只打印结果，不写文件")
    args = parser.parse_args()
    if args.stdout:
        for row in generate():
            print(f"{row[0]:.1f},{row[1]:.1f},{row[2]:.5f},{row[3]:.5f}")
    else:
        path = main()
        print(f"wrote {path} ({len(generate())} rows, seed={SEED}, "
              f"内参 {INTRINSICS.width}x{INTRINSICS.height})")
