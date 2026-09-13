"""重新生成 data/sample_track.csv 的脚本，不要手改 CSV。

固定 seed 调用 aim.sim.make_occluded_track，保证重跑结果逐字节一致。

    python data/make_sample_track.py

脚本会自行把 src 加入 sys.path，从任意工作目录运行都可以。
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src"))

from aim import CameraIntrinsics, make_occluded_track  # noqa: E402

# 生成参数集中在此，README 里描述的关键数值必须与这里一致。
N = 120
DT_MS = 20.0
PIXEL0 = (200.0, 240.0)
VELOCITY_PX_S = (80.0, 0.0)
NOISE_PX = 0.3
SEED = 20240913
GAP_LEN = 15
GAP_START = 60


def main() -> Path:
    """生成 CSV 并返回写入路径。"""
    intrinsics = CameraIntrinsics.default_for(640, 480)
    observations = make_occluded_track(
        N, DT_MS, PIXEL0, VELOCITY_PX_S, intrinsics,
        noise_px=NOISE_PX, seed=SEED, gap_start=GAP_START, gap_len=GAP_LEN,
    )
    out_path = _ROOT / "data" / "sample_track.csv"
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t_ms", "px", "py", "confidence"])
        for obs in observations:
            writer.writerow([
                f"{obs.t_ms:.1f}",
                f"{obs.x:.4f}",
                f"{obs.y:.4f}",
                f"{obs.confidence:.1f}",
            ])
    return out_path


if __name__ == "__main__":
    path = main()
    print(f"wrote {path} ({N} rows, seed={SEED}, gap=[{GAP_START}, {GAP_START + GAP_LEN}))")
