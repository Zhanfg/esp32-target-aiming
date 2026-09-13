"""棋盘格内参标定，输出 JSON 存档和一份可粘进 config.h 的 C 代码。

跑法：
    python calibrate_intrinsics.py <图片目录> --cols 9 --rows 6 --square 25

cols/rows 是内角点数，不是方格总数。--square 只影响外参平移量，内参本身与它无关，
但填错会误导后面按角点算距离，还是按实际量。

依赖 opencv-python，缺失时给出安装提示后退出。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from common import c_float, die, grade_rmse, warn

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")


def import_cv2():
    """延迟导入 OpenCV，缺失时返回 None，让 --help 和参数校验仍可用。"""
    try:
        import cv2  # noqa: F401
    except ImportError:
        return None
    return cv2


def collect_images(directory: Path, suffixes=IMAGE_SUFFIXES) -> list[Path]:
    return [p for p in sorted(directory.iterdir())
            if p.is_file() and p.suffix.lower() in suffixes]


def calibrate(cv2, images: list[Path], cols: int, rows: int, square: float):
    """检测角点并标定，返回 (内参, 畸变, 各视图 RMS, 图像尺寸, 失败列表)。"""
    pattern = (cols, rows)
    objp = np.zeros((rows * cols, 3), np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    object_points, image_points = [], []
    per_view_rms = []
    size = None
    failed = []
    for path in images:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            failed.append((path.name, "读不出图像"))
            continue
        if size is None:
            size = (gray.shape[1], gray.shape[0])
        elif (gray.shape[1], gray.shape[0]) != size:
            failed.append((path.name, f"尺寸 {gray.shape[1]}x{gray.shape[0]} 与首张不一致"))
            continue
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not found:
            failed.append((path.name, "没找到棋盘格角点"))
            continue
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(objp)
        image_points.append(corners)

    if not object_points:
        return None, None, [], size, failed

    ok, camera_matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, size, None, None)
    if not ok:
        return None, None, [], size, failed

    # 逐视图重投影 RMS 比只看总体值更能定位坏图。
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist)
        err = img.reshape(-1, 2) - projected.reshape(-1, 2)
        per_view_rms.append(float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))))
    return camera_matrix, dist.reshape(-1), per_view_rms, size, failed


def format_c_code(camera_matrix, dist, size, rpe: float) -> str:
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    k1, k2, p1, p2, k3 = (list(dist) + [0.0] * 5)[:5]
    lines = [
        "// 由 tools/calibrate_intrinsics.py 生成。",
        f"// 标定分辨率 {size[0]}x{size[1]}，重投影 RMSE {rpe:.4f} px。",
        "// config.h 目前没有内参宏，固件运行时从 NVS 读（见 calibration.h）；",
        "// 下面这段可作为编译期兜底值加进 config.h 的 VISION 段，或抄进 NVS 初始值。",
        f"#define CAM_FX        {c_float(fx)}",
        f"#define CAM_FY        {c_float(fy)}",
        f"#define CAM_CX        {c_float(cx)}",
        f"#define CAM_CY        {c_float(cy)}",
        f"#define CAM_DIST_K1   {c_float(k1)}",
        f"#define CAM_DIST_K2   {c_float(k2)}",
        f"#define CAM_DIST_P1   {c_float(p1)}",
        f"#define CAM_DIST_P2   {c_float(p2)}",
        f"#define CAM_DIST_K3   {c_float(k3)}",
    ]
    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="棋盘格内参标定，输出 JSON 和可粘进 config.h 的 C 代码。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("images", help="棋盘格图片所在目录")
    parser.add_argument("--cols", type=int, default=9, help="内角点列数")
    parser.add_argument("--rows", type=int, default=6, help="内角点行数")
    parser.add_argument("--square", type=float, default=25.0, help="方格边长（mm）")
    parser.add_argument("--json", default="calibrate_intrinsics.json", help="JSON 存档输出路径")
    parser.add_argument("--c-code", default="calibrate_intrinsics_config.h.txt",
                        help="C 代码片段输出路径")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.cols < 2 or args.rows < 2:
        die("棋盘格内角点至少 2x2", hint="常见板子是 9x6 或 7x5，按实物填")
    if args.square <= 0:
        die("--square 必须为正数")

    directory = Path(args.images)
    if not directory.is_dir():
        die(f"目录不存在：{directory}", hint="传入存放棋盘格照片的目录")
    images = collect_images(directory)
    if not images:
        die(
            f"{directory} 里没有图片",
            hint="支持 " + "、".join(IMAGE_SUFFIXES) + "；照片里棋盘格要完整、清晰",
        )

    cv2 = import_cv2()
    if cv2 is None:
        die(
            "缺少 opencv-python，无法做棋盘格标定",
            hint="pip install opencv-python；或 python -m pip install -r tools/requirements.txt",
        )

    print(f"棋盘格内参标定：{directory}，{len(images)} 张图片")
    camera_matrix, dist, per_view_rms, size, failed = calibrate(
        cv2, images, args.cols, args.rows, args.square)
    for name, reason in failed:
        warn(f"{name}：{reason}")

    if camera_matrix is None:
        die(
            "没有一张图片成功检测到棋盘格",
            hint=f"确认内角点数是 {args.cols}x{args.rows}，且整块棋盘都在画面内",
        )

    rpe = float(np.sqrt(np.mean(np.square(per_view_rms))))
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

    print(f"  成功检测 {len(per_view_rms)} 张，尺寸 {size[0]}x{size[1]}")
    print(f"  fx={fx:.3f}  fy={fy:.3f}  cx={cx:.3f}  cy={cy:.3f}")
    dist_list = (list(dist) + [0.0] * 5)[:5]
    print("  畸变 (k1,k2,p1,p2,k3)=" + ", ".join(f"{v:.6f}" for v in dist_list))
    print(f"  各视图 RMS：最小 {min(per_view_rms):.4f}，最大 {max(per_view_rms):.4f} px")
    print(f"  重投影 RMSE（RPE）：{rpe:.4f} px")
    print(f"  第 10 节分级（RPE 按像素套同一张表）：{grade_rmse(rpe)}")
    if rpe > 0.5:
        print("  建议：换更多姿态、覆盖画面四角再标一次，单靠中心区域标定边缘会不准")

    c_code = format_c_code(camera_matrix, dist_list, size, rpe)
    Path(args.c_code).write_text(c_code, encoding="utf-8")
    record = {
        "source_dir": str(directory),
        "pattern_cols": args.cols,
        "pattern_rows": args.rows,
        "square_mm": args.square,
        "image_size": {"width": size[0], "height": size[1]},
        "image_count": len(per_view_rms),
        "failed_images": [{"name": n, "reason": r} for n, r in failed],
        "camera_matrix": camera_matrix.tolist(),
        "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
        "dist_coeffs": [float(v) for v in dist_list],
        "rpe_px": rpe,
        "per_view_rms_px": per_view_rms,
        "grade": grade_rmse(rpe),
    }
    Path(args.json).write_text(json.dumps(record, ensure_ascii=False, indent=2),
                               encoding="utf-8")
    print(f"  已写出：{args.json}，{args.c_code}")
    print("  下一步：把内参交给 calibrate_axis.py（--intrinsics-json），然后在真机上打点")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
