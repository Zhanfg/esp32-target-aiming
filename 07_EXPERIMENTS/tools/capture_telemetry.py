"""串口采集固件遥测，只留 AIM 数据行，写 CSV 并打印采样情况。

跑法：
    python capture_telemetry.py --port COM5 --duration 20 --out run1.csv
    python capture_telemetry.py --list

依赖 pyserial，缺失时打印安装提示后退出。
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from common import TELEMETRY_FIELDS, parse_aim_line, state_name

# 首行写成注释，read_aim_telemetry 会自然跳过，文件本身还能看懂列含义。
FIELD_HEADER = "# " + ",".join(["AIM", *TELEMETRY_FIELDS])


def import_serial():
    try:
        import serial  # noqa: F401
    except ImportError:
        return None
    return serial


def list_serial_ports(serial_mod) -> list:
    from serial.tools import list_ports as list_ports_mod
    return list(list_ports_mod.comports())


def describe_ports(serial_mod) -> str:
    ports = list_serial_ports(serial_mod)
    if not ports:
        return "  当前没有检测到串口"
    return "\n".join(f"  {p.device}  {p.description or '无描述'}" for p in ports)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="采集固件以 AIM, 开头的遥测行，写 CSV 并统计采样率与状态分布。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=None, help="串口名，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--duration", type=float, default=None,
                        help="采集时长（秒），与 --lines 二选一，都不给按 10 秒")
    parser.add_argument("--lines", type=int, default=None, help="采集到这么多 AIM 行后停止")
    parser.add_argument("--out", default="telemetry.csv", help="输出 CSV 路径")
    parser.add_argument("--list", action="store_true", help="列出可用串口后退出")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    serial = import_serial()
    if serial is None:
        print("错误：缺少 pyserial，无法访问串口", file=sys.stderr)
        print("提示：pip install pyserial；或 python -m pip install -r 07_EXPERIMENTS/tools/requirements.txt",
              file=sys.stderr)
        return 2

    if args.list:
        print("可用串口：")
        print(describe_ports(serial))
        return 0

    if not args.port:
        print("错误：没有指定 --port", file=sys.stderr)
        print("提示：先跑 python capture_telemetry.py --list 看有哪些串口", file=sys.stderr)
        return 2
    if args.duration is not None and args.duration <= 0:
        print("错误：--duration 必须为正数", file=sys.stderr)
        return 2
    if args.lines is not None and args.lines <= 0:
        print("错误：--lines 必须为正数", file=sys.stderr)
        return 2
    duration = args.duration if args.duration is not None else (
        None if args.lines is not None else 10.0)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"错误：打不开串口 {args.port}：{exc}", file=sys.stderr)
        print("提示：确认串口名没写错、没被串口监视器占用。当前可用：", file=sys.stderr)
        print(describe_ports(serial), file=sys.stderr)
        return 3

    out_path = Path(args.out)
    kept = skipped = 0
    state_counts: dict[int, int] = {}
    kept_times: list[float] = []
    deadline = time.monotonic() + duration if duration else None
    stop_reason = "时长到"

    print(f"采集 {args.port} @ {args.baud}，输出 {out_path}"
          + (f"，时长 {duration:.1f}s" if duration else ""))
    try:
        with out_path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow([FIELD_HEADER])
            while True:
                if args.lines is not None and kept >= args.lines:
                    stop_reason = "行数到"
                    break
                if deadline is not None and time.monotonic() >= deadline:
                    break
                try:
                    raw = ser.readline()
                except serial.SerialException as exc:
                    print(f"警告：串口读取中断：{exc}", file=sys.stderr)
                    print("提示：检查 USB 连接与供电，重新插拔后重跑；已采集的数据保留在 "
                          + str(out_path), file=sys.stderr)
                    stop_reason = "串口断开"
                    break
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace")
                row = parse_aim_line(text)
                if row is None:
                    skipped += 1
                    continue
                writer.writerow([text.strip()])
                kept += 1
                kept_times.append(time.monotonic())
                state_counts[int(row["state"])] = state_counts.get(int(row["state"]), 0) + 1
    finally:
        ser.close()

    print(f"  停止：{stop_reason}")
    print(f"  AIM 行 {kept}，丢弃非 AIM/畸形行 {skipped}")
    if len(kept_times) >= 2:
        span = kept_times[-1] - kept_times[0]
        rate = (len(kept_times) - 1) / span if span > 0 else 0.0
        print(f"  实测采样率 {rate:.1f} Hz（按到达时间算，{span:.2f}s 窗口）")
    else:
        print("  样本太少，算不出采样率")
    if state_counts:
        print("  状态分布：")
        for code in sorted(state_counts):
            share = state_counts[code] / kept * 100.0
            print(f"    {state_name(code):<11} {state_counts[code]:>6}  {share:5.1f}%")
    else:
        print("  没有采到 AIM 行，检查波特率与 TELEMETRY 输出开关")
    print(f"  已写出 {out_path}，用 analyze_telemetry.py 继续算精度")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
