"""工具链共用件：algo_reference 路径注入、遥测行解析、config.h 宏读取。

字段顺序和错误提示只在这里定义一次，免得固件改了列顺序两边对不上。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ALGO_SRC = PROJECT_ROOT / "05_FIRMWARE" / "algo_reference" / "src"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "05_FIRMWARE" / "include" / "config.h"
DEFAULT_SAMPLE_AXIS_CSV = Path(__file__).resolve().parent / "sample_axis_points.csv"

# 调用的就是固件照抄的那份几何函数，PC 端不另写一份数学。
if str(ALGO_SRC) not in sys.path:
    sys.path.insert(0, str(ALGO_SRC))


class ToolError(Exception):
    """面向使用者的错误，附带一句能照做的提示。"""

    def __init__(self, message: str, hint: str | None = None):
        super().__init__(message)
        self.hint = hint


def die(message: str, hint: str | None = None, code: int = 2) -> None:
    print(f"错误：{message}", file=sys.stderr)
    if hint:
        print(f"提示：{hint}", file=sys.stderr)
    raise SystemExit(code)


def warn(message: str) -> None:
    print(f"警告：{message}", file=sys.stderr)


MST_PREFIX = "MST,"

# 顺序与 05_FIRMWARE/src/comms/telemetry.cpp 的 emitLine() printf 逐字一致：
#   "MST,%lu,%u,%s,%s,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u,%s\n"
# 字段含义、单位、取值范围见 08_DATA/DATA_SCHEMA.md 第 1 节。
TELEMETRY_FIELDS = [
    "timestamp", "experiment_id", "prototype_version", "mechanism_version",
    "mode", "boundary_state", "preload_state", "membrane_id", "payload_id",
    "cycle_count", "stage1_event", "stage2_event", "mechanism_recovered",
    "magazine_position", "magazine_index_ok", "fault_code", "operator_note",
]

# %s 打印的字段读成字符串，其余 %lu/%u 打印的字段读成整数。
_STRING_FIELDS = {"prototype_version", "mechanism_version", "operator_note"}

DIAG_PREFIX = "DIAG,"

# 顺序与 05_FIRMWARE/src/comms/telemetry.cpp 的 telemetryEmitDiag() printf 逐字一致：
#   "DIAG,%lu,%lu,%u,%u,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u\n"
# 每控制拍一行，用于算指向精度、控制周期与锁定建立时间，与 MST, 行互不影响。
DIAG_FIELDS = [
    "timestamp_ms", "loop_us", "obs_valid", "obs_dropped", "px", "py", "confidence",
    "pan_deg", "tilt_deg", "pan_target_deg", "tilt_target_deg",
    "err_pan_deg", "err_tilt_deg", "err_deg", "aim_state", "has_feedback",
]

# 整数字段，其余按小数解析。
_DIAG_INT_FIELDS = {"timestamp_ms", "loop_us", "obs_valid", "obs_dropped",
                    "aim_state", "has_feedback"}

# 以下枚举值与 05_FIRMWARE/src/aim_types.h 的顺序一致。遥测行只直接输出
# fault_code / mode / boundary_state / preload_state，其余状态在标定外壳的 ST, 行里。
CYCLE_STATE_NAMES = {
    0: "BOOT", 1: "SAFE", 2: "HOME", 3: "READY", 4: "AIM_ALLOWED",
    5: "PRELOAD", 6: "ARMED", 7: "RELEASE", 8: "RECOVER", 9: "INDEX",
    10: "FAULT",
}
AIM_STATE_NAMES = {
    0: "IDLE", 1: "CALIBRATING", 2: "SEARCHING", 3: "TRACKING",
    4: "LOCKED", 6: "CALIB_MODE",
}
PRELOAD_STATE_NAMES = {0: "IDLE", 1: "PRELOADING", 2: "ARMED"}
BOUNDARY_STATE_NAMES = {0: "BOUNDARY_0", 1: "BOUNDARY_1"}
FIRE_MODE_NAMES = {0: "AIR_ONLY", 1: "SOFT_PAYLOAD"}
FAULT_CODE_NAMES = {
    0: "NONE", 1: "PERIPH_INIT", 2: "HOME_TIMEOUT", 3: "HOME_SWITCH_CONFLICT",
    4: "RELEASE_GATE_SELFTEST", 5: "OBS_LINK", 6: "INDEX_FAIL", 7: "WATCHDOG",
    8: "MECH_STUCK", 9: "ESTOP", 10: "MEMBRANE_RUPTURE", 11: "DOUBLE_FEED",
    12: "FIRE_INHIBIT_SHORT", 13: "I2C_BUS_HANG", 14: "STATE_ILLEGAL",
}
# 分级与 aim_types.h 的 faultSeverity() 一致：0 无，2/5/6/7/9/11 软，其余硬。
FAULT_SEVERITY_NAMES = {
    0: "NONE",
    2: "SOFT", 5: "SOFT", 6: "SOFT", 7: "SOFT", 9: "SOFT", 11: "SOFT",
    1: "HARD", 3: "HARD", 4: "HARD", 8: "HARD", 10: "HARD", 12: "HARD",
    13: "HARD", 14: "HARD",
}


def _enum_name(table, code, fallback: str = "UNKNOWN") -> str:
    try:
        return table.get(int(code), f"{fallback}({int(code)})")
    except (TypeError, ValueError):
        return f"{fallback}({code})"


def cycle_state_name(code) -> str:
    return _enum_name(CYCLE_STATE_NAMES, code)


def aim_state_name(code) -> str:
    return _enum_name(AIM_STATE_NAMES, code)


def preload_state_name(code) -> str:
    return _enum_name(PRELOAD_STATE_NAMES, code)


def boundary_state_name(code) -> str:
    return _enum_name(BOUNDARY_STATE_NAMES, code)


def fire_mode_name(code) -> str:
    return _enum_name(FIRE_MODE_NAMES, code)


def fault_code_name(code) -> str:
    return _enum_name(FAULT_CODE_NAMES, code)


def fault_severity_name(code) -> str:
    return _enum_name(FAULT_SEVERITY_NAMES, code)


def parse_mst_line(line: str):
    """解析一行 MST 遥测，前缀不对、列数不对或数值字段非法都返回 None。

    固件上电时打印的表头行也以 MST, 开头，但数值列是字段名，会在转 int 时被拒。
    """
    text = line.strip()
    if not text.startswith(MST_PREFIX):
        return None
    parts = text[len(MST_PREFIX):].split(",")
    if len(parts) != len(TELEMETRY_FIELDS):
        return None
    row = {}
    for name, token in zip(TELEMETRY_FIELDS, parts):
        token = token.strip()
        if name in _STRING_FIELDS:
            row[name] = token
            continue
        try:
            row[name] = int(token)
        except ValueError:
            return None
    return row


def read_mst_telemetry(path) -> tuple[list[dict], dict]:
    """返回 (MST 行列表, 计数统计)，统计键为 total/kept/skipped。"""
    rows: list[dict] = []
    total = 0
    with Path(path).open("r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            total += 1
            row = parse_mst_line(raw)
            if row is not None:
                rows.append(row)
    return rows, {"total": total, "kept": len(rows), "skipped": total - len(rows)}


def parse_diag_line(line: str):
    """解析一行 DIAG 帧轨迹，前缀不对、列数不对或数值字段非法都返回 None。

    固件上电时打印的 DIAG 表头行字段名会被转 float/int 时拒掉，与 MST 表头同理。
    """
    text = line.strip()
    if not text.startswith(DIAG_PREFIX):
        return None
    parts = text[len(DIAG_PREFIX):].split(",")
    if len(parts) != len(DIAG_FIELDS):
        return None
    row = {}
    for name, token in zip(DIAG_FIELDS, parts):
        token = token.strip()
        try:
            row[name] = int(token) if name in _DIAG_INT_FIELDS else float(token)
        except ValueError:
            return None
    return row


def read_diag_telemetry(path) -> tuple[list[dict], dict]:
    """返回 (DIAG 行列表, 计数统计)，统计键为 total/kept/skipped。"""
    rows: list[dict] = []
    total = 0
    with Path(path).open("r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            total += 1
            row = parse_diag_line(raw)
            if row is not None:
                rows.append(row)
    return rows, {"total": total, "kept": len(rows), "skipped": total - len(rows)}


def read_telemetry(path) -> tuple[list[dict], list[dict], dict]:
    """一次读入混合 CSV，按前缀分流成 (MST 行, DIAG 行, 计数)。

    计数键：total 总行数，mst 命中 MST 行数，diag 命中 DIAG 行数，skipped 两者都不匹配
    且非注释非空的行数。空行与 `#` 注释行不计入 skipped。
    """
    mst_rows: list[dict] = []
    diag_rows: list[dict] = []
    total = 0
    skipped = 0
    with Path(path).open("r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            total += 1
            text = raw.strip()
            if not text or text.startswith("#"):
                continue
            mst = parse_mst_line(raw)
            if mst is not None:
                mst_rows.append(mst)
                continue
            diag = parse_diag_line(raw)
            if diag is not None:
                diag_rows.append(diag)
                continue
            skipped += 1
    counts = {"total": total, "mst": len(mst_rows), "diag": len(diag_rows),
              "skipped": skipped}
    return mst_rows, diag_rows, counts


_MACRO_RE = re.compile(r"^\s*#define\s+([A-Za-z_]\w*)\s+(.+?)\s*$")
_IDENT_RE = re.compile(r"[A-Za-z_]\w*")
_ARITH_RE = re.compile(r"[A-Za-z_0-9\s+\-*/().]+")
_NUM_RE = re.compile(r"\(?\s*([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)\s*[fFlL]?\s*\)?")


def _as_float(body: str):
    text = body.strip()
    if not text or text.startswith('"'):
        return None
    match = _NUM_RE.fullmatch(text)
    if match:
        return float(match.group(1))
    return None


def _eval_expr(body: str, table: dict):
    text = body.strip()
    if not _ARITH_RE.fullmatch(text):
        return None
    for ident in set(_IDENT_RE.findall(text)):
        if ident not in table:
            return None
        value = table[ident]
        if not isinstance(value, (int, float)):
            return None
        text = re.sub(rf"\b{re.escape(ident)}\b", f"({float(value)!r})", text)
    try:
        return float(eval(text, {"__builtins__": {}}, {}))
    except (SyntaxError, ZeroDivisionError, ValueError):
        return None


def parse_config_macros(path=DEFAULT_CONFIG_PATH) -> dict:
    """读取 config.h 的对象式宏，能算出数值的转 float，其余保留原文。

    函数式宏不匹配正则，直接跳过。宏体里的表达式按依赖关系多轮代入，像
    `CONTROL_LOOP_BUDGET_MS = 1000 / CONTROL_LOOP_HZ` 也能算出来。
    """
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    raw: dict[str, str] = {}
    for line in text.splitlines():
        line = line.split("//", 1)[0]
        match = _MACRO_RE.match(line)
        if match:
            raw[match.group(1)] = match.group(2).strip()

    resolved: dict[str, object] = {}
    for _ in range(32):
        progressed = False
        for name, body in raw.items():
            if name in resolved:
                continue
            value = _as_float(body)
            if value is None:
                value = _eval_expr(body, resolved)
            if value is not None:
                resolved[name] = value
                progressed = True
        if not progressed:
            break
    for name, body in raw.items():
        resolved.setdefault(name, body)
    return resolved


def macro_number(table: dict, *names, default=None):
    for name in names:
        value = table.get(name)
        if isinstance(value, (int, float)):
            return float(value)
    return default


# 内参宏名候选。固件当前从 NVS 读内参，这些留给编译期兜底值。
_CX_NAMES = ("CAM_CX", "VISION_CX", "CAM_INTRINSICS_CX")
_CY_NAMES = ("CAM_CY", "VISION_CY", "CAM_INTRINSICS_CY")
_FX_NAMES = ("CAM_FX", "VISION_FX", "CAM_INTRINSICS_FX")
_FY_NAMES = ("CAM_FY", "VISION_FY", "CAM_INTRINSICS_FY")
_K_NAMES = (
    ("CAM_DIST_K1", "VISION_DIST_K1"),
    ("CAM_DIST_K2", "VISION_DIST_K2"),
    ("CAM_DIST_P1", "VISION_DIST_P1"),
    ("CAM_DIST_P2", "VISION_DIST_P2"),
    ("CAM_DIST_K3", "VISION_DIST_K3"),
)


def intrinsics_from_macros(table: dict):
    from aim.types import CameraIntrinsics

    width = macro_number(table, "VISION_SRC_W", "CAM_INT_WIDTH", default=320)
    height = macro_number(table, "VISION_SRC_H", "CAM_INT_HEIGHT", default=240)
    fx = macro_number(table, *_FX_NAMES)
    fy = macro_number(table, *_FY_NAMES)
    cx = macro_number(table, *_CX_NAMES)
    cy = macro_number(table, *_CY_NAMES)
    if fx is None or fy is None or cx is None or cy is None:
        return None
    dist = tuple(macro_number(table, *pair, default=0.0) for pair in _K_NAMES)
    return CameraIntrinsics(width=int(width), height=int(height),
                            fx=fx, fy=fy, cx=cx, cy=cy, dist_coeffs=dist)


def apply_intrinsics_overrides(intr, *, width=None, height=None, fx=None, fy=None,
                               cx=None, cy=None, dist=(None, None, None, None, None)):
    from aim.types import CameraIntrinsics

    base = list(intr.dist_coeffs[:5]) + [0.0] * (5 - len(intr.dist_coeffs[:5]))
    for idx, value in enumerate(dist):
        if value is not None:
            base[idx] = float(value)
    return CameraIntrinsics(
        width=int(width if width is not None else intr.width),
        height=int(height if height is not None else intr.height),
        fx=float(fx if fx is not None else intr.fx),
        fy=float(fy if fy is not None else intr.fy),
        cx=float(cx if cx is not None else intr.cx),
        cy=float(cy if cy is not None else intr.cy),
        dist_coeffs=tuple(base),
    )


def default_intrinsics_from_config(path=DEFAULT_CONFIG_PATH):
    """返回 (内参, 来源说明)；config.h 没有标定宏时用图像尺寸生成占位内参。"""
    from aim.types import CameraIntrinsics

    table = parse_config_macros(path)
    found = intrinsics_from_macros(table)
    if found is not None:
        return found, f"{Path(path).name} 中的内参宏"
    width = int(macro_number(table, "VISION_SRC_W", "CAM_INT_WIDTH", default=320))
    height = int(macro_number(table, "VISION_SRC_H", "CAM_INT_HEIGHT", default=240))
    intr = CameraIntrinsics.default_for(width, height)
    return intr, f"{Path(path).name} 图像尺寸 {width}x{height} 的占位内参（未标定）"


def grade_rmse(value: float) -> str:
    """按 02_REQUIREMENTS/标定与几何推导.md 第 10 节的分级给评价。"""
    if value < 0.2:
        return "优秀，可直接用于静态打靶"
    if value < 0.5:
        return "可用，满足初版目标"
    if value <= 1.5:
        return "偏大，检查光轴与轴系对齐，检查畸变系数"
    return "不可用，系统存在结构性误差"


def c_float(value: float) -> str:
    return f"{float(value):.6f}f"
