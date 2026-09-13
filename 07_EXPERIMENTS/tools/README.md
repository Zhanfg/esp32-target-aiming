# 工具链（07_EXPERIMENTS/tools）

PC 端工具链，在开发机上跑，不烧进固件。作用是把固件、算法参考实现和真实舵盘接起来：
标定相机、标定轴角、采遥测、评精度。

依赖见 requirements.txt：numpy、opencv-python、pyserial、pytest。

```
python -m pip install -r 07_EXPERIMENTS/tools/requirements.txt
```

算法数学不在这些脚本里实现，全部 import `05_FIRMWARE/algo_reference/src/aim`。`common.py` 把该目录
挂进 `sys.path`，之后 `from aim.geometry import ...` 直接可用。要改数学去改
algo_reference，改这里没用。

## 典型顺序

1. `calibrate_intrinsics.py` 棋盘格标内参，得到 fx/fy/cx/cy 和畸变系数
2. `calibrate_axis.py` 打点解出像素角度到舵盘角的仿射
3. `capture_telemetry.py` 串口采固件遥测，MST 记录行与 DIAG 帧轨迹一起收
4. `analyze_telemetry.py` 出机制指标与指向/时序验收指标

## 遥测两条线

固件同时输出两条遥测线，前缀不同，用途不同，解析互不影响。

`MST,` 是规格书 §19 的实验记录，每发一条，共 17 个字段：时间戳、实验号、原型与机构版本、
模式、边界与预载状态、膜片与载荷编号、循环计数、两级事件计数、机构恢复、工位、索引到位、
故障码、操作备注。机制类指标用它算。

`DIAG,` 是每控制拍的帧轨迹，由 `AIM_VERBOSE_TELEMETRY` 开关控制，开发期打开、发布期关闭。
共 16 个字段，顺序如下：

```text
DIAG,timestamp_ms,loop_us,obs_valid,obs_dropped,px,py,confidence,
     pan_deg,tilt_deg,pan_target_deg,tilt_target_deg,
     err_pan_deg,err_tilt_deg,err_deg,aim_state,has_feedback
```

| 字段 | 单位 | 含义 |
|---|---|---|
| `timestamp_ms` | ms | 本拍 `millis()` |
| `loop_us` | us | 本拍控制环耗时，不含遥测打印 |
| `obs_valid` | 无 | 本拍是否有新鲜有效观测，1/0 |
| `obs_dropped` | 无 | 本拍尝试取帧但失败，1/0 |
| `px` `py` | px | 目标质心像素坐标 |
| `confidence` | 无 | 质心置信度 0..1 |
| `pan_deg` `tilt_deg` | 度 | 两轴当前角 |
| `pan_target_deg` `tilt_target_deg` | 度 | 两轴本拍目标角 |
| `err_pan_deg` `err_tilt_deg` | 度 | 目标质心相对光轴的角偏差分量 |
| `err_deg` | 度 | 角偏差合成量，静态指向 RMSE 按它算 |
| `aim_state` | 无 | `AimState` 整数值，3=TRACKING，4=LOCKED |
| `has_feedback` | 无 | 两轴任一有位置反馈 |

`obs_valid=0` 时 `px`/`py`/`confidence`/`err_*` 都写 0，统计时按 `obs_valid=1` 过滤。

发布固件（`AIM_VERBOSE_TELEMETRY=0`）不输出 `DIAG,` 行，只保留 `MST,` 低频心跳；这时三项
验收指标无法计算，`analyze_telemetry.py` 会给出提示并照常输出机制类指标。

## 脚本

### calibrate_intrinsics.py

棋盘格内参标定，用 OpenCV。检测一个目录下所有图片的角点，标出内参和畸变系数，输出
JSON 存档和一段 C 宏。

```
python calibrate_intrinsics.py calib_images --cols 9 --rows 6 --square 25
```

`--cols`/`--rows` 是内角点数，常见板子 9x6 或 7x5。`--square` 是方格边长（mm）。

注意：`05_FIRMWARE/include/config.h` 目前没有相机内参宏，固件的内参存在 NVS 里
（见 `05_FIRMWARE/src/calibration/calibration.h`）。脚本输出的 C 片段按 config.h 现有的
`CAM_` 命名习惯给出（`CAM_FX` 等），想把它当编译期兜底值就自行加进 config.h 的 VISION
段，或者抄进 NVS 初始值。

缺 opencv-python 时会打印安装提示后退出，不抛 traceback。

### calibrate_axis.py

轴角标定。输入一张对应点表，每行是目标像素和该点在画面中央时的舵盘角：

```
px,py,pan_deg,tilt_deg
120.0,80.0,3.2450,-1.8720
...
```

流程是像素先经 `pixel_to_angles` 转 bearing/elevation（度），再用
`solve_affine_angle_to_pan_tilt` 最小二乘解 2x3 仿射 A，满足
`q = A · (bearing, elevation, 1)`，输出 pan/tilt。随机留出一部分点（固定 seed）只做评估，
用它们的 RMSE 按推导文档第 10 节给评价。

```
python calibrate_axis.py points.csv --holdout 0.3 --seed 0
```

内参优先用 `--intrinsics-json`（calibrate_intrinsics.py 的输出），否则读 config.h 的内参宏，
都没有就用 config.h 图像尺寸生成占位内参。也可以用 `--fx --fy --cx --cy` 等直接覆盖。

脚本还会算标定点的包围盒占比，全挤在画面中央会警告：外推区的误差会被放大。输出 JSON
存档和一段 C 数组，字段名对应 `CalibrationData::affine`。

### capture_telemetry.py

串口采集固件遥测，`MST,` 与 `DIAG,` 两条线都收，原样写进同一个 CSV。两条线靠行前缀区分，
文件开头有两行以 `#` 起的表头注释。这样一次采集就能同时拿到机构记录和帧轨迹，不用开两个
串口或跑两遍。

```
python capture_telemetry.py --port COM5 --duration 20 --out run1.csv
python capture_telemetry.py --lines 3000 --out run2.csv
python capture_telemetry.py --list
```

`--duration` 和 `--lines` 二选一，都不给按 10 秒采；`--lines` 数的是两条线的合计数据行。
采完打印两线各自行数、实测采样率、MST 的预载/边界/故障分布，以及 DIAG 的 AimState 分布。
串口打不开会列出当前可用串口；中途断开会保留已采数据并提示重插。缺 pyserial 时打印安装
提示后退出。发布固件没有 DIAG 行，脚本会提示这一点。

### analyze_telemetry.py

读采集到的 CSV，按前缀分流后出两类指标：

机制类（数据源 `MST,`）：

- 运行信息：实验号、原型级别、机构变体
- 采样间隔：相邻两行 `timestamp` 差值的均值、最大、95 分位
- 机构恢复率：`mechanism_recovered` 为 1 的比例，目标 100%
- 索引到位率：`magazine_index_ok` 为 1 的比例，目标 100%
- 故障：非零 `fault_code` 的行数与分布，带软/硬分级
- 循环计数跨度、级间事件累计、工位覆盖、预载/边界/模式分布

指向与验收类（数据源 `DIAG,`，对照 `02_REQUIREMENTS/方案设计.md` §11.1）：

- 静态指向 RMSE：`obs_valid=1` 且 `aim_state=LOCKED` 的样本，按 `err_deg` 算均方根，目标 ≤ 0.5°
- 控制周期：`loop_us` 换算成毫秒，给均值与最大值，目标 ≤ 5 ms
- 锁定建立时间：从首个有效观测到首次进入 LOCKED，目标 ≤ 500 ms

```
python analyze_telemetry.py run1.csv
python analyze_telemetry.py run1.csv --json report.json
```

输入只有 `MST,` 没有 `DIAG,` 时，脚本不报错，机制类报告照常输出，验收指标区给出
「未发现 DIAG, 行」的提示，不打印空表。

### make_sample_axis_points.py

生成 `sample_axis_points.csv`。样例用默认 QVGA 内参和一组虚拟仿射反算得到，带 0.03 度
噪声，只为让 calibrate_axis.py 能端到端跑通，不代表任何真实设备。真实标定自己打点。

```
python make_sample_axis_points.py
```

## 测试

```
cd 07_EXPERIMENTS/tools
python -m pytest -q
```

只测不依赖硬件和 OpenCV 的部分：遥测行解析、config.h 宏解析、精度统计、仿射解算。
