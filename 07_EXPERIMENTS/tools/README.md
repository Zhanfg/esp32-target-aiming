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
3. `capture_telemetry.py` 串口采固件 AIM 遥测行
4. `analyze_telemetry.py` 算指向误差、跟踪误差、锁定时间、控制周期

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

串口采集固件以 `AIM,` 开头的遥测行，原样写入 CSV，统计采样率和状态分布。

```
python capture_telemetry.py --port COM5 --duration 20 --out run1.csv
python capture_telemetry.py --lines 3000 --out run2.csv
python capture_telemetry.py --list
```

`--duration` 和 `--lines` 二选一，都不给按 10 秒采。串口打不开会列出当前可用串口；
中途断开会保留已采数据并提示重插。缺 pyserial 时打印安装提示后退出。

### analyze_telemetry.py

读采集到的 CSV，输出指标报告：

- 稳态指向误差：LOCKED 且观测有效时，像素误差 `sqrt((px-cx)² + (py-cy)²)` 的均值、RMS、95 分位，并换算成角度
- 跟踪误差：TRACKING/LOCKED 时的视线角误差，以及轴位置与指令的差
- 锁定建立时间：首个有效观测到首次进入 LOCKED 的毫秒差
- 控制周期：`loop_us` 的均值、最大、std、95 分位，对照方案设计第 11.1 节的 ≤5 ms
- 状态分布与丢观测比例

```
python analyze_telemetry.py run1.csv
python analyze_telemetry.py run1.csv --json report.json
```

不达标的项会标出来，并按推导文档第 11 节的误差预算提示排查方向（分割稳定性和机械间隙
是两个主导项）。

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
