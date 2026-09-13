# MST-01 微型定向脉冲平台固件（第二期）

摄像头与脉冲核心同轴装在旋转云台上。pan 为水平旋转，tilt 为俯仰。两轴各由一台位置
舵机驱动，ESP32-S3 直出 50Hz PWM，或有总线反馈的舵机时读回位置与负载。整机由三块组成：
找目标的视觉、指向目标的云台、产生脉冲的两级串联失稳机械核心。

第二期执行层：舵机指向、两级脉冲核心、旋转供给盘。第一期的电机闭环、TB6612、编码器层
已整体删除。状态机做成双区域：区域 A 管发射周期，区域 B 管指向质量，跨区域用守卫矩阵
约束。故障锁存后唯一清除路径是 `SAFE → HOME`。

架构与安全设计见 `00_MASTER_SPEC/统一架构方案.md` 第 6 节，引脚见 `05_FIRMWARE/io-map.md`，
故障码见 `05_FIRMWARE/fault-codes.md`。

## 1. 构建 / 烧录 / 监视

```bash
cd 05_FIRMWARE

# 编译（开发环境，带逐帧遥测）
pio run

# 编译并烧录
pio run -t upload

# 打开串口监视器（115200）
pio device monitor

# 发布环境（-O2、关闭逐帧遥测）
pio run -e esp32-s3-devkitc-1-release
```

> 环境：VSCode + PlatformIO（Arduino 框架）+ C++17。外部库只依赖 `espressif/esp32-camera`，
> LEDC、I2C、Preferences 都来自 Arduino-ESP32 内置的 ESP-IDF 组件。

## 2. 上电前核对

`include/config.h` 里带 `// 待实测标定` 的引脚和参数都是占位默认值。首次烧录前重点确认：

- 摄像头 DVP 数据线 `CAM_PIN_Y2..Y9` 的顺序。接错会黑屏或花屏，一般不会损坏硬件。
- 舵机 PWM 脚（PAN_CMD=14、TILT_CMD=21）与摄像头、USB、Flash/PSRAM 占用脚不冲突。
- I2C0（GPIO45/46）是 strapping 脚，外接上拉会影响复位电平，必须核对。
- MCP23017 输入的外部上拉/下拉已按信号极性就位：许可类（FIRE_INHIBIT、EXTERNAL_ALLOW）
  外部下拉、开路即禁止；`MECH_RECOVERED` 常闭干接点对地、外部上拉。
- 参考开关 `PAN_HOME`、`TILT_HOME` 的触发方向与 `HOME_*_DIR` 一致，否则归零会超时进 FAULT。
- 释放链硬件与门树、单稳态与负载开关已搭好，且逐条断开输入时释放无动作（架构文档 §6.11）。

## 3. 模块职责

| 文件 | 职责 |
|------|------|
| `platformio.ini` | 构建配置：芯片、PSRAM、Flash 分区、依赖、编译开关、发布环境 |
| `include/config.h` | 全部引脚、机械参数、安全阈值、通信参数 |
| `src/aim_types.h` | 双区域状态、故障码、跨模块数据结构 |
| `src/math/angle_utils.*` | 角度归一化、角差、限速趋近 |
| `src/hal/servo.*` | 舵机抽象：PWM 实现 + 总线占位实现，编译期选择 |
| `src/hal/io_expander.*` | MCP23017 的 I2C0 驱动，读失败落安全侧 |
| `src/hal/safety_gate.*` | 软件互锁判定与武装锁存，硬件与门之外的第二层 |
| `src/hal/transmitter.*` | 第一期发射器抽象，第二期由四路命令层取代，保留占位 |
| `src/control/servo_axis.*` | 单轴指向控制：限幅、变化率限制、有反馈时位置校正 |
| `src/control/home.*` | 上电归零五步流程，参考开关 + 3 秒超时 |
| `src/calibration/*` | 内参、畸变、仿射标定的存取与最小二乘解算 |
| `src/vision/*` | 取帧、降采样、HSV、连通域、亚像素质心 |
| `src/comms/telemetry.*` | 串口 CSV 遥测与事件行 |
| `src/main.cpp` | 初始化、双区域状态机、安全策略、看门狗 |

## 4. 遥测格式

固件走两条遥测线，前缀不同，用途不同。

`MST,` 是规格书 §19 的实验记录，每发一条，17 个字段：

```
MST,timestamp,experiment_id,prototype_version,mechanism_version,mode,boundary_state,
    preload_state,membrane_id,payload_id,cycle_count,stage1_event,stage2_event,
    mechanism_recovered,magazine_position,magazine_index_ok,fault_code,operator_note
EVT,<t_ms>,<tag>,<message>
```

`DIAG,` 是每控制拍的帧轨迹，由 `AIM_VERBOSE_TELEMETRY` 控制，开发期打开、发布期关闭，
16 个字段：

```
DIAG,timestamp_ms,loop_us,obs_valid,obs_dropped,px,py,confidence,
     pan_deg,tilt_deg,pan_target_deg,tilt_target_deg,
     err_pan_deg,err_tilt_deg,err_deg,aim_state,has_feedback
```

`err_deg` 是目标质心相对光轴的角偏差合成量，静态指向 RMSE 按它算；`loop_us` 是控制周期；
`aim_state` 的 3 与 4 对应 TRACKING 与 LOCKED，据此算锁定建立时间。字段口径见
`src/comms/telemetry.h`。

发布环境（`AIM_VERBOSE_TELEMETRY=0`）只留 `MST,` 低频心跳行，`DIAG,` 行不输出。故障码见
`fault-codes.md`。

## 5. 安全要点

- 释放链的物理门控由外部与门与负载开关保证，软件互锁只是第一道闸（架构文档 §6.6）。
- `FAULT` 锁存，`FAULT → READY` 不存在；软故障用 `CLEAR` 后走 `SAFE → HOME`，硬故障需断电。
- 归零期间释放链强制断开，参考开关未全中不进入 `READY`。
- 无反馈 PWM 舵机不能检测失速，失速检测依赖总线舵机反馈或电流监控，需台架验证。

## 6. 批次编号命令

膜片与载荷的批次号供实验追溯，来源只能是操作者输入。串口命令写入后存 NVS（命名空间
`aim_batch`，键 `membrane`、`payload`），断电不丢，上电由 `setup()` 读回，组装遥测时填入
`membrane_id`、`payload_id`。两字段都是 uint16，取值 0 .. 65535，0 表示未设置。

```
SET MEMBRANE <id>    # 设膜片批次编号
SET PAYLOAD <id>     # 设载荷批次编号
STATUS               # 输出 ST,membrane_id,<n> 与 ST,payload_id,<n>
```

参数非数字或超出 0 .. 65535 会报错并保持原值；NVS 写入失败也会回退原值。这两条命令在
`calib_shell` 的 `HELP` 里也能查到。
