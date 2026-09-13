# ESP32-S3 双轴目标锁定舵盘固件

摄像头与发射器同轴装在旋转舵盘上。pan 为水平旋转，限位 ±180°；tilt 为俯仰，
限位 -30°~+60°。两轴各由一台 JGB37-520 12V 有刷减速电机（自带霍尔编码器）驱动，
经一片 TB6612FNG 双路 H 桥，A 路接 pan，B 路接 tilt。ESP32-S3 用 PCNT 硬件四倍频
解码编码器，控制层是位置环（外）+ 速度环（内）串级 PID，闭环目的是让目标保持在
画面中心。

## 1. 构建 / 烧录 / 监视

```bash
# 进入固件目录
cd firmware

# 编译（开发环境，带逐帧遥测）
pio run

# 编译并烧录
pio run -t upload

# 打开串口监视器（115200）
pio device monitor

# 发布环境（-O2、关闭逐帧遥测）
pio run -e esp32-s3-devkitc-1-release
```

> 环境：VSCode + PlatformIO 插件（或 PlatformIO Core CLI），Arduino 框架，C++17。
> 首次编译会下载 `espressif32` 平台与 `espressif/esp32-camera`，需要联网。

## 2. 引脚先核对

`include/config.h` 里带 `// 待实测标定` 的引脚和参数都是占位默认值，不是实测值。

首次烧录前，对照自己开发板的原理图逐条确认。重点看这几处：

- 摄像头 DVP 数据线 `CAM_PIN_Y2..Y9` 的顺序。不同厂商的 S3-CAM 板差别很大，接错
  会黑屏或花屏，一般不会损坏硬件。
- 电机方向脚与 PWM 脚不能和摄像头、USB、Flash/PSRAM 占用脚冲突。PSRAM 走 OPI 时
  GPIO33~37 不可用。
- 编码器 A/B 相引脚。
- 发射器控制脚默认 GPIO3，它是 JTAG 相关 strap 脚，用 JTAG 调试时要换脚。

摄像头 XCLK 由 LEDC 的 timer2/channel2 产生，电机 PWM 用 timer0/1 的 channel0/1，
两者不冲突。改动任一 LEDC 资源后要重新检查是否抢占。

## 3. 接线表（待填写）

把实际连线填进下表后再烧录。所有模块与 ESP32-S3 必须共地，电机和 H 桥用独立且
电流足够的 12V 电源。

### 3.1 电源

| 电源 | 供给对象 | 电压 | 备注 |
|------|----------|------|------|
| VM | TB6612FNG 电机电源 | 12V | 与逻辑电源共地，加去耦电容 |
| VCC | TB6612FNG 逻辑电源 | 3.3V/5V | 按模块要求 |
| 3V3 | ESP32-S3、编码器 | 3.3V | 编码器供电按实物规格 |
| GND | 全部模块 | — | 共地 |

### 3.2 ESP32-S3 与 TB6612FNG、电机

| TB6612 引脚 | ESP32-S3 引脚（默认） | 连接对象 | 填写实际 |
|-------------|----------------------|----------|----------|
| PWMA | GPIO14 | pan 电机 + / - | ______ |
| AIN1 | GPIO1 | — | ______ |
| AIN2 | GPIO2 | — | ______ |
| PWMB | GPIO21 | tilt 电机 + / - | ______ |
| BIN1 | GPIO47 | — | ______ |
| BIN2 | GPIO48 | — | ______ |
| STBY | GPIO38 | — | ______ |
| AO1/AO2 | — | pan 电机 | ______ |
| BO1/BO2 | — | tilt 电机 | ______ |
| VM | 12V | 电机电源 | ______ |
| VCC | 3.3V/5V | 逻辑电源 | ______ |
| GND | GND | 公共地 | ______ |

### 3.3 ESP32-S3 与编码器

| 编码器 | ESP32-S3 引脚（默认） | 填写实际 |
|--------|----------------------|----------|
| pan A 相 | GPIO39 | ______ |
| pan B 相 | GPIO40 | ______ |
| tilt A 相 | GPIO41 | ______ |
| tilt B 相 | GPIO42 | ______ |
| VCC | 3.3V | ______ |
| GND | GND | ______ |

### 3.4 摄像头与发射器

| 信号 | ESP32-S3 引脚（默认） | 填写实际 |
|------|----------------------|----------|
| 摄像头各脚 | 见 `config.h` 的 CAMERA PINS | ______ |
| 发射器控制 | GPIO3（待核对） | ______ |

## 4. 模块职责

| 文件 | 职责 |
|------|------|
| `platformio.ini` | 构建配置：芯片、PSRAM、Flash 分区、依赖、编译开关、发布环境 |
| `include/config.h` | 全部引脚、机械参数、PID 增益、安全阈值、通信参数 |
| `src/aim_types.h` | 跨模块共享的数据结构与枚举（不依赖 Arduino） |
| `src/math/angle_utils.*` | 角度归一化、角差、限速趋近（360° 环绕落点） |
| `src/math/pid.*` | 通用 PID：抗积分饱和、微分低通、死区 |
| `src/hal/motor_driver.*` | TB6612FNG 驱动：LEDC PWM、方向脚、死区补偿、上限保护 |
| `src/hal/encoder.*` | PCNT 硬件四倍频解码、角度与角速度估计 |
| `src/hal/transmitter.*` | 发射器抽象接口与占位实现（安全互锁） |
| `src/control/axis.*` | 单轴双环 PID、模式切换、软限位、duty 变化率限制 |
| `src/control/turntable.*` | 两轴组合、alpha-beta 跟踪 + P 控制（待替换为 AimSolver） |
| `src/calibration/*` | 内参、畸变、仿射标定的存取与最小二乘解算 |
| `src/vision/*` | 取帧、降采样、HSV、连通域、亚像素质心 |
| `src/comms/telemetry.*` | 串口 CSV 遥测与事件行 |
| `src/main.cpp` | 初始化、非阻塞状态机、安全策略、看门狗 |

## 5. 遥测格式

每帧一行，PC 端按前缀过滤：

```
AIM,t_ms,state,obs_valid,px,py,conf,pan_deg,tilt_deg,pan_target,tilt_target,pan_rate,tilt_rate,enc_pan,enc_tilt,loop_us
EVT,<t_ms>,<tag>,<message>
```

发布环境（`AIM_VERBOSE_TELEMETRY=0`）只留低频心跳行。

## 6. 上电前核对清单

1. 引脚：对照原理图核对 `config.h` 里全部摄像头、电机、编码器、发射器引脚。
2. 电机与减速比：确认 JGB37-520 的实际减速比（30/56/90），改 `GEAR_RATIO_*`。
3. 编码器 PPR：确认每转脉冲数（常见 11 或 13），改 `ENCODER_PPR`。
4. 编码器计数方向：上电给正指令，确认编码器角度增大；反了就调 `MOTOR_INVERT_*`
   或 `ENCODER_INVERT_*`，两者改一个即可。
5. 供电与共地：12V 电机电源和逻辑电源共地，电流余量够，H 桥散热到位。
6. 发射器安全：上电前确认发射器物理断开或处于安全状态。固件 `setup()` 期间发射器
   保持 SAFE，只有进入 `LOCKED` 并显式调用 `aimArmTransmitter(true)` 后才可能触发。
7. 限位：手动确认 pan/tilt 机械限位与 `config.h` 一致，防止软限位失效时撞限位。
