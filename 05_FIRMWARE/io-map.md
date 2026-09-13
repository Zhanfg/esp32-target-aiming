# 引脚分配与扩展预留

本文汇总 `05_FIRMWARE/include/config.h` 里的引脚与片内外设占用，给出空闲资源和扩展方案。
第二期执行层改为舵机 + 两级脉冲核心 + 旋转供给盘，电机、编码器、TB6612 层已删除，
本文随之同步。改动 `config.h` 的任一引脚宏后，本文的三张表要同步更新。

## 1. 引脚分配表

从 `config.h` 汇总。核对实物一列在首次烧录前逐条填写，"待核对"表示还没有对照原理图确认。

| 用途 | GPIO | 外设资源 | 核对实物 |
|---|---|---|---|
| RELEASE_CMD | 3 | 普通 GPIO，strapping | 待核对 |
| 摄像头 SCCB SDA | 4 | GPIO 模拟 SCCB（I2C1） | 待核对 |
| 摄像头 SCCB SCL | 5 | GPIO 模拟 SCCB（I2C1） | 待核对 |
| 摄像头 VSYNC | 6 | 普通 GPIO | 待核对 |
| 摄像头 HREF | 7 | 普通 GPIO | 待核对 |
| 摄像头 D2 | 8 | 普通 GPIO | 待核对 |
| 摄像头 D1 | 9 | 普通 GPIO | 待核对 |
| 摄像头 D3 | 10 | 普通 GPIO | 待核对 |
| 摄像头 D0 | 11 | 普通 GPIO | 待核对 |
| 摄像头 D4 | 12 | 普通 GPIO | 待核对 |
| 摄像头 PCLK | 13 | 普通 GPIO | 待核对 |
| PAN_CMD | 14 | LEDC ch3 / timer3，50Hz | 待核对 |
| 摄像头 XCLK | 15 | LEDC ch2 / timer2，20MHz | 待核对 |
| 摄像头 D7 | 16 | 普通 GPIO | 待核对 |
| 摄像头 D6 | 17 | 普通 GPIO | 待核对 |
| 摄像头 D5 | 18 | 普通 GPIO | 待核对 |
| TILT_CMD | 21 | LEDC ch4 / timer3，50Hz | 待核对 |
| STAGE1_STATE | 40 | 普通 GPIO，中断，失稳事件计时 | 待核对 |
| STAGE2_STATE | 41 | 普通 GPIO，中断，判定两级顺序 | 待核对 |
| I2C0 SDA | 45 | I2C0，strapping | 必须核对 |
| I2C0 SCL | 46 | I2C0，strapping | 必须核对 |

摄像头 D0-D7 对应 `config.h` 的 `CAM_PIN_Y2..Y9`，其中 D0=Y2，D7=Y9。数据线顺序因板而异，接错会黑屏或花屏。

RELEASE_CMD 的 GPIO3 是 JTAG 相关 strapping 脚，用 JTAG 调试时要换脚。I2C0 用的 45/46 也是
strapping 脚，外接上拉会影响复位瞬间电平，接线前必须核对模块规格书。

## 2. MCP23017 位分配

I2C0 接一片 MCP23017，地址 0x20（A2A1A0 全低）。位编号 0-7 = GPA0-GPA7，8-15 = GPB0-GPB7。

| 位 | 名称 | 方向 | 说明 |
|---|---|---|---|
| 0 | BOUNDARY_CMD | 输出 | 边界研究，慢速 |
| 1 | PRELOAD_CMD | 输出 | 预载使能，硬件串 `MECH_RECOVERED_N` |
| 2 | MAG_INDEX_CMD | 输出 | 供给盘索引一步 |
| 3 | STATUS_LED | 输出 | 指示 |
| 4 | MAG_HOME | 输入 | 供给盘零位 |
| 5 | MAG_INDEX_OK | 输入 | 索引到位/对齐确认 |
| 6 | MECH_RECOVERED | 输入 | 常闭干接点对地，复位时为低 |
| 7 | PAN_HOME | 输入 | pan 参考开关 |
| 8 | TILT_HOME | 输入 | tilt 参考开关 |
| 9 | FIRE_INHIBIT | 输入 | 有效为高，外部下拉，开路禁止 |
| 10 | EXTERNAL_ALLOW | 输入 | 有效为高，外部下拉，开路拒绝 |

MCP23017 内部上拉不启用。安全输入按架构文档 §6.7 要求外部加下拉，许可类主动拉高，
开路即禁止；`MECH_RECOVERED` 是常闭干接点，要外部上拉。上电前必须确认外部电阻已就位。

## 3. 空闲 GPIO 表

| GPIO | 可用性 | 限制 |
|---|---|---|
| 0 | 受限可用 | BOOT 键兼 strapping，只作输入或按键 |
| 1,2 | 空闲 | 前一期电机方向脚，已释放 |
| 38,39,42,47,48 | 空闲 | 前一期 STBY/编码器/方向脚，已释放；部分开发板兼 RGB LED，用前核对 |
| 19 | 条件可用 | 原生 USB D-。用 USB 调试或烧录时不可占用，改用 UART0 后可释放 |
| 20 | 条件可用 | 原生 USB D+，同上 |
| 22-25 | 不存在 | 芯片不引出 |
| 26-32 | 不可用 | Flash/PSRAM 的 SPI 占用 |
| 33-37 | 不可用 | N16R8 八线 OPI PSRAM 的额外数据线占用 |
| 43-44 | 不可用 | UART0，默认串口监视器 |

结论：真正无约束的空闲脚有限，且 0、19、20、45、46 都带复位瞬间约束。扩展时优先用片内
外设或 I2C，不要指望再加很多引脚。

如果换成 QSPI PSRAM 的型号（N8R2 等），GPIO33-37 里的部分脚会释放，届时以实际模块
规格书为准，`platformio.ini` 的 `board_build.arduino.memory_type` 也要从 `qio_opi` 改掉。

## 4. 外设资源表

| 资源 | 总量 | 已用 | 剩余 | 说明 |
|---|---|---|---|---|
| LEDC 通道 | 8 | ch2（摄像头 XCLK）、ch3/ch4（两舵机） | ch0/ch1/ch5-ch7 | 两舵机共用 timer3 |
| LEDC 定时器 | 4 | timer2（XCLK）、timer3（舵机） | timer0/timer1 | 每个定时器可挂 2 个通道 |
| PCNT 单元 | 4 | 无 | unit0-unit3 | 编码器层已删 |
| UART | 3 | UART0（遥测与标定外壳共用 Serial） | UART1、UART2 | 总线舵机预留 UART1 |
| I2C | 2 | I2C1（摄像头 SCCB） | I2C0（MCP23017，可再挂电流监控） | - |
| SPI | SPI0/1 内部占用，SPI2、SPI3 可用 | 无 | SPI2、SPI3 | 需要引脚 |
| ADC | ADC1 十通道、ADC2 十通道 | 无 | 无可用引脚 | 电流/电压采样走 I2C ADC |
| RMT | 8 通道 | 无 | 全部 | 可留给红外或单总线外设 |

## 5. 资源余量

第二期去掉电机闭环后，Flash 与 RAM 仍充足，约束在引脚与外设，不在存储。需要注意的
瓶颈：

- GPIO：几乎没有无约束空脚，新外设要么走 I2C，要么复用已释放脚。
- CPU 时间：控制环 100Hz，单拍预算 10ms。视觉与发射时序共用一个 CPU，第二期必须实测
  最坏情况抖动（架构文档第 9 节）。
- 舵机行程与失速：无编码器，失速检测只能靠电流/负载（优先总线舵机反馈），要在台架标定。

## 6. 代码层扩展点

### 6.1 换舵机实现

边界在 `hal/servo.h` 的 `ServoDrive` 接口，`SERVO_DRIVE_TYPE` 编译期选择。总线协议落地后
在 `ServoBus` 里补齐帧格式、波特率、读位置与负载命令字即可，`servo_axis` 与 `main` 不改。

### 6.2 加第三个轴

改动集中在四处：`config.h` 加轴宏与引脚/LEDC 资源、`servo.h` 加读写接口、`servo_axis.cpp`
的轴表加一项、`aim_types.h` 的 `TurretSolution` 与遥测加字段。PC 端解析同步改。

### 6.3 换目标检测算法

边界在 `vision.h`：`visionCapture` 返回 `TargetObservation`，控制与解算不关心内部是阈值
分割还是神经网络。`confidence` 保持 [0,1]。

### 6.4 换传感器（IMU、电流监控）

放 `hal/` 下新建模块，挂 I2C0 与 MCP23017 共总线。IMU 姿态可喂给指向跟踪做前馈，电流
可用于舵机失速判定。新增传感器不要在 `main.cpp` 里堆逻辑，保持 `main` 只做编排。

## 7. 串口标定命令

与遥测共用 115200 的 Serial。每行一条命令，不分大小写，参数空格分隔。命令响应以 `OK,`、
`ERR,` 或字段前缀（`ST,`、`PT,`、`SOLVE,`）输出，和 `MST,`、`EVT,` 遥测行区分。

| 命令 | 作用 |
|---|---|
| `HELP` | 列出全部命令 |
| `STATUS` | 打印区域 A/区域 B 状态、故障码、两轴角度与标定 |
| `CAL START` | 进入手动标定模式，仅在 READY 可用，释放链强制断开 |
| `CAL JOG <pan_deg> <tilt_deg>` | 设两轴目标角，走限幅与变化率限制 |
| `CAL MARK [label]` | 记录当前有效像素观测与当前实际角，无效观测会被拒绝 |
| `CAL LIST` / `CAL DEL <n>` / `CAL CLEAR` | 点表管理 |
| `CAL SOLVE` | 最小二乘解算，分别报告训练与留出 RMSE |
| `CAL SAVE` / `CAL LOAD` | 写读 NVS |
| `CAL EXIT` | 退出标定模式 |
| `ESTOP` | 立即切断舵机与释放链并锁存 FAULT |
| `CLEAR` | 清除软故障并送 SAFE，随后自动重新 HOME；硬故障无效 |

点表最多 32 个点。`CAL SOLVE` 按记录顺序每第 3 个点抽为留出集，其余参与解算，训练点不足 3 个时
退化为全点解算，不再报留出残差。`CAL JOG` 越界会夹到机械限位并回报。
