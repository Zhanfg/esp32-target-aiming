# MST-01 接口定义（interfaces.md）

本文把机械、电子、固件三方需要对齐的接口固定下来，让各自按同一份数字和极性做，装配和联调时能对上。
数据来源：主规格 §10/§11/§12/§18/§19，统一架构 §3/§4/§6/§7，`05_FIRMWARE/io-map.md` 与
`05_FIRMWARE/include/config.h`。

约定两条：

- 引脚、扩展器位、宏名以 `config.h` 为准；本文与代码不一致时算本文的缺陷，立即回填。
- 规格书和代码都没冻结的条目标"待定"，后面写清定它的前置条件，不拿猜测值当接口。

接口编号按类别分：`IF-ELEC`（对外电气）、`IF-MECH`（机械）、`IF-EXP`（扩展器）、
`IF-HOST`（上位机）、`IF-GATE`（硬件安全门）。

---

## 1. 对外电气接口（IF-ELEC）

### 1.1 电源输入

整机分三条轨：执行轨（舵机与释放执行器）、逻辑轨（3.3V 主控与传感）、辅助轨（5V，摄像头或部分舵机）。
执行轨和逻辑轨分开供电，只在一点共地。

| 项目 | 规格 | 状态与前置条件 |
|---|---|---|
| 主输入电压 | 待定。候选 12V DC 或 2S 锂电 7.4V | 舵机与释放执行器选型后冻结，见 `SERVO_DRIVE_TYPE` |
| 逻辑轨 | 3.3V，ESP32-S3 主控 | 由主输入降压，必须与执行轨隔离 |
| 辅助轨 | 5V | 依摄像头与舵机型号决定是否独立 |
| 峰值电流 | 待定 | 由释放执行器浪涌和舵机堵转电流决定；选型后按最坏同时动作估算，留 2 倍余量 |
| 连接器 | 待定。要求键控、防反插、带锁扣 | 逻辑轨与执行轨必须用不同型号，插错插不进 |
| 防反接 | 逻辑轨入口串肖特基或 P-MOS 理想二极管 | 执行轨若有感性执行器，反并续流二极管 |
| 短路保护 | 各轨入口串自恢复保险丝 | 参数按实测稳态与峰值电流整定 |
| 接地 | 单点共地，禁止地环路 | 电机/执行器线不得与摄像头排线并捆 |

主电源的切断由急停回路承担，不经过 MCU，见 1.2。

### 1.2 急停回路

急停用常闭蘑菇头，主触点直接串在执行轨供电里，按下即断开执行轨。这条回路不经过 MCU，也不受固件状态影响，
对应架构 §6.5 的 H6。

- 主触点：常闭，串执行轨。断路即断电。
- 辅助触点：接 MCU 或接入 `FIRE_INHIBIT` 链，让软件知道急停已动作并锁存。辅助触点只做状态感知，
  不承担断电功能。
- 线径与连接器：待定。前置条件是执行轨峰值电流定下来。
- 失效方向：回路任何断开都落到断电侧。触点粘合会让急停失效，属于单点失效，处理见
  `safety-boundary.md` 第 6 节。

### 1.3 EXTERNAL_ALLOW（上级许可）

| 项目 | 规格 |
|---|---|
| 语义 | 上级授权这一发，绿灯 |
| 电平 | 3.3V 逻辑电平 |
| 极性 | 主动驱动到高为许可 |
| 静态偏置 | 外部下拉 10kΩ 到 GND，MCP23017 内部上拉不启用 |
| 接入 | MCP23017 位 10（`EXP_EXTERNAL_ALLOW_BIT`），见第 3 节 |
| 有效窗口 | 仅在 READY/AIM_ALLOWED/PRELOAD/ARMED 期间监视；上升沿作为释放触发条件之一 |
| 双通道 | 预留。用两个冗余触点交叉校验，两路不一致按禁止处理；是否必须做双通道由安全完整性实验确认 |

### 1.4 FIRE_INHIBIT（安全否决）

| 项目 | 规格 |
|---|---|
| 语义 | 安全装置否决释放，红灯。网络名记 `FIRE_INHIBIT_N`，高电平表示"未否决、允许" |
| 来源 | 舱门开关、光幕、外接急停、机械钥匙等安全装置 |
| 电平 | 3.3V 逻辑电平 |
| 极性 | 高为未否决（允许），反转后进释放与门 |
| 静态偏置 | 外部下拉 10kΩ 到 GND，开路即否决 |
| 接入 | MCP23017 位 9（`EXP_FIRE_INHIBIT_BIT`） |
| 监视窗口 | PRELOAD 与 ARMED 期间持续监视，出现上升沿（否决生效）立即安全卸载 |
| 双通道 | 预留，理由同 EXTERNAL_ALLOW |

软件侧把原始电平取反后使用：`safetyFireInhibitAsserted()` 返回真表示否决生效。先判 `FIRE_INHIBIT`
再判 `EXTERNAL_ALLOW`，日志分开记录，事后能区分是安全装置拦截还是上级撤回授权。

### 1.5 三种失效下的行为

逐条列对外通路。短路一列区分短到允许侧（3V3）和短到禁止侧（GND），前者是危险方向，后者落安全。

| 通路 | 开路 | 掉电 | 短路 | 落点 |
|---|---|---|---|---|
| 主电源输入 | 整机断电 | 整机断电 | 输出短路由自恢复保险限流 | 断电侧，安全 |
| 急停回路主触点 | 执行轨断电 | 执行轨断电 | 触点粘合则急停失效，靠冗余或点检；短到 GND 仍断路 | 安全，粘合除外 |
| EXTERNAL_ALLOW | 下拉到低，拒绝 | 下拉到低，拒绝 | 短到 GND 仍拒绝；短到 3V3 会伪许可，需双通道检出 | 开路/掉电安全，短到 3V3 危险 |
| FIRE_INHIBIT_N | 下拉到低，否决 | 下拉到低，否决 | 短到 GND 仍否决；短到 3V3 会失去否决，需双通道与卡死诊断 | 开路/掉电安全，短到 3V3 危险 |
| RELEASE_CMD 通路 | 无请求，释放不导通 | 无请求 | 短到 GND 关断；短到 3V3 依赖硬件单稳态钳位脉冲宽度 | 开路/掉电安全，短到 3V3 需硬件兜底 |
| PRELOAD 使能通路 | 预载不动作 | 预载不动作 | 短到 GND 关断；短到 3V3 需 `MECH_RECOVERED_N` 常闭触点串接兜底 | 开路/掉电安全，短到 3V3 需硬件兜底 |

危险方向（短到允许侧）目前靠冗余触点和硬件门控兜底，安全完整性做到哪一级要实验确认。设计上先按双通道留位置。

---

## 2. 机械接口（IF-MECH）

只列跨模块配合面。规格书给的是工程 envelope，不是冻结尺寸，下面凡"待定"的都不能拿去开模或打印。

| 编号 | 接口 | 基准或要求 | 状态与前置条件 |
|---|---|---|---|
| IF-MECH-01 | 云台输出法兰与炮头安装面 | 提供安装孔阵列与定位面；两轴 0° 时炮口轴线与视觉光轴保持固定夹角 | 待定。前置条件：舵机云台选型 + CAD 冻结 |
| IF-MECH-02 | 视觉光轴与发射轴线对齐 | 装配后记录相对姿态，交给标定吸收残余偏差 | 待定。前置条件：摄像头支架与炮头装配基准冻结 |
| IF-MECH-03 | 供给盘与炮头的对接面 | 中心定位孔 + 端面贴合 + 防转键；每个 pocket 与发射孔同轴；有唯一方向防呆、零位标记、机械止挡 | 待定。前置条件：`MAG_POSITIONS` 定 6 还是 8，供给盘 CAD 冻结 |
| IF-MECH-04 | 供给盘零位靶与 `MAG_HOME` 传感器 | 零位标记与传感靶同相位，重复定位可测；靶可旋拧拆下做对照 | 待定。前置条件：`MAG_HOME` 传感器选型（霍尔锁存候选） |
| IF-MECH-05 | 载荷对齐检测靶与 `MAG_INDEX_OK` | 对齐信号是安全相关，必须接进硬件与门；模式 A 用继电器旁路为真 | 待定。前置条件：H3 与门方案冻结 |
| IF-MECH-06 | `PAN_HOME` / `TILT_HOME` 参考开关 | 开关装在固定框架上，触发件装在运动件上；位置可调，全行程不干涉；触发重复定位精度要测 | 待定。前置条件：舵机选型与装配基准冻结 |
| IF-MECH-07 | 膜片压环的拆装空间 | 不拆核心即可更换膜片；留出工具回转空间；压环可重复装拆 | 待定。前置条件：`DIAPHRAGM_MODULE` CAD |
| IF-MECH-08 | `STAGE1_STATE` / `STAGE2_STATE` 传感靶 | 磁靶会影响 snap 动力学，靶件用 304 不锈钢等近无磁材料；靶必须可拆，做带靶与不带靶的 force-displacement 对照 | 待定。前置条件：A1/B1/C1 对照实验确认扰动可接受 |
| IF-MECH-09 | 释放执行器与脉冲核心的机械耦合 | 释放执行器（电磁铁、sear 或预载反向）与两级核心接口对齐 | 待定。前置条件：架构 §6.11 第 1 项 H1 定型 |
| IF-MECH-10 | 两轴机械止挡 | 物理止挡是末端防线，软限位必须提前生效 | 待定。前置条件：舵机行程标定（`SERVO_PAN_MIN/MAX_DEG` 等） |

`REQ-127` 要求 CAD 至少拆成 FRAME、STAGE_1、STAGE_2、BOUNDARY_MODULE、DIAPHRAGM_MODULE、PAYLOAD_MODULE、
MAGAZINE_MODULE 七个模块，上表的配合面都落在模块边界上。

---

## 3. 固件与扩展器的接口（IF-EXP）

片内 GPIO 已分配完，慢速离散 I/O 全部走一片 MCP23017。

### 3.1 总线与器件

| 项目 | 值 | 来源 |
|---|---|---|
| 总线 | I2C0 | `config.h` |
| SDA / SCL | GPIO45 / GPIO46（strapping 脚，上电电平必须核对） | `I2C0_SDA_PIN` / `I2C0_SCL_PIN` |
| 速率 | 400 kHz | `I2C0_FREQ_HZ` |
| 从地址 | 0x20（A2A1A0 全低） | `MCP23017_ADDR` |
| 寄存器布局 | IOCON = 0，BANK = 0，地址自动递增 | `io_expander.cpp` |
| 事务超时 | 20 ms | `io_expander.cpp` |
| 内部上拉 | 不启用，上拉/下拉全部外置 | `io_expander.cpp` |
| 读失败行为 | 返回 0 并把健康标志置假 | `ioExpanderReadInputsSnapshot` / `ioExpanderHealthy` |
| 输出上电默认 | 全 0（BOUNDARY/PRELOAD/MAG_INDEX 断，STATUS_LED 灭） | `io_expander.cpp` |

### 3.2 位分配

位编号 0-7 = GPA0-GPA7，8-15 = GPB0-GPB7。方向、极性、上电默认如下。输出掩码 `EXP_OUTPUT_MASK`
只含位 0 到 3，其余配置为输入。

| 位 | 名称 | 宏 | 方向 | 有效极性 | 上电默认 | 说明 |
|---:|---|---|---|---|---|---|
| 0 | `BOUNDARY_CMD` | `EXP_BOUNDARY_CMD_BIT` | 输出 | 高 = 激活边界状态 | 0 | 慢速，边界研究，`MST_BOUNDARY_RESEARCH_ACTIVE` 控制是否置起 |
| 1 | `PRELOAD_CMD` | `EXP_PRELOAD_CMD_BIT` | 输出 | 高 = 预载 | 0 | 硬件串 `MECH_RECOVERED_N` 常闭触点 |
| 2 | `MAG_INDEX_CMD` | `EXP_MAG_INDEX_CMD_BIT` | 输出 | 高 = 索引一步 | 0 | 慢速 |
| 3 | `STATUS_LED` | `EXP_STATUS_LED_BIT` | 输出 | 高 = 亮 | 0 | 指示 |
| 4 | `MAG_HOME` | `EXP_MAG_HOME_BIT` | 输入 | 高 = 触发 | 读入 | 供给盘零位 |
| 5 | `MAG_INDEX_OK` | `EXP_MAG_INDEX_OK_BIT` | 输入 | 高 = 到位/对齐 | 读入 | 对齐检测并入本通道（C-11） |
| 6 | `MECH_RECOVERED` | `EXP_MECH_RECOVERED_BIT` | 输入 | 低 = 已复位 | 读入 | 常闭干接点对地，外部上拉；`safety_gate` 取反后使用 |
| 7 | `PAN_HOME` | `EXP_PAN_HOME_BIT` | 输入 | 高 = 触发 | 读入 | pan 参考开关 |
| 8 | `TILT_HOME` | `EXP_TILT_HOME_BIT` | 输入 | 高 = 触发 | 读入 | tilt 参考开关 |
| 9 | `FIRE_INHIBIT` | `EXP_FIRE_INHIBIT_BIT` | 输入 | 高 = 未否决/允许 | 读入 | 外部下拉，开路即否决 |
| 10 | `EXTERNAL_ALLOW` | `EXP_EXTERNAL_ALLOW_BIT` | 输入 | 高 = 许可 | 读入 | 外部下拉，开路即拒绝 |
| 11-15 | 预留 | 无 | 输入 | 按信号定义外置偏置 | 读入 | 未分配 |

### 3.3 读写模型

固件每拍调一次 `safetyGateUpdate()`，它内部通过 `ioExpanderReadInputsSnapshot()` 一次事务读完 16 位，
再用 `ioExpanderHealthy()` 确认这同一次事务成功，避免逐位读取时口径不一致。总线失败时
`safety_gate` 走 `forceSafe()`，把全部输入按禁止侧解释，不允许"读不到就默认机构已复位"。

输出写通过 `ioExpanderWritePin()`，写失败只记录健康标志，不清除已写值，影子寄存器保留末次写入的位型。
这一点在 `safety-boundary.md` 第 6 节作为单点失效讨论。

### 3.4 已知文档冲突（不改文件，先记录）

- `config.h` 第 181 行注释写"其余为输入并启用内部上拉"，与 `io_expander.cpp` 显式关闭内部上拉
  矛盾。以 `io-map.md` 和 `io_expander.cpp` 为准，`config.h` 该行注释需要更新。
- `io_expander.cpp` 头注释把 `MECH_RECOVERED` 写成"断言为高"，与 `safety_gate.cpp`
  的取反逻辑及 `io-map.md` 的"复位时为低"矛盾。以 `safety_gate.cpp` 和 `io-map.md` 为准。
- `io-map.md` 第 7 节把 `CAL JOG` 描述为"走双环 PID"，`calib_shell.cpp` 第 145 行帮助文本
  同误，二期已删除 PID 层，应改为限幅与变化率限制。

---

## 4. 固件与上位机的接口（IF-HOST）

### 4.1 串口参数

| 项目 | 值 |
|---|---|
| 端口 | UART0（`DEBUG_TX` 预留可映射到 USB CDC，见 C-10） |
| 波特率 | 115200（`TELEMETRY_BAUD`） |
| 帧格式 | 8N1 |
| 行分隔 | `\n`；命令接受 `\r\n` |
| 命令大小写 | 不敏感 |
| 单行上限 | 96 字符，超长整行丢弃并回 `ERR` |
| 逐帧遥测开关 | `AIM_VERBOSE_TELEMETRY`；关闭时只发 1 Hz 心跳（`TELEMETRY_HEARTBEAT_MS`） |

### 4.2 遥测数据行

前缀 `MST,`，字段与顺序按主规格 §19，共 17 个，字段名和顺序不可改。字符串字段不得含逗号。

| # | 字段 | 类型 | 单位 | 取值范围 | 说明 |
|---:|---|---|---|---|---|
| 1 | `timestamp` | uint32 | ms | 0 .. 4294967295 | 毫秒时间戳，会回绕 |
| 2 | `experiment_id` | uint16 | 无 | 0 .. 65535 | `MST_EXPERIMENT_ID` |
| 3 | `prototype_version` | string | 无 | 不含逗号 | 如 `MST-01C` |
| 4 | `mechanism_version` | string | 无 | 不含逗号 | 机制变体 |
| 5 | `mode` | uint8 | 无 | 0 / 1 | 0 = Mode A 空气，1 = Mode B 软载荷 |
| 6 | `boundary_state` | uint8 | 无 | 0 / 1 | `BOUNDARY_0` / `BOUNDARY_1` |
| 7 | `preload_state` | uint8 | 无 | 0 / 1 / 2 | 0 空闲，1 预载中，2 已武装 |
| 8 | `membrane_id` | uint16 | 无 | 0 .. 65535 | 膜片批次或试样编号 |
| 9 | `payload_id` | uint16 | 无 | 0 .. 65535 | 载荷批次或编号 |
| 10 | `cycle_count` | uint32 | 次 | 0 .. 4294967295 | 完成索引的动作计数 |
| 11 | `stage1_event` | uint8 | 无 | 0 / 1 | 本周期是否捕获到 Stage 1 |
| 12 | `stage2_event` | uint8 | 无 | 0 / 1 | 本周期是否捕获到 Stage 2 |
| 13 | `mechanism_recovered` | uint8 | 无 | 0 / 1 | 机构复位确认 |
| 14 | `magazine_position` | uint8 | 无 | 0 .. 5 | 当前工位，`MAG_POSITIONS` 为 6 |
| 15 | `magazine_index_ok` | uint8 | 无 | 0 / 1 | 索引到位 |
| 16 | `fault_code` | uint16 | 无 | 0 .. 9 | `FaultCode` 整数值，定义见 `aim_types.h` |
| 17 | `operator_note` | string | 无 | 不含逗号，默认 `-` | 操作备注 |

事件行与数据行用不同前缀区分：`EVT,<t_ms>,<tag>,<msg>`，`t_ms` 为毫秒，`tag` 与 `msg` 不含逗号。

### 4.3 标定外壳命令集

与遥测共用 115200 的 Serial，每行一条命令，参数空格分隔。响应前缀为 `OK,`、`ERR,`，以及
`ST,` / `PT,` / `SOLVE,` / `LIST,` / `LOAD,` / `CMD,` 等字段前缀。

| 命令 | 作用 | 关键约束 |
|---|---|---|
| `HELP` | 列出全部命令 | 无 |
| `STATUS` | 打印区域 A/区域 B 状态、故障码、两轴角度与标定 | 无 |
| `CAL START` | 进入手动标定模式 | 仅 READY 可用；释放链强制断开 |
| `CAL JOG <pan_deg> <tilt_deg>` | 设两轴目标角 | 越界夹到机械限位并回报；走限幅与变化率限制 |
| `CAL MARK [label]` | 记录当前有效像素观测与当前实际角 | 无效观测拒绝记录 |
| `CAL LIST` | 列出点表 | 最多 32 点 |
| `CAL DEL <n>` | 删除第 n 个点 | 序号从 1 开始 |
| `CAL CLEAR` | 清空点表 | 无 |
| `CAL SOLVE` | 最小二乘解算，报告训练与留出 RMSE | 至少 3 点；按顺序每第 3 点留出 |
| `CAL SAVE` | 把当前仿射写入 NVS | 仿射无效则拒绝 |
| `CAL LOAD` | 从 NVS 读回并应用 | 无记录则报错 |
| `CAL EXIT` | 退出标定模式 | 无 |
| `ESTOP` | 切断舵机与释放链并锁存 FAULT | 无条件立即生效 |
| `CLEAR` | 清除软故障并送 SAFE，随后自动重新 HOME | 硬故障无效 |

上位机脚本在 `07_EXPERIMENTS/tools/`，解析这些行，改动字段或命令集必须同步改脚本，见第 6 节。

---

## 5. 固件与硬件安全门的接口（IF-GATE）

释放路径的物理门控由一块与门树实现，软件 `safety_gate` 只是第二层，不替代硬件。

### 5.1 与门树

```
Q1 = AND( FIRE_INHIBIT_N, EXTERNAL_ALLOW, MECH_RECOVERED_N, POCKET_ALIGNED_or_MODE_A )
Q2 = AND( Q1, ARMED_LATCH, MCU_RELEASE_CMD, MASTER_ENABLE )
Q2 -> 负载开关使能 -> 释放执行器供电
```

器件与实现要点见架构 §6.6：一片 74HC21 双 4 输入与门级联即可覆盖；每个输入按极性加下拉或上拉，
开路即安全侧；Q2 输出加下拉，浮空等于关断；释放命令经 74HC123 单稳态钳位脉冲宽度，即使 MCU 卡在
输出高也只放一个固定短脉冲；负载开关用低边 N-MOSFET 串回流路径，栅极下拉，感性负载反并续流二极管，
串自恢复保险丝。

### 5.2 参与与门的信号

| 信号 | 作用 | 极性（有效侧） | 软件侧对应 |
|---|---|---|---|
| `FIRE_INHIBIT_N` | 安全否决，任一红灯亮即断开 | 高 = 未否决 | `safetyFireInhibitAsserted()`（取反） |
| `EXTERNAL_ALLOW` | 上级许可，绿灯 | 高 = 许可 | `safetyExternalAllow()` |
| `MECH_RECOVERED_N` | 机构复位确认 | 高 = 已复位（节点取自常闭/反相触点，命名里的 N 指此意，进与门前按高为允许接入） | `safetyMechRecovered()` |
| `POCKET_ALIGNED` | Mode B 载荷对齐；Mode A 由继电器旁路为真 | 高 = 对齐 | `safetyMagIndexOk()`（对齐检测并入 `MAG_INDEX_OK`，C-11） |
| `ARMED_LATCH` | 武装锁存，防止未武装时误放 | 高 = 已武装 | `safetyLatchArmed()` / `safetyArmedLatched()` |
| `MCU_RELEASE_CMD` | 释放请求 | 高 = 请求 | `digitalWrite(RELEASE_CMD_PIN, HIGH)`，`advanceCycle()` 的 ARMED 分支 |
| `MASTER_ENABLE` | 主使能，人工总开关或钥匙 | 高 = 有效 | 暂无对应宏与引脚，待定 |

`FIRE_INHIBIT_N` 与 `MECH_RECOVERED_N` 命名里的 N 表示取自常闭或反相节点，在与门树里都按高电平为允许侧接入。
具体电平方向在 H1 台架联调时用万用表逐点确认并回填本文。

### 5.3 预载路径的第二道硬件

预载使能串 `MECH_RECOVERED_N` 常闭触点，机构未复位时预载执行器物理上无法被驱动（H2）。软件侧由
`safetyCanEnterPreload()` 复查，只用于迁移判定与日志。

### 5.4 释放门控自检

`safetyReleaseGateSelfTest()` 确认扩展器健康且未武装时 `safetyCanRelease()` 为假。这是软件自检，
与门本身的验证要在台架上逐条断开输入、故意让 MCU 输出常高，确认释放无动作，方法见架构 §6.11。

### 5.5 代码里尚未落地的接口

`MASTER_ENABLE` 与 `POCKET_ALIGNED` 目前没有独立宏或引脚，`POCKET_ALIGNED` 走 `MAG_INDEX_OK` 合并通道，
`MASTER_ENABLE` 只存在于硬件与门树。两条的状态都是待定，前置条件是 H1 定型。

---

## 6. 接口变更流程

改任一接口，必须在同一次提交里改齐下游文件，不允许接口文档和代码各说各话。

| 改动对象 | 必须同步 |
|---|---|
| 任一 `*_PIN` 宏 | `config.h` → `io-map.md` 第 1 节 → `硬件接线与选型.md` 第 2 节 → 本文 §1/§3 |
| 任一 `EXP_*_BIT` 或输出掩码 | `config.h` → `io-map.md` 第 2 节 → 本文 §3 → `safety_gate.cpp` 的位解释 |
| 遥测字段或顺序 | `telemetry.h` / `telemetry.cpp` → `REQUIREMENTS.md` REQ-124 → 本文 §4 → `07_EXPERIMENTS/tools/` 解析脚本 |
| 标定命令集或响应格式 | `calib_shell.cpp` / `calib_shell.h` → `io-map.md` 第 7 节 → 本文 §4 → `07_EXPERIMENTS/tools/` |
| 与门树或信号极性 | 统一架构 §6.6/§6.7 → `safety_gate.cpp` / `safety_gate.h` → `main.cpp` 释放分支 → 本文 §5 → `safety-boundary.md` |
| 机械配合面或基准 | `03_CAD/` → 本文 §2 → `方案设计.md` 坐标与角度约定 |
| 接口新增或删除 | 本文 → 对应源文件 → `REQUIREMENTS.md` 的需求条目 |

下游文件没改完之前，接口改动不算完成。发现本文与代码不一致时，先以代码为准恢复一致性，再更新本文。
