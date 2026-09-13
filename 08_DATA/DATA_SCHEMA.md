# MST-01 数据规范（DATA_SCHEMA.md）

对应主规格 §33 的交付项，覆盖 §15.1 输入变量、§15.2 输出变量、§16 数据文件名、§17 循环寿命、
§18 供给盘验收记录项、§19 十七字段，以及 `05_FIRMWARE/src/comms/telemetry.*` 的实际输出。

本文有两类读者。做实验的人照着建表和填表，PC 端写解析脚本的人照着取值和枚举。凡与代码冲突的地方，
以 `telemetry.h`、`telemetry.cpp`、`aim_types.h` 为准，并在文中标出冲突点，不在这里改代码。

数据文件落在 `07_EXPERIMENTS/` 对应级别目录，整理规范后归档到 `08_DATA/`。两处的列定义相同，
归档只换目录，不改列。

---

## 1. 固定遥测行

固件通过 UART0 以 115200 8N1 输出两类行，用前缀区分：

```text
MST,<17 个字段>          数据行，本节定义
EVT,<t_ms>,<tag>,<msg>   事件行，见 1.4
```

`telemetryInit()` 在上电时先打印两行注释和一行表头：

```text
# MST telemetry; header:
MST,timestamp,experiment_id,prototype_version,mechanism_version,mode,boundary_state,preload_state,membrane_id,payload_id,cycle_count,stage1_event,stage2_event,mechanism_recovered,magazine_position,magazine_index_ok,fault_code,operator_note
```

表头行以 `MST,` 开头，与数据行前缀相同，PC 端若逐行解析要先跳过含字段名的首行。行尾为 `\n`。
发布固件（`AIM_VERBOSE_TELEMETRY=0`）不逐帧刷屏，按 `TELEMETRY_HEARTBEAT_MS`（默认 1000 ms）发一行心跳，
字段与格式不变，只有 `timestamp` 一定取当拍 `millis()`。

### 1.1 字段定义

顺序与 `telemetry.cpp` 的 `emitLine()` 逐字对齐。printf 格式串：

```c
"MST,%lu,%u,%s,%s,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u,%s\n"
```

| 序号 | 字段名 | C 类型 | 单位 | 取值范围 | 含义 | 非法值表示 |
|---:|---|---|---|---|---|---|
| 1 | `timestamp` | uint32 | ms | 0 .. 4294967295，约 49.7 天回绕 | 本行生成时的 `millis()`。逐帧模式调用方传 0 时回退为当拍 `millis()` | 无专用哨兵。上电后第 0 毫秒即 0，无法与"未填"区分 |
| 2 | `experiment_id` | uint16 | 无 | 0 .. 65535 | 实验编号，取 `MST_EXPERIMENT_ID`，当前为 0 | 无。0 是合法实验号 |
| 3 | `prototype_version` | 字符串 | 无 | 不含逗号；`MST-01A` .. `MST-01E` | 原型级别，取 `MST_PROTOTYPE_VERSION` | 空指针打印 `-`；空字符串打印为空字段 |
| 4 | `mechanism_version` | 字符串 | 无 | 不含逗号；`MST-01A` .. `MST-01E` | 机构变体，取 `MST_MECHANISM_VERSION` | 同 `prototype_version` |
| 5 | `mode` | uint8 | 无 | 0 / 1 | `FireMode` 整数值，0 空气，1 软载荷 | 无。越界值只能来自代码错误 |
| 6 | `boundary_state` | uint8 | 无 | 0 / 1 | 当前边界状态，取 `MST_BOUNDARY_RESEARCH_ACTIVE` | 无。代码里没有 `BoundaryState` 枚举，见 2.3 |
| 7 | `preload_state` | uint8 | 无 | 0 / 1 / 2 | 0 空闲，1 预载中，2 已武装 | 无 |
| 8 | `membrane_id` | uint16 | 无 | 0 .. 65535 | 膜片批次或试样编号。由串口 `SET MEMBRANE <id>` 写入 NVS，上电读回，`main.cpp` 组装记录时填入 | 0 表示未设置，是默认值 |
| 9 | `payload_id` | uint16 | 无 | 0 .. 65535 | 载荷批次或编号。由串口 `SET PAYLOAD <id>` 写入 NVS，上电读回，`main.cpp` 组装记录时填入 | 0 表示未设置，是默认值。同 `membrane_id` |
| 10 | `cycle_count` | uint32 | 次 | 0 .. 4294967295 | 完成索引的动作计数，`INDEX` 到位时自增 | 无 |
| 11 | `stage1_event` | uint8 | 次 | 0 .. 255，溢出回绕 | Stage 1 中断累计次数，`stage1Isr()` 里 `s_stage1_count++`，全程不清零 | 无。与 `interfaces.md` §4.2 的"0/1 本周期标志"口径冲突，见 1.3 |
| 12 | `stage2_event` | uint8 | 次 | 0 .. 255，溢出回绕 | Stage 2 中断累计次数，口径同 `stage1_event` | 同 `stage1_event` |
| 13 | `mechanism_recovered` | uint8 | 无 | 0 / 1 | `safetyMechRecovered()` 的布尔结果 | 无 |
| 14 | `magazine_position` | uint8 | 无 | 0 .. `MAG_POSITIONS`-1，当前 0 .. 5 | 当前供给盘工位，`INDEX` 到位后 `(pos+1) % MAG_POSITIONS` | 无。工位数由 `MAG_POSITIONS` 决定，改宏要同步本文 |
| 15 | `magazine_index_ok` | uint8 | 无 | 0 / 1 | `safetyMagIndexOk()` 的布尔结果，载荷对齐并入本通道 | 无 |
| 16 | `fault_code` | uint16 | 无 | 0 .. 14 | `FaultCode` 整数值，见 4.3 | 无。未知码按原值输出 |
| 17 | `operator_note` | 字符串 | 无 | 不含逗号，≤ 23 字符，默认 `-` | 操作备注。缓冲区 `s_operator_note[24]`，超长截断 | 空指针打印 `-`；空字符串打印为空字段 |

字符串字段进 `emitLine()` 前都会做空指针兜底，`rec.prototype_version ? ... : "-"`。也就是说
空指针和空字符串在输出上不同：前者是 `-`，后者是一个空字段，会让 `split(',')` 得到空串。
写代码时统一给 `-` 或给 `nullptr`，不要给 `""`。

### 1.2 类型宽度与格式串

`telemetry.cpp` 用 `%u` 打印 uint8/uint16，用 `%lu` 打印 uint32。ESP32 上 `int` 为 32 位，
默认参数提升后 `%u` 读取 32 位无符号数，小整数补零，行为正确。`%lu` 对应 `unsigned long`，
ESP32 上是 32 位。格式串里没有空格，字段间只有逗号。

### 1.3 与接口文档的两处冲突

- `interfaces.md` §4.2 把 `stage1_event`、`stage2_event` 写成"本周期是否捕获到"的 0/1 标志，
  代码里是累计计数。以代码为准，本文按累计计数记录；要恢复 0/1 语义需要改固件在每周期清零，
  这属于接口变更，按 `interfaces.md` §6 走同步流程。
- `membrane_id`、`payload_id` 已按 §19 要求接入固件，不再是恒 0 的占位。串口命令
  `SET MEMBRANE <id>`、`SET PAYLOAD <id>` 写 NVS（命名空间 `aim_batch`，键 `membrane`、`payload`），
  上电读回后由 `main.cpp` 组装 `TelemetryRecord` 时填入，`STATUS` 可查询。取值 0 .. 65535，
  0 表示未设置。此条原为缺口记录，现按固件实际更新。

### 1.4 事件行

```text
EVT,<t_ms>,<tag>,<msg>
```

`t_ms` 为 `millis()`，uint32。`tag` 与 `msg` 不含逗号，空指针打印 `-`。目前固件发这几类：

| tag | msg 形式 | 触发点 |
|---|---|---|
| `STAGE1` | `t=<微秒>` | Stage 1 中断计数变化时 |
| `STAGE2` | `t=<微秒>` | Stage 2 中断计数变化时 |
| `WD` | 文本 | 控制环超预算降帧 |
| 故障名或 `STATE_ILLEGAL` 等 | 文本 | `enterFault()` 及守卫失守 |

事件行里只有 `STAGE1`/`STAGE2` 带微秒时刻，其余是文本。要做级间延时分析，读 `msg` 里的
`t=<微秒>`，不要用 `t_ms` 相减。

---

## 2. 实验数据记录格式

§19 的十七字段是"每次动作保存"的记录。固件把同一组字段直接当作固定遥测行输出，两者字段集合相同，
区别在用途：遥测行是控制环里的连续快照，动作记录是每发动作归档下来的一行。归档时按同一列序落盘，
PC 端一套解析器通吃。列定义见 1.1，本节补语义、来源和枚举。

### 2.1 记录字段

| 字段名 | 类型 | 单位 | 取值范围或枚举 | 来源 | 必填 |
|---|---|---|---|---|---|
| `timestamp` | uint32 | ms | 0 .. 4294967295 | 自动 | 是 |
| `experiment_id` | uint16 | 无 | 0 .. 65535 | 自动（`MST_EXPERIMENT_ID`），实验前人工设 | 是 |
| `prototype_version` | 字符串 | 无 | `MST-01A` .. `MST-01E` | 自动（`MST_PROTOTYPE_VERSION`） | 是 |
| `mechanism_version` | 字符串 | 无 | `MST-01A` .. `MST-01E` | 自动（`MST_MECHANISM_VERSION`） | 是 |
| `mode` | uint8 | 无 | 0 / 1，见 4.4 | 自动 | 是 |
| `boundary_state` | uint8 | 无 | 0 / 1，见 2.3 | 自动（编译期宏），实验时人工置状态 | 是 |
| `preload_state` | uint8 | 无 | 0 / 1 / 2，见 2.3 | 自动 | 是 |
| `membrane_id` | uint16 | 无 | 0 .. 65535 | 自动（串口 `SET MEMBRANE` 写 NVS，上电读回）；0 = 未设置 | 是 |
| `payload_id` | uint16 | 无 | 0 .. 65535 | 自动（串口 `SET PAYLOAD` 写 NVS，上电读回）；0 = 未设置 | 是 |
| `cycle_count` | uint32 | 次 | 0 .. 4294967295 | 自动 | 是 |
| `stage1_event` | uint8 | 次 | 0 .. 255 累计 | 自动 | 是 |
| `stage2_event` | uint8 | 次 | 0 .. 255 累计 | 自动 | 是 |
| `mechanism_recovered` | uint8 | 无 | 0 / 1 | 自动 | 是 |
| `magazine_position` | uint8 | 无 | 0 .. 5 | 自动 | 是 |
| `magazine_index_ok` | uint8 | 无 | 0 / 1 | 自动 | 是 |
| `fault_code` | uint16 | 无 | 0 .. 14，见 4.3 | 自动 | 是 |
| `operator_note` | 字符串 | 无 | 不含逗号，≤ 23 字符，默认 `-` | 人工 | 是（无内容填 `-`） |

REQ-124 要求十七个字段每发都保存，没有可选项。人工填的只有 `operator_note`，其余由固件或编译期宏给。

### 2.2 `experiment_id` 编号约定

规格书没有给编号规则。README 里要求 `experiment_id` 带上原型级别，这里落成可执行口径：

| experiment_id | 级别 |
|---:|---|
| 0 | 未指定 / 台架联调 |
| 1 | A 级 MST-01A |
| 2 | B 级 MST-01B |
| 3 | C 级 MST-01C |
| 4 | D 级 MST-01D |
| 5 | E 级 MST-01E |
| 6 | 循环寿命 |
| 7 | 供给盘独立验收 M1 |

这套编号是本文件新增定义，改 `MST_EXPERIMENT_ID` 时同步更新本表。

### 2.3 枚举字段的取值来源

`mode`、`fault_code` 在 `aim_types.h` 里有正式枚举，见第 4 节。

`boundary_state`、`preload_state` 没有对应的 C++ 枚举类。`preload_state` 的 0/1/2 来自
`telemetry.h` 的字段注释，`boundary_state` 的 0/1 来自 `interfaces.md` §4.2 和 §16 的
`BOUNDARY_0`/`BOUNDARY_1`。两者都是本文件按现有文档固定下来的取值，将来若加正式枚举，
只能追加在末尾。

`prototype_version` 与 `mechanism_version` 是字符串，取值来自 `config.h` 的
`MST_PROTOTYPE_VERSION`、`MST_MECHANISM_VERSION`，当前都是 `"MST-01C"`。按 §13 的原型序列，
合法值是 `MST-01A` 到 `MST-01E`，另可带后缀区分同一级的不同机构变体，后缀不得含逗号。

---

## 3. CSV 文件约定

### 3.1 通用约定

| 项目 | 约定 |
|---|---|
| 编码 | UTF-8，不带 BOM。固件只输出 ASCII，人工文本用 UTF-8 |
| 换行 | 以 `\n` 为准。读取端接受 `\r\n`，不要依赖单一换行 |
| 分隔符 | 半角逗号 `,`。字段内不得出现半角逗号，见 3.7 |
| 表头 | 每个 CSV 首行为表头。遥测原始文件首行是 `# MST,...` 注释形式，第二行才是表头 |
| 注释行 | 以 `#` 开头，读取端跳过 |
| 缺失值 | 数值字段留空；字符串字段填 `-`。不用 `NaN`、`NULL`、`N/A` |
| 时间戳 | 毫秒，uint32，时基为板上电后的 `millis()`。跨上电周期的绝对时间靠文件名和 `*_notes.md` 里的墙钟记录 |
| 微秒时刻 | 只在事件行和事件表的 `event_time_us` 列出现，uint32 |
| 小数 | 定点十进制，点号做小数点，不使用科学计数法 |

`timestamp` 会回绕。单次实验连续跑不满 49.7 天，正常不会遇到；跨回绕的长寿命测试要在
`*_notes.md` 里记一次上电时刻，用于对齐。

### 3.2 每次动作记录表（§19）

文件名建议 `RUN_<experiment_id>_<日期>.csv`，一行动作，列序严格按 1.1 的十七列。归档到 `08_DATA/`。
这是主表，曲线文件和事件文件通过 `cycle_count` 与它关联。

### 3.3 力-位移表

用于 `A1_force.csv`、`B1_force.csv`、`C1_force.csv`、`D1_B0_force.csv`、`D1_B1_force.csv`。
列定义为本文件新增，规格书只给了文件名和"force-displacement curve"。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `t_ms` | uint32 | ms | 采样时刻，同一动作内相对起始 |
| 2 | `displacement_mm` | float | mm | 执行器或膜片位移 |
| 3 | `force_n` | float | N | 力传感器读数 |
| 4 | `cycle_count` | uint32 | 次 | 对应动作记录的 `cycle_count`，用于关联 |
| 5 | `membrane_id` | uint16 | 无 | 对应膜片编号 |
| 6 | `operator_note` | 字符串 | 无 | 可选，默认 `-` |

### 3.4 位移-时间表

用于 `A1_displacement.csv`、`B1_displacement.csv`、`C1_displacement.csv`、`D1_B0_displacement.csv`、
`D1_B1_displacement.csv`、`E1_displacement.csv`。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `t_ms` | uint32 | ms | 采样时刻 |
| 2 | `displacement_mm` | float | mm | 膜片位移 |
| 3 | `cycle_count` | uint32 | 次 | 关联用 |
| 4 | `membrane_id` | uint16 | 无 | 关联用 |
| 5 | `operator_note` | 字符串 | 无 | 可选，默认 `-` |

### 3.5 压力-时间表

用于 `A1_pressure.csv`、`B1_pressure.csv`、`C1_pressure.csv`、`D1_B0_pressure.csv`、
`D1_B1_pressure.csv`、`E1_pressure.csv`。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `t_ms` | uint32 | ms | 采样时刻 |
| 2 | `pressure_pa` | float | Pa | 低量程压力传感器读数。量程在 A1 前定，写进 `*_notes.md` |
| 3 | `cycle_count` | uint32 | 次 | 关联用 |
| 4 | `membrane_id` | uint16 | 无 | 关联用 |
| 5 | `operator_note` | 字符串 | 无 | 可选，默认 `-` |

### 3.6 事件表

用于 `B1_events.csv`、`C1_events.csv`、`D1_events.csv`。事件从 `STAGE1_STATE`、`STAGE2_STATE`
中断和固件 `EVT` 行来，一行一个事件。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `event_index` | uint16 | 无 | 文件内序号，从 1 起 |
| 2 | `event` | 字符串 | 无 | `STAGE1`、`STAGE2`、`RECOVER`、`INDEX_OK` 之一 |
| 3 | `edge` | 字符串 | 无 | `rise` 或 `fall` |
| 4 | `t_ms` | uint32 | ms | 毫秒时刻 |
| 5 | `event_time_us` | uint32 | 微秒 | 中断捕获的微秒时刻，来自 `EVT` 的 `msg` |
| 6 | `cycle_count` | uint32 | 次 | 关联用 |
| 7 | `operator_note` | 字符串 | 无 | 可选，默认 `-` |

级间延时 = 同一 `cycle_count` 下 `STAGE2.event_time_us` 减 `STAGE1.event_time_us`。

### 3.7 人工文本的逗号与换行

固件的 `MST,` 行不做转义，`operator_note` 里出现半角逗号会多切出一列，出现换行会多切出一行。
`setNote()` 写入 `s_operator_note[24]`，超长截断，也不做转义。规则分两层：

- 进固件、要出现在 `MST,` 行里的文本：写入前把半角逗号替换为全角 `，` 或分号 `;`，把换行替换为
  空格，控制在 23 字符内。不做这一步，遥测列数会错。
- 人工在 PC 端填进 CSV 的文本：按 RFC 4180 处理，含逗号、换行或双引号的字段用双引号包住，
  字段内的双引号写成 `""`。含换行的字段读取时按引号配对还原，不能按行切。

两种写法不要混用。归档整理时以固件行转出来的记录为准，人工补注放在独立的 `*_notes.md`，
避免把带引号的字段再灌回固件格式。

### 3.8 循环寿命表

用于 `LIFE_100.csv`、`LIFE_500.csv`、`LIFE_1000.csv`。一行一个循环，检查项只在里程碑填，
其余循环留空。检查项按 §17。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `cycle_count` | uint32 | 次 | 累计循环数 |
| 2 | `t_ms` | uint32 | ms | 时刻 |
| 3 | `mechanism_version` | 字符串 | 无 | 被测机构变体 |
| 4 | `stage_combination` | 字符串 | 无 | `single-passive`、`single-snap`、`dual-snap` |
| 5 | `snap_threshold_n` | float | N | snap 阈值，漂移判据 ≤ 10% |
| 6 | `peak_displacement_mm` | float | mm | 峰值位移 |
| 7 | `displacement_decay_pct` | float | % | 相对基线衰减，判据 ≤ 10% |
| 8 | `recover_ok` | uint8 | 无 | 0 / 1，恢复失败要求 0 次 |
| 9 | `crack_visible` | 字符串 | 无 | `yes` / `no` / `-` |
| 10 | `plastic_deform` | 字符串 | 无 | `yes` / `no` / `-` |
| 11 | `relaxation_pct` | float | % | 膜片松弛量，未测填 `-` |
| 12 | `screw_loose` | 字符串 | 无 | `yes` / `no` / `-` |
| 13 | `sensor_false` | 字符串 | 无 | `yes` / `no` / `-` |
| 14 | `operator_note` | 字符串 | 无 | 可选，默认 `-` |

### 3.9 供给盘索引表

用于 `M1_index.csv`。记录项来自 §18：`index_command`、`index_detected`、`jam`、`double_feed`、
`misalignment`、`manual_reset`。一行一次索引，连续 50 次。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `attempt` | uint16 | 无 | 第几次索引，1 起 |
| 2 | `t_ms` | uint32 | ms | 时刻 |
| 3 | `index_command` | uint8 | 无 | 0 / 1，是否发出索引命令 |
| 4 | `index_detected` | uint8 | 无 | 0 / 1，`MAG_INDEX_OK` 是否到位 |
| 5 | `jam` | uint8 | 无 | 0 / 1 |
| 6 | `double_feed` | uint8 | 无 | 0 / 1 |
| 7 | `misalignment` | uint8 | 无 | 0 / 1 |
| 8 | `manual_reset` | uint8 | 无 | 0 / 1，人工复位次数合计要 ≤ 1 |
| 9 | `magazine_position` | uint8 | 无 | 索引后工位 |
| 10 | `operator_note` | 字符串 | 无 | 可选，默认 `-` |

### 3.10 E 级集成周期表

用于 `E1_cycle.csv`。列序取 1.1 的十七列，末尾追加两列，符合第 5 节的追加规则。

| # | 列名 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1..17 | §19 十七字段 | 见 1.1 | 见 1.1 | 每发动作记录 |
| 18 | `recovery_time_ms` | uint32 | ms | 从 RELEASE 到 `MECH_RECOVERED` 为真的时间 |
| 19 | `index_retry_count` | uint8 | 次 | 本发后索引重试次数，超 `INDEX_MAX_RETRY` 升级故障 |

### 3.11 视频与笔记

`*_video.*` 是原始视频，不在这里定义格式，用采集设备的默认容器。`*_notes.md` 是自由文本，
至少记墙钟日期时间、室温、装夹方式、操作者、压力传感器量程、异常现象。视频文件名前缀与同组
CSV 一致，便于对照。

---

## 4. 枚举对照表

以 `aim_types.h` 为准。PC 端脚本按整数值解析，值错了整条链路都错。改动只能追加在末尾。

### 4.1 CycleState

区域 A，发射周期状态。底层 `uint8_t`，取值 0 到 10，共 11 个。

| 整数 | 名称 |
|---:|---|
| 0 | `BOOT` |
| 1 | `SAFE` |
| 2 | `HOME` |
| 3 | `READY` |
| 4 | `AIM_ALLOWED` |
| 5 | `PRELOAD` |
| 6 | `ARMED` |
| 7 | `RELEASE` |
| 8 | `RECOVER` |
| 9 | `INDEX` |
| 10 | `FAULT` |

### 4.2 AimState

区域 B，指向质量状态。底层 `uint8_t`，有效取值 0 到 6，共 6 个，整数 5 是保留空洞。

| 整数 | 名称 | 备注 |
|---:|---|---|
| 0 | `IDLE` | |
| 1 | `CALIBRATING` | |
| 2 | `SEARCHING` | |
| 3 | `TRACKING` | |
| 4 | `LOCKED` | 发射门判据用 |
| 5 | 保留 | 第一期 `FAULT` 旧址，已移入 `CycleState`。不得复用 |
| 6 | `CALIB_MODE` | 手动标定模式 |

### 4.3 FaultCode

底层 `uint16_t`，取值 0 到 14，共 15 个。软硬分级和触发条件见 `05_FIRMWARE/fault-codes.md`。

| 整数 | 名称 | 软/硬 |
|---:|---|---|
| 0 | `NONE` | 无 |
| 1 | `PERIPH_INIT` | 硬 |
| 2 | `HOME_TIMEOUT` | 软 |
| 3 | `HOME_SWITCH_CONFLICT` | 硬 |
| 4 | `RELEASE_GATE_SELFTEST` | 硬 |
| 5 | `OBS_LINK` | 软 |
| 6 | `INDEX_FAIL` | 软 |
| 7 | `WATCHDOG` | 软 |
| 8 | `MECH_STUCK` | 硬 |
| 9 | `ESTOP` | 软 |
| 10 | `MEMBRANE_RUPTURE` | 硬 |
| 11 | `DOUBLE_FEED` | 软 |
| 12 | `FIRE_INHIBIT_SHORT` | 硬 |
| 13 | `I2C_BUS_HANG` | 硬 |
| 14 | `STATE_ILLEGAL` | 硬 |

### 4.4 FireMode

底层 `uint8_t`，取值 0 到 1，共 2 个。遥测 `mode` 字段直接输出。

| 整数 | 名称 | 含义 |
|---:|---|---|
| 0 | `AIR_ONLY` | Mode A，纯空气脉冲 |
| 1 | `SOFT_PAYLOAD` | Mode B，超轻软载荷 |

### 4.5 其他枚举

`TransmitterState`（`SAFE`、`ARMED`、`FIRING`、`COOLDOWN`、`FAULT`）是第一期发射器抽象的占位，
没有指定整数值，也不进遥测，PC 端不需要解析。`FaultSeverity`（`NONE`、`SOFT`、`HARD`）只在
`faultSeverity()` 里用，由 `FaultCode` 映射得到，同样不进遥测。

---

## 5. 版本与兼容

### 5.1 字段只追加

遥测字段和枚举值只能追加在末尾。不得在中间插入、不得删除、不得改字段含义、不得重排顺序。

理由是 PC 端脚本按位置解析。`07_EXPERIMENTS/tools/common.py` 的 `TELEMETRY_FIELDS` 与
`parse_aim_line()` 用 `zip(TELEMETRY_FIELDS, parts)` 把第 n 个 token 直接绑到第 n 个字段名，
中间插一列，后面所有列全部错位。枚举同理，`AimState` 的整数 5 特意留成空洞不复用，就是为了
不让 `CALIB_MODE` 及其后的取值平移。

`MST,` 行的表头行带着字段名，将来可以改成按表头名解析，那样插入字段的代价会小。当前工具链
没有按名解析，冻结契约仍是按位置。改成按名解析之前，追加规则照旧。

### 5.2 真要改的时候

两个可选路径，按改动规模选：

- 追加字段：直接在十七列后加，PC 端解析器对多出来的尾部字段可以选择忽略。这是默认做法。
- 破坏性改动：新增一个行前缀，例如 `MST2,`，把新字段集放在新前缀下，`MST,` 保持原样。
  解析器按前缀分流，老脚本继续读老行。不要在 `MST,` 里改字段含义。

若走新增行前缀，`telemetry.h` 的表头、`interfaces.md` §4、`REQUIREMENTS.md` REQ-124、
`07_EXPERIMENTS/tools/` 解析脚本要在同一次提交里改齐，流程见 `interfaces.md` §6。

### 5.3 本文自身的版本

本文随字段集变化更新。改动记在文末变更记录里，写清改了哪一列、为什么、谁同步。字段没动而只是
补说明的改动也记一笔。

---

## 6. 与 TEST_PLAN.md 的对应

`07_EXPERIMENTS/TEST_PLAN.md` 逐级规定了数据文件名。下表把每个文件和它用的列定义表对上，
照着建表即可。曲线、事件、视频、笔记放在 `07_EXPERIMENTS/<级别>/`，整理后归档到 `08_DATA/`。

| 级别 | 数据文件 | 列定义表 |
|---|---|---|
| A | `A/A1_force.csv` | 表 3.3 力-位移 |
| A | `A/A1_displacement.csv` | 表 3.4 位移-时间 |
| A | `A/A1_pressure.csv` | 表 3.5 压力-时间 |
| A | `A/A1_video.*`、`A/A1_notes.md` | 见 3.11 |
| B | `B/B1_force.csv` | 表 3.3 |
| B | `B/B1_displacement.csv` | 表 3.4 |
| B | `B/B1_pressure.csv` | 表 3.5 |
| B | `B/B1_events.csv` | 表 3.6 事件 |
| B | `B/B1_video.*`、`B/B1_notes.md` | 见 3.11 |
| C | `C/C1_force.csv` | 表 3.3 |
| C | `C/C1_displacement.csv` | 表 3.4 |
| C | `C/C1_pressure.csv` | 表 3.5 |
| C | `C/C1_events.csv` | 表 3.6 |
| C | `C/C1_video.*`、`C/C1_notes.md` | 见 3.11 |
| D | `D/D1_B0_force.csv`、`D/D1_B1_force.csv` | 表 3.3 |
| D | `D/D1_B0_displacement.csv`、`D/D1_B1_displacement.csv` | 表 3.4 |
| D | `D/D1_B0_pressure.csv`、`D/D1_B1_pressure.csv` | 表 3.5 |
| D | `D/D1_events.csv` | 表 3.6 |
| D | `D/D1_video.*`、`D/D1_notes.md` | 见 3.11 |
| E | `E/E1_cycle.csv` | 表 3.10（§19 十七列 + 两列） |
| E | `E/E1_displacement.csv` | 表 3.4 |
| E | `E/E1_pressure.csv` | 表 3.5 |
| E | `E/E1_video.*`、`E/E1_notes.md` | 见 3.11 |
| E | `E/M1_index.csv` | 表 3.9 供给盘 |
| E | `E/M1_video.*`、`E/M1_notes.md` | 见 3.11 |
| B 或 C | `LIFE_100.csv`、`LIFE_500.csv`、`LIFE_1000.csv` | 表 3.8 寿命 |
| B 或 C | `LIFE_notes.md` | 见 3.11 |
| 各级 | 每次动作记录 | 表 3.2（§19） |

寿命文件放被测机构所在级，单级放 B，双级放 C。E 级的 `M1_index.csv` 是进入前置，先于 E1。
D 级的 BOUNDARY_0 / BOUNDARY_1 两组文件列定义相同，靠文件名区分状态。

每一发动作都要在动作记录表（表 3.2）里占一行，曲线和事件表通过 `cycle_count` 关联到它。
这样任意一条曲线都能回溯到当时的 `mechanism_version`、`boundary_state`、`preload_state` 和故障码。

---

## 变更记录

| 日期 | 改动 | 说明 |
|---|---|---|
| 2026-09-13 | 初版 | 固定遥测行按 `telemetry.cpp` 对齐，补 §19 记录、CSV 约定、枚举对照和 TEST_PLAN 对应 |
