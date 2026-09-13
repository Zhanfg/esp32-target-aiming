# 论文提纲

本文件给出一篇论文的骨架。实验还没有做，第 5 节的结果部分是一份待填的图表清单，逐条注明由哪个实验或
哪个模型产出、数据文件在仓库的哪个路径。清单里没有任何编造的数字。关键词按规格书 §24 列。

引用来源统一用 `01_PRIOR_ART/SOURCE_DIGEST.md` 的 SRC-/PAT- 编号，prior-art 的技术元素与证据对应关系
直接引 `01_PRIOR_ART/CLAIM_MATRIX.md`，不另行编号。数据引用 `08_DATA/`，图引用 `09_PLOTS/`，本目录不另存。
对外提交前按 `README.md` 的约定把安全互锁的具体接法拿掉。

## 0. 题名与关键词

题名沿研究问题的口径，建议围绕"串联弹性失稳机构与被动膜片的瞬态脉冲耦合"，不要出现武器或杀伤相关词。
关键词取 §24：

- sequential elastic instability
- snap-through
- transient impulse shaping
- diaphragm coupling
- compact compliant actuator
- nonlinear mechanics
- tunable boundary conditions

## 1. 问题

问题陈述用 §24 的正确表述：研究紧凑串联弹性失稳机构与膜片负载耦合后，边界状态、级间非线性和膜片柔度
如何共同影响瞬态机械脉冲的形成。不需要实验回答的版本见 §35（本文提纲不重复，引用即可）。本节要交代
三点：现有 sequential snap 与 snap 流体脉冲的研究已经很多（第 2 节展开）；缺的是多级非线性核心直接
耦合被动膜片后的动态规律；本文用 A/B/C 三级对照去回答单元数量、边界状态、预载、膜片柔度各自贡献了什么。

## 2. 相关工作

按 `CLAIM_MATRIX.md` 的技术元素组织，每段末尾用证据编号收口。分五块：

多级失稳与可编程响应。SRC-01、SRC-02、SRC-03、SRC-04。已覆盖多个失稳单元串联、多阶段力—位移曲线、
失稳顺序可调、locked / released 边界刚度调节。本文与这一块的区别只在"这些机制作用到被动膜片后的动态
结果"，且这层还是待验证。

失稳到流体脉冲。SRC-05 把 snap-through 与 pulsatile pumping 连起来，PAT-01 用膜片 snap 产生短空气脉冲
（1994 优先权），PAT-02 用失稳式空气脉冲推动软质载荷（2007 优先权）。这一块已经把"弹性能转成空气脉冲
再作用到软载荷"封死。本文的膜片只作被动负载，驱动的来源是多级机械核心，这两点的组合是待研究的点。

膜片与阀。SRC-06 是双稳软膜气动开关。本文的膜片不承担开关功能，只把机械位移转成短时空气体积位移，
相关工作段要明确切分。

柔顺机构与小型化。SRC-07 的单体柔顺发射器与尺度规律、PAT-06 的 MEMS 双稳机构。小型化本身不是贡献，
只能在方法里作为工程约束出现。SRC-08 说明双稳态弹射小物体也已有先例。

供给系统。PAT-03、PAT-04、PAT-05 覆盖气动旋转索引、电机旋转软镖弹仓、软弹加弹仓。MST-01E 的旋转盘是
工程实现，写进系统描述，不写进贡献。

本节的收尾一段，把 `CLAIM_MATRIX.md` 末三行原样带出，说明本文的候选贡献落在这三个点上，并写明离线
资料未包含完全同构单一来源、系统级组合待验证。

## 3. 方法

本节要让人看出软硬件资产是真做出来的东西，逐项给出仓库路径。

### 3.1 参数化机械设计与构件

尺寸与布局集中在 `03_CAD/lib/params.py`，参数来源登记在 `PARAM_SOURCES`。几何模块分 FRAME、STAGE_1 /
STAGE_2（von Mises truss，弹性片为独立件，见 `03_CAD/lib/stage.py`）、DIAPHRAGM_MODULE
（`03_CAD/lib/membrane.py`）、RELEASE_MODULE（自锁卡榫加电磁铁拔销，`03_CAD/lib/release.py`）。
A/B/C 三级各自有 `build.py` 装配并做几何自检，导出的 STL 在 `03_CAD/MST-01A/exports/`、
`03_CAD/MST-01B/exports/`、`03_CAD/MST-01C/exports/`。B 级比 A 级多 STAGE_1 与 RELEASE_MODULE，
C 级比 B 级多一个 STAGE_2 和级间垫片 `STAGE_GAP_shim.stl`。失稳单元几何是 a=8 mm、L=10 mm、h0=6 mm，
与 `04_SIMULATION/models/params.py` 的 `A_VMT` / `L_VMT` 对齐。三级轴向长度分别为 32 / 38 / 44 mm，
落在 §1.2 envelope 内。

### 3.2 释放门控硬件

`03_CAD/electronics/` 是释放路径的硬件门控板。`release-gate.kicad_sch` 与 `release-gate.kicad_pcb`
给出原理图与板图，导出件为同目录 PDF / SVG，物料在 `bom.csv`，板级验证步骤在 `test-procedure.md`。
逻辑是两级与门树 Q1 / Q2（一片 74HC21），加 74HC123 单稳态把释放脉宽钳在约 33 ms，74HC74 做故障锁存与
武装锁存。这一块把架构 §6.6 的 H1 到 H6 做成实体，对应 §10.3 的 `RELEASE_CMD` 与 §11 的四条互锁。
方法一节要说明 MCU 只发状态与释放请求，物理门控独立于固件，芯片卡死也放不出一次释放。

### 3.3 固件与状态机

固件工程在 `05_FIRMWARE/`，PlatformIO + Arduino + C++17，主控 ESP32-S3。状态机做成双区域：区域 A 管发射
周期，区域 B 管指向质量，跨区域用守卫矩阵约束，细节见 `05_FIRMWARE/state-machine.md`。软件互锁在
`05_FIRMWARE/src/hal/safety_gate.*`，是硬件与门之外的第二层。故障码与软硬分级见
`05_FIRMWARE/fault-codes.md`。遥测按 §19 的 17 个字段输出，格式在 `05_FIRMWARE/README.md` 第 4 节。
论文里报告的是第二期执行层：`BOUNDARY` / `PRELOAD` / `RELEASE` / `MAG_INDEX` 四路命令，与
`STAGE1_STATE` / `STAGE2_STATE` 两路事件捕获。

### 3.4 数值模型

`04_SIMULATION/` 是纯 Python 与 numpy 的力学模型，不依赖有限元软件与 scipy。膜片用 Föppl 大挠度薄膜
理论的单模态近似（`models/membrane.py`），失稳单元用经典 von Mises truss 势能加可滑动底端的边界弹簧
（`models/vmt.py`），A/B/C 都装配成一维串联链（`models/chain.py`），准静态用位移控牛顿法、瞬态用速度
Verlet 辛积分（`models/solver.py`）。三个 study 对应 §23.1 准静态、§23.2 瞬态、§23.3 参数扫描，见
`studies/quasi_static.py`、`studies/transient.py`、`studies/parameter_study.py`。模型的假设、局限与
参数出处写在 `04_SIMULATION/README.md`，论文里要如实转述，尤其是忽略气腔与流体耦合、材料粘弹这两条。

### 3.5 实验方法

实验纪律与通过标准见 `07_EXPERIMENTS/TEST_PLAN.md`。测量链是力传感器、位移传感器或高速视频、低量程
压力传感器、事件 GPIO 中断、高速视频。执行次序固定 A → B → C → D → E，每级达标才进下一级（§14、
REQ-108）。重复性口径按 TEST_PLAN 第 0.3 节。数据写入 `08_DATA/`，图从那里读。

## 4. 实验

按 TEST_PLAN 的 A 到 E 五级写，每级给进入条件、测试项、测量变量、通过标准与不通过时的处置。

- A1 普通膜片基线，无 snap（§16）。产出 A1 的力、位移、压力、视频与笔记。
- B1 单级 snap，与 A1 对照（§16）。判定方向：膜片响应是否明显更快、是否可重复、振铃是否过大。
  不通过触发 STOP-01。
- C1 双级串联 snap（§16）。判定两个事件是否可分辨、顺序是否固定、是否偶发同时 snap、是否比 B 更有价值。
  不通过触发 STOP-02。
- D1 边界变化，至少 BOUNDARY_0 与 BOUNDARY_1 两个状态（§16）。不通过触发 STOP-03。
- E 级：M1 供给盘连续 50 次索引（§18）作为前置，E1 是派生的完整周期集成测试（§9.2）。卡料失败按
  STOP-04。
- 循环寿命按 100 / 500 / 1000 阶梯（§17）。

## 5. 结果（待填图表清单）

实验数据尚未产出。下表的实验数据文件路径按 `07_EXPERIMENTS/TEST_PLAN.md` 定义，文件本身还不存在；
模型数据文件已经落盘，可在 `04_SIMULATION/results/` 找到。实验完成后逐条核对路径再填。

| 编号 | 内容 | 产出实验或模型 | 数据文件 |
|---|---|---|---|
| 图 1 | 三级原型装配与截面 | CAD 模型（待渲染） | `03_CAD/MST-01A/exports/MST-01A_assembly.stl`、`03_CAD/MST-01B/exports/`、`03_CAD/MST-01C/exports/MST-01C_assembly.stl` |
| 图 2 | A/B/C 准静态力—位移曲线 | 模型 + 实验 | 模型 `04_SIMULATION/results/A_force_displacement.csv`、`B_force_displacement.csv`、`C_force_displacement.csv`；实验 `07_EXPERIMENTS/A/A1_force.csv`、`07_EXPERIMENTS/B/B1_force.csv`、`07_EXPERIMENTS/C/C1_force.csv` |
| 图 3 | snap 点、稳定分支与级序 | 模型 + 实验 | 模型 `04_SIMULATION/results/C_force_displacement.csv`；实验 `07_EXPERIMENTS/C/C1_events.csv` |
| 图 4 | 边界刚度敏感性 | 模型 | `04_SIMULATION/results/boundary_sensitivity.csv` |
| 图 5 | 瞬态膜片速度与位移时间曲线 | 模型 + 实验 | 模型 `04_SIMULATION/results/transient_A.csv`、`transient_B.csv`、`transient_C.csv`、`transient_summary.csv`；实验 `07_EXPERIMENTS/A/A1_displacement.csv`、`07_EXPERIMENTS/B/B1_displacement.csv`、`07_EXPERIMENTS/C/C1_displacement.csv` |
| 图 6 | 两级事件时刻与级间时延 | 模型 + 实验 | 模型 `04_SIMULATION/results/transient_summary.csv`（`stage_interval_s`）、`04_SIMULATION/results/sweep_mismatch.csv`；实验 `07_EXPERIMENTS/C/C1_events.csv` |
| 图 7 | 低量程压力—时间 | 实验（模型暂无气腔） | `07_EXPERIMENTS/A/A1_pressure.csv`、`07_EXPERIMENTS/B/B1_pressure.csv`、`07_EXPERIMENTS/C/C1_pressure.csv` |
| 图 8 | 同条件多周期重复性叠加 | 实验 | `07_EXPERIMENTS/A/A1_displacement.csv`（每条件至少 10 次）、`07_EXPERIMENTS/B/B1_displacement.csv`、`07_EXPERIMENTS/C/C1_displacement.csv` |
| 图 9 | 边界状态对照 | 实验 | `07_EXPERIMENTS/D/D1_B0_force.csv`、`D1_B0_displacement.csv`、`D1_B0_pressure.csv`、`D1_B1_force.csv`、`D1_B1_displacement.csv`、`D1_B1_pressure.csv` |
| 图 10 | 参数扫描趋势 | 模型 | `04_SIMULATION/results/sweep_boundary.csv`、`sweep_preload.csv`、`sweep_mismatch.csv`、`sweep_membrane.csv`、`sweep_damping.csv` |
| 图 11 | 循环寿命漂移 | 实验 | `07_EXPERIMENTS/C/LIFE_100.csv`、`LIFE_500.csv`、`LIFE_1000.csv`（被测机构在 B 级时路径改到 `07_EXPERIMENTS/B/`） |
| 图 12 | 供给盘索引可靠性 | 实验 | `07_EXPERIMENTS/E/M1_index.csv` |
| 图 13 | 完整周期时序 | 实验 | `07_EXPERIMENTS/E/E1_cycle.csv`、`E1_displacement.csv`、`E1_pressure.csv` |
| 表 1 | 实验变量与观测变量 | TEST_PLAN | `07_EXPERIMENTS/TEST_PLAN.md` 第 0.2 节，字段对应规格书 §15.1、§15.2、§19 |
| 表 2 | prior-art 矩阵 | 离线台账 | `01_PRIOR_ART/CLAIM_MATRIX.md` |
| 表 3 | STOP 条件与判定 | TEST_PLAN | `07_EXPERIMENTS/TEST_PLAN.md` 的 STOP 对照表 |

## 6. 讨论

讨论分四块。

耦合结论。按第 5 节的图和表说单元数量、边界状态、预载、膜片柔度各自对脉冲时间历程的贡献，以及哪些
交互项在数据里显现。没有观测到的交互要说明是没测还是没影响。

模型与实验的差异。`04_SIMULATION/README.md` 列了模型的忽略项，气压反作用与流体耦合、材料粘弹、作动器
自身动力学是定量失真的主要来源。讨论里逐条对照，不掩盖。

失败情形。若 STOP-01 或 STOP-02 成立，按 `RESEARCH_FRAMING.md` 第 6 节的口径写。阴性结果限定在名义参数
区间，不做一般化断言。记录归到 `10_FAILURE_ANALYSIS/`。

适用边界与安全。装置只做低能量空气脉冲和超轻软载荷，不讨论伤害、穿透、射程。§28 的两张清单在这里
再执行一次，讨论段不得出现"全球首创""从未有人做过"这类措辞。

## 7. 结论

结论回答 §35 的假设：在给定参数区间里，单元数量和边界状态是否产生了稳定、可重复、可区分的脉冲时间
历程，以及相对普通驱动或单级 snap 是否有可测的工程价值。若结论是回退单级，就如实写回退；若结论支持
多级，也只写到"本离线资料未包含完全同构单一来源"，把 claim-level 查新留给后续。

## 附录 A 数据与复现

数据字段见 §19，数据文件归 `08_DATA/`，图从 `08_DATA/` 读、放 `09_PLOTS/`，不在本目录重存。模型重跑
命令见 `04_SIMULATION/README.md`，固件构建命令见 `05_FIRMWARE/README.md`，CAD 重建命令见
`03_CAD/README.md`。

## 附录 B 图表命名与状态

图只放结果，推导在 `02_REQUIREMENTS/`，模型细节在 `04_SIMULATION/`。每张图标注数据来源路径与生成脚本，
保证换个人能重画。第 5 节清单里的实验项在数据落盘前保持"待填"状态，不要预填占位数值。
