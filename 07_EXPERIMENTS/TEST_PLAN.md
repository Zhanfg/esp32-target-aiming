# MST-01 实验计划（TEST_PLAN.md）

本文件对应主规格 §13 的原型序列、§14 的不可跳级、§16 的第一阶段流程、§17 的循环寿命、
§18 的供给盘独立实验，以及 §33 交付项。

执行纪律固定为 A → B → C → D → E，每一级达到通过标准才进入下一级（REQ-108）。任何一级不通过，
按 §30 对应条目处理，不得绕过继续。

规格书没有给出量化的通过标准，只给了判定方向（如 B1 看是否明显更快、重复、振铃过大）。
下文里的通过标准是可执行口径，属于本文件新增定义，评审确认后生效；判据中的百分比在拿到 A1 基线数据后
可以收紧或放宽，调整要记进本文件变更记录。

数据文件放在 `07_EXPERIMENTS/` 下对应的 A/B/C/D/E 目录。命名沿用 §16 的 `A1_force.csv` 风格，
同一测试的曲线、视频、笔记用同一前缀。所有记录同时满足 §19 的数据字段要求。

---

## 0. 通用约定

### 0.1 测试台与测量链

- 力：力传感器，采 force-displacement，采样率要能分辨 snap 事件。
- 位移：膜片位移传感器或高速视频跟踪，采 membrane displacement-time。
- 压力：低量程压力传感器，采 low-range pressure-time。量程在 A1 前确定并记录。
- 事件：STAGE1_STATE、STAGE2_STATE 直连 GPIO 中断，捕获 snap 事件时刻。磁钢靶的动力学扰动按 RSK-13 对照。
- 视频：高速视频记录机构动作，用于事件可分辨性与顺序核对。
- 环境：温度、装夹方式、操作者记进 notes。

### 0.2 每次动作的记录字段

输入变量按 §15.1，输出变量按 §15.2。

- 记录变量（输入）：mechanism version、boundary state、preload state、membrane material、
  membrane mounting、stage combination、cycle count。
- 测量变量（输出）：force-displacement curve、Stage 1 event time、Stage 2 event time、
  membrane displacement-time、low-range pressure-time、recovery time、repeatability、
  indexing success、fatigue drift。
- 结构化字段按 §19 的 timestamp 到 operator_note 全列出，fault_code 用枚举值（REQ-125）。

### 0.3 重复性口径

本文的「相对标准差」指同一条件下多次有效动作的样本标准差除以均值。除特别说明，默认每个条件
至少 10 次有效动作。

---

## A 级：MST-01A 普通膜片基线

### 进入条件

项目起点，没有上一级结论。前置是 MST-01A 的 CAD、测试台测量链、第一期的云台与状态机联调平台可用。
Mode A 纯空气模式（REQ-100）。

### 测试项

Test A1 普通膜片（§16）。低速执行器直接推动膜片，无 snap-through。

### 测量变量

force-displacement、membrane displacement-time、low-range pressure-time、recovery time、repeatability。

### 记录变量

mechanism version（A）、boundary state（默认 fixed）、preload state、membrane material、
membrane mounting、stage combination（single-passive）、cycle count。

### 数据文件名

```text
07_EXPERIMENTS/A/A1_force.csv
07_EXPERIMENTS/A/A1_displacement.csv
07_EXPERIMENTS/A/A1_pressure.csv
07_EXPERIMENTS/A/A1_video.*
07_EXPERIMENTS/A/A1_notes.md
```

### 通过标准

- 三条曲线都能稳定采到，无丢帧、无饱和，采样率足以分辨推入过程。
- 同一条件下膜片峰值位移与压力峰值的相对标准差不超过 10%，作为后续对照的重复性基线。
- force-displacement 与位移、压力时间曲线在复装膜片后仍可复现。
- 至少比较一种弹性体膜与一种聚酯膜（REQ-081），两种材料的数据都在。

### 不通过时的行动

属于测量链或装夹问题，修传感器、装夹、膜片后重测。不涉及 STOP 条件，不进 B 级。

---

## B 级：MST-01B 单级 snap

### 进入条件

A1 通过，基线数据与重复性口径确认，MST-01B CAD 与 STAGE1_STATE 采集通道可用（架构 §8）。
必须回放 A1 数据作为对照。

### 测试项

Test B1 单级 snap（§16）。一个失稳单元，与 A1 比较。

### 测量变量

force-displacement、Stage 1 event time、membrane displacement-time、low-range pressure-time、
recovery time、repeatability。

### 记录变量

同 A1，stage combination 改为 single-snap；boundary state 默认 fixed；增加弹性元件材料与厚度。

### 数据文件名

```text
07_EXPERIMENTS/B/B1_force.csv
07_EXPERIMENTS/B/B1_displacement.csv
07_EXPERIMENTS/B/B1_pressure.csv
07_EXPERIMENTS/B/B1_events.csv
07_EXPERIMENTS/B/B1_video.*
07_EXPERIMENTS/B/B1_notes.md
```

### 通过标准

判定方向来自 §16：膜片响应是否明显更快、是否可重复、是否产生过大振铃。可执行口径：

- 相对 A1，膜片到达峰值位移的时间中位数改善不低于 20%，或压力上升时间明显缩短。
- Stage 1 snap 事件能被 STAGE1_STATE 稳定捕获，事件时刻的相对标准差不超过 10%。
- 振铃不过大：主峰之后的振荡衰减到主峰 20% 以内的时间，不超过主峰上升时间的三倍。口径可调。
- 重复性不差于 A1，峰值位移与压力峰值相对标准差不超过 10%。

#### 模型先验预测（来自 `04_SIMULATION/`，纯软件，非实测）

以下预测用在实验里证伪或证实，不替代上面的通过标准。模型假设：`k_bar` 未标定、膜片线弹性且无粘弹滞后、
忽略摩擦与重力气压反作用与流体耦合、作动器理想化、一维对称质量集中。

- 模型给出单级相对普通膜片的膜片峰值速度为 8.939 对 0.469 m/s，约 19.1 倍；输入功 19.963 对 2.615 mJ。
- B1 实测若复现这个量级的优势，STOP-01 不触发，可继续 C 级，模型此条被证实。
- 这条预测也是 C1 的对照基准：模型认为单级已经拿到大部分收益（19.1 倍），双级只再多 18%。若 B1 复现，
  STOP-02 里「双级机构的复杂度与收益不匹配」这一面的先验成立，最终判定仍要等 C1。
- B1 实测若与模型相反，单级对普通膜片无可测优势，先触发 STOP-01。此时最可能失真的模型假设是
  `k_bar` 未实测、膜片粘弹滞后未建模、流体耦合缺失，应先核查这三项，再判断是模型偏差还是单级本身无价值。

### 不通过时的行动

**STOP-01**：单级相对普通膜片没有可测优势，停止双级研究（REQ-142）。按架构 §6.8 落成构建期机构
变体常量加研究门控标志，进 FAULT 之外的研究降级，结果记录到 10_FAILURE_ANALYSIS。也顺带检查
MECH_RECOVERED 判定可靠性（架构 §9.3、§6.11 末的 ARMED 卸载能力）。

---

## C 级：MST-01C 双级串联 snap

### 进入条件

B1 通过，单级相对普通膜片的优势可测且可重复。MST-01C CAD 与 STAGE2_STATE 采集通道可用。
这一级才开始判断 sequential snap 是否真有价值（§13）。

### 测试项

Test C1 双级 snap（§16）。两个失稳单元串联。

### 测量变量

force-displacement、Stage 1 event time、Stage 2 event time、inter-stage delay、
membrane displacement-time、low-range pressure-time、recovery time、repeatability。

### 记录变量

同 B1，stage combination 改为 dual-snap；记录两级弹性元件的几何与厚度、级间 gap。

### 数据文件名

```text
07_EXPERIMENTS/C/C1_force.csv
07_EXPERIMENTS/C/C1_displacement.csv
07_EXPERIMENTS/C/C1_pressure.csv
07_EXPERIMENTS/C/C1_events.csv
07_EXPERIMENTS/C/C1_video.*
07_EXPERIMENTS/C/C1_notes.md
```

### 通过标准

判定方向来自 §16：两个事件是否可分辨、顺序是否固定、是否偶发同时 snap、是否比 B 更有价值。
可执行口径：

- 两个 snap 事件在不少于 90% 的有效动作中可分辨，且顺序固定。
- 偶发同时 snap 的比例低于 5%。
- inter-stage delay 的相对标准差不超过 15%。
- 相对 B1，至少一项有工程价值的改善（脉冲峰值、上升时间、重复性之一），且寿命相关指标不显著下降。

#### 模型先验预测（来自 `04_SIMULATION/`，纯软件，非实测）

以下预测用在实验里证伪或证实，不替代上面的通过标准。模型假设同 B1：`k_bar` 未标定、膜片线弹性且
无粘弹滞后、忽略摩擦与重力气压反作用与流体耦合、作动器理想化、一维对称质量集中。

- 模型给出 C 相对 B 的膜片峰值速度为 10.573 对 8.939 m/s，只有约 18% 的增益；输入功从 19.963 升到
  37.623 mJ，接近翻倍。
- 模型准静态下两级各距 fold 只有 0.008 mm，几乎同时跳变；瞬态下级序不稳定，刚度失配 k_bar2/k_bar1
  到 1.5 倍即从 (1,0) 翻为 (0,1)，级间间隔在 2.32 到 7.20 ms 之间漂移。
- C1 实测若复现上述倾向，命中的是 STOP-02 里「更不稳定」这一分句，并支持「双级机构的复杂度与收益不匹配」。
  但「响应没有区别」不成立，模型自己就给出 18% 差异；「寿命下降严重」要等 §17 的 100/500/1000 阶梯。
  三个分句要一起看，不能只凭级序不稳就宣布命中 STOP-02。
- C1 实测若与模型相反，双级相对单级确有实质增益（明显超过 18%，且级序稳定），则模型在当前假设下失效。
  最可能失真的假设是 `k_bar` 未实测（边界刚度与等效杆刚度都依赖它）、膜片粘弹滞后未建模、流体耦合缺失。
  这三项要先通过标定和带气腔与压力的实验补齐，再重做解释。

### 不通过时的行动

**STOP-02**：双级比单级明显更差（更不稳定、响应无区别、寿命下降严重），回退单级（REQ-143）。
按架构 §6.8 走构建期回退，记录到 10_FAILURE_ANALYSIS。C 级通过后才允许进 D。

---

## D 级：MST-01D 可切换边界研究版

### 进入条件

C1 通过，双级相对单级有可测价值。MST-01D CAD 与 BOUNDARY_CMD 通道可用，边界状态可记录。
D 版定位为科研变量验证，不等于专利点（REQ-110）。

### 测试项

Test D1 边界变化（§16）。至少两个状态：BOUNDARY_0、BOUNDARY_1。

### 测量变量

force-displacement、stage order、inter-stage delay、membrane response、repeatability、fatigue drift。

### 记录变量

同 C1，boundary state 在 BOUNDARY_0 与 BOUNDARY_1 间切换；记录边界切换方式（locked/released）与预载状态。

### 数据文件名

```text
07_EXPERIMENTS/D/D1_B0_force.csv
07_EXPERIMENTS/D/D1_B0_displacement.csv
07_EXPERIMENTS/D/D1_B0_pressure.csv
07_EXPERIMENTS/D/D1_B1_force.csv
07_EXPERIMENTS/D/D1_B1_displacement.csv
07_EXPERIMENTS/D/D1_B1_pressure.csv
07_EXPERIMENTS/D/D1_events.csv
07_EXPERIMENTS/D/D1_video.*
07_EXPERIMENTS/D/D1_notes.md
```

### 通过标准

判定方向来自 §16 与 STOP-03：边界切换要有稳定可复现的输出差异。可执行口径：

- BOUNDARY_0 与 BOUNDARY_1 在 force-displacement、stage order、inter-stage delay、membrane response
  中至少两项存在可区分差异，建议以均值差超过合并标准差为判据。
- 两个状态各自的重复性仍满足 A1 口径。
- 跨天重测时差异方向和量级一致，即差异可复现。

### 不通过时的行动

**STOP-03**：boundary switching 没有稳定可复现的输出差异，D 版不进入论文主线（REQ-144）。
记录并关闭边界研究分支（架构 §6.8），核心机械照常推进。D 通过才允许多花精力在 E 之前继续边界研究。

---

## E 级：MST-01E 自动供给完整版

§16 只定义到 D1，E 级没有编号测试项。下文的 E1 是本文件派生的集成测试，判定门槛引用 §9.2 的工作逻辑
与 §18 的供给盘验收。E 级的前置是 M1 已通过。

### 进入条件

D1 通过（或边界分支按 STOP-03 关闭但核心机械通过）。M1 供给盘独立验收通过。MST-01E 的 MAG_* 通道、
Mode B 载荷与旋转盘装配完成。

### 测试项

- Test M1 供给盘独立实验（§18）作为进入前置。
- Test E1 完整发射周期集成（派生）：READY → 动作 → RECOVER → 确认复位 → INDEX → INDEX_OK → READY（§9.2）。

### 测量变量

recovery time、indexing success、membrane displacement-time、low-range pressure-time、repeatability。

### 记录变量

增加 magazine_position、magazine_index_ok、payload_id；mode 取 Mode B；记录每发载荷位与是否人工干预。

### 数据文件名

```text
07_EXPERIMENTS/E/E1_cycle.csv
07_EXPERIMENTS/E/E1_displacement.csv
07_EXPERIMENTS/E/E1_pressure.csv
07_EXPERIMENTS/E/E1_video.*
07_EXPERIMENTS/E/E1_notes.md
```

### 通过标准

- M1 已通过（人工复位不超过 1 次）。
- 完整周期连续不少于 30 发，无人工干预；INDEX_OK 失败不超过 1 次。
- 每发都能确认机械核心复位（MECH_RECOVERED 为真）后才开始索引。
- Mode B 出膛一致：视频核对无卡弹、双片、错位。
- 每发按 §19 写全字段。

### 不通过时的行动

卡料相关失败按 **STOP-04** 处理，先修供给，不与炮头集成（REQ-145）。其他失败按现象定位到具体一级，
回退到 C 或 D 重测。集成期若机构在储能态被顶死，升级 FAULT 并保持硬件断开（架构 §6.8）。

---

## 循环寿命测试（§17）

### 适用范围

在 B/C 中证明有价值的失稳机构上做，包括对应的膜片与弹性元件。测试对象是机械核心，不带供给盘。

### 阶梯与检查项

按 100、500、1000 次阶梯，每级结束检查：snap threshold 漂移、裂纹、塑性变形、膜片松弛、螺丝松动、
传感误判、恢复失败（§17、REQ-119）。

### 数据文件名

```text
07_EXPERIMENTS/C/LIFE_100.csv
07_EXPERIMENTS/C/LIFE_500.csv
07_EXPERIMENTS/C/LIFE_1000.csv
07_EXPERIMENTS/C/LIFE_notes.md
```

路径按被测机构所在级调整，单级放 B，双级放 C。

### 通过标准

- snap threshold 漂移不超过 10%。阈值口径在 B1 数据后确定。
- 无可见裂纹，无明显塑性变形。
- 膜片松弛导致的峰值位移衰减不超过 10%。
- 紧固件无松动，传感无误判，恢复失败 0 次。
- 每级的数据与 A1 重复性口径可比。

### 不通过时的行动

100 次已经明显漂移则不得进入集成版（§17、REQ-120）。换弹簧钢弹性件或改几何后重测；
连续两轮仍漂移则重评失稳方案。寿命结论记录到 10_FAILURE_ANALYSIS。

---

## 供给盘独立验收（§18）

### 前置

供给模块脱离炮头单独运行（REQ-121）。M1 在 E 级集成之前必须完成。

### 测试项

Test M1 连续 50 次索引（§18）。

### 测量与记录变量

```text
index_command
index_detected
jam
double_feed
misalignment
manual_reset
```

### 数据文件名

```text
07_EXPERIMENTS/E/M1_index.csv
07_EXPERIMENTS/E/M1_video.*
07_EXPERIMENTS/E/M1_notes.md
```

### 通过标准

- 50 次索引中人工复位不超过 1 次（REQ-123）。
- 出现 jam、double_feed、misalignment 时系统能按架构 §6.8 自动重试或安全停止，且不损伤机构。
- 每次索引都能读到 MAG_INDEX_OK，错位检测有效（载荷对齐参与释放门控，架构 §6.5 H3）。

### 不通过时的行动

**STOP-04**：50 次人工复位超过 1 次，先修供给，不与炮头集成。修完重跑 M1，通过后才做 E1。

---

## STOP 条件与实验通过标准对照

| STOP | 性质 | 对应实验 | 不通过时的动作 |
|---|---|---|---|
| STOP-01 | 实验结论 | B1 | 停止双级研究，研究降级，记录到 10_FAILURE_ANALYSIS |
| STOP-02 | 实验结论 | C1 | 回退单级，构建期机构变体回退，记录到 10_FAILURE_ANALYSIS |
| STOP-03 | 实验结论 | D1 | D 版不进入论文主线，记录并关闭边界研究分支 |
| STOP-04 | 运行期条件 | M1、E1 | 进软故障，单次 INDEX 重试；超门槛先修供给，不与炮头集成 |
| STOP-05 | 设计期预算 | BOM 校验 | 按 §21 顺序砍件，不得删状态检测、防脱、密封、互锁 |

STOP-05 不在实验通过标准里，它由设计评审的 BOM 校验门槛承担（架构 §6.8），
但每次出图后要按 `06_BOM/bom.csv` 核对。
