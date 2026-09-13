# MST-01 微型定向低能量脉冲平台
# Offline Agent Master Specification / 离线 Agent 完整交接文档

> **版本**：v2.0  
> **日期**：2026-09-13  
> **状态**：方案收敛、证据修订、可交给完全不联网的 Agent  
> **阅读要求**：Agent 必须从头到尾读完本文件后再开始工作。  
> **外部依赖**：无。理解项目、做第一轮机械设计、仿真规划、控制接口和实验规划，不需要访问互联网，也不需要向用户补充询问。  
> **特别说明**：本文件已经纠正 v1.0 中把“打蚊子”误解为实际用途的问题，也修正了 v1.0 对“可变边界 + 串联 snap-through”新颖性的过高判断。  

---

# 0. 先读这一页：项目到底是什么

## 0.1 用户真正想做的东西

用户想做的是一个：

- **很小**；
- **有炮台/云台式外形和指向能力**；
- **由单片机系统控制**；
- **动作很快**；
- **机械炮头尽量低成本**；
- **发射/脉冲执行原理希望有研究新意**；

的微型定向执行平台。

用户之前说“类似打蚊子”，**“打蚊子”只是为了表达体量、自动指向和动作形式的类比，不是实际用途**。

### “打蚊子”不是项目需求

本项目：

- 不以蚊虫为目标；
- 不要求蚊虫识别；
- 不要求翼频检测；
- 不要求计算机视觉；
- 不要求自动找虫；
- 不要求研究杀虫方法；
- 不应该把任何蚊虫论文、Photonic Fence、IR 翼频分类等内容作为项目设计依据。

**任何 Agent 不得再次把“打蚊子”当作实际应用场景。**

如果需要一句最准确的项目描述：

> **MST-01 是一个 MCU 可控制、火柴盒级、低能量、可模块化供给超轻软载荷的二维指向式快速机械脉冲执行平台。**

---

# 0.2 “炮台”一词的含义

本文沿用“炮台/炮头”只是为了沟通方便。

工程上应理解为：

> **directed low-energy impulse platform / 定向低能量脉冲平台**

不是以伤害、穿透、最大射程或高速度为目标的武器系统。

---

# 0.3 安全与设计边界

本项目允许：

- 纯空气脉冲；
- EVA / EPP / 毛毡 / 轻纸质等超轻软载荷；
- 低能量近距离封闭实验；
- 机构动力学、顺序失稳、脉冲整形研究；
- 自动索引供给；
- MCU 状态控制。

本项目不允许：

- 金属弹丸；
- 玻璃/陶瓷弹丸；
- 硬质高密度实心塑料弹丸；
- 针、箭、飞镖、尖锐载荷；
- 火药、爆燃、燃烧推进；
- 高压储气瓶；
- 以提高伤害、穿透或最大射程为优化目标；
- 人体或动物测试。

设计优先级固定为：

> **安全 > 可重复性 > 可靠性 > 小型化 > 低成本 > 输出强度**

---

# 1. 冻结的用户约束

## 1.1 成本

用户给出的 **¥50** 约束，只针对：

> **炮头机械架构本身**

### 计入 ¥50

- 弹性片；
- 柔性膜片；
- 轴、销、铜套、小轴承；
- O 型圈、密封件；
- 磁铁；
- 紧固件；
- 低密度软载荷；
- 非打印的小型机械件。

### 不计入 ¥50

- MCU；
- 主控板；
- 传感器后台；
- 云台舵机；
- 云台电机；
- 发射/蓄能所用外部执行电机或舵机；
- 电源；
- PCB；
- 线材；
- 通信模块；
- 3D 打印材料成本；
- CAD/仿真/实验仪器成本。

因此：

> **不要为了省 MCU、舵机或打印材料去破坏机械方案。**

---

# 1.2 尺寸

目标不是一个固定的毫米数字，而是：

> **尽可能小，并保持可靠。**

建议目标：

- 核心脉冲模块：约 **30–50 mm 长度级**；
- 核心横向尺寸：约 **15–25 mm 级**；
- 加供给模块后允许略大；
- 整体仍属于掌心/火柴盒级微型装置。

这些数字是**工程 envelope**，不是冻结尺寸。

如果 Agent 的仿真表明更大 10–20% 可以显著提升可靠性，可以先做可靠原型，再缩小。

---

# 1.3 控制

单片机是系统控制核心。

但 MCU 不应该承担“直接产生高速机械动作”的任务。

原则：

> **MCU 控制状态，机械结构产生快速动作。**

MCU 负责：

- 归零；
- 瞄准接口；
- 机械预载状态；
- 边界状态；
- 触发许可；
- 状态检测；
- 供给索引；
- 故障锁定；
- 循环计数；
- 日志。

---

# 2. v1.0 必须修正的两个错误

## 2.1 错误一：把“打蚊子”当作应用

错误。

用户只是类比。

### v2.0 处理

删除：

- 蚊虫检测设计；
- IR 翼频传感；
- Photonic Fence；
- 蚊虫分类；
- 目标生物特性。

这些内容不再属于项目。

---

## 2.2 错误二：高估“可变边界 + 串联 snap-through”的新颖性

v1.0 曾将：

> 可变边界 + 串联失稳

作为较强候选创新核心。

这不够严谨。

### 已发现的直接证据

2026 年 Filipe A. Santos 的论文：

**3D-Printed Serial Snap-Through Architectures for Programmable Mechanical Response**

已经明确公开：

- 两到四个串联 von Mises truss bistable units；
- sequential snap-through；
- programmable multistage response；
- 每个单元有 sliding support；
- 支撑可以 locked / released；
- 通过改变 boundary stiffness 改变 snap-through 和 post-snap 行为；
- 通过 geometry、engagement gap、support constraint 调整整体响应。

因此：

> **“串联 snap-through + 可切换边界条件”本身不能作为 MST-01 的新颖性核心。**

这是 v2.0 最重要的修正之一。

---

# 3. 当前最终工程基线

虽然单个元素都有 prior art，仍然保留下面架构作为**工程/研究验证基线**：

> **低速预载 → 两级串联失稳 → 直接机械耦合柔性膜片 → 瞬态低能量空气脉冲**

并支持：

- Mode A：纯空气脉冲；
- Mode B：超轻软质载荷；
- 可拆旋转供给盘；
- MCU 状态控制。

为什么仍然做它：

1. 尺寸适合；
2. 成本低；
3. 易于 A/B 测试；
4. 有明确可测力学变量；
5. 可以验证“多级失稳对脉冲波形是否真的有价值”；
6. 即便最后专利新颖性不足，也仍可以形成机械研究数据。

### 关键态度

**它是研究基线，不是已经证明的新发明。**

---

# 4. 系统总架构

```text
┌──────────────────────────────┐
│        外部上层控制/目标坐标      │
│   （具体来源不属于本项目范围）      │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│             MCU              │
│ HOME / READY / PRELOAD       │
│ RELEASE / RECOVER / INDEX    │
│ FAULT / SAFE                 │
└───────┬───────────────┬──────┘
        │               │
        │               └──────────► 供给索引执行器
        │
        ├──────────────────────────► 两轴指向机构
        │
        └──────────────────────────► 预载/边界/释放执行器
                                      │
                                      ▼
                        ┌────────────────────────┐
                        │ 机械脉冲核心             │
                        │ Stage 1                │
                        │      ↓                 │
                        │ Stage 2                │
                        │      ↓                 │
                        │ 柔性膜片                │
                        └──────────┬─────────────┘
                                   │
                          瞬态低能量空气位移
                                   │
                   ┌───────────────┴───────────────┐
                   ▼                               ▼
              Mode A                         Mode B
             纯空气模式                  超轻软载荷模块
                                                │
                                                ▼
                                        6–8 位旋转供给盘
```

---

# 5. 机械核心定义

## 5.1 Stage 1

功能：

- 接收低速位移输入；
- 储存弹性能量；
- 在达到结构临界状态时快速 snap-through。

Stage 1 不负责直接接触软载荷。

---

## 5.2 Stage 2

功能：

- 接收 Stage 1 的机械位移；
- 提供第二个非线性转变；
- 对输出时间历程进行改变；
- 驱动膜片接口。

Stage 2 的价值需要实验验证。

不能预设：

> 两级一定优于一级。

---

# 5.3 为什么必须保留单级对照

已有研究已经证明多级 snap-through 可以形成复杂多段 force-displacement response。

但这并不自动证明：

> 对微型膜片空气脉冲有好处。

因此必须有：

- A：普通慢推膜片；
- B：单级 snap；
- C：双级 snap；

三个原型。

如果 C 比 B：

- 更复杂；
- 更不稳定；
- 脉冲没有明显改善；

则双级应被淘汰。

---

# 5.4 边界调整模块

可以保留：

- fixed boundary；
- released/sliding boundary；
- 不同预载。

但它的定位是：

> **研究变量**

而不是：

> **默认专利新颖性。**

原因见 [SRC-04]。

---

# 6. 柔性膜片模块

## 6.1 功能

膜片用于将：

> 快速机械位移

转换为：

> 短时空气体积位移。

它不是高压储气装置。

---

# 6.2 材料候选

按优先级：

### 候选 A：薄硅胶膜

优点：

- 柔软；
- 密封好；
- 易更换；
- 疲劳较友好。

缺点：

- 材料滞后；
- 太软时会吸收 snap 输出。

### 候选 B：TPU 薄膜

优点：

- 耐磨；
- 方便做可更换片；
- 可与打印框架配合。

缺点：

- 批次刚度变化需要测量。

### 候选 C：PET / Mylar 薄膜

优点：

- 刚度稳定；
- 低滞后；
- 厚度一致。

缺点：

- 大变形能力低于硅胶。

### 第一轮建议

先至少比较：

- 一种弹性体膜；
- 一种聚酯薄膜。

不要在没有数据时指定唯一膜片材料。

---

# 6.3 膜片安装

采用：

> **可拆压环**

而不是永久胶粘。

原因：

- 方便更换；
- 方便改变材料；
- 方便检查损伤；
- 方便做 A/B 对照；
- 可以减少胶层成为隐藏变量。

---

# 7. 失稳元件材料

## 7.1 不建议把高循环失稳核心完全交给普通 FDM 柔性梁

FDM 打印：

- 层间性能变化；
- 疲劳寿命不稳定；
- 尺寸缩小后误差比例增大。

### 推荐结构

打印件负责：

- 框架；
- 支撑；
- 限位；
- 边界切换；
- 模块定位。

独立弹性元件负责：

- snap-through 主变形。

---

# 7.2 弹性元件候选

### A. 301 / 304 弹簧不锈钢薄片

优点：

- 易获得；
- 高弹性；
- 循环性能较好；
- 片材加工方便。

### B. PET 弹性片

优点：

- 很便宜；
- 易激光切割/手工打样；
- 安全；
- 适合早期验证。

缺点：

- 长期蠕变；
- 温度敏感；
- 疲劳性能需验证。

### C. 磷青铜薄片

优点：

- 弹性稳定；
- 易加工。

缺点：

- 成本略高；
- 对本项目不一定有必要。

### 原型顺序

1. PET 做几何验证；
2. 弹簧钢做寿命/重复性版。

---

# 8. 软载荷定义

## 8.1 主方案

推荐：

> **EVA 或 EPP 超轻软质圆片**

不是球。

---

# 8.2 选择圆片的工程原因

- 可以平铺在旋转盘内；
- 不会自由滚动；
- 每个位置可以独立约束；
- 供给通道短；
- 弹仓厚度低；
- 低密度；
- 空气阻力大；
- 可做不同颜色用于状态识别；
- 易冲切。

---

# 8.3 备用载荷

允许：

- 毛毡片；
- 轻纸片；
- 纸浆软片。

早期验证可优先纸片/毛毡。

---

# 8.4 禁止兼容硬载荷

机械接口不要设计成可以无修改装入：

- BB；
- 钢珠；
- 玻璃珠；
- 陶瓷珠；
- 金属柱；
- 飞镖；
- 尖头件。

这既是安全要求，也是设计约束。

---

# 9. 供给系统

## 9.1 最终工程选择

> **6–8 位旋转供给盘**

注意：

> 旋转弹仓/供给盘不是创新点。

已有明确专利 prior art，见 [PAT-03]、[PAT-04]。

---

# 9.2 工作逻辑

```text
当前载荷位置
    ↓
READY
    ↓
执行一次低能量动作
    ↓
RECOVER
    ↓
确认机械核心复位
    ↓
INDEX
    ↓
供给盘转动一个固定索引
    ↓
检测 INDEX_OK
    ↓
READY
```

---

# 9.3 为什么不是直线弹匣

软圆片在狭长通道中更容易：

- 翘曲；
- 摩擦；
- 叠片；
- 卡住。

旋转盘的每个载荷都有独立 pocket。

因此第一版优先可靠性。

---

# 9.4 自动换弹的准确含义

自动：

> **切换下一载荷位置。**

不自动：

> **自动拆掉空盘并装入新的供给盘。**

空盘由人工整体更换。

---

# 9.5 供给盘接口

要求：

- 可拔出；
- 有唯一方向防呆；
- 有零位标记；
- 有机械止挡；
- MCU 可以知道零位；
- 手动装回后可以重新 HOME。

---

# 10. MCU 控制规范

## 10.1 固定状态机

```text
BOOT
  ↓
SAFE
  ↓
HOME
  ↓
READY
  ↓
AIM_ALLOWED
  ↓
PRELOAD
  ↓
ARMED
  ↓
RELEASE
  ↓
RECOVER
  ↓
INDEX
  ↓
READY
```

任何异常：

```text
ANY_STATE
   ↓
FAULT
   ↓
SAFE
```

---

# 10.2 必须预留的逻辑输入

```text
PAN_HOME
TILT_HOME
MAG_HOME
MAG_INDEX_OK
STAGE1_STATE
STAGE2_STATE
MECH_RECOVERED
FIRE_INHIBIT
EXTERNAL_ALLOW
```

实际实现可以：

- Hall；
- 光电；
- 微动；
- 编码器；
- 电机电流辅助判定。

---

# 10.3 必须预留的输出

```text
PAN_CMD
TILT_CMD
BOUNDARY_CMD
PRELOAD_CMD
RELEASE_CMD
MAG_INDEX_CMD
STATUS_LED
DEBUG_TX
```

---

# 10.4 MCU 不应做什么

MCU 不应该：

- 用软件时序替代机械互锁；
- 在不知道机构状态时重复触发；
- 在未复位时索引供给盘；
- 将输出强度作为无限制可调参数。

---

# 11. 机械安全互锁

至少要有以下逻辑：

## Interlock 1

只有：

```text
MECH_RECOVERED = TRUE
```

才能进入 PRELOAD。

## Interlock 2

只有：

```text
MAG_INDEX_OK = TRUE
```

才能执行 Mode B 动作。

## Interlock 3

如果：

```text
FIRE_INHIBIT = TRUE
```

则 RELEASE 永远禁止。

## Interlock 4

FAULT 后必须重新 HOME。

---

# 12. 炮头模式

## Mode A — Air-only

没有载荷。

目的：

- 机械调试；
- 膜片测试；
- 压力波形；
- 寿命测试；
- 对照实验。

Mode A 应该是整个项目的第一工作版本。

---

## Mode B — Soft payload

加入：

- 超轻软圆片；
- 旋转供给盘。

目的：

- 验证供给；
- 验证系统集成；
- 验证动作一致性。

Mode B 不是研究输出强度的版本。

---

# 13. 原型序列

## MST-01A — 普通膜片基线

没有 snap-through。

低速执行器直接推动膜片。

目的：

> 建立基线。

---

## MST-01B — 单级 snap

一个失稳单元。

目的：

> 判断 snap-through 是否值得使用。

---

## MST-01C — 双级串联 snap

两个失稳单元。

目的：

> 判断 sequential snap 是否真的改善输出时间历程或重复性。

---

## MST-01D — 可切换边界研究版

加入：

- locked / released support；
- 或其他可逆边界改变；
- 多种预载状态。

目的：

> 建立 boundary state → mechanical response 的实验映射。

注意：

> 这个概念已有非常接近的 2026 prior art，因此 D 版主要是科研变量，不是自动等于专利点。

---

## MST-01E — 自动供给完整版

加入：

- 6–8 位供给盘；
- 索引；
- Mode B。

目的：

> 完整工程演示。

---

# 14. 不能跳过的 A/B/C 对照

Agent 不得直接做 E。

必须：

```text
A → B → C → D → E
```

每一级只有达到通过标准才继续。

否则无法知道问题来自：

- snap；
- 膜片；
- 第二级；
- 边界；
- 供给盘；

中的哪一个。

---

# 15. 实验变量

## 15.1 输入变量

允许记录：

- mechanism version；
- boundary state；
- preload state；
- membrane material；
- membrane mounting；
- stage combination；
- cycle count。

---

# 15.2 输出变量

测：

- force-displacement curve；
- Stage 1 event time；
- Stage 2 event time；
- membrane displacement-time；
- low-range pressure-time；
- recovery time；
- repeatability；
- indexing success；
- fatigue drift。

---

# 15.3 不作为优化目标

不要建立：

- “最大杀伤”；
- “最大穿透”；
- “最大危险射程”；
- “最重弹丸”。

这样的指标。

---

# 16. 第一阶段实验流程

## Test A1：普通膜片

目标：

- 获取普通慢推的位移和压力时间曲线。

保存：

```text
A1_force.csv
A1_displacement.csv
A1_pressure.csv
A1_video.*
A1_notes.md
```

---

## Test B1：单级 snap

比较 A1。

判断：

- 是否出现明显更快的膜片响应；
- 是否可重复；
- 是否产生过大的振铃。

---

## Test C1：双级 snap

重点：

- 两个事件是否可分辨；
- 顺序是否固定；
- 是否偶发同时 snap；
- 是否比 B 更有价值。

---

## Test D1：边界变化

至少两个状态：

```text
BOUNDARY_0
BOUNDARY_1
```

比较：

- force-displacement；
- stage order；
- inter-stage delay；
- membrane response。

---

# 17. 循环寿命

推荐阶梯：

```text
100 cycles
500 cycles
1000 cycles
```

每个阶段检查：

- snap threshold 漂移；
- 裂纹；
- 塑性变形；
- 膜片松弛；
- 螺丝松动；
- 传感误判；
- 恢复失败。

如果 100 次已经明显漂移：

> 不进入集成版。

---

# 18. 供给盘独立实验

供给模块必须先脱离炮头单独运行。

### Test M1

连续：

> 50 次索引。

记录：

```text
index_command
index_detected
jam
double_feed
misalignment
manual_reset
```

第一版最低要求：

> 50 次中需要人工复位不超过 1 次。

---

# 19. 数据格式

每次动作保存：

```text
timestamp
experiment_id
prototype_version
mechanism_version
mode
boundary_state
preload_state
membrane_id
payload_id
cycle_count
stage1_event
stage2_event
mechanism_recovered
magazine_position
magazine_index_ok
fault_code
operator_note
```

---

# 20. 目录结构

```text
MST-01/
├── 00_MASTER_SPEC/
│   └── MST-01_Offline_Agent_Master_Spec_v2.0.md
│
├── 01_PRIOR_ART/
│   ├── SOURCE_DIGEST.md
│   ├── CLAIM_MATRIX.md
│   └── VERSION_CORRECTIONS.md
│
├── 02_REQUIREMENTS/
│   ├── requirements.md
│   ├── interfaces.md
│   └── safety-boundary.md
│
├── 03_CAD/
│   ├── MST-01A/
│   ├── MST-01B/
│   ├── MST-01C/
│   ├── MST-01D/
│   └── MST-01E/
│
├── 04_SIMULATION/
│   ├── quasi-static/
│   ├── transient/
│   └── parameter-study/
│
├── 05_FIRMWARE/
│   ├── state-machine.md
│   ├── io-map.md
│   └── fault-codes.md
│
├── 06_BOM/
│   └── bom.csv
│
├── 07_EXPERIMENTS/
│   ├── A/
│   ├── B/
│   ├── C/
│   ├── D/
│   └── E/
│
├── 08_DATA/
├── 09_PLOTS/
├── 10_FAILURE_ANALYSIS/
└── 11_PUBLICATION_NOTES/
```

---

# 21. BOM 目标

此表是设计预算，不是实时市场报价。

| 项目 | 目标成本 |
|---|---:|
| 弹性片 | ¥3–8 |
| 膜片材料 | ¥2–5 |
| 轴/销/铜套 | ¥2–5 |
| 小轴承（如果确有必要） | ¥2–8 |
| 小磁铁 | ¥1–3 |
| O 型圈/密封件 | ¥2–5 |
| M2/M3 紧固件 | ¥2–4 |
| 软质实验载荷 | ¥1–3 |
| 其他机械杂件 | ¥2–5 |
| **目标总计** | **¥17–46** |

成本目标：

- 推荐 ≤ ¥40；
- 硬限制 < ¥50。

超过预算时依次减少：

1. 非必要轴承；
2. CNC/金属定制件；
3. 多余装饰壳；
4. 不必要传动级。

不要删：

- 状态检测；
- 防脱结构；
- 必要密封；
- 安全互锁。

---

# 22. CAD 设计原则

## 22.1 模块化

必须至少拆成：

```text
FRAME
STAGE_1
STAGE_2
BOUNDARY_MODULE
DIAPHRAGM_MODULE
PAYLOAD_MODULE
MAGAZINE_MODULE
```

---

# 22.2 可测试性

每个核心模块必须：

- 可以单独拆；
- 可以单独测；
- 能被照片/高速视频看到；
- 可以更换弹性件；
- 不需要破坏外壳才能换膜片。

---

# 22.3 不要过早一体化

第一版不要追求：

- 最漂亮；
- 最薄；
- 最少螺丝；
- 全部一体打印。

研究原型首先要：

> 看得见、拆得开、测得到。

---

# 23. 仿真要求

## 23.1 Quasi-static

至少得到：

- force-displacement；
- stable branches；
- snap point；
- Stage 1 / Stage 2 顺序；
- boundary sensitivity。

---

# 23.2 Transient

至少研究：

- snap 后速度；
- stage interval；
- membrane motion；
- structural ringing。

---

# 23.3 参数扫描

不要盲目扫所有几何尺寸。

优先：

```text
boundary stiffness
preload
stage mismatch
membrane stiffness
mechanical damping
```

---

# 24. 研究问题的正确表述

不要写：

> 我们发明了一种新的炮。

更合适：

> **研究紧凑串联弹性失稳机构与膜片负载耦合后，边界状态、级间非线性和膜片柔度如何共同影响瞬态机械脉冲的形成。**

论文关键词可以围绕：

- sequential elastic instability；
- snap-through；
- transient impulse shaping；
- diaphragm coupling；
- compact compliant actuator；
- nonlinear mechanics；
- tunable boundary conditions。

---

# 25. 当前 IP / 新颖性结论

## 25.1 已经不能主张的新颖点

下列都已经有明确 prior art：

| 想法 | 结论 |
|---|---|
| bistable / snap-through | 已有 |
| sequential snap-through | 已有 |
| serial snap-through | 已有 |
| programmable multistage snap | 已有 |
| locked/released boundary 调节 snap | 已有 |
| bistable membrane valve | 已有 |
| snap-action diaphragm air pulse | 已有 |
| snap action + soft projectile air launch | 已有 |
| compliant projectile launcher | 已有 |
| automatic rotary magazine | 已有 |
| motorized soft-dart rotary magazine | 已有 |
| 微型/MEMS bistable mechanism | 已有 |

---

# 25.2 当前仍可研究、但不能声称全球首创的部分

可以继续验证：

> **一个低成本、微型、直接机械耦合的多级失稳—膜片负载系统，在不同边界状态下是否能够产生可重复、可选择的瞬态脉冲时间历程。**

这可以成为：

- 论文研究问题；
- 新的系统集成方法候选；
- 后续专利继续查新的种子。

但不能在离线状态下直接得出：

> “具有全球专利新颖性”。

---

# 26. 非常重要：最接近的 prior art

下面资料是本项目最关键的离线证据。

Agent 必须理解，不需要联网。

---

# [SRC-01] Snapping Mechanical Metamaterials under Tension

**完整书目信息**

- Authors: Ahmad Rafsanjani, Abdolhamid Akbarzadeh, Damiano Pasini
- Title: *Snapping Mechanical Metamaterials under Tension*
- Journal: Advanced Materials
- Year: 2015
- Volume: 27
- Issue: 39
- Pages: 5931–5935
- DOI: **10.1002/adma.201502809**

**核心内容摘要**

研究利用具有局部弹性不稳定性的结构单元，在拉伸下产生 snap-through。多个单元的响应可以依次发生，从而形成明显的 sequential snap-through 和多阶段力学响应。

**对 MST-01 的意义**

- sequential snap-through 不是新概念；
- “多个单元依次跳变”不能作为核心专利主张；
- MST-01 必须证明的是它和膜片负载耦合后的具体瞬态行为，而不是重新证明 sequential snap。

---

# [SRC-02] Multi-step deformation mechanical metamaterials

**完整书目信息**

- Authors: Zhiqiang Meng, Mingchao Liu, Yafei Zhang, Chang Qing Chen
- Title: *Multi-step deformation mechanical metamaterials*
- Journal: Journal of the Mechanics and Physics of Solids
- Volume: 144
- Year: 2020
- Article: 104095
- DOI: **10.1016/j.jmps.2020.104095**

**核心内容摘要**

文章建立多阶段变形机械超材料，通过 sequential snap-through、buckling 等不同失稳过程实现多个明显的机械阶段；几何设计可以调节不同阶段发生的位置和顺序。

**对 MST-01 的意义**

- “多阶段力—位移曲线可设计”已经成熟；
- 不能把“两级结构会出现两个事件”当创新；
- MST-01 应关注不同阶段作用于动态膜片后的输出，而非静态多阶段本身。

---

# [SRC-03] Exploring sequential snapping bifurcation through a tunable energy landscape

**完整书目信息**

- First author: Ke Huang
- Coauthors include Jiaying Zhang, Weicheng Huang, Qingyun Wang 等
- Title: *Exploring sequential snapping bifurcation through a tunable energy landscape*
- Journal: Physical Review Applied
- Volume: 25
- Issue: 6
- Article: 064029
- Year: 2026
- DOI: **10.1103/nxpg-92kv**

**核心内容摘要**

研究直接围绕多个失稳单元的能量景观与 sequential snapping 分岔展开，说明通过改变系统参数可以改变失稳路径和顺序。

**对 MST-01 的意义**

- “调参数改变 snapping sequence”本身也已有明确研究；
- 不可简单把“MCU 改预载 → 顺序变化”声明为新发明。

---

# [SRC-04] 3D-Printed Serial Snap-Through Architectures for Programmable Mechanical Response

**完整书目信息**

- Author: Filipe A. Santos
- Title: *3D-Printed Serial Snap-Through Architectures for Programmable Mechanical Response*
- Journal: Advanced Engineering Materials
- Volume: 28
- Issue: 6
- Article: e202502854
- First published: 2026-02-04
- DOI: **10.1002/adem.202502854**

**这是 v2.0 最关键的新证据。**

**文章做了什么**

- 使用 monolithic 3D-printed von Mises truss (VMT) units；
- 将 2–4 个 bistable unit 串联；
- 每个单元可以出现 snap-through；
- 组合后形成 multistage response；
- 通过不同 engagement gaps 控制不同单元开始作用的位置；
- 通过 geometry 调节响应；
- 使用 guided sliding support；
- 支撑可以 locked 或 released；
- locked/released 会改变 boundary stiffness；
- boundary stiffness 改变 snap-through 和 post-snap response；
- 文章建立 inverted-compliance model；
- 目标是 programmable staged response。

**对 MST-01 的直接影响**

原先的：

> serial snap-through + switchable boundary

不能再作为独立核心创新。

MST-01D 可以继续做，但用途应是：

> 验证这种已有力学概念在微型膜片脉冲负载中的耦合行为。

---

# [SRC-05] Snap Pump: A Snap-Through Mechanism for a Pulsatile Pump

**完整书目信息**

- Authors: Kazuki Arakawa, Francesco Giorgio-Serchi, Hiromi Mochiyama
- Title: *Snap Pump: A Snap-Through Mechanism for a Pulsatile Pump*
- Journal: IEEE Robotics and Automation Letters
- Volume: 6
- Issue: 2
- Pages: 803–810
- Year: 2021
- DOI: **10.1109/LRA.2021.3052416**

**核心内容摘要**

该研究利用 snap-through 快速释放弹性能，通过流体腔产生 pulsatile pumping。核心意义在于：snap-through 与 fluid displacement / pulsatile output 的结合已经有直接学术先例。

**对 MST-01 的意义**

不能主张：

> “用 snap-through 产生流体/空气脉冲”

本身是新的。

同时说明：

> 弹性能转化到流体输出会有损失。

所以 MST-01 必须和普通膜片执行做 A/B 对照，而不能凭概念假定效率更高。

---

# [SRC-06] A soft, bistable valve for autonomous control of soft actuators

**完整书目信息**

- First author: Philipp Rothemund
- Authors include Alar Ainla, Lee Belding, Douglas J. Preston, et al.
- Title: *A soft, bistable valve for autonomous control of soft actuators*
- Journal: Science Robotics
- Volume: 3
- Issue: 16
- Article: eaar7986
- Date: 2018-03-21
- DOI: **10.1126/scirobotics.aar7986**

**核心内容摘要**

文章使用具有两个稳定状态的弹性膜作为机械开关来控制气流，展示 bistable membrane 可以参与软体气动系统的自主状态切换。

**对 MST-01 的意义**

- bistable membrane；
- pneumatic switching；
- soft actuator control；

均不是空白领域。

---

# [SRC-07] Monolithic scalable compliant mechanisms

**完整书目信息**

- Authors:
  - Jared R. Hunter
  - Bethany Parkinson
  - Jacob L. Sheffield
  - Mark B. Rober
  - Brian D. Jensen
  - Spencer P. Magleby
  - Nathan S. Usevitch
  - Larry L. Howell
- Title: *Monolithic scalable compliant mechanisms*
- Journal: PLOS ONE
- Volume: 21
- Issue: 1
- Article: e0340272
- Published: 2026-01-21
- DOI: **10.1371/journal.pone.0340272**

**核心内容摘要**

文章研究 displacement-driven compliant mechanisms 的尺度规律，指出在特定几何相似缩放条件下最大机械应力可以具有尺度不变性。文章使用多个示例，其中明确包括：

> one-piece, fully compliant projectile launcher

而且展示多个不同尺度版本。

**对 MST-01 的意义**

- 小型化 compliant launcher 已经有非常直接的先例；
- “单件柔顺机构 + 很小”不能作为新颖点；
- 如果后续缩小 MST-01，可参考其“先在较大尺度验证，再按相似关系缩放”的研究思想，但不能复制其 launcher 作为创新主张。

---

# [SRC-08] Magneto-sensitive bistable soft actuators: Experiments, simulations, and applications

**书目信息**

- Title: *Magneto-sensitive bistable soft actuators: Experiments, simulations, and applications*
- Journal: Applied Physics Letters
- Volume: 113
- Issue: 22
- Article: 221902
- Year: 2018
- DOI: **10.1063/1.5062490**

**核心相关点**

该研究展示 magneto-sensitive bistable soft actuator，并包含快速弹射小物体的 catapult 类应用。

**对 MST-01 的意义**

> bistability + ejecting small objects

已有先例。

---

# [PAT-01] US5628411A — Valve devices for use in sorting apparatus ejectors

**专利信息**

- Publication: **US5628411A**
- Title: *Valve devices for use in sorting apparatus ejectors*
- Inventors: Stewart J. Mills, Kenneth C. Henderson
- Original Assignee: Sortex Ltd
- Priority: **1994-12-01**
- Publication: **1997-05-13**

**核心技术**

专利描述用于自动分选设备的气动 ejector：

- pressurized air source；
- diaphragm valve；
- diaphragm 含 piezoelectric element；
- computer/microprocessor 根据传感器信号选择 ejector；
- diaphragm 在压力与弹性预载作用下实现 snap-action opening / closing；
- 输出短空气脉冲；
- 用空气脉冲改变飞行颗粒路径；
- 输出口位置可以调整，从而改变 diaphragm 的工作状态。

**为什么对 MST-01 非常重要**

这意味着下列组合在 1990 年代就已有非常接近的先例：

> 电子控制信号  
> → diaphragm 快速 snap  
> → air pulse  
> → 对飞行小物体施加定向作用

因此：

- “MCU/计算机触发膜片空气脉冲”不是创新；
- “微型快速 air ejector”不是创新；
- “可调膜片工作点”也不是完全空白。

---

# [PAT-02] US8590519B2 — Projectile launching devices particularly useful in toys

**专利信息**

- Publication: **US8590519B2**
- Application: US12/314,008
- Inventor: Benjamin J. Barish
- Priority: **2007-12-05**
- Filing: 2008-12-02
- Grant/Publication: **2013-11-26**

**核心技术**

该专利公开：

- housing + barrel；
- air pulsator；
- pump；
- projectile chamber；
- pressure-responsive valve；
- 先提高 chamber pressure；
- 到阈值后 valve snap-action opening；
- 产生 air pulse；
- air pulse 推动轻质软球或其他载荷；
- launch force / range 可以通过压力相关结构预设；
- 还公开 feeder 结构和多个软球连续供给。

专利甚至明确使用：

> soft, light-weight, spongy cellular material

作为载荷。

**为什么这是 MST-01 最重要的风险先例之一**

这已经非常接近：

> snap action → air pulse → soft payload

因此 MST-01 绝不能把：

> “失稳机构产生空气脉冲发射软载荷”

本身当作核心发明。

---

# [PAT-03] US9086251B2 — Indexing pneumatic launcher for multiple toy rocket projectiles

**专利信息**

- Publication: **US9086251B2**
- Inventor: Peter Cummings
- Original Assignee: KHA Concepts Ltd
- Priority/Filing: **2013-10-15**
- Publication: **2015-07-21**

**公开内容**

包含：

- pneumatic toy launcher；
- multiple projectiles；
- carousel；
- indexing wheel；
- 自动 reload / indexing；
- rotating drum magazine 类概念；
- air pulse 触发载荷。

**MST-01 影响**

> 气动 + rotating carousel + 自动索引

不是新颖点。

---

# [PAT-04] CA2624593A1 — Toy soft dart launcher

**专利信息**

- Publication: **CA2624593A1**
- Inventors:
  - Kenlip Ong
  - Kok Fai Tam
  - Tak To Lee
- Current assignee listed: Mattel Inc
- Priority: **2005-09-30**
- Filing: 2006-09-30
- Publication: **2007-04-12**

**核心公开**

专利 claim 直接包含：

- soft dart magazine；
- magazine 围绕 central axis 旋转；
- 多个 bore 圆周分布；
- motor；
- magazine rotator；
- cyclic uniform angular indexing；
- air cylinder + piston；
- 同一套驱动协调 piston 和 magazine rotation。

**MST-01 影响**

> 电机自动旋转软载荷弹仓

已经是明确 prior art。

因此 MST-01E 的旋转盘只是工程实现，不是专利亮点。

---

# [PAT-05] US8640683B2 — Soft-projectile launching device

**专利信息**

- Publication: **US8640683B2**
- Publication/grant: 2014-02-04
- Family priority: 2010-05-10

**核心相关内容**

公开软质载荷系统：

- soft projectiles；
- magazine；
- holder；
- spring / air pressure / other suitable firing mechanisms；
- magazine mating；
- 多种供给方式。

载荷之一是 super absorbent polymer soft projectile。

**MST-01 影响**

“软弹 + magazine + 多种机械/空气驱动”是高度成熟领域。

本项目选择 EVA/EPP 只是安全与工程决策。

---

# [PAT-06] CN101654216B — Arc MEMS compliant bistable mechanism

**专利信息**

- Publication: **CN101654216B**
- Title: 弧形 MEMS 柔顺双稳态机构
- Assignee: Shanghai Jiao Tong University
- Inventors:
  - 吴义伯
  - 王娟
  - 毛胜平
  - 丁桂甫
  - 张丛春
  - 汪红
- Priority/Filing: **2009-09-28**
- Grant publication: **2011-05-04**

**核心技术**

包括：

- curved beam；
- flexure spring；
- lumped mass；
- base；
- bistable behavior；
- MEMS-scale implementation。

专利公开的示例结构本身已经是微米尺度。

**MST-01 影响**

> “把 bistable mechanism 做得很小”

本身当然不是新颖点。

---

# 27. Prior-Art Matrix

| 技术元素 | 代表证据 | 风险等级 |
|---|---|---:|
| bistable mechanism | PAT-06 | 极高 |
| sequential snap | SRC-01, SRC-02 | 极高 |
| tunable sequential snapping | SRC-03 | 极高 |
| serial snap chain | SRC-04 | 极高 |
| switchable boundary support | SRC-04 | 极高 |
| snap → fluid pulse | SRC-05 | 极高 |
| bistable pneumatic valve | SRC-06 | 极高 |
| snap diaphragm → air pulse | PAT-01 | 极高 |
| snap air pulse → soft payload | PAT-02 | 极高 |
| small compliant launcher | SRC-07 | 极高 |
| rotary indexing magazine | PAT-03 | 极高 |
| motorized soft-dart magazine | PAT-04 | 极高 |
| soft projectile magazine | PAT-05 | 极高 |
| **多级失稳与被动膜片负载的特定瞬态耦合规律** | 未在本离线资料中发现完全同构单一来源 | 待研究 |
| **小型系统中不同边界状态对输出脉冲时间历程的可重复映射** | 邻近 prior art 很多 | 中高，不能直接主张 |
| **具体结构细节形成的新组合** | 需要未来 claim-level 检索 | 未定 |

---

# 28. Agent 对“创新”的正确处理

Agent 必须遵守：

## 可以说

- “候选研究贡献”；
- “本离线资料未包含完全同构单一来源”；
- “需要后续专利检索确认”；
- “系统级组合待验证”。

## 不可以说

- “全球首创”；
- “从未有人做过”；
- “具有确定专利新颖性”；
- “可直接申请专利且一定通过”。

---

# 29. 为什么还值得继续

尽管 prior art 很多，项目仍值得做，因为问题可以从：

> “发明一个没人见过的发射器”

改成更严谨的：

> **研究一个微型低成本非线性机械脉冲核心在被动膜片负载下的瞬态耦合规律，并把它做成可模块化、MCU 管理的实验平台。**

最终可能得到：

1. 一个可工作的工程装置；
2. 一套力—位移与动态脉冲数据；
3. 一个缩放研究；
4. 一个可复现实验平台；
5. 若具体结构组合确有差异，再进入正式专利 claim-level 查新。

---

# 30. 失败条件

出现以下情况，应立即停止当前分支而不是硬做：

## STOP-01

单级 snap 相比普通膜片没有可测优势。

→ 停止双级研究。

## STOP-02

双级比单级明显更差：

- 更不稳定；
- 响应没有区别；
- 寿命下降严重。

→ 回退单级。

## STOP-03

boundary switching 没有稳定可复现的输出差异。

→ D 版不进入论文主线。

## STOP-04

旋转供给盘 50 次测试频繁卡料。

→ 先修供给，不与炮头集成。

## STOP-05

机械本体预计成本 > ¥50。

→ 简化零件。

---

# 31. Agent 不得向用户追问的事项

如果没有额外信息，Agent 自己按以下规则决定：

### 材料不知道选哪个

先：

> PET 原型 → 弹簧钢验证版。

### 膜片不知道选哪个

至少并行测试：

> 弹性体膜 + PET 膜。

### 轴承还是铜套

低载荷、小角度：

> 优先铜套/打印导向。

只有确有必要才加轴承。

### 6 还是 8 个载荷位

如果 8 位导致整体明显变大：

> 先 6 位。

可靠后再 8 位。

### 结构太大

先保证：

> 重复性。

然后缩小。

### 某个方案漂亮但不好测

选：

> 好测的。

---

# 32. 离线 Agent 的第一阶段具体任务

拿到文件后不要问用户。

依次完成：

## TASK-01

建立：

```text
REQUIREMENTS.md
```

从本文件提取所有 MUST / MUST NOT / SHOULD。

---

## TASK-02

做三个机械概念：

```text
MST-01A
MST-01B
MST-01C
```

先不要做弹仓。

---

## TASK-03

为 A/B/C 建立对照实验矩阵。

---

## TASK-04

建立有限元模型：

- A：膜片；
- B：单 snap + 膜片；
- C：双 snap + 膜片。

---

## TASK-05

输出：

```text
DESIGN_DECISION_LOG.md
```

每个决策写：

- decision；
- reason；
- alternative；
- evidence；
- validation；
- failure condition。

---

## TASK-06

只有 B/C 通过后，再设计 D。

---

## TASK-07

只有核心机械通过后，再做 E 供给盘。

---

# 33. 必须交付的文件

最终至少：

```text
REQUIREMENTS.md
SYSTEM_ARCHITECTURE.md
INTERFACES.md
DESIGN_DECISION_LOG.md
BOM.csv
RISK_REGISTER.md
TEST_PLAN.md
FAILURE_MODES.md
STATE_MACHINE.md
DATA_SCHEMA.md
MST-01A CAD
MST-01B CAD
MST-01C CAD
MST-01D CAD
MST-01E CAD
```

---

# 34. 不要做的事情

Agent 不要：

- 重新把项目定义成灭蚊器；
- 研究目标检测；
- 重做视觉系统；
- 搜集蚊虫资料；
- 默认高功率激光；
- 换成火药；
- 换成高压气瓶；
- 换成硬弹；
- 为了“看起来像炮”牺牲研究可测性；
- 为了“创新”故意增加没有价值的复杂机构；
- 删除 A/B/C 对照实验；
- 把 prior art 已覆盖的元素写成首创。

---

# 35. 最终研究假设

当前唯一需要实验回答的核心问题：

> **当一个紧凑串联非线性弹性机构直接耦合到一个柔性膜片负载时，单元数量、边界状态、预载和膜片柔度是否可以产生稳定、可重复、可区分的瞬态脉冲时间历程；这种响应是否相较普通直接驱动或单级 snap 有实际工程价值？**

注意：

> 这是一条**研究假设**，不是专利声明。

---

# 36. 交接时 Agent 应该记住的 12 条

1. “打蚊子”只是尺寸/形态类比。
2. 项目不是灭蚊器。
3. 机械炮头预算 < ¥50。
4. MCU/电机/舵机/打印材料不算 ¥50。
5. 尺寸目标是火柴盒/掌心级，不是固定毫米。
6. 第一版先做纯空气模式。
7. 软载荷只用低密度柔软材料。
8. 旋转供给盘是工程选型，不是创新。
9. sequential snap 已有。
10. serial snap + switchable boundary 已有。
11. snap diaphragm air pulse 已有。
12. 真正需要验证的是“多级非线性核心 + 膜片负载”的具体动态价值。

---

# 37. v2.0 自查

## 用户原始意图

- [x] 很小；
- [x] 类炮台形态；
- [x] MCU 为主要控制；
- [x] 希望新原理；
- [x] 要查论文和专利；
- [x] 炮头机械架构 < ¥50；
- [x] MCU/舵机/电机不算；
- [x] 3D 打印材料不算；
- [x] 可自动切换下一软载荷；
- [x] 软质载荷；
- [x] 希望一份 Agent 看完即可执行的文档。

## 错误纠正

- [x] 删除蚊虫检测；
- [x] 明确“打蚊子”只是类比；
- [x] 修正 serial + variable-boundary 新颖性判断；
- [x] 加入 Santos 2026 直接 prior art；
- [x] 加入 1994 优先权 snap diaphragm air ejector 专利；
- [x] 加入 2007 优先权 air-pulsator soft projectile 专利；
- [x] 明确旋转仓已有专利；
- [x] 明确 compliant launcher 已有论文。

## 工程内容

- [x] 系统架构；
- [x] Stage 1；
- [x] Stage 2；
- [x] 膜片；
- [x] 材料候选；
- [x] 模式 A/B；
- [x] 供给盘；
- [x] MCU 状态机；
- [x] I/O；
- [x] 互锁；
- [x] 原型序列；
- [x] 实验计划；
- [x] 寿命；
- [x] 数据格式；
- [x] 目录结构；
- [x] BOM；
- [x] CAD 原则；
- [x] 仿真；
- [x] 失败条件；
- [x] Agent 无需追问的默认决策；
- [x] 离线任务；
- [x] 最终交付物。

## 证据完整性

- [x] 每一类关键 prior art 都给出标题或专利号；
- [x] 论文给出 DOI；
- [x] 专利给出 priority/publication 等必要元数据；
- [x] 每条来源都有“对 MST-01 意味着什么”；
- [x] 未把检索不到等同于全球不存在。

---

# 38. 最终冻结结论

从现在开始：

## 不再讨论“是不是做灭蚊”

不是。

## 不再把以下作为创新

- sequential snap；
- serial snap；
- variable boundary snap；
- snap air pulse；
- soft projectile；
- rotary magazine。

## 工程基线冻结为

> **A/B/C/D/E 五级原型验证路线**

## 研究核心冻结为

> **多级非线性机械核心与柔性膜片负载之间的瞬态耦合、可重复脉冲整形与小型化。**

## 专利态度冻结为

> **高 prior-art 密度，只有具体结构和系统组合完成后，才值得继续做正式 claim-level 新颖性检索。**

---

# 39. 给离线 Agent 的启动指令

```text
你接手的是 MST-01 微型定向低能量脉冲平台。

本文件是完整上下文，不需要联网，不需要向用户询问背景。

先完整阅读文件，再工作。

最重要的纠正：
“打蚊子”只是用户描述尺寸和自动炮台形态的类比，项目不是灭蚊器，不做蚊虫检测。

冻结约束：
- 机械炮头本体 < ¥50；
- MCU、舵机、电机、电源、PCB、传感后台、3D 打印材料不计入；
- 尽可能小，目标火柴盒/掌心级；
- 只允许低能量空气脉冲和超轻软质载荷；
- 禁止硬质弹丸、爆燃、高压储气和伤害/穿透优化；
- 先 A 普通膜片，再 B 单级 snap，再 C 双级，再 D 可切换边界，最后 E 自动旋转供给盘；
- sequential snap、serial snap、variable boundary、snap air pulse、soft projectile 和 rotary magazine 都有 prior art，不得称首创；
- 研究核心是多级非线性机械机构与柔性膜片负载的瞬态耦合及可重复脉冲整形。

立即执行：
1. 提取 REQUIREMENTS.md；
2. 建立 A/B/C 系统架构；
3. 做设计决策日志；
4. 做仿真和实验计划；
5. 只有 A/B/C 证明双级有价值后才进入 D/E；
6. 遇到未规定的参数按“安全 > 可重复性 > 可靠性 > 小型化 > 成本 > 输出强度”决策；
7. 不要重新发散项目用途；
8. 不要声称全球首创。
```

---

# 40. 文档结束

这份文件应被视为：

> **MST-01 当前唯一主规格。**

v1.0 中与本文件冲突的内容：

> **全部以 v2.0 为准。**
