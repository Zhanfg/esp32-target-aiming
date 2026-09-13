# MST-01 Prior Art 来源摘要台账（离线版）

这份台账把规格书 §26 的 14 条来源压成可检索的条目，论文 8 条（SRC-01 到 SRC-08），专利 6 条（PAT-01 到 PAT-06）。
整理过程没有联网。它是从离线资料里摘出来的，不代表全球检索已经做完。§25.2 讲得很清楚，凭这份台账得不出「具有全球专利新颖性」这种结论，
也不能把「这里没收录」当成「世上不存在」。后面做 claim-level 查新时，这些编号是检索起点，不是终点。

条目的排序看约束强度，不按编号。排在前面的是已经把某个想法封死的来源，越往后越接近「只能当研究变量」或「只是工程选型」。
每条都带一个 `约束类型`，便于快速判断这条来源到底砍掉了什么。

## 约束类型速查

| 编号 | 一句话 | 约束类型 |
|---|---|---|
| SRC-04 | 3D 打印串联 snap + 可锁/可释放支撑 | 直接封死 |
| PAT-01 | 1994 优先权的膜片 snap 空气喷射阀 | 直接封死 |
| PAT-02 | 2007 优先权的 air-pulsator 发射软质载荷 | 直接封死 |
| SRC-05 | snap-through 驱动脉冲泵 | 直接封死 |
| SRC-01 | 拉伸下 snapping 超材料，sequential snap | 直接封死 |
| SRC-02 | 多阶段变形超材料 | 直接封死 |
| SRC-03 | 可调能量景观下的 sequential snapping 分岔 | 直接封死，顺序可调降为研究变量 |
| PAT-06 | 弧形 MEMS 柔顺双稳态机构 | 封死子元素 |
| SRC-07 | 单体可缩放柔顺机构，含柔顺发射器 | 封死子元素 |
| SRC-06 | 软体双稳态阀 | 封死子元素 |
| SRC-08 | 磁敏双稳态软执行器，含弹射小物体 | 封死子元素 |
| PAT-03 | 气动索引式玩具火箭发射器 | 封死工程实现 |
| PAT-04 | 电机旋转软镖弹仓 | 封死工程实现 |
| PAT-05 | 软质载荷发射装置 | 封死工程实现 |

---

## 第一组｜直接封死

这一组来源已经把 MST-01 原本想主张的组合或子组合公开过。落在这里的条目，对应的想法不能再当作贡献写进文档。

### SRC-04｜3D-Printed Serial Snap-Through Architectures for Programmable Mechanical Response

- 作者：Filipe A. Santos
- 出处：Advanced Engineering Materials, Vol. 28, Issue 6, Article e202502854
- DOI：10.1002/adem.202502854
- 日期：首次发表 2026-02-04
- 约束类型：直接封死

技术要点：

1. 用 monolithic 3D 打印 von Mises truss 单元，把 2 到 4 个 bistable unit 串联，单元依次 snap-through 形成 multistage response。
2. 每个单元配 guided sliding support，支撑可以 locked 也可以 released；这直接改变 boundary stiffness，进而改变 snap-through 与 post-snap 行为。
3. 用 engagement gap 和 geometry 调节各单元开始作用的位置，论文给出 inverted-compliance 模型，落点是 programmable staged response。

对 MST-01 的约束含义：

- 「串联 snap-through + 可切换边界」这个组合不能再作为独立核心创新。D 版继续做，身份是研究变量。
- 设计文档里不要再用「可锁/可释放支撑」「engagement gap 调节」这类措辞把自己包装成首次提出；这些描述已被占用。
- 两级串联本身也已经被覆盖，因此 §14 的 A/B/C 对照要老老实实做，C 版的价值只能靠膜片耦合后的动态数据来说话。
- 要把差异收窄到微型膜片作为被动负载这一层，并且这层还只是待验证，不是既成结论。

### PAT-01｜US5628411A — Valve devices for use in sorting apparatus ejectors

- 发明人：Stewart J. Mills, Kenneth C. Henderson
- 原始受让人：Sortex Ltd
- 出处：美国专利 US5628411A
- 优先权：1994-12-01；公开：1997-05-13
- 约束类型：直接封死

技术要点：

1. 自动分选设备用的气动 ejector，有 pressurized air source 和 diaphragm valve，膜片里带 piezoelectric element。
2. 计算机/微处理器依据传感器信号选择 ejector，膜片在气压与弹性预载共同作用下做 snap-action opening/closing，输出短空气脉冲改变飞行颗粒路径。
3. 输出口位置可调，用来改变膜片工作状态。

对 MST-01 的约束含义：

- 「MCU/计算机触发膜片空气脉冲」不能作为创新点。MST-01 的 MCU 只能是状态管理器，这点和 §1.3 一致。
- 「微型快速 air ejector」也已被覆盖，不能写成新装置。
- 「可调膜片工作点」不算空白，别把它当成独有的可调性卖点。

### PAT-02｜US8590519B2 — Projectile launching devices particularly useful in toys

- 发明人：Benjamin J. Barish
- 申请：US12/314,008
- 优先权：2007-12-05；申请日：2008-12-02；授权：2013-11-26
- 约束类型：直接封死

技术要点：

1. housing + barrel + air pulsator + pump + projectile chamber + pressure-responsive valve，先把腔内压力抬高到阈值，阀再做 snap-action opening。
2. 打开的阀产生 air pulse 推动轻质软球或其他载荷，launch force 与 range 可通过压力相关结构预设。
3. 公开了 feeder 结构和多软球连续供给，载荷明确写了 soft, light-weight, spongy cellular material。

对 MST-01 的约束含义：

- 「失稳机构产生空气脉冲发射软载荷」不能当核心发明，这是 MST-01 最重要的风险先例之一。
- 软载荷选型（EVA/EPP/毛毡/纸片）属于安全与工程决策，不能包装成贡献。
- 连续供给也不必当成新东西讲，供给本身另有专利覆盖。

### SRC-05｜Snap Pump: A Snap-Through Mechanism for a Pulsatile Pump

- 作者：Kazuki Arakawa, Francesco Giorgio-Serchi, Hiromi Mochiyama
- 出处：IEEE Robotics and Automation Letters, Vol. 6, Issue 2, pp. 803–810
- DOI：10.1109/LRA.2021.3052416
- 日期：2021
- 约束类型：直接封死

技术要点：

1. 用 snap-through 快速释放弹性能，经流体腔产生 pulsatile pumping。
2. 把 snap-through 与 fluid displacement / pulsatile output 直接连起来。
3. 弹性能向流体输出转化存在损失。

对 MST-01 的约束含义：

- 「用 snap-through 产生流体/空气脉冲」不能主张为新。
- 必须保留普通膜片驱动作为对照，不能凭概念假定 snap 方案效率更高。
- 效率类表述要留有余地，不能写成 snap 一定带来增益。

### SRC-01｜Snapping Mechanical Metamaterials under Tension

- 作者：Ahmad Rafsanjani, Abdolhamid Akbarzadeh, Damiano Pasini
- 出处：Advanced Materials, Vol. 27, Issue 39, pp. 5931–5935
- DOI：10.1002/adma.201502809
- 日期：2015
- 约束类型：直接封死

技术要点：

1. 利用局部弹性不稳定单元，在拉伸下产生 snap-through。
2. 多个单元的响应可以依次发生，形成 sequential snap-through 与多阶段力学响应。
3. 单元排布决定响应的阶段结构。

对 MST-01 的约束含义：

- sequential snap-through 不是新概念，「多个单元依次跳变」不能作为核心专利主张。
- C 版「两个事件可分辨」只能作为现象记录，不能拿它当结论。
- 必须把重心移到膜片耦合后的动态量：stage interval、membrane displacement-time、low-range pressure-time。

### SRC-02｜Multi-step deformation mechanical metamaterials

- 作者：Zhiqiang Meng, Mingchao Liu, Yafei Zhang, Chang Qing Chen
- 出处：Journal of the Mechanics and Physics of Solids, Vol. 144, Article 104095
- DOI：10.1016/j.jmps.2020.104095
- 日期：2020
- 约束类型：直接封死

技术要点：

1. 用 sequential snap-through、buckling 等不同失稳过程拼出多个明显的机械阶段。
2. 几何设计可以调节不同阶段发生的位置和顺序。
3. 多阶段力—位移曲线是可设计的。

对 MST-01 的约束含义：

- 「多阶段力—位移曲线可设计」已经成熟，静态曲线只能当仿真验证手段，不能当成结果创新。
- 「两级结构会出现两个事件」不能当卖点。
- 静态多阶段与动态膜片输出要分开讲，别混着说。

### SRC-03｜Exploring sequential snapping bifurcation through a tunable energy landscape

- 主要作者：Ke Huang，合作者包括 Jiaying Zhang, Weicheng Huang, Qingyun Wang 等
- 出处：Physical Review Applied, Vol. 25, Issue 6, Article 064029
- DOI：10.1103/nxpg-92kv
- 日期：2026
- 约束类型：直接封死，顺序可调降为研究变量

技术要点：

1. 围绕多个失稳单元的能量景观与 sequential snapping 分岔展开。
2. 改变系统参数可以改变失稳路径和顺序。
3. 顺序不是固定的，受参数控制。

对 MST-01 的约束含义：

- 「调参数改变 snapping sequence」已有明确研究，不能把「MCU 改预载 → 顺序变化」声称为新发明。
- 顺序变化可以作为实验变量记录（见 §15.1 boundary state、preload state 的字段），但它只服务于研究问题，不服务于专利主张。
- §13 的 D 版目的写成 boundary state → mechanical response 映射，措辞按研究口径来。

---

## 第二组｜封死子元素

这一组来源各自砍掉一个具体子元素。单个元素不能主张，组合起来的系统是否有新意仍需未来 claim-level 检索。

### PAT-06｜CN101654216B — 弧形 MEMS 柔顺双稳态机构

- 受让人：上海交通大学
- 发明人：吴义伯、王娟、毛胜平、丁桂甫、张丛春、汪红
- 出处：中国专利 CN101654216B
- 优先权/申请：2009-09-28；授权公开：2011-05-04
- 约束类型：封死子元素

技术要点：

1. 结构包含 curved beam、flexure spring、lumped mass 与 base。
2. 呈现 bistable behavior。
3. 示例结构本身已是微米尺度的 MEMS 实现。

对 MST-01 的约束含义：

- 「把 bistable mechanism 做得很小」不是新颖点，尺寸小本身不能单独成主张。
- 小型化只能当作工程目标（§1.2），用来描述 envelope，不能写成创新。
- 若后续缩到 MEMS 量级，要另做检索，不能沿用这份台账的结论。

### SRC-07｜Monolithic scalable compliant mechanisms

- 作者：Jared R. Hunter, Bethany Parkinson, Jacob L. Sheffield, Mark B. Rober, Brian D. Jensen, Spencer P. Magleby, Nathan S. Usevitch, Larry L. Howell
- 出处：PLOS ONE, Vol. 21, Issue 1, Article e0340272
- DOI：10.1371/journal.pone.0340272
- 日期：2026-01-21
- 约束类型：封死子元素

技术要点：

1. 研究 displacement-driven compliant mechanisms 的尺度规律，特定几何相似缩放下最大机械应力可具尺度不变性。
2. 示例中明确包含 one-piece, fully compliant projectile launcher。
3. 展示多个不同尺度版本。

对 MST-01 的约束含义：

- 「单件柔顺机构 + 很小」不能作为新颖点，小型化柔顺发射器已有直接先例。
- 尺度缩放的研究思路可以借鉴（先在较大尺度验证再按相似关系缩小），但不能把其 launcher 复制过来当自己的主张。
- §29 里提到的缩放研究要单独定位成研究产出，不附带专利暗示。

### SRC-06｜A soft, bistable valve for autonomous control of soft actuators

- 主要作者：Philipp Rothemund，合作者包括 Alar Ainla, Lee Belding, Douglas J. Preston 等
- 出处：Science Robotics, Vol. 3, Issue 16, Article eaar7986
- DOI：10.1126/scirobotics.aar7986
- 日期：2018-03-21
- 约束类型：封死子元素

技术要点：

1. 用有两个稳定状态的弹性膜作机械开关来控制气流。
2. 展示 bistable membrane 参与软体气动系统的自主状态切换。
3. 落点是软执行器的自主控制。

对 MST-01 的约束含义：

- bistable membrane、pneumatic switching、soft actuator control 都不是空白领域。
- MST-01 的膜片只作为被动负载接口，不要写成具有自主切换能力的元件。
- 膜片相关的表述集中在其作为负载时的瞬态响应，别扩到气动开关。

### SRC-08｜Magneto-sensitive bistable soft actuators: Experiments, simulations, and applications

- 出处：Applied Physics Letters, Vol. 113, Issue 22, Article 221902
- DOI：10.1063/1.5062490
- 日期：2018
- 约束类型：封死子元素

技术要点：

1. 展示 magneto-sensitive bistable soft actuator。
2. 包含快速弹射小物体的 catapult 类应用。
3. 实验与仿真并行，给出应用示例。

对 MST-01 的约束含义：

- 「bistability + ejecting small objects」已有先例，不能当作新组合。
- 磁驱动路径不进入 MST-01 设计，避免引入额外变量。
- 引用时只用于说明双稳态弹射的普遍性。

---

## 第三组｜封死工程实现

这一组来源针对的是具体实现方式。它们的结论很直接：这些实现属于成熟工程，选型可以，主张不行。

### PAT-03｜US9086251B2 — Indexing pneumatic launcher for multiple toy rocket projectiles

- 发明人：Peter Cummings
- 原始受让人：KHA Concepts Ltd
- 出处：美国专利 US9086251B2
- 优先权/申请：2013-10-15；公开：2015-07-21
- 约束类型：封死工程实现

技术要点：

1. 气动玩具发射器，支持多个 projectiles。
2. 含 carousel、indexing wheel，可自动 reload / indexing。
3. 用 air pulse 触发载荷，属于 rotating drum magazine 类概念。

对 MST-01 的约束含义：

- 「气动 + rotating carousel + 自动索引」不是新颖点。
- 6–8 位旋转供给盘是工程选型，E 版不要写成亮点（§9.1 已注明）。
- 供给盘的设计重点放在可靠性与可测试性，不放在概念包装。

### PAT-04｜CA2624593A1 — Toy soft dart launcher

- 发明人：Kenlip Ong, Kok Fai Tam, Tak To Lee
- 当前受让人记录：Mattel Inc
- 出处：加拿大专利 CA2624593A1
- 优先权：2005-09-30；申请日：2006-09-30；公开：2007-04-12
- 约束类型：封死工程实现

技术要点：

1. 权利要求包含 soft dart magazine，弹仓绕 central axis 旋转，多个 bore 圆周分布。
2. 含 motor 与 magazine rotator，做 cyclic uniform angular indexing。
3. 同一套驱动协调 piston 与 magazine rotation。

对 MST-01 的约束含义：

- 「电机自动旋转软载荷弹仓」已是明确 prior art。
- E 版旋转盘是工程实现，不承担专利亮点。
- 旋转索引机构的描述按已有成熟机构来写，不给自己加新意。

### PAT-05｜US8640683B2 — Soft-projectile launching device

- 出处：美国专利 US8640683B2
- 家族优先权：2010-05-10；授权：2014-02-04
- 约束类型：封死工程实现

技术要点：

1. 公开软质载荷系统：soft projectiles、magazine、holder。
2. 驱动方式涵盖 spring / air pressure / other suitable firing mechanisms，弹仓有 mating 结构。
3. 载荷之一是 super absorbent polymer soft projectile。

对 MST-01 的约束含义：

- 「软弹 + magazine + 多种机械/空气驱动」是高度成熟领域。
- 选 EVA/EPP 是安全与工程决策，不能写成独有设计。
- 供给与载荷部分按工程实现对待，不进入创新叙事。

---

## 按元素归类

把上面 14 条收敛到元素层面，方便和 §25.1、§27 对照。

### 彻底不能主张

- bistable / snap-through 机制本身（PAT-06、SRC-01）
- sequential / serial / programmable multistage snap（SRC-01、SRC-02、SRC-03、SRC-04）
- locked/released 边界调节 snap（SRC-04）
- snap → fluid/air pulse（SRC-05）
- bistable pneumatic valve / bistable membrane（SRC-06）
- snap diaphragm → air pulse，含可调工作点（PAT-01）
- snap air pulse → soft payload（PAT-02）
- bistability + 弹射小物体（SRC-08）
- 小型/单体柔顺发射器（SRC-07）
- 旋转索引供给仓（PAT-03）
- 电机旋转软载荷弹仓（PAT-04）
- 软弹 + magazine 供给（PAT-05）

### 只能作为研究变量

这些在离线资料里没有找到完全同构的单一来源，但邻近证据很多，不能直接主张，只能通过实验建立映射。

- 多级失稳与被动膜片负载的特定瞬态耦合规律
- 小型系统中不同边界状态对输出脉冲时间历程的可重复映射
- 具体结构细节形成的新组合

这三条对应 §27 矩阵最后三行，展开见 `CLAIM_MATRIX.md`。它们决定了 A/B/C/D 实验到底要测什么。
