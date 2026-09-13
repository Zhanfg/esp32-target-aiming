# MST-01 固件状态机

本文从实际代码抽出，不是设计文档的复述。事实来源：

- `src/aim_types.h`：`CycleState`、`AimState`、`FaultCode`、`FaultSeverity`、`FireMode`
- `src/main.cpp`：`advanceCycle()`、`cycleGuardAllows()`、`changeCycle()`、`setAim()`、
  `updateAimQuality()`、`enterFault()`、`faultClear()`
- 设计意图参照 `00_MASTER_SPEC/统一架构方案.md` 第 6 节。两者不一致时以代码为准，本文标出差异。

调度入口在 `loop()`：每拍先 `calibShellPoll()`，再按 `CONTROL_LOOP_HZ` 节拍采样互锁、
取帧，然后（未锁存故障时）跑 `advanceCycle()` 与 `updateTracking()`，下发舵机、发遥测、
跑看门狗。`setup()` 完成后进入 `SAFE`。

---

## 1. 双区域模型

| 区域 | 枚举 | 语义 | 取值 |
|---|---|---|---|
| A | `CycleState` | 发射周期，谁在储能、谁在释放 | 0..10，共 11 个 |
| B | `AimState` | 指向质量，有没有看到目标、误差是否在死区 | 0..4、6，整数 5 留空洞 |

两件事天然并发：`PRELOAD` 期间可以仍处于 `TRACKING`，`READY` 期间也可以处于 `SEARCHING`。
把它们压进一个枚举会得到 11×7 的笛卡尔空间，其中绝大多数组合非法（例如「正在 `RELEASE`
但没锁定目标」「`CALIB_MODE` 同时 `ARMED`」）。用扁平枚举，就是用「某个值不存在」表达非法
组合；只要有一处漏判，系统就会走进不可能状态。发射链是安全相关代码，靠人工维护一张合法性
矩阵恰恰是最容易出错的地方。

因此拆成两个区域，各自有独立的转移规则，跨区域只用一张守卫矩阵约束。代码里唯一的跨区域
约束点是 `main.cpp` 的 `cycleGuardAllows(CycleState next)`：它按当前 `s_aim` 决定区域 A
能往上走到哪。守卫单向，只限制进入发射链，不强制回退（丢锁由各状态分支自己处理）。

---

## 2. 区域 A（`CycleState`）状态清单

转移全部由 `advanceCycle()` 驱动。`changeCycle()` 负责落状态：目标是自身时直接返回；
目标不是 `FAULT` 且守卫拒绝时发 `GUARD` 事件并放弃；离开 `ARMED` 去非 `RELEASE` 时解除武装
锁存；去 `READY`/`SAFE` 时断开释放链。

| 值 | 状态 | 进入条件 | 允许的下一状态 | 退出条件（代码分支） |
|---|---|---|---|---|
| 0 | `BOOT` | 上电初值 `s_cycle = BOOT` | `SAFE` | `advanceCycle` 的 `BOOT` 分支无条件 `changeCycle(SAFE,"boot")` |
| 1 | `SAFE` | 首次推进，或 `faultClear()` 清软故障后 | `HOME` | `homeStart(now)` 返回真，设 `AIM` 后 `changeCycle(HOME,"homing start")` |
| 2 | `HOME` | `SAFE` 发起归零 | `READY`、`FAULT` | `homeUpdate()`：`DONE`→`READY`；`TIMEOUT`→`FAULT(HOME_TIMEOUT)`；`CONFLICT`→`FAULT(HOME_SWITCH_CONFLICT)` |
| 3 | `READY` | 归零完成；`AIM_ALLOWED` 丢锁；`PRELOAD` 中止；`RECOVER`(Mode A) 复位；`INDEX` 到位 | `AIM_ALLOWED`，及守卫允许的 `BOOT/SAFE/HOME` | `updateAimQuality()` 后 `s_aim==LOCKED` → `AIM_ALLOWED("lock acquired")` |
| 4 | `AIM_ALLOWED` | `READY` 且已 `LOCKED` | `READY`、`PRELOAD` | `s_aim!=LOCKED` → `READY("lock lost")`；`safetyCanEnterPreload()` 真 → `preloadSet(true)`、`PRELOAD("interlock1 ok")` |
| 5 | `PRELOAD` | `AIM_ALLOWED` 且互锁 1 通过 | `ARMED`、`READY` | 丢锁且超过 `PRELOAD_ABORT_MS` → `stopPreload()`、`READY`；预载满 `PRELOAD_TIME_MS` → `s_preload_state=2`、`safetyLatchArmed(true)`、`ARMED` |
| 6 | `ARMED` | `PRELOAD` 预载完成 | `RELEASE`、`FAULT` | `LOCKED` 且停留 ≥`ARMED_MIN_DWELL_MS` 且（自动释放或 `EXTERNAL_ALLOW` 上升沿）：`safetyCanRelease()` 假 → `FAULT(RELEASE_GATE_SELFTEST)`；真 → 拉高 `RELEASE_CMD_PIN`、`RELEASE` |
| 7 | `RELEASE` | `ARMED` 许可成立 | `RECOVER` | 脉冲到达 `RELEASE_PULSE_MS` → `releaseChainOff()`、`safetyLatchArmed(false)`、`RECOVER` |
| 8 | `RECOVER` | `RELEASE` 脉冲结束 | `INDEX`(Mode B)、`READY`(Mode A)、`FAULT` | `safetyMechRecovered()` 真：Mode B 发索引→`INDEX`，Mode A→`READY`；超过 `RECOVER_TIMEOUT_MS` → `FAULT(MECH_STUCK)` |
| 9 | `INDEX` | `RECOVER`(Mode B) | `READY`、`FAULT` | `safetyMagIndexOk()` 真 → 工位前进、`cycle_count++`、`READY`；超 `INDEX_TIMEOUT_MS` 重试，`s_index_attempts>=INDEX_MAX_RETRY` → `s_mode=AIR_ONLY`、`FAULT(INDEX_FAIL)` |
| 10 | `FAULT` | `enterFault()` 任意时刻 | `SAFE`（仅 `faultClear()`） | `advanceCycle` 的 `FAULT` 分支为空，无自动退出 |

说明：

- `RELEASE` 不可逆，`changeCycle` 里 `ARMED → RELEASE` 之外没有其它出口。
- `RECOVER → INDEX` 只有 `s_mode == FireMode::SOFT_PAYLOAD` 才走；`AIR_ONLY` 直接回 `READY`。
- `INDEX` 连续失败会把 `s_mode` 强制成 `AIR_ONLY`（STOP-04 落地），此后不再进 `INDEX`。
- 守卫特例：`cycleGuardAllows()` 开头判断当前区域 A 是否落在 `[PRELOAD, INDEX]`，是则直接
  放行。即已进入发射链后，链内前进不再要求持续 `LOCKED`（对应 §6.2），锁定丢失交给
  `PRELOAD` 的中止窗口和 `ARMED` 的释放前复查处理。

---

## 3. 区域 B（`AimState`）状态清单

区域 B 由 `updateAimQuality()` 评估，只在 `READY`、`AIM_ALLOWED`、`PRELOAD`、`ARMED`
四个状态的每个节拍被调用；`HOME` 分支用 `setAim()` 直接置位；`CALIB_MODE` 期间
`updateAimQuality()` 与 `updateTracking()` 都在函数开头 `return`，状态冻结。

| 值 | 状态 | 进入条件 | 允许的下一状态 | 退出条件 |
|---|---|---|---|---|
| 0 | `IDLE` | 上电初值 `s_aim = IDLE` | 无（代码里不再回到 `IDLE`） | 无 `setAim(IDLE)` 调用点 |
| 1 | `CALIBRATING` | `s_cal_ok==false`；归零开始/完成时标定无效；退出 `CALIB_MODE` 且仍无效 | `SEARCHING`、`CALIB_MODE` | 观测新鲜且标定有效后进入跟踪评估 |
| 2 | `SEARCHING` | `!obs_fresh`（观测缺失或超 `OBS_TIMEOUT_MS`）；归零后标定有效；退出 `CALIB_MODE` | `TRACKING`、`LOCKED`、`CALIBRATING` | 有新鲜观测且两轴进入死区评估 |
| 3 | `TRACKING` | 观测新鲜但未进死区；或进死区但锁定窗口未满足（`settling`） | `LOCKED`、`SEARCHING`、`CALIBRATING` | `servoAxisInDeadband()` 且窗口与帧数同时满足 |
| 4 | `LOCKED` | 两轴在死区、`now-s_lock_start_ms >= LOCK_WINDOW_MS`、视觉帧增量 `>= LOCK_MIN_VISION_FRAMES` | `TRACKING`、`SEARCHING`、`CALIBRATING`、`CALIB_MODE` | 任一条件不再成立 |
| 5 | （空洞） | 第一期 `FAULT` 已迁入区域 A 的 `CycleState::FAULT`，整数 5 保留不重排 | （无） | （无） |
| 6 | `CALIB_MODE` | `shellSetCalibrationMode(true)`，要求未锁存故障且当前 `s_cycle==READY` | `SEARCHING`、`CALIBRATING` | `setCalibrationMode(false)`，按标定有效性回 `SEARCHING`/`CALIBRATING` |

`IDLE` 只作为初值存在。上电后第一次区域 A 推进（`BOOT→SAFE`）之前 `s_aim` 一直是 `IDLE`；
`SAFE→HOME` 或 `HOME→READY` 会用 `setAim()` 把它改写为 `SEARCHING` 或 `CALIBRATING`，
之后代码里没有回到 `IDLE` 的路径。

---

## 4. 跨区域守卫矩阵

判定函数：`main.cpp` 的 `static bool cycleGuardAllows(CycleState next)`。调用点只有
`changeCycle()` 一处：

```
if (next != CycleState::FAULT && !cycleGuardAllows(next)) {
    telemetryEmitEvent("GUARD", why ? why : "blocked");
    return;   // 转移被拒
}
```

`FAULT` 不受守卫约束；目标等于当前状态时 `changeCycle` 直接返回，不触发判定。

判定逻辑，逐条反推：

1. 若当前 `s_cycle` 落在 `[PRELOAD, INDEX]`，返回 `true`（链内放行，见 §2 守卫特例）。
2. 否则看 `s_aim`：
   - `LOCKED`：返回 `true`。
   - `IDLE` / `SEARCHING` / `TRACKING` / `CALIBRATING` / `CALIB_MODE`：
     返回 `(uint8_t)next <= (uint8_t)CycleState::READY`。
   - 其余（当前枚举里没有别的值）：返回 `false`。

由此得到矩阵。下表「被禁止的区域 A 转移」指从非链内状态出发、且 `next != FAULT` 时会被
`cycleGuardAllows` 拒绝的边。

| 区域 B | 允许的最高区域 A | 被禁止的区域 A 转移 |
|---|---|---|
| `LOCKED` | 全链 | 无（`BOOT/SAFE/HOME/READY/AIM_ALLOWED/PRELOAD/ARMED/RELEASE/RECOVER/INDEX` 全放行） |
| `IDLE` | `READY` | `→ AIM_ALLOWED`、`→ PRELOAD`、`→ ARMED`、`→ RELEASE`、`→ RECOVER`、`→ INDEX` |
| `SEARCHING` | `READY` | 同上 |
| `TRACKING` | `READY` | 同上 |
| `CALIBRATING` | `READY` | 同上 |
| `CALIB_MODE` | `READY` | 同上 |

补充：

- `BOOT/SAFE/HOME/READY` 的枚举值都 `<= READY`，所以在任何区域 B 取值下都可进入。
- 守卫是映射到「允许的最高目标」而不是一张显式边表。新增区域 A 状态时，若其枚举值排在
  `READY` 之后，默认会被所有非 `LOCKED` 的区域 B 拒绝，必须显式评估是否放行。
- 守卫不负责回退。`READY`、`AIM_ALLOWED`、`PRELOAD` 各自在分支里处理丢锁，例如
  `AIM_ALLOWED` 丢锁回 `READY`、`PRELOAD` 超中止窗口回 `READY`。

### 4.1 当前组合的合法性核对

守卫矩阵管的是迁移，`changeCycle()` 在每次转移前调用一次，判断「目标状态是否准进」。
它管不到「已经处于的组合是否是漏进来的」。`main.cpp` 的 `cycleStateLegal()` 补这一层，
每个周期核对一次当前 `s_cycle` 与 `s_aim` 的组合：

```
static bool cycleStateLegal() {
    if (s_cycle == CycleState::FAULT) return true;
    return cycleGuardAllows(s_cycle);
}
```

它把当前状态当作 `next` 喂回 `cycleGuardAllows()`，复用同一张矩阵，没有复制阈值常量，
也没有改动原函数的语义。`FAULT` 是任意时刻都合法的锁存态，单独放行。等价的直白表述是：
非 `LOCKED` 的区域 B 下，当前区域 A 若落在 `[PRELOAD, INDEX]` 之外且枚举值大于 `READY`，
即判非法。当前枚举里能被这条规则抓到的是 `AIM_ALLOWED` 与非 `LOCKED` 的区域 B 同时出现，
也就是「没锁定却站在允许发射的门上」。

核对点在 `loop()` 里 `advanceCycle()` 之前，看的是上一拍收敛后的稳定组合，不会碰到
单拍内的瞬态（例如 `AIM_ALLOWED` 分支在丢锁的同一拍就回 `READY`）。判非法后先发
`STATE_ILLEGAL` 事件，内容为当时的 `cycle`、`aim` 整数值，再 `enterFault()` 进硬故障，
不做自动恢复。

---

## 5. 故障体系

`FaultCode`（`aim_types.h`）与软硬分级由 `faultSeverity()` 判定。`fault_code` 遥测字段
直接输出枚举整数值，明细另见 `05_FIRMWARE/fault-codes.md`。

| 码 | 名称 | 分级 | 触发条件（代码位置） | 清除路径 |
|---|---|---|---|---|
| 0 | `NONE` | 无 | 无故障 | 不适用 |
| 1 | `PERIPH_INIT` | 硬 | `setup()`：`ioExpanderInit()` 或 `servoAxisInit()` 失败 | 软件清除无效；断电/人工排查后重新上电 |
| 2 | `HOME_TIMEOUT` | 软 | `homeUpdate()` 返回 `TIMEOUT`（3 秒内未触发或触发后丢失） | `faultClear()` → `SAFE` → `HOME` |
| 3 | `HOME_SWITCH_CONFLICT` | 硬 | `homeUpdate()` 返回 `CONFLICT`（确认窗口内回跳，或退回后仍闭合） | 软件清除无效；断电检查开关装配与极性 |
| 4 | `RELEASE_GATE_SELFTEST` | 硬 | `setup()` 释放门控自检失败；或 `ARMED` 释放前 `safetyCanRelease()` 复查不通过 | 软件清除无效；断电检查与门树与输入极性 |
| 5 | `OBS_LINK` | 软 | 摄像头初始化失败；或 `s_capture_fail_streak >= OBS_LINK_FAIL_N` | `faultClear()` → `SAFE` → `HOME` |
| 6 | `INDEX_FAIL` | 软 | `INDEX` 重试达 `INDEX_MAX_RETRY`（STOP-04），同时 `s_mode` 置 `AIR_ONLY` | `faultClear()` → `SAFE` → `HOME`，并排查卡料 |
| 7 | `WATCHDOG` | 软 | `s_overrun_streak >= WATCHDOG_STREAK_N`，或单拍 `loop_us/1000 > WATCHDOG_HARD_OVERRUN_MS` | `faultClear()` → `SAFE` → `HOME` |
| 8 | `MECH_STUCK` | 硬 | `RECOVER` 超 `RECOVER_TIMEOUT_MS` 仍 `!safetyMechRecovered()` | 软件清除无效；断电人工处理 |
| 9 | `ESTOP` | 软 | 串口 `ESTOP`（`shellEmergencyStop()`） | `faultClear()` → `SAFE` → `HOME` |
| 10 | `MEMBRANE_RUPTURE` | 硬 | 膜片破裂或气路泄漏，脉冲通道完整性无法由软件确认。触发点待硬件或后续实现 | 软件清除无效；断电更换膜片并做气密检查后重新上电 |
| 11 | `DOUBLE_FEED` | 软 | 供给盘检出双片（检出后禁 Mode B，保留 Mode A）。触发点待硬件或后续实现，依赖供给盘位置传感器与索引逻辑 | `faultClear()` → `SAFE` → `HOME`，并清除叠片；储能态机构顶死时按硬故障处理 |
| 12 | `FIRE_INHIBIT_SHORT` | 硬 | `FIRE_INHIBIT` 短路到许可侧，安全否决失效。触发点待硬件或后续实现，依赖双通道冗余输入 | 软件清除无效；断电排查双通道输入与接线后重新上电 |
| 13 | `I2C_BUS_HANG` | 硬 | `io_expander.cpp` 维护运行期连续失败计数，达到 `I2C_BUS_HANG_FAIL_N` 后 `main.cpp` 周期里 `ioExpanderBusSuspectedHang()` 为真，`enterFault()` 置位 | 软件清除无效；断电重启总线与扩展器后重新上电 |
| 14 | `STATE_ILLEGAL` | 硬 | `main.cpp` 周期里 `cycleStateLegal()` 判当前 cycle/aim 组合落在 `cycleGuardAllows()` 之外，`enterFault()` 置位并发事件记录 A、B 取值 | 软件清除无效；断电检查守卫矩阵后重新上电 |

码 10..14 的枚举值与软硬分级已在 `aim_types.h` 就位。码 13、14 的触发点已接：
`I2C_BUS_HANG` 的运行期连续失败计数在 `io_expander.cpp`（初始化事务不计入，上电失败仍走
`PERIPH_INIT`），`STATE_ILLEGAL` 的当前组合核对在 `main.cpp` 的 `cycleStateLegal()`。码 10、11、
12 的触发点待硬件或后续实现，`main.cpp` 里没有对应的 `enterFault()` 调用点，逐码的触发与清除
明细见 `fault-codes.md`。

`enterFault(code, why)` 的固定动作，先是锁存保护，再是安全动作：

1. `if (s_fault_latched) return;` 保留首个故障码。
2. 记 `s_fault = code`、`s_fault_latched = true`。
3. `releaseChainOff()`、`safetyLatchArmed(false)`、`homeAbort()`、`stopPreload()`、
   `servoAxisFaultSafe()`。不立即释能，因为开环 tilt 断电会因重力下坠。
4. `s_cycle = CycleState::FAULT;` 直接写，不走 `changeCycle()`，保证守卫拦不住进 `FAULT`。
5. 记录 `operator_note` 并发 `FAULT` 事件。

`faultClear()` 只接受软故障：无锁存返回假；`faultSeverity(s_fault)==HARD` 时发
`"clear rejected: hard fault"` 返回假；否则清锁存与故障码、发 `"fault cleared"`、调用
`changeCycle(CycleState::SAFE, "fault cleared -> SAFE")`。清除后必须真实走完
`SAFE → HOME`，`HOME` 里参考开关全中才可能进 `READY`。

---

## 6. 硬转移与软转移

硬转移由物理通道决定，固件即使卡死或输出错误电平也完不成。软件实现只是第二层。

| 编号 | 硬约束 | 固件里的第二层 | 代码注释 |
|---|---|---|---|
| H1 | 释放能量路径导通需 `FIRE_INHIBIT` 未断言、`EXTERNAL_ALLOW` 有效、`MECH_RECOVERED`、模式 B 载荷对齐、`ARMED` 锁存、主使能有效 | `safetyCanRelease()` 复查与门树 Q1 与 `ARMED_LATCH` 的软件对应部分 | `safety_gate.h`：软件侧互锁是「硬件与门之外的第二层」；`safety_gate.cpp` `safetyCanRelease()`：`Q1 = AND(FIRE_INHIBIT_N, EXTERNAL_ALLOW, MECH_RECOVERED_N, 对齐或 Mode A)`，`Q2 = AND(Q1, ARMED_LATCH, MCU_RELEASE_CMD, MASTER_ENABLE)`，并注明「对齐位与主使能由上层状态机补，硬件与门与单稳态另在板级实现」 |
| H2 | 机构未复位时预载执行器无法被驱动 | `safetyCanEnterPreload()` 只返回 `s_mech_recovered` 作迁移判定与日志 | `safety_gate.cpp`：`MECH_RECOVERED_N` 常闭触点串在预载使能回路里，「真正的断开由……常闭触点」完成 |
| H3 | 模式 B 未对齐时释放无法完成 | `safetyCanIndexPayload()`（`s_mech_recovered && s_mag_index_ok`） | 对齐开关接入 H1 与门，`safety_gate.h` Interlock 2 |
| H4 | 故障/急停锁存，未重新归零不能复位 | `s_fault_latched` 软件锁存 + `faultClear()` | `main.cpp` 安全策略：「FAULT 锁存，唯一清除路径是 SAFE → HOME」 |
| H5 | 超出机械限位 | `servoAxisSetTargetDeg()` / `servo.cpp clampAxisDeg()` 的软件夹紧 | `servo.cpp`：「无反馈舵机指令越界会撞止挡，必须在最底层拦住」 |
| H6 | 主电源急停 | 无固件对应 | 常闭蘑菇头直接串执行器供电，不经 MCU |

软转移是纯软件判定：`LOCKED` 判定（`updateAimQuality()`）、观测新鲜度与链路失败计数、
许可授予（`AIM_ALLOWED`）、预载到位（`PRELOAD_TIME_MS`）、索引完成与重试、冷却与超时、
看门狗、故障码分级（`faultSeverity()`）、模式选择（`s_mode`）、跨区域守卫矩阵。软转移可以
当第一道闸，但不是唯一一道。

安全策略的原文在两处：

- `main.cpp` 文件头：「释放链只在全部互锁满足时导通，且真正的物理门控由外部与门保证
  （架构文档 §6.6）」。
- `main.cpp` `ARMED` 分支：「释放前最后一瞬重新确认 LOCKED，并复查全部互锁（§6.5 H1 的
  软件侧）」。

`setup()` 期间释放链保持在禁止侧：先把 `RELEASE_CMD_PIN` 置低、`releaseChainOff()`，
再初始化外设；`safetyReleaseGateSelfTest()` 确认锁存未置位时 `safetyCanRelease()` 必为假。

---

## 7. `FAULT` 的唯一出口

事实链：

1. `s_fault_latched` 一旦置位，`enterFault()` 直接返回，保留首个故障码，不会被后续故障覆盖。
2. `loop()` 里 `if (!s_fault_latched)` 才调用 `advanceCycle()`，锁存期间区域 A 的推进逻辑
   根本不执行。
3. `advanceCycle()` 的 `case CycleState::FAULT:` 是空分支，没有自发转移。
4. 唯一能把 `s_cycle` 写离 `FAULT` 的函数是 `faultClear()`，它无论如何都只
   `changeCycle(CycleState::SAFE, ...)`，从不写 `READY`。
5. `FAULT` 的入口走 `enterFault()` 里的 `s_cycle = CycleState::FAULT`，绕过 `changeCycle()`
   的守卫，所以任何地方都能安全地进入 `FAULT`。
6. `shellSetCalibrationMode(true)` 在 `s_fault_latched` 时直接返回假，故障期间不能进标定。

因此 `FAULT → READY` 这条边不是被守卫挡住的，而是没有任何调用点产生它。需要说明的细节：
`cycleGuardAllows()` 对 `LOCKED` 返回真，若真有人从 `FAULT` 调
`changeCycle(CycleState::READY, ...)` 且此刻 `s_aim==LOCKED`，守卫其实会放行。保证来自
「只有 `faultClear()` 写出 `FAULT`，且它只写 `SAFE`」这一构造事实，不来自守卫。加新转移时
不要在 `FAULT` 分支里直接写 `READY`。

软件侧的锁存是 `s_fault_latched`。硬件侧的锁存器（SR 锁存或自保持继电器，复位接
`PAN_HOME & TILT_HOME & MAG_HOME`）见 `统一架构方案.md` §6.5 H4，属板级实现，
本固件只做 `faultClear()` 这个请求端。`faultClear()` 成功不代表硬件锁存已复位。

---

## 8. 状态与遥测字段的对应

枚举都是 `uint8_t`，底下就是整数，PC 端脚本按整数解析。

- 标定外壳的 `STATUS` 命令（`calib_shell.cpp`）输出
  `ST,cycle,<整数>,<名字>`、`ST,state,<整数>,<名字>`、`ST,fault_code,<整数>`、
  `ST,membrane_id,<整数>`、`ST,payload_id,<整数>`。
  名字来自 `cycleName()` 与 `stateName()` 的 switch。
- 遥测数据行 `MST,...`：`fault_code = (uint16)s_fault`；`mode = (uint8_t)s_mode`
  （`FireMode`：0=空气，1=软载荷）；`preload_state`（0=空闲、1=预载中、2=已武装）；
  `membrane_id`、`payload_id` 是批次追溯编号，由串口 `SET MEMBRANE <id>`、`SET PAYLOAD <id>`
  写入 NVS（命名空间 `aim_batch`，键 `membrane`、`payload`），上电读回，uint16 取值 0 .. 65535，
  0 表示未设置；`cycle_count` 在 `INDEX` 到位时自增；`magazine_position` 为当前工位。

取值速查：

```
CycleState: BOOT=0 SAFE=1 HOME=2 READY=3 AIM_ALLOWED=4 PRELOAD=5 ARMED=6
            RELEASE=7 RECOVER=8 INDEX=9 FAULT=10
AimState:   IDLE=0 CALIBRATING=1 SEARCHING=2 TRACKING=3 LOCKED=4 (5 保留空洞) CALIB_MODE=6
FaultCode:  NONE=0 PERIPH_INIT=1 HOME_TIMEOUT=2 HOME_SWITCH_CONFLICT=3
            RELEASE_GATE_SELFTEST=4 OBS_LINK=5 INDEX_FAIL=6 WATCHDOG=7
            MECH_STUCK=8 ESTOP=9 MEMBRANE_RUPTURE=10 DOUBLE_FEED=11
            FIRE_INHIBIT_SHORT=12 I2C_BUS_HANG=13 STATE_ILLEGAL=14
```

加新状态的纪律：

1. 只能追加在枚举末尾，不得插队重排。`AimState` 的整数 5 是第一期遗留空洞，不得复用。
2. 同步 `calib_shell.cpp` 里 `cycleName()` / `stateName()` 的 switch，否则 `STATUS` 显示
   `UNKNOWN`，但整数仍照常输出。
3. `cycleGuardAllows()` 是「允许的最高目标」映射。新区域 A 状态若排在 `READY` 之后，
   默认被所有非 `LOCKED` 的区域 B 拒绝；新区域 B 状态落入 switch 的 `default`，会让区域 A
   最多只到 `READY`。两种情况都要显式决定是否放行。
4. 遥测字段（`fault_code`、`mode`、`preload_state`）与 PC 解析脚本要一起更新。
   故障码同样只追加，见 `fault-codes.md`。
