# MST-01 故障码表

对应规格书 §20 的 `05_FIRMWARE/fault-codes.md`，并与 `src/aim_types.h` 的
`FaultCode` 枚举一一对应。遥测 §19 的 `fault_code` 字段直接输出码值整数值。

故障体系按架构文档 §6.4 分两级：

- 软故障：允许 `FAULT → SAFE → HOME → READY`，且 `HOME` 必须真实完成、参考开关全中。
- 硬故障：保持锁存，软件清除无效，必须断电或人工处理。

`FAULT` 锁存后不自动恢复，唯一清除路径是 `SAFE → HOME`，`FAULT → READY` 这条边不存在。
清除命令只把系统送到 `SAFE`，不直接到 `READY`。

| 码值 | 名称 | 软/硬 | 触发条件 | 清除方式 |
|---|---|---|---|---|
| 0 | `NONE` | 无 | 无故障 | 不适用 |
| 1 | `PERIPH_INIT` | 硬 | MCP23017、舵机或其它外设初始化失败 | 断电排查后重新上电 |
| 2 | `HOME_TIMEOUT` | 软 | 归零 3 秒内参考开关未触发，或确认窗口内丢失触发 | 发故障清除 → `SAFE` → 重新归零 |
| 3 | `HOME_SWITCH_CONFLICT` | 硬 | 参考开关触发后退回时仍闭合，或确认窗口内抖动丢失 | 断电检查开关装配与极性 |
| 4 | `RELEASE_GATE_SELFTEST` | 硬 | 释放门控自检失败；释放前互锁复查不通过 | 断电检查与门树与输入极性 |
| 5 | `OBS_LINK` | 软 | 摄像头初始化失败，或连续取帧失败超过阈值 | 发故障清除 → `SAFE` → 重新归零 |
| 6 | `INDEX_FAIL` | 软 | 索引连续失败达到重试上限（STOP-04），同时禁 Mode B | 发故障清除 → `SAFE` → 重新归零，并排查卡料 |
| 7 | `WATCHDOG` | 软 | 控制环连续超预算，或单拍严重超时 | 发故障清除 → `SAFE` → 重新归零 |
| 8 | `MECH_STUCK` | 硬 | 触发后机构在超时窗口内未回报复位 | 断电人工处理卡死 |
| 9 | `ESTOP` | 软 | 串口 `ESTOP` 或等效急停触发 | 发故障清除 → `SAFE` → 重新归零 |
| 10 | `MEMBRANE_RUPTURE` | 硬 | 膜片破裂或气路泄漏，脉冲通道完整性无法由软件确认。触发点待硬件或后续实现 | 断电更换膜片并做气密检查后重新上电 |
| 11 | `DOUBLE_FEED` | 软 | 供给盘检出双片（检出后禁 Mode B、保留 Mode A）。触发点待硬件或后续实现，依赖供给盘位置传感器与索引逻辑 | 清除叠片后 `CLEAR` → `SAFE` → 重新归零；若发生在储能态且机构顶死，升级 `FAULT` 并按硬故障处理 |
| 12 | `FIRE_INHIBIT_SHORT` | 硬 | `FIRE_INHIBIT` 短路到许可侧，安全否决失效。触发点待硬件或后续实现，依赖双通道冗余输入 | 断电排查双通道输入与接线后重新上电 |
| 13 | `I2C_BUS_HANG` | 硬 | 已实现：运行期 I2C0 连续失败达到 `I2C_BUS_HANG_FAIL_N`（`config.h`），扩展器状态不可信。计数器在 `io_expander.cpp`，`main.cpp` 周期里查 `ioExpanderBusSuspectedHang()` 后进故障 | 断电重启总线与扩展器后重新上电 |
| 14 | `STATE_ILLEGAL` | 硬 | 已实现：`main.cpp` 周期里 `cycleStateLegal()` 判当前 cycle/aim 组合落在 `cycleGuardAllows()` 之外。进入时发 `STATE_ILLEGAL` 事件记录当时的区域 A、B 取值 | 断电检查固件守卫矩阵后重新上电 |

新增码的分级依据架构文档 §6.4 与 `02_REQUIREMENTS/safety-boundary.md` 第 3 节。
`MEMBRANE_RUPTURE`、`FIRE_INHIBIT_SHORT`、`I2C_BUS_HANG`、`STATE_ILLEGAL` 都落在软件无法确认
真实安全状态的地方：膜片破裂后脉冲通道完整性未知，安全否决短路后 fail-safe 丢失，运行期
I2C 卡死时扩展器锁存输出无法由软件物理复位，非法状态组合说明守卫矩阵已经失守。
`DOUBLE_FEED` 按 §6.8 的 STOP-04 归为运行期软故障，检出即禁 Mode B，条件成立时再升级。
`PERIPH_INIT` 仍只覆盖上电初始化失败，运行期总线故障用 `I2C_BUS_HANG` 单列。

实现状态：`I2C_BUS_HANG`(13) 与 `STATE_ILLEGAL`(14) 已有触发点，位置见上表；二者都是硬故障，
`faultSeverity()` 判 `HARD`，软件清除无效。`MEMBRANE_RUPTURE`(10)、`DOUBLE_FEED`(11)、
`FIRE_INHIBIT_SHORT`(12) 的触发点待硬件或后续实现，当前不设占位触发点，避免制造假信号。

## 硬件侧的锁存

故障锁存器由 SR 锁存或自保持继电器实现，复位条件接
`PAN_HOME & TILT_HOME & MAG_HOME`，机构没有真回零位就复位不了（架构文档 §6.5 H4）。
软件侧的 `faultClear()` 只是请求，最终能否复位取决于这三个开关的物理与。

## 与遥测的对应

- `fault_code`：本表码值。
- `mechanism_recovered`：`MECH_RECOVERED` 当前电平。
- `magazine_index_ok`：`MAG_INDEX_OK` 当前电平。
