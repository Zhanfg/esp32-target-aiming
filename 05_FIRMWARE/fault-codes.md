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

## 硬件侧的锁存

故障锁存器由 SR 锁存或自保持继电器实现，复位条件接
`PAN_HOME & TILT_HOME & MAG_HOME`，机构没有真回零位就复位不了（架构文档 §6.5 H4）。
软件侧的 `faultClear()` 只是请求，最终能否复位取决于这三个开关的物理与。

## 与遥测的对应

- `fault_code`：本表码值。
- `mechanism_recovered`：`MECH_RECOVERED` 当前电平。
- `magazine_index_ok`：`MAG_INDEX_OK` 当前电平。
