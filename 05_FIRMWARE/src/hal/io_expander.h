#pragma once
// MCP23017 的 I2C0 驱动。位编号 0-7 = GPA0-GPA7，8-15 = GPB0-GPB7，用途见 config.h 的 EXP_*_BIT。
// 读失败一律返回 0（安全侧）；极性判定在 safety_gate，失败即禁止的约定见 io_expander.cpp。

#include <cstdint>

bool ioExpanderInit(uint8_t addr);

uint8_t ioExpanderReadPin(uint8_t n);

// 写失败只记录健康标志，不清除已写值。
void ioExpanderWritePin(uint8_t n, uint8_t level);

void ioExpanderSetPinMode(uint8_t n, bool input);

uint16_t ioExpanderReadAll();

uint16_t ioExpanderReadInputsSnapshot();

bool ioExpanderHealthy();

// 运行期连续 I2C 事务失败达到 I2C_BUS_HANG_FAIL_N 时置真，提示总线可能卡死。
// 初始化阶段的事务不计入；上电失败仍由 PERIPH_INIT 表示。
bool ioExpanderBusSuspectedHang();
