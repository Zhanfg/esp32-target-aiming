#pragma once
/**
 * io_expander.h
 * MCP23017 的 I2C0 驱动。片内 GPIO 已在二期分配完，慢速离散 I/O 全走这片扩展器。
 * 位编号 0-7 = GPA0-GPA7，8-15 = GPB0-GPB7，具体用途见 config.h 的 EXP_*_BIT。
 *
 * 读失败一律返回 0 并置健康标志为假；信号极性（哪些高为断言、哪些低为断言）
 * 由 safety_gate 按信号定义判定。开路、断线、总线故障都落到禁止，不允许因为读不到
 * 就默认机构已复位。
 */

#include <cstdint>

bool ioExpanderInit(uint8_t addr);

// 读单个输入位；读失败返回 0（安全侧）。
uint8_t ioExpanderReadPin(uint8_t n);

// 写单个输出位（0/1）。写失败只记录健康标志，不清除已写值。
void ioExpanderWritePin(uint8_t n, uint8_t level);

// 配置某位为输入（true）或输出（false）。内部上拉不启用，上拉/下拉由外部电阻决定。
void ioExpanderSetPinMode(uint8_t n, bool input);

// 读全部 16 位；读失败返回 0。
uint16_t ioExpanderReadAll();

// 只保留输入位的快照，输出位清零；读失败返回 0。
uint16_t ioExpanderReadInputsSnapshot();

// 最近一次 I2C 事务是否成功。
bool ioExpanderHealthy();
