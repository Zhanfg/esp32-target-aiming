/**
 * io_expander.cpp
 * MCP23017 的 I2C0 驱动。寄存器用 BANK=0 布局：A 组在低位寄存器，B 组在高位。
 *
 * 安全侧约定：
 *   输入位读不到 → 0（未断言）。MECH_RECOVERED / MAG_INDEX_OK / *_HOME 断言为高，
 *   0 表示"未复位/未对齐/未触发"，是禁止侧；FIRE_INHIBIT / EXTERNAL_ALLOW 有效为高，
 *   0 表示"禁止/未许可"。任何 I2C 失败都让这两类信号落到禁止。
 *   输出位上电默认全 0，即 BOUNDARY/PRELOAD/MAG_INDEX 全断、STATUS_LED 灭。
 */

#include "io_expander.h"

#include <Arduino.h>
#include <driver/i2c.h>

#include "config.h"
#include "freertos/FreeRTOS.h"

namespace {

constexpr uint8_t kRegIodirA = 0x00;
constexpr uint8_t kRegIodirB = 0x01;
constexpr uint8_t kRegIocon  = 0x0A;
constexpr uint8_t kRegGppuA  = 0x0C;
constexpr uint8_t kRegGppuB  = 0x0D;
constexpr uint8_t kRegGpioA  = 0x12;
constexpr uint8_t kRegGpioB  = 0x13;
constexpr uint8_t kRegOlatA  = 0x14;
constexpr uint8_t kRegOlatB  = 0x15;

constexpr TickType_t kI2cTimeout = pdMS_TO_TICKS(20);

bool s_ready = false;
bool s_healthy = false;
uint8_t s_addr = 0;
uint16_t s_shadow_out = 0;

bool writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, s_addr, buf, sizeof(buf), kI2cTimeout);
    s_healthy = (err == ESP_OK);
    return s_healthy;
}

bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
    esp_err_t err = i2c_master_write_read_device(I2C_NUM_0, s_addr, &reg, 1, buf, len, kI2cTimeout);
    s_healthy = (err == ESP_OK);
    return s_healthy;
}

void writeWord(uint8_t reg_lo, uint8_t reg_hi, uint16_t value) {
    writeReg(reg_lo, (uint8_t)(value & 0xFF));
    writeReg(reg_hi, (uint8_t)(value >> 8));
}

} // namespace

bool ioExpanderInit(uint8_t addr) {
    s_addr = addr;

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C0_SDA_PIN;
    conf.scl_io_num = I2C0_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C0_FREQ_HZ;
    conf.clk_flags = 0;

    if (i2c_param_config(I2C_NUM_0, &conf) != ESP_OK) {
        Serial.println("[expander] I2C0 参数配置失败");
        return false;
    }
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        Serial.println("[expander] I2C0 驱动安装失败");
        return false;
    }

    // IOCON = 0：BANK=0、地址自动递增，与上面的寄存器常量一致。
    if (!writeReg(kRegIocon, 0x00)) {
        Serial.println("[expander] MCP23017 无应答");
        return false;
    }

    const uint16_t out_mask = (uint16_t)EXP_OUTPUT_MASK;
    if (!writeReg(kRegIodirA, (uint8_t)(~out_mask & 0xFF)) ||
        !writeReg(kRegIodirB, (uint8_t)((~out_mask >> 8) & 0xFF))) {
        Serial.println("[expander] 方向寄存器写入失败");
        return false;
    }
    // 不启用 MCP23017 内部上拉。安全输入按 §6.7 要求外部加下拉（许可类主动拉高，
    // 开路即禁止）；MECH_RECOVERED 是常闭干接点对地，也要外部上拉。若这里强加上拉，
    // 会与许可信号的外部下拉对拉，破坏 fail-safe。上电前必须确认外部电阻已就位。
    writeWord(kRegGppuA, kRegGppuB, 0);

    // 输出上电默认安全侧全 0。
    s_shadow_out = 0;
    writeWord(kRegOlatA, kRegOlatB, s_shadow_out);

    s_ready = true;
    return true;
}

uint8_t ioExpanderReadPin(uint8_t n) {
    if (n > 15) return 0;
    const uint16_t all = ioExpanderReadAll();
    return (uint8_t)((all >> n) & 0x01u);
}

void ioExpanderWritePin(uint8_t n, uint8_t level) {
    if (!s_ready || n > 15) return;
    const uint16_t bit = (uint16_t)(1u << n);
    if (level) {
        s_shadow_out |= bit;
    } else {
        s_shadow_out &= (uint16_t)~bit;
    }
    writeWord(kRegOlatA, kRegOlatB, s_shadow_out);
}

void ioExpanderSetPinMode(uint8_t n, bool input) {
    if (!s_ready || n > 15) return;

    uint8_t iodir = 0;
    if (!readRegs(n < 8 ? kRegIodirA : kRegIodirB, &iodir, 1)) return;
    const uint8_t bit = (uint8_t)(1u << (n % 8));
    if (input) {
        iodir |= bit;
    } else {
        iodir &= (uint8_t)~bit;
    }
    writeReg(n < 8 ? kRegIodirA : kRegIodirB, iodir);
    // 内部上拉始终保持关闭，上拉/下拉由外部电阻按信号极性决定。
}

uint16_t ioExpanderReadAll() {
    if (!s_ready) return 0;
    uint8_t buf[2] = { 0, 0 };
    if (!readRegs(kRegGpioA, buf, sizeof(buf))) {
        return 0; // 读失败按全 0 安全侧解释
    }
    return (uint16_t)(((uint16_t)buf[0]) | ((uint16_t)buf[1] << 8));
}

uint16_t ioExpanderReadInputsSnapshot() {
    const uint16_t in_mask = (uint16_t)((uint16_t)EXP_OUTPUT_MASK ^ 0xFFFFu);
    return (uint16_t)(ioExpanderReadAll() & in_mask);
}

bool ioExpanderHealthy() { return s_ready && s_healthy; }
