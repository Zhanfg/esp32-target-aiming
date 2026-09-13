/**
 * safety_gate.cpp
 * safety_gate.h 的实现。只依赖 io_expander 与 config.h 的位定义。
 */

#include "safety_gate.h"

#include "config.h"
#include "io_expander.h"

namespace {

bool s_fire_allow = false;      // FIRE_INHIBIT 原始电平，高=允许
bool s_external_allow = false;  // EXTERNAL_ALLOW，高=许可
bool s_mech_recovered = false;
bool s_mag_index_ok = false;
bool s_pan_home = false;
bool s_tilt_home = false;
bool s_mag_home = false;
bool s_armed_latched = false;

bool readBit(uint16_t snapshot, uint8_t bit) { return ((snapshot >> bit) & 0x01u) != 0; }

void forceSafe() {
    s_fire_allow = false;
    s_external_allow = false;
    s_mech_recovered = false;
    s_mag_index_ok = false;
    s_pan_home = false;
    s_tilt_home = false;
    s_mag_home = false;
}

} // namespace

void safetyGateInit() {
    // 上电先落到禁止侧，不信任任何未采样的输入。
    forceSafe();
    s_armed_latched = false;
}

void safetyGateUpdate(uint32_t now_ms) {
    (void)now_ms;
    // 一次事务读完 16 位，健康标志对应这同一次事务，避免逐位读取时口径不一致。
    const uint16_t snap = ioExpanderReadInputsSnapshot();
    if (!ioExpanderHealthy()) {
        // 总线失败：全部按禁止侧解释，不允许因为读不到就默认机构已复位。
        forceSafe();
        return;
    }

    s_fire_allow = readBit(snap, EXP_FIRE_INHIBIT_BIT);
    s_external_allow = readBit(snap, EXP_EXTERNAL_ALLOW_BIT);
    // MECH_RECOVERED 是常闭干接点对地（§6.6），外部上拉、机构复位时触点为低。
    s_mech_recovered = !readBit(snap, EXP_MECH_RECOVERED_BIT);
    s_mag_index_ok = readBit(snap, EXP_MAG_INDEX_OK_BIT);
    s_pan_home = readBit(snap, EXP_PAN_HOME_BIT);
    s_tilt_home = readBit(snap, EXP_TILT_HOME_BIT);
    s_mag_home = readBit(snap, EXP_MAG_HOME_BIT);
}

bool safetyFireInhibitAsserted() { return !s_fire_allow; }
bool safetyExternalAllow() { return s_external_allow; }
bool safetyMechRecovered() { return s_mech_recovered; }
bool safetyMagIndexOk() { return s_mag_index_ok; }
bool safetyPanHome() { return s_pan_home; }
bool safetyTiltHome() { return s_tilt_home; }
bool safetyMagHome() { return s_mag_home; }

void safetyLatchArmed(bool armed) { s_armed_latched = armed; }
bool safetyArmedLatched() { return s_armed_latched; }

bool safetyCanEnterPreload() {
    // Interlock 1（§6.6 第 1 条）：机构未复位时物理禁止预载，软件这里复查用于
    // 迁移判定与日志；真正的断开由 MECH_RECOVERED_N 常闭触点串在预载使能回路里。
    return s_mech_recovered;
}

bool safetyCanRelease() {
    // Interlock 3（§6.6 第 3 条）：FIRE_INHIBIT 断言时释放永远禁止。
    // 这里只实现与门树 Q1 与 ARMED_LATCH 的软件对应部分：
    //   Q1 = AND(FIRE_INHIBIT_N, EXTERNAL_ALLOW, MECH_RECOVERED_N, 对齐或 Mode A)
    //   Q2 = AND(Q1, ARMED_LATCH, MCU_RELEASE_CMD, MASTER_ENABLE)
    // 对齐位与主使能由上层状态机补，硬件与门与单稳态另在板级实现。
    if (!s_fire_allow) return false;
    if (!s_external_allow) return false;
    if (!s_mech_recovered) return false;
    if (!s_armed_latched) return false;
    return true;
}

bool safetyCanIndexPayload() {
    // Interlock 2（§6.6 第 2 条）：MAG_INDEX_OK 才准 Mode B。
    // 机构先复位是驱动供给盘的前提，对齐在位才允许带载荷动作。
    return s_mech_recovered && s_mag_index_ok;
}

bool safetyReleaseGateSelfTest() {
    // 扩展器读不到就无法确认真实电平，视为自检失败。
    if (!ioExpanderHealthy()) return false;
    // 未武装时释放门必须关死。若这里为真，说明缓存或锁存有误。
    if (s_armed_latched) return false;
    if (safetyCanRelease()) return false;
    return true;
}
