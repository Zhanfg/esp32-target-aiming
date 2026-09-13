/**
 * main.cpp
 * 固件入口与双区域状态机。区域 A（CycleState）管发射周期，区域 B（AimState）管
 * 指向质量，跨区域用守卫矩阵约束。上电初始化后按固定节拍跑"视觉->解算->指向->遥测"。
 *
 * 安全策略：
 *   1. 释放链只在全部互锁满足时导通，且真正的物理门控由外部与门保证（架构文档 §6.6）；
 *   2. setup() 期间释放链保持在禁止侧，归零完成前不使能；
 *   3. FAULT 锁存，唯一清除路径是 SAFE → HOME，FAULT → READY 这条边不存在；
 *   4. loop() 非阻塞，calibShellPoll() 放在最前，否则 100Hz 轮询会漏字符。
 */

#include <Arduino.h>

#include <cmath>
#include <cstdio>

#include "config.h"
#include "aim_types.h"
#include "data/batch_tags.h"
#include "calibration/calibration.h"
#include "comms/calib_shell.h"
#include "comms/telemetry.h"
#include "control/home.h"
#include "control/servo_axis.h"
#include "hal/io_expander.h"
#include "hal/safety_gate.h"
#include "hal/servo.h"
#include "math/angle_utils.h"
#include "vision/vision.h"

static CycleState s_cycle = CycleState::BOOT;
static AimState s_aim = AimState::IDLE;
static FaultCode s_fault = FaultCode::NONE;
static bool s_fault_latched = false;
static FireMode s_mode = (FireMode)FIRE_MODE_DEFAULT;

static bool s_vision_ok = false;
static bool s_cal_ok = false;
static CalibrationData s_cal;

// 批次追溯编号：由操作者经 SET 命令写入并存 NVS，上电读回。0 表示未设置。
static BatchTags s_batch;

static TargetObservation s_obs;
static uint32_t s_last_capture_ms = 0;
static uint32_t s_vision_frame_count = 0;
static uint32_t s_capture_fail_streak = 0;
static uint32_t s_frame_divider = 1;
static uint32_t s_overrun_streak = 0;

static uint32_t s_cycle_enter_ms = 0;
static uint32_t s_phase_start_ms = 0;
static uint32_t s_lock_start_ms = 0;
static uint32_t s_lock_start_frames = 0;
static uint32_t s_lock_lost_ms = 0;
static uint32_t s_cycle_count = 0;
static uint8_t s_mag_position = 0;
static uint8_t s_preload_state = 0;
static uint32_t s_index_attempts = 0;
static uint32_t s_index_phase_start_ms = 0;
static uint32_t s_release_start_ms = 0;
static bool s_prev_external_allow = false;

static volatile uint32_t s_stage1_us = 0;
static volatile uint32_t s_stage2_us = 0;
static volatile uint8_t s_stage1_count = 0;
static volatile uint8_t s_stage2_count = 0;

static char s_operator_note[24] = "-";

// alpha-beta 跟踪状态（作用于相对光轴的 bearing/elevation）。
static bool s_has_track = false;
static float s_bearing_deg = 0.0f;
static float s_elevation_deg = 0.0f;
static float s_bearing_rate_dps = 0.0f;
static float s_elevation_rate_dps = 0.0f;

static void enterFault(FaultCode code, const char* why);
static void changeCycle(CycleState next, const char* why);
static void setAim(AimState next, const char* why);
static void updateAimQuality(bool obs_fresh, uint32_t now_ms);
static bool faultClear();

static void releaseChainOff() {
    ioExpanderWritePin(EXP_PRELOAD_CMD_BIT, 0);
    ioExpanderWritePin(EXP_MAG_INDEX_CMD_BIT, 0);
    digitalWrite(RELEASE_CMD_PIN, LOW);
}

static void preloadSet(bool on) { ioExpanderWritePin(EXP_PRELOAD_CMD_BIT, on ? 1 : 0); }
static void magIndexSet(bool on) { ioExpanderWritePin(EXP_MAG_INDEX_CMD_BIT, on ? 1 : 0); }
static void stopPreload() {
    preloadSet(false);
    s_preload_state = 0;
}

static void setNote(const char* s) {
    std::snprintf(s_operator_note, sizeof(s_operator_note), "%s", s ? s : "-");
}

static void IRAM_ATTR stage1Isr() {
    s_stage1_us = micros();
    s_stage1_count++;
}
static void IRAM_ATTR stage2Isr() {
    s_stage2_us = micros();
    s_stage2_count++;
}

static void enterFault(FaultCode code, const char* why) {
    if (s_fault_latched) return; // 已锁存则保留首个故障码
    s_fault = code;
    s_fault_latched = true;

    // 统一入口：先断释放链与预载，再把舵机驱到安全位并保持力矩。
    // 不立即释能是因为开环 tilt 断电会因重力下坠（§6.9 第 13 条）。
    releaseChainOff();
    safetyLatchArmed(false);
    homeAbort();
    stopPreload();
    servoAxisFaultSafe();

    s_cycle = CycleState::FAULT;
    s_cycle_enter_ms = millis();
    setNote(why ? why : "fault");
    telemetryEmitEvent("FAULT", why ? why : "unknown");
}

// 区域 B 守卫矩阵（§6.1）：区域 B 决定区域 A 能走多远。
// 特例：已进入发射链（PRELOAD..INDEX）后按 §6.2 不再要求持续 LOCKED，
// 链内前进放行，锁定丢失由中止窗口处理；守卫只约束进入链之前。
static bool cycleGuardAllows(CycleState next) {
    const uint8_t cur = (uint8_t)s_cycle;
    if (cur >= (uint8_t)CycleState::PRELOAD && cur <= (uint8_t)CycleState::INDEX) {
        return true;
    }
    switch (s_aim) {
        case AimState::LOCKED:
            return true;
        case AimState::IDLE:
        case AimState::SEARCHING:
        case AimState::TRACKING:
        case AimState::CALIBRATING:
        case AimState::CALIB_MODE:
            return (uint8_t)next <= (uint8_t)CycleState::READY;
    }
    return false;
}

// 当前组合的合法性查询，与 cycleGuardAllows 职责分开：后者判「能否迁移到 next」，
// 这里判「当前 cycle/aim 组合是否本就不该存在」。把当前状态当作 next 喂回守卫，
// 复用同一张矩阵而不复制常量；FAULT 是任意时刻都合法的锁存态，单独放行。
static bool cycleStateLegal() {
    if (s_cycle == CycleState::FAULT) return true;
    return cycleGuardAllows(s_cycle);
}

static void changeCycle(CycleState next, const char* why) {
    if (next == s_cycle) return;
    if (next != CycleState::FAULT && !cycleGuardAllows(next)) {
        telemetryEmitEvent("GUARD", why ? why : "blocked");
        return;
    }
    // 离开含能态统一卸载：解除武装锁存。
    if (s_cycle == CycleState::ARMED && next != CycleState::RELEASE) {
        safetyLatchArmed(false);
    }
    if (next == CycleState::READY || next == CycleState::SAFE) {
        releaseChainOff();
    }
    s_cycle = next;
    s_cycle_enter_ms = millis();
    telemetryEmitEvent("STATE", why ? why : "");
}

static void setAim(AimState next, const char* why) {
    if (next == s_aim) return;
    s_aim = next;
    telemetryEmitEvent("AIM", why ? why : "");
}

// 故障清除：软故障才接受，只把系统送到 SAFE，随后由 SAFE → HOME 真实归零恢复。
// 硬故障软件清除无效；FAULT → READY 这条边不存在。
static bool faultClear() {
    if (!s_fault_latched) return false;
    if (faultSeverity(s_fault) == FaultSeverity::HARD) {
        telemetryEmitEvent("FAULT", "clear rejected: hard fault");
        return false;
    }
    s_fault_latched = false;
    s_fault = FaultCode::NONE;
    setNote("fault cleared");
    changeCycle(CycleState::SAFE, "fault cleared -> SAFE");
    return true;
}

static void updateAimQuality(bool obs_fresh, uint32_t now_ms) {
    if (s_aim == AimState::CALIB_MODE) return;
    if (!s_cal_ok) {
        setAim(AimState::CALIBRATING, "calibration invalid");
        return;
    }
    if (!obs_fresh) {
        s_lock_start_ms = 0;
        s_lock_lost_ms = (s_lock_lost_ms == 0) ? now_ms : s_lock_lost_ms;
        setAim(AimState::SEARCHING, "no observation");
        return;
    }
    if (servoAxisInDeadband()) {
        if (s_lock_start_ms == 0) {
            s_lock_start_ms = now_ms;
            s_lock_start_frames = s_vision_frame_count;
        }
        const bool window_ok = (now_ms - s_lock_start_ms) >= LOCK_WINDOW_MS;
        const bool frames_ok = (s_vision_frame_count - s_lock_start_frames) >= LOCK_MIN_VISION_FRAMES;
        if (window_ok && frames_ok) {
            setAim(AimState::LOCKED, "lock window satisfied");
        } else {
            setAim(AimState::TRACKING, "settling");
        }
    } else {
        s_lock_start_ms = 0;
        setAim(AimState::TRACKING, "tracking");
    }
}

// 视觉观测 -> 相对光轴角 -> alpha-beta -> 叠加到当前轴角。
static void updateTracking(const TargetObservation& obs, float dt_s) {
    if (s_aim == AimState::CALIB_MODE) return; // 标定时由 CAL JOG 直接给目标
    if (!obs.valid || obs.centroid.confidence <= 0.0f) return;

    float bearing_deg = 0.0f, elevation_deg = 0.0f;
    if (!calibrationPixelToAngles(s_cal, obs.centroid.x, obs.centroid.y,
                                  bearing_deg, elevation_deg)) {
        return;
    }

    if (!s_has_track) {
        s_bearing_deg = bearing_deg;
        s_elevation_deg = elevation_deg;
        s_bearing_rate_dps = 0.0f;
        s_elevation_rate_dps = 0.0f;
        s_has_track = true;
    } else {
        const float pred_b = s_bearing_deg + s_bearing_rate_dps * dt_s;
        const float pred_e = s_elevation_deg + s_elevation_rate_dps * dt_s;
        const float res_b = angleDiffDeg(pred_b, bearing_deg);
        const float res_e = angleDiffDeg(pred_e, elevation_deg);
        const float alpha = 0.50f; // 待实测标定
        const float beta = 0.20f;  // 待实测标定
        s_bearing_deg = pred_b + alpha * res_b;
        s_elevation_deg = pred_e + alpha * res_e;
        s_bearing_rate_dps += beta * res_b / dt_s;
        s_elevation_rate_dps += beta * res_e / dt_s;
    }

    const float cur_pan = servoAxisState(AXIS_PAN).position_deg;
    const float cur_tilt = servoAxisState(AXIS_TILT).position_deg;
    servoAxisSetTargetDeg(AXIS_PAN, cur_pan + s_bearing_deg);
    servoAxisSetTargetDeg(AXIS_TILT, cur_tilt + s_elevation_deg);
}

static void advanceCycle(uint32_t now_ms, bool obs_fresh) {
    switch (s_cycle) {
        case CycleState::BOOT:
            changeCycle(CycleState::SAFE, "boot");
            break;

        case CycleState::SAFE:
            // SAFE → HOME：上电与故障清除后都从这里开始归零。
            if (homeStart(now_ms)) {
                setAim(s_cal_ok ? AimState::SEARCHING : AimState::CALIBRATING, "homing");
                changeCycle(CycleState::HOME, "homing start");
            }
            break;

        case CycleState::HOME: {
            const HomeResult r = homeUpdate(now_ms);
            if (r == HomeResult::DONE) {
                // §6.10 步骤 5：归零完成后再校验标定数据。
                setAim(s_cal_ok ? AimState::SEARCHING : AimState::CALIBRATING, "homing done");
                changeCycle(CycleState::READY, "homing done");
            } else if (r == HomeResult::TIMEOUT) {
                enterFault(FaultCode::HOME_TIMEOUT, "home timeout");
            } else if (r == HomeResult::CONFLICT) {
                enterFault(FaultCode::HOME_SWITCH_CONFLICT, "home switch conflict");
            }
            break;
        }

        case CycleState::READY:
            updateAimQuality(obs_fresh, now_ms);
            if (s_aim == AimState::LOCKED) {
                changeCycle(CycleState::AIM_ALLOWED, "lock acquired");
            }
            break;

        case CycleState::AIM_ALLOWED:
            updateAimQuality(obs_fresh, now_ms);
            if (s_aim != AimState::LOCKED) {
                changeCycle(CycleState::READY, "lock lost");
                break;
            }
            // Interlock 1：机构复位才准预载。
            if (safetyCanEnterPreload()) {
                preloadSet(true);
                s_preload_state = 1;
                s_phase_start_ms = now_ms;
                s_lock_lost_ms = 0;
                changeCycle(CycleState::PRELOAD, "interlock1 ok");
            }
            break;

        case CycleState::PRELOAD:
            updateAimQuality(obs_fresh, now_ms);
            if (s_aim != AimState::LOCKED) {
                // 预载未完成且锁定丢失超过中止窗口：受控卸载退回 READY。
                if (s_lock_lost_ms == 0) s_lock_lost_ms = now_ms;
                if ((now_ms - s_lock_lost_ms) > PRELOAD_ABORT_MS) {
                    stopPreload();
                    s_lock_lost_ms = 0;
                    changeCycle(CycleState::READY, "lock lost abort");
                    break;
                }
            } else {
                s_lock_lost_ms = 0;
            }
            if (s_preload_state == 1 && (now_ms - s_phase_start_ms) >= PRELOAD_TIME_MS) {
                s_preload_state = 2;
                safetyLatchArmed(true); // 武装锁存参与 H1 与门
                changeCycle(CycleState::ARMED, "preload done");
            }
            break;

        case CycleState::ARMED: {
            updateAimQuality(obs_fresh, now_ms);
            const bool external = safetyExternalAllow();
            const bool rise = external && !s_prev_external_allow;
            s_prev_external_allow = external;
            // 释放前最后一瞬重新确认 LOCKED，并复查全部互锁（§6.5 H1 的软件侧）。
            if (s_aim == AimState::LOCKED &&
                (now_ms - s_cycle_enter_ms) >= ARMED_MIN_DWELL_MS &&
                (RELEASE_AUTO_ENABLED != 0 || rise)) {
                if (!safetyCanRelease()) {
                    enterFault(FaultCode::RELEASE_GATE_SELFTEST, "release interlock rejected");
                } else {
                    digitalWrite(RELEASE_CMD_PIN, HIGH);
                    s_release_start_ms = now_ms;
                    changeCycle(CycleState::RELEASE, "release");
                }
            }
            break;
        }

        case CycleState::RELEASE:
            if ((now_ms - s_release_start_ms) >= RELEASE_PULSE_MS) {
                releaseChainOff();
                safetyLatchArmed(false);
                changeCycle(CycleState::RECOVER, "release pulse done");
            }
            break;

        case CycleState::RECOVER:
            if (safetyMechRecovered()) {
                if (s_mode == FireMode::SOFT_PAYLOAD) {
                    s_index_attempts = 0;
                    s_index_phase_start_ms = now_ms;
                    magIndexSet(true);
                    changeCycle(CycleState::INDEX, "recovered, index");
                } else {
                    changeCycle(CycleState::READY, "recovered");
                }
            } else if ((now_ms - s_cycle_enter_ms) > RECOVER_TIMEOUT_MS) {
                enterFault(FaultCode::MECH_STUCK, "mechanism not recovered");
            }
            break;

        case CycleState::INDEX:
            if (safetyMagIndexOk()) {
                magIndexSet(false);
                s_mag_position = (uint8_t)((s_mag_position + 1) % MAG_POSITIONS);
                s_cycle_count++;
                changeCycle(CycleState::READY, "index ok");
            } else if ((now_ms - s_index_phase_start_ms) > INDEX_TIMEOUT_MS) {
                magIndexSet(false);
                s_index_attempts++;
                if (s_index_attempts >= INDEX_MAX_RETRY) {
                    // STOP-04：连续失败禁 Mode B，升级软故障。
                    s_mode = FireMode::AIR_ONLY;
                    enterFault(FaultCode::INDEX_FAIL, "index retries exhausted");
                } else {
                    s_index_phase_start_ms = now_ms;
                    magIndexSet(true);
                }
            }
            break;

        case CycleState::FAULT:
            break;
    }
}

static void shellGetObservation(TargetObservation& out) { out = s_obs; }
static void shellGetAxes(AxisState& pan, AxisState& tilt) { servoAxisGetState(pan, tilt); }
static AimState shellGetState() { return s_aim; }
static CycleState shellGetCycleState() { return s_cycle; }
static FaultCode shellGetFaultCode() { return s_fault; }
static CalibrationData* shellGetCalibration() { return &s_cal; }
static void shellApplyCalibration() { s_cal_ok = s_cal.valid; }
static void shellJog(float pan_deg, float tilt_deg) { servoAxisSetAxesDeg(pan_deg, tilt_deg); }
static void shellEmergencyStop() { enterFault(FaultCode::ESTOP, "shell estop"); }

static uint16_t shellGetMembraneId() { return s_batch.membrane_id; }
static uint16_t shellGetPayloadId() { return s_batch.payload_id; }

// 先改内存再落 NVS，写失败回退，保证参数校验失败或存储失败时都保持原值。
static bool shellSetMembraneId(uint16_t id) {
    const uint16_t prev = s_batch.membrane_id;
    s_batch.membrane_id = id;
    if (!batchTagsSave(s_batch)) {
        s_batch.membrane_id = prev;
        return false;
    }
    return true;
}

static bool shellSetPayloadId(uint16_t id) {
    const uint16_t prev = s_batch.payload_id;
    s_batch.payload_id = id;
    if (!batchTagsSave(s_batch)) {
        s_batch.payload_id = prev;
        return false;
    }
    return true;
}

static bool shellSetCalibrationMode(bool enable) {
    if (enable) {
        if (s_fault_latched) return false;
        if (s_cycle != CycleState::READY) return false; // 只能在 READY 发起
        if (s_aim == AimState::CALIB_MODE) return true;
        // 标定期间主动断开释放链，并清武装锁存。
        releaseChainOff();
        safetyLatchArmed(false);
        stopPreload();
        setAim(AimState::CALIB_MODE, "manual calibration");
        return true;
    }
    if (s_aim != AimState::CALIB_MODE) return false;
    setAim(s_cal_ok ? AimState::SEARCHING : AimState::CALIBRATING, "leave calibration");
    return true;
}

void setup() {
    telemetryInit(TELEMETRY_BAUD);

    Serial.println();
    Serial.println("=== MST-01 微型定向脉冲平台 ===");
    Serial.printf("fw version: %s  proto: %s  mech: %s  stages: %d\n",
                  AIM_BUILD_VERSION, MST_PROTOTYPE_VERSION, MST_MECHANISM_VERSION,
                  MST_MECHANISM_STAGES);
    Serial.printf("loop: %dHz  pan limit: %.0f~%.0f  tilt limit: %.0f~%.0f\n",
                  CONTROL_LOOP_HZ, PAN_MIN_DEG, PAN_MAX_DEG, TILT_MIN_DEG, TILT_MAX_DEG);
    Serial.printf("servo drive: %s\n",
                  (SERVO_DRIVE_TYPE == SERVO_DRIVE_BUS) ? "bus" : "pwm");

    // 批次追溯编号从 NVS 读回，断电不丢。无记录时保持 0，0 表示未设置。
    if (batchTagsLoad(s_batch)) {
        Serial.printf("[batch] membrane_id=%u payload_id=%u\n",
                      (unsigned)s_batch.membrane_id, (unsigned)s_batch.payload_id);
    } else {
        Serial.println("[batch] 无批次记录，膜片/载荷编号均为 0（未设置）");
    }

    // 释放链相关引脚先落到安全侧，任何后续初始化失败都不会让它误动。
    pinMode(RELEASE_CMD_PIN, OUTPUT);
    digitalWrite(RELEASE_CMD_PIN, LOW);
    pinMode(STAGE1_STATE_PIN, INPUT_PULLUP);
    pinMode(STAGE2_STATE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(STAGE1_STATE_PIN), stage1Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STAGE2_STATE_PIN), stage2Isr, CHANGE);

    safetyGateInit();
    homeInit();

    // 外设初始化采用受限分支：任一致命失败立即进 FAULT 并停止后续推进。
    if (!ioExpanderInit(MCP23017_ADDR)) {
        enterFault(FaultCode::PERIPH_INIT, "MCP23017 init failed");
        return;
    }
    ioExpanderWritePin(EXP_BOUNDARY_CMD_BIT, MST_BOUNDARY_RESEARCH_ACTIVE ? 1 : 0);
    ioExpanderWritePin(EXP_STATUS_LED_BIT, 0);
    releaseChainOff();

    if (!servoAxisInit()) {
        enterFault(FaultCode::PERIPH_INIT, "servo init failed");
        return;
    }

    // 标定外壳尽早注入，摄像头等后续失败时仍可查询 STATUS。
    CalibShellHooks shell_hooks;
    shell_hooks.getObservation = shellGetObservation;
    shell_hooks.getAxes = shellGetAxes;
    shell_hooks.getState = shellGetState;
    shell_hooks.getCycleState = shellGetCycleState;
    shell_hooks.getFaultCode = shellGetFaultCode;
    shell_hooks.calibration = shellGetCalibration;
    shell_hooks.applyCalibration = shellApplyCalibration;
    shell_hooks.setCalibrationMode = shellSetCalibrationMode;
    shell_hooks.jog = shellJog;
    shell_hooks.emergencyStop = shellEmergencyStop;
    shell_hooks.clearFault = faultClear;
    shell_hooks.getMembraneId = shellGetMembraneId;
    shell_hooks.getPayloadId = shellGetPayloadId;
    shell_hooks.setMembraneId = shellSetMembraneId;
    shell_hooks.setPayloadId = shellSetPayloadId;
    if (!calibShellInit(shell_hooks)) {
        Serial.println("[cal] 标定外壳未启用：回调注入不完整");
    }

    // PSRAM 检查：缺失只告警不终止（视觉会退回内部 RAM，余量小）。
    const size_t psram = ESP.getPsramSize();
    Serial.printf("PSRAM: %u bytes\n", (unsigned)psram);
    if (psram == 0) {
        telemetryEmitEvent("PSRAM", "未检测到 PSRAM，已回退内部 RAM");
    }

    // 摄像头：带重试。失败只进 FAULT，不继续推进周期（§6.9 第 12 条）。
    for (int attempt = 0; attempt < 3 && !s_vision_ok; ++attempt) {
        s_vision_ok = visionInit();
        if (!s_vision_ok) {
            Serial.printf("[vision] init 失败，重试 %d/3\n", attempt + 1);
            delay(50); // setup 阶段允许短暂等待，loop 内不允许
        }
    }

    // 标定：失败则用内置占位默认值。
    if (calibrationLoad(s_cal)) {
        s_cal_ok = s_cal.valid;
        Serial.printf("[cal] 已加载：%dx%d valid=%d\n", s_cal.width, s_cal.height, s_cal.valid);
    } else {
        calibrationSetDefaults(s_cal, VISION_SRC_W, VISION_SRC_H);
        s_cal_ok = false;
        Serial.println("[cal] 无标定记录，使用内置占位默认值");
    }

    // 释放门控自检：锁存未置位时释放必须关死。
    if (!safetyReleaseGateSelfTest()) {
        enterFault(FaultCode::RELEASE_GATE_SELFTEST, "release gate self test failed");
    }
    if (!s_vision_ok && !s_fault_latched) {
        enterFault(FaultCode::OBS_LINK, "camera init failed");
    }

    if (!s_fault_latched) {
        changeCycle(CycleState::SAFE, "setup done");
    }
    telemetryEmitEvent("BOOT", "setup done");
}

void loop() {
    // 串口命令在最前面轮询：loop 未到控制节拍时会提前 return，
    // 放在这里才能按 UART 到达速率收字节，避免长命令被硬件 FIFO 丢掉。
    calibShellPoll();

    const uint32_t t_loop_start = micros();
    const uint32_t now_ms = millis();

    // 固定节拍：未到下一拍直接返回，绝不阻塞。
    static uint32_t next_us = 0;
    if (next_us == 0) {
        next_us = t_loop_start;
    }
    if ((int32_t)(t_loop_start - next_us) < 0) {
        return;
    }
    if ((int32_t)(t_loop_start - next_us) > (int32_t)((uint32_t)CONTROL_LOOP_BUDGET_MS * 1000u)) {
        next_us = t_loop_start;
    }
    next_us += (uint32_t)CONTROL_LOOP_BUDGET_MS * 1000u;

    // 输入采样每拍都做，故障态也刷新，便于 STATUS 显示现场电平。
    safetyGateUpdate(now_ms);

    // I2C0 运行期连续失败达到阈值：扩展器状态不再可信，判总线卡死（硬故障）。
    // 上一行的 safetyGateUpdate() 已在本拍完成读事务并刷新连续失败计数。
    if (!s_fault_latched && ioExpanderBusSuspectedHang()) {
        enterFault(FaultCode::I2C_BUS_HANG, "i2c0 bus hang");
    }

    // 失稳事件时间戳由中断捕获，这里把增量转成日志（§15.2 需要精确时间）。
    static uint8_t last_stage1_count = 0;
    static uint8_t last_stage2_count = 0;
    if (s_stage1_count != last_stage1_count) {
        last_stage1_count = s_stage1_count;
        char buf[24];
        std::snprintf(buf, sizeof(buf), "t=%luus", (unsigned long)s_stage1_us);
        telemetryEmitEvent("STAGE1", buf);
    }
    if (s_stage2_count != last_stage2_count) {
        last_stage2_count = s_stage2_count;
        char buf[24];
        std::snprintf(buf, sizeof(buf), "t=%luus", (unsigned long)s_stage2_us);
        telemetryEmitEvent("STAGE2", buf);
    }

    // 取帧（按目标帧率 + 看门狗降帧倍数节流）
    bool obs_fresh = false;
    bool obs_dropped = false; // 本拍尝试取帧但未得到有效观测
    if (!s_fault_latched && s_vision_ok) {
        const uint32_t vision_period_ms = (1000u / VISION_TARGET_FPS) * s_frame_divider;
        if ((now_ms - s_last_capture_ms) >= vision_period_ms) {
            s_last_capture_ms = now_ms;
            TargetObservation obs;
            if (visionCapture(obs) && obs.valid) {
                s_obs = obs; // 仅在有效时更新，避免无效帧覆盖上一有效观测
                s_vision_frame_count++;
                s_capture_fail_streak = 0;
            } else {
                obs_dropped = true;
                if (s_capture_fail_streak < 0xFFFFFFFFul) s_capture_fail_streak++;
            }
        }
    }
    if (s_obs.valid && (now_ms - s_obs.t_ms) <= OBS_TIMEOUT_MS) {
        obs_fresh = true;
    }

    // 双区域组合核对：守卫矩阵只约束迁移，这里补一次对当前稳定组合的核对，
    // 捕捉逻辑漏洞落进来的矩阵外组合。非法即进硬故障，并把当时的 A/B 取值留档。
    if (!s_fault_latched && !cycleStateLegal()) {
        char combo[32];
        std::snprintf(combo, sizeof(combo), "cycle=%u aim=%u",
                      (unsigned)s_cycle, (unsigned)s_aim);
        telemetryEmitEvent("STATE_ILLEGAL", combo);
        enterFault(FaultCode::STATE_ILLEGAL, combo);
    }

    if (!s_fault_latched) {
        advanceCycle(now_ms, obs_fresh);

        // 指向跟踪只在链上或 READY 态运行，归零/故障时由 home/fault 分支控制目标。
        const bool tracking_phase = (s_cycle == CycleState::READY ||
                                     s_cycle == CycleState::AIM_ALLOWED ||
                                     s_cycle == CycleState::PRELOAD ||
                                     s_cycle == CycleState::ARMED);
        if (tracking_phase) {
            const float dt_s = (float)CONTROL_LOOP_BUDGET_MS / 1000.0f;
            updateTracking(obs_fresh ? s_obs : TargetObservation(), dt_s);
        }
    }

    // 舵机指令下发，HOME 分支已在上面设好目标。
    servoAxisUpdate((float)CONTROL_LOOP_BUDGET_MS / 1000.0f);

    // 观测链路异常：连续取帧失败升级软故障。
    if (!s_fault_latched && s_capture_fail_streak >= OBS_LINK_FAIL_N) {
        enterFault(FaultCode::OBS_LINK, "capture link lost");
    }

    // 遥测：§19 的 17 个字段。
    TelemetryRecord rec;
    rec.timestamp = now_ms;
    rec.experiment_id = MST_EXPERIMENT_ID;
    rec.prototype_version = MST_PROTOTYPE_VERSION;
    rec.mechanism_version = MST_MECHANISM_VERSION;
    rec.mode = (uint8_t)s_mode;
    rec.boundary_state = MST_BOUNDARY_RESEARCH_ACTIVE;
    rec.preload_state = s_preload_state;
    rec.membrane_id = s_batch.membrane_id;
    rec.payload_id = s_batch.payload_id;
    rec.cycle_count = s_cycle_count;
    rec.stage1_event = s_stage1_count;
    rec.stage2_event = s_stage2_count;
    rec.mechanism_recovered = safetyMechRecovered() ? 1 : 0;
    rec.magazine_position = s_mag_position;
    rec.magazine_index_ok = safetyMagIndexOk() ? 1 : 0;
    rec.fault_code = (uint16_t)s_fault;
    rec.operator_note = s_operator_note;
    telemetryEmit(rec);

    // 看门狗：超预算只降帧；连续超预算或单拍严重超时直接进 FAULT（§6.9 第 9 条）。
    const uint32_t loop_us = micros() - t_loop_start;

    // 帧轨迹诊断行（DIAG,）：与上面的 MST, 各走各的，只服务指向精度、控制周期与锁定建立时间。
    // loop_us 量的是本拍计算耗时，不含遥测打印，避免打印本身抬高控制周期读值。
    {
        DiagRecord diag;
        diag.timestamp_ms = now_ms;
        diag.loop_us = loop_us;
        diag.obs_valid = obs_fresh ? 1 : 0;
        diag.obs_dropped = obs_dropped ? 1 : 0;
        if (obs_fresh) {
            diag.px = s_obs.centroid.x;
            diag.py = s_obs.centroid.y;
            diag.confidence = s_obs.centroid.confidence;
            float bearing_deg = 0.0f, elevation_deg = 0.0f;
            if (calibrationPixelToAngles(s_cal, diag.px, diag.py, bearing_deg, elevation_deg)) {
                diag.err_pan_deg = bearing_deg;
                diag.err_tilt_deg = elevation_deg;
                diag.err_deg = std::sqrt(bearing_deg * bearing_deg + elevation_deg * elevation_deg);
            }
        }
        const AxisState pan_state = servoAxisState(AXIS_PAN);
        const AxisState tilt_state = servoAxisState(AXIS_TILT);
        diag.pan_deg = pan_state.position_deg;
        diag.tilt_deg = tilt_state.position_deg;
        diag.pan_target_deg = pan_state.target_deg;
        diag.tilt_target_deg = tilt_state.target_deg;
        diag.aim_state = (uint8_t)s_aim;
        diag.has_feedback = (pan_state.has_feedback || tilt_state.has_feedback) ? 1 : 0;
        telemetryEmitDiag(diag);
    }

    if (loop_us > (uint32_t)CONTROL_LOOP_BUDGET_MS * 1000u) {
        if (s_overrun_streak < 0xFFFFFFFFul) s_overrun_streak++;
        if (s_overrun_streak == 1 || (s_overrun_streak % 50u) == 0u) {
            telemetryEmitEvent("WD", "loop over budget, reducing vision fps");
        }
        if (s_frame_divider < 8u) {
            s_frame_divider++;
        }
        if (!s_fault_latched &&
            (s_overrun_streak >= WATCHDOG_STREAK_N ||
             (loop_us / 1000u) > WATCHDOG_HARD_OVERRUN_MS)) {
            enterFault(FaultCode::WATCHDOG, "watchdog timeout");
        }
    } else {
        s_overrun_streak = 0;
        if (s_frame_divider > 1u) s_frame_divider--; // 恢复
    }
}
