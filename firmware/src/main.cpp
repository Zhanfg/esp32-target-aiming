/**
 * main.cpp
 * 固件入口与非阻塞状态机。上电初始化后按固定节拍跑"视觉->解算->控制->遥测"，
 * 同时执行安全策略：发射器互锁、故障切断、看门狗降帧。
 *
 * 位于依赖图顶层，依赖 hal、control、vision、calibration、comms。
 *
 * 安全策略：
 *   1. 发射器只在 LOCKED 且显式 aimArmTransmitter(true) 后才允许触发；
 *   2. setup() 期间不使能发射器，始终 SAFE；
 *   3. FAULT 或紧急停止无条件切断发射器与电机输出；
 *   4. loop() 内不用超过 1ms 的 delay()，看门狗超预算只降帧不阻塞。
 */

#include <Arduino.h>

#include "config.h"
#include "aim_types.h"
#include "calibration/calibration.h"
#include "comms/calib_shell.h"
#include "comms/telemetry.h"
#include "control/axis.h"
#include "control/turntable.h"
#include "hal/encoder.h"
#include "hal/motor_driver.h"
#include "hal/transmitter.h"
#include "vision/vision.h"

static MotorTransmitter g_transmitter;

static AimState s_state = AimState::IDLE;
static bool s_fault_latched = false;
static bool s_vision_ok = false;
static bool s_cal_ok = false;
static CalibrationData s_cal;

static TargetObservation s_obs;       // 最近一次视觉观测
static uint32_t s_last_capture_ms = 0; // 最近一次尝试取帧的时刻

static uint32_t s_lock_frames = 0;    // 连续处于 deadband 的帧数
static uint32_t s_overrun_streak = 0; // 连续超预算次数（看门狗）
static uint32_t s_frame_divider = 1;  // 降帧倍数（看门狗触发时增大）

static AxisState s_pan_state;
static AxisState s_tilt_state;
static TurretSolution s_sol; // 最近一次解算结果（遥测用）

// 前向声明
static void enterFault(const char* why);
static void changeState(AimState next, const char* why);

// 显式武装/解除发射器。解除永远允许；武装仅在 LOCKED 且无故障时允许。
bool aimArmTransmitter(bool enable) {
    if (!enable) {
        g_transmitter.arm(false);
        telemetryEmitEvent("ARM", "disarm");
        return true;
    }
    if (s_fault_latched) {
        telemetryEmitEvent("ARM", "拒绝：FAULT");
        return false;
    }
    if (s_state == AimState::CALIB_MODE) {
        telemetryEmitEvent("ARM", "拒绝：标定模式");
        return false;
    }
    if (s_state != AimState::LOCKED) {
        telemetryEmitEvent("ARM", "拒绝：未锁定");
        return false;
    }
    bool ok = g_transmitter.arm(true);
    telemetryEmitEvent("ARM", ok ? "armed" : "arm failed");
    return ok;
}

// 触发发射。必须处于 LOCKED 且已武装；否则拒绝。
bool aimFireTransmitter() {
    if (s_fault_latched || s_state != AimState::LOCKED) {
        telemetryEmitEvent("FIRE", "拒绝：非锁定/故障");
        return false;
    }
    bool ok = g_transmitter.fire();
    telemetryEmitEvent("FIRE", ok ? "fired" : "拒绝：未武装");
    return ok;
}

// 进入故障：无条件切断一切输出。
static void enterFault(const char* why) {
    s_fault_latched = true;
    s_state = AimState::FAULT;
    g_transmitter.emergencyStop();       // 切断发射器
    motorDriverSetStandby(false);        // 硬件级切断电机
    turretEmergencyStop();               // 两轴滑行
    telemetryEmitEvent("FAULT", why ? why : "unknown");
}

// 状态切换（带事件日志；离开 LOCKED 时自动解除发射器武装）。
static void changeState(AimState next, const char* why) {
    if (next == s_state) {
        return;
    }
    if (s_state == AimState::LOCKED && next != AimState::LOCKED) {
        // 安全：一旦不再锁定，立即解除武装。
        g_transmitter.arm(false);
    }
    s_state = next;
    telemetryEmitEvent("STATE", why ? why : "");
}

// 标定外壳接口：回调注入，calib_shell 不直接依赖 main 的静态变量。
static void shellGetObservation(TargetObservation& out) { out = s_obs; }
static void shellGetAxes(AxisState& pan, AxisState& tilt) { turretGetState(pan, tilt); }
static AimState shellGetState() { return s_state; }
static CalibrationData* shellGetCalibration() { return &s_cal; }
static void shellApplyCalibration() {
    s_cal_ok = s_cal.valid;
    turretSetCalibration(s_cal);
}
static void shellJog(float pan_deg, float tilt_deg) { turretSetAxesDeg(pan_deg, tilt_deg); }
static void shellEmergencyStop() { enterFault("shell estop"); }

static bool shellSetCalibrationMode(bool enable) {
    if (enable) {
        if (s_fault_latched) return false;
        if (s_state == AimState::CALIB_MODE) return true;
        changeState(AimState::CALIB_MODE, "manual calibration");
        // 标定期间发射器物理断开，且 aimArmTransmitter 会继续拒绝武装。
        g_transmitter.emergencyStop();
        telemetryEmitEvent("CAL", "enter manual calibration");
        return true;
    }
    if (s_state != AimState::CALIB_MODE) return false;
    turretClearHold();
    changeState(s_cal_ok ? AimState::SEARCHING : AimState::CALIBRATING, "leave calibration");
    telemetryEmitEvent("CAL", "exit manual calibration");
    return true;
}

void setup() {
    telemetryInit(TELEMETRY_BAUD);

    Serial.println();
    Serial.println("=== ESP32-S3 双轴目标锁定舵盘 ===");
    Serial.printf("fw version: %s\n", AIM_BUILD_VERSION);
    Serial.printf("loop: %dHz  pan limit: %.0f~%.0f  tilt limit: %.0f~%.0f\n",
                  CONTROL_LOOP_HZ, PAN_MIN_DEG, PAN_MAX_DEG, TILT_MIN_DEG, TILT_MAX_DEG);
    Serial.printf("encoder: PPR=%d gear(pan/tilt)=%d/%d CPR(pan/tilt)=%d/%d\n",
                  ENCODER_PPR, GEAR_RATIO_PAN, GEAR_RATIO_TILT,
                  COUNTS_PER_OUTPUT_REV_PAN, COUNTS_PER_OUTPUT_REV_TILT);

    // PSRAM 检查：缺失只告警不终止（视觉会退回内部 RAM，余量小）。
    size_t psram = ESP.getPsramSize();
    Serial.printf("PSRAM: %u bytes\n", (unsigned)psram);
    if (psram == 0) {
        telemetryEmitEvent("PSRAM", "未检测到 PSRAM，已回退内部 RAM");
    }

    // 电机驱动 → 编码器（上电零位标定）
    if (!motorDriverInit()) {
        enterFault("motor driver init failed");
    }
    if (!encoderInit()) {
        enterFault("encoder init failed");
    } else {
        encoderReset(AXIS_PAN);
        encoderReset(AXIS_TILT);
    }

    // 舵盘初始化并归零到安全位
    if (!turretInit()) {
        enterFault("turret init failed");
    } else {
        turretSetAxesDeg(CONTROL_HOME_PAN_DEG, CONTROL_HOME_TILT_DEG);
    }

    // 发射器：初始化后立即置 SAFE，setup 期间绝不武装。
    if (!g_transmitter.init()) {
        enterFault("transmitter init failed");
    } else {
        g_transmitter.arm(false);
    }

    // 摄像头：带重试
    for (int attempt = 0; attempt < 3 && !s_vision_ok; ++attempt) {
        s_vision_ok = visionInit();
        if (!s_vision_ok) {
            Serial.printf("[vision] init 失败，重试 %d/3\n", attempt + 1);
            delay(50); // setup 阶段允许短暂等待，loop 内不允许
        }
    }
    if (!s_vision_ok) {
        enterFault("camera init failed");
    }

    // 标定：失败则用内置占位默认值并进入 CALIBRATING
    if (calibrationLoad(s_cal)) {
        s_cal_ok = s_cal.valid;
        Serial.printf("[cal] 已加载：%dx%d valid=%d\n", s_cal.width, s_cal.height, s_cal.valid);
    } else {
        calibrationSetDefaults(s_cal, VISION_SRC_W, VISION_SRC_H);
        s_cal_ok = false;
        Serial.println("[cal] 无标定记录，使用内置占位默认值");
    }
    turretSetCalibration(s_cal);

    // 串口标定外壳：注入状态访问与动作回调。
    CalibShellHooks shell_hooks;
    shell_hooks.getObservation = shellGetObservation;
    shell_hooks.getAxes = shellGetAxes;
    shell_hooks.getState = shellGetState;
    shell_hooks.calibration = shellGetCalibration;
    shell_hooks.applyCalibration = shellApplyCalibration;
    shell_hooks.setCalibrationMode = shellSetCalibrationMode;
    shell_hooks.jog = shellJog;
    shell_hooks.emergencyStop = shellEmergencyStop;
    if (!calibShellInit(shell_hooks)) {
        Serial.println("[cal] 标定外壳未启用：回调注入不完整");
    }

    // 收尾状态
    if (s_fault_latched) {
        // 保持 FAULT
    } else if (!s_cal_ok) {
        changeState(AimState::CALIBRATING, "no calibration");
    } else {
        changeState(AimState::IDLE, "ready");
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
    // 若已经落后超过一个周期，重新对齐到"现在 + 周期"，避免追赶式突发。
    if ((int32_t)(t_loop_start - next_us) > (int32_t)((uint32_t)CONTROL_LOOP_BUDGET_MS * 1000u)) {
        next_us = t_loop_start;
    }
    next_us += (uint32_t)CONTROL_LOOP_BUDGET_MS * 1000u;

    // 取帧（按目标帧率 + 看门狗降帧倍数节流）
    const uint32_t vision_period_ms =
        (1000u / VISION_TARGET_FPS) * s_frame_divider;
    if (s_vision_ok && (now_ms - s_last_capture_ms) >= vision_period_ms) {
        s_last_capture_ms = now_ms;
        TargetObservation obs;
        if (visionCapture(obs) && obs.valid) {
            s_obs = obs; // 仅在有效时更新，避免无效帧覆盖上一有效观测
        }
    }

    // 观测新鲜度
    bool obs_fresh = s_obs.valid && (now_ms - s_obs.t_ms) <= OBS_TIMEOUT_MS;

    // 状态机
    if (!s_fault_latched) {
        if (s_state == AimState::CALIB_MODE) {
            // 手动标定：状态由 calib_shell 的 CAL START/EXIT 控制，不跑自动跟踪判定。
        } else if (!s_vision_ok) {
            enterFault("vision not ready");
        } else if (!s_cal_ok) {
            changeState(AimState::CALIBRATING, "calibration invalid");
        } else if (!obs_fresh) {
            s_lock_frames = 0;
            changeState(AimState::SEARCHING, "no valid observation");
        } else if (turretInDeadband()) {
            if (s_lock_frames < 0xFFFFFFFFul) s_lock_frames++;
            if (s_lock_frames >= LOCK_FRAMES_N) {
                changeState(AimState::LOCKED, "lock acquired");
            } else {
                changeState(AimState::TRACKING, "settling");
            }
        } else {
            s_lock_frames = 0;
            changeState(AimState::TRACKING, "tracking");
        }
    }

    // 控制：故障时已切断，正常时每拍都跑（含归零保持）。
    if (!s_fault_latched) {
        if (s_state == AimState::CALIB_MODE) {
            // 标定模式忽略视觉，只维持 CAL JOG 设定的保持目标；未 JOG 时保持当前位置。
            s_sol = turretUpdate(TargetObservation(), now_ms);
        } else {
            s_sol = turretUpdate(obs_fresh ? s_obs : TargetObservation(), now_ms);
        }
        // 解算器移植后，s_sol 将来自 AimSolver。
        turretGetState(s_pan_state, s_tilt_state);
    } else {
        turretGetState(s_pan_state, s_tilt_state);
    }

    // 发射器时序推进（非阻塞）
    g_transmitter.update(now_ms);

    // 看门狗：单次 loop 超预算只记录并降帧，绝不阻塞。
    const uint32_t t_loop_end = micros();
    const uint32_t loop_us = t_loop_end - t_loop_start;
    telemetrySetLoopUs(loop_us);
    if (loop_us > (uint32_t)CONTROL_LOOP_BUDGET_MS * 1000u) {
        if (s_overrun_streak < 0xFFFFFFFFul) s_overrun_streak++;
        if (s_overrun_streak == 1 || (s_overrun_streak % 50u) == 0u) {
            telemetryEmitEvent("WD", "loop over budget, reducing vision fps");
        }
        if (s_frame_divider < 8u) {
            s_frame_divider++;
        }
        // 连续 100 次严重超预算（>5 倍）判为故障，切断输出。
        if (s_overrun_streak >= 100u &&
            loop_us > (uint32_t)CONTROL_LOOP_BUDGET_MS * 5000u && !s_fault_latched) {
            enterFault("watchdog timeout");
        }
    } else {
        s_overrun_streak = 0;
        if (s_frame_divider > 1u) s_frame_divider--; // 恢复
    }

    telemetryEmit(s_obs, s_sol, s_pan_state, s_tilt_state, s_state);
}
