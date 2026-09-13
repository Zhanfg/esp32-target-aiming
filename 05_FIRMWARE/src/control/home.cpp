/**
 * home.cpp
 */

#include "home.h"

#include <cmath>

#include "config.h"
#include "../hal/safety_gate.h"
#include "../hal/servo.h"
#include "servo_axis.h"

namespace {

enum class AxisPhase { SEEK, BACKOFF, DONE };

struct AxisHome {
    AxisPhase phase = AxisPhase::SEEK;
    float trigger_cmd = 0.0f;
    float backoff_deg = 0.0f;
    int8_t dir = -1;
    uint32_t confirm_start_ms = 0;
    uint32_t backoff_start_ms = 0;
    uint32_t last_step_ms = 0;
    bool switch_ok = true;
};

AxisHome s_pan;
AxisHome s_tilt;
bool s_active = false;
uint32_t s_start_ms = 0;

bool axisSwitchTriggered(uint8_t axis) {
    return (axis == AXIS_PAN) ? safetyPanHome() : safetyTiltHome();
}

void stepSeek(uint8_t axis, AxisHome& h, uint32_t now_ms) {
    if (now_ms - h.last_step_ms < HOME_SEEK_PERIOD_MS) return;
    h.last_step_ms = now_ms;
    const float pos = servoAxisState(axis).position_deg;
    servoAxisSetTargetDeg(axis, pos + (float)h.dir * HOME_SEEK_STEP_DEG);
}

bool updateAxis(uint8_t axis, AxisHome& h, uint32_t now_ms) {
    switch (h.phase) {
        case AxisPhase::SEEK:
            if (axisSwitchTriggered(axis)) {
                // 触发后要求稳定闭合一小段，消抖也避开开关抖动。
                if (h.confirm_start_ms == 0) {
                    h.confirm_start_ms = now_ms;
                } else if (now_ms - h.confirm_start_ms >= HOME_CONFIRM_MS) {
                    h.trigger_cmd = servoAxisState(axis).position_deg;
                    h.phase = AxisPhase::BACKOFF;
                    h.backoff_start_ms = now_ms;
                    // 开关在参考方向一侧，退回已知角即朝反方向让开。
                    servoAxisSetTargetDeg(axis, h.trigger_cmd - (float)h.dir * h.backoff_deg);
                }
            } else {
                // 确认窗口内丢失触发：判为参考信号矛盾，避免把抖动量当真基准。
                if (h.confirm_start_ms != 0 && (now_ms - h.confirm_start_ms) > HOME_CONFIRM_MS) {
                    h.switch_ok = false;
                }
                h.confirm_start_ms = 0;
                stepSeek(axis, h, now_ms);
            }
            break;

        case AxisPhase::BACKOFF:
            if (!h.switch_ok) break;
            if (now_ms - h.backoff_start_ms < HOME_SETTLE_MS) break;
            // 退回已知角后开关应已释放；仍闭合说明开关卡死或极性反了。
            if (axisSwitchTriggered(axis)) {
                h.switch_ok = false;
            } else {
                h.phase = AxisPhase::DONE;
            }
            break;

        case AxisPhase::DONE:
            break;
    }
    return h.switch_ok && h.phase == AxisPhase::DONE;
}

} // namespace

void homeInit() {
    s_pan = AxisHome();
    s_tilt = AxisHome();
    s_active = false;
}

bool homeStart(uint32_t now_ms) {
    if (s_active) return true;
    s_pan = AxisHome();
    s_tilt = AxisHome();
    s_pan.dir = (HOME_PAN_DIR < 0) ? -1 : 1;
    s_tilt.dir = (HOME_TILT_DIR < 0) ? -1 : 1;
    s_pan.backoff_deg = HOME_PAN_BACKOFF_DEG;
    s_tilt.backoff_deg = HOME_TILT_BACKOFF_DEG;
    s_pan.last_step_ms = now_ms;
    s_tilt.last_step_ms = now_ms;
    s_start_ms = now_ms;
    // 归零需要力矩，先使能再动。
    servoDrive().setTorqueEnabled(true);
    s_active = true;
    return true;
}

HomeResult homeUpdate(uint32_t now_ms) {
    if (!s_active) return HomeResult::DONE;

    if (now_ms - s_start_ms > HOME_TIMEOUT_MS) {
        s_active = false;
        return HomeResult::TIMEOUT;
    }

    const bool pan_done = updateAxis(AXIS_PAN, s_pan, now_ms);
    const bool tilt_done = updateAxis(AXIS_TILT, s_tilt, now_ms);

    if (!s_pan.switch_ok || !s_tilt.switch_ok) {
        s_active = false;
        return HomeResult::CONFLICT;
    }
    if (pan_done && tilt_done) {
        s_active = false;
        return HomeResult::DONE;
    }
    return HomeResult::RUNNING;
}

bool homeActive() { return s_active; }

void homeAbort() { s_active = false; }
