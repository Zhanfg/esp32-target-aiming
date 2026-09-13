/**
 * servo_axis.cpp
 * 无反馈实现里 position 等于当前指令角，只是估计值，不能当实测角度用。
 */

#include "servo_axis.h"

#include <cmath>

#include "config.h"
#include "../hal/servo.h"
#include "../math/angle_utils.h"

namespace {

struct AxisRuntime {
    float min_deg = 0.0f;
    float max_deg = 0.0f;
    float max_slew_dps = 0.0f;
    float deadband_deg = 0.0f;
    bool wrap_360 = false;
    float target_deg = 0.0f;
    float cmd_deg = 0.0f;
    AxisState state;
    bool initialized = false;
};

AxisRuntime s_axes[AXIS_COUNT];
bool s_initialized = false;

float axisErrorDeg(uint8_t axis) {
    const AxisRuntime& a = s_axes[axis];
    return angleDiffAxisDeg(a.state.position_deg, a.target_deg, a.wrap_360);
}

void writeCommand(uint8_t axis, float deg) {
    if (axis == AXIS_PAN) {
        servoDrive().writePanDeg(deg);
    } else {
        servoDrive().writeTiltDeg(deg);
    }
}

} // namespace

bool servoAxisInit() {
    if (!servoDrive().init()) {
        return false;
    }

    s_axes[AXIS_PAN].min_deg = PAN_MIN_DEG;
    s_axes[AXIS_PAN].max_deg = PAN_MAX_DEG;
    s_axes[AXIS_PAN].max_slew_dps = PAN_MAX_SLEW_DPS;
    s_axes[AXIS_PAN].deadband_deg = PAN_DEADBAND_DEG;
    s_axes[AXIS_PAN].wrap_360 = (ANGLE_WRAP_360_ENABLED != 0);

    s_axes[AXIS_TILT].min_deg = TILT_MIN_DEG;
    s_axes[AXIS_TILT].max_deg = TILT_MAX_DEG;
    s_axes[AXIS_TILT].max_slew_dps = TILT_MAX_SLEW_DPS;
    s_axes[AXIS_TILT].deadband_deg = TILT_DEADBAND_DEG;
    s_axes[AXIS_TILT].wrap_360 = false;

    for (int i = 0; i < AXIS_COUNT; ++i) {
        s_axes[i].target_deg = CONTROL_HOME_PAN_DEG;
        s_axes[i].cmd_deg = 0.0f;
        s_axes[i].state = AxisState();
        s_axes[i].initialized = true;
    }
    s_axes[AXIS_TILT].target_deg = CONTROL_HOME_TILT_DEG;

    s_initialized = true;
    return true;
}

void servoAxisSetTargetDeg(uint8_t axis, float deg) {
    if (axis >= AXIS_COUNT) return;
    if (!std::isfinite(deg)) return;
    if (s_axes[axis].wrap_360) {
        s_axes[axis].target_deg = angleWrap360(deg);
    } else {
        if (deg < s_axes[axis].min_deg) deg = s_axes[axis].min_deg;
        if (deg > s_axes[axis].max_deg) deg = s_axes[axis].max_deg;
        s_axes[axis].target_deg = deg;
    }
}

void servoAxisSetAxesDeg(float pan_deg, float tilt_deg) {
    servoAxisSetTargetDeg(AXIS_PAN, pan_deg);
    servoAxisSetTargetDeg(AXIS_TILT, tilt_deg);
}

void servoAxisUpdate(float dt_s) {
    if (!s_initialized || !(dt_s > 1e-6f)) return;

    for (int i = 0; i < AXIS_COUNT; ++i) {
        AxisRuntime& a = s_axes[i];
        if (!a.initialized) continue;

        const float prev_cmd = a.cmd_deg;
        float desired = a.target_deg;

        // 有反馈时按实测角与目标角的差做一次比例校正，补偿回差与温漂。
        if (servoDrive().hasFeedback()) {
            const float actual = (i == AXIS_PAN) ? servoDrive().readPanDeg() : servoDrive().readTiltDeg();
            if (std::isfinite(actual)) {
                const float err = angleDiffAxisDeg(actual, desired, a.wrap_360);
                desired += SERVO_POS_CORR_KP * err;
            }
            a.state.has_feedback = true;
            a.state.feedback_deg = actual;
        } else {
            a.state.has_feedback = false;
            a.state.feedback_deg = a.cmd_deg;
        }

        // 变化率限制：限制单拍指令跳变，抑制舵机冲击与过冲。
        const float max_step = a.max_slew_dps * dt_s;
        float delta = angleDiffAxisDeg(a.cmd_deg, desired, a.wrap_360);
        if (delta > max_step) delta = max_step;
        if (delta < -max_step) delta = -max_step;

        float cmd = a.cmd_deg + delta;
        bool at_limit = false;
        if (a.wrap_360) {
            cmd = angleWrap360(cmd);
        } else {
            if (cmd <= a.min_deg + SOFT_LIMIT_MARGIN_DEG) at_limit = true;
            if (cmd >= a.max_deg - SOFT_LIMIT_MARGIN_DEG) at_limit = true;
            if (cmd < a.min_deg) cmd = a.min_deg;
            if (cmd > a.max_deg) cmd = a.max_deg;
        }
        a.cmd_deg = cmd;
        writeCommand((uint8_t)i, cmd);

        a.state.position_deg = cmd;
        a.state.velocity_dps = (cmd - prev_cmd) / dt_s;
        a.state.target_deg = a.target_deg;
        a.state.at_limit = at_limit;
    }
}

AxisState servoAxisState(uint8_t axis) {
    if (axis >= AXIS_COUNT) return AxisState();
    return s_axes[axis].state;
}

void servoAxisGetState(AxisState& pan, AxisState& tilt) {
    pan = servoAxisState(AXIS_PAN);
    tilt = servoAxisState(AXIS_TILT);
}

bool servoAxisHasFeedback(uint8_t axis) {
    (void)axis;
    return servoDrive().hasFeedback();
}

void servoAxisFaultSafe() {
    // 先把指令压到安全角并保持力矩；是否再释能由上层按机械阻尼决定。
    servoAxisSetAxesDeg(CONTROL_HOME_PAN_DEG, CONTROL_HOME_TILT_DEG);
    servoDrive().setTorqueEnabled(true);
    if (s_initialized) {
        for (int i = 0; i < AXIS_COUNT; ++i) {
            s_axes[i].cmd_deg = s_axes[i].target_deg;
            writeCommand((uint8_t)i, s_axes[i].cmd_deg);
            s_axes[i].state.position_deg = s_axes[i].cmd_deg;
            s_axes[i].state.velocity_dps = 0.0f;
            s_axes[i].state.target_deg = s_axes[i].target_deg;
        }
    }
}

void servoAxisEmergencyRelease() { servoDrive().emergencyRelease(); }

bool servoAxisInDeadband() {
    return std::fabs(servoAxisPanErrorDeg()) <= PAN_DEADBAND_DEG &&
           std::fabs(servoAxisTiltErrorDeg()) <= TILT_DEADBAND_DEG;
}

float servoAxisPanErrorDeg() {
    if (!s_initialized) return 0.0f;
    return axisErrorDeg(AXIS_PAN);
}

float servoAxisTiltErrorDeg() {
    if (!s_initialized) return 0.0f;
    return axisErrorDeg(AXIS_TILT);
}
