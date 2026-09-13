/**
 * telemetry.cpp
 * telemetry.h 的实现，用 Arduino Serial 与 config.h。
 *
 * CSV 表头（与 telemetry.h 一致）：
 *   AIM,t_ms,state,obs_valid,px,py,conf,pan_deg,tilt_deg,pan_target,tilt_target,pan_rate,tilt_rate,enc_pan,enc_tilt,loop_us
 */

#include "telemetry.h"

#include <Arduino.h>

#include "config.h"

static uint32_t s_loop_us = 0;

void telemetryInit(uint32_t baud) {
    Serial.begin(baud);
    // 等串口就绪，但绝不阻塞过久：只等很短时间，且 loop 前调用一次无妨。
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 200) {
        delay(1);
    }
    Serial.println();
    Serial.println("# AIM telemetry; header:");
    Serial.println("AIM,t_ms,state,obs_valid,px,py,conf,pan_deg,tilt_deg,"
                   "pan_target,tilt_target,pan_rate,tilt_rate,enc_pan,enc_tilt,loop_us");
}

void telemetrySetLoopUs(uint32_t loop_us) {
    s_loop_us = loop_us;
}

void telemetryEmit(const TargetObservation& obs, const TurretSolution& sol,
                   const AxisState& pan, const AxisState& tilt, AimState state) {
#if AIM_VERBOSE_TELEMETRY
    uint32_t t_ms = (sol.t_ms != 0) ? sol.t_ms : millis();
    Serial.printf(
        "AIM,%lu,%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%lu\n",
        (unsigned long)t_ms,
        (int)state,
        obs.valid ? 1 : 0,
        obs.centroid.x,
        obs.centroid.y,
        obs.centroid.confidence,
        pan.position_deg,
        tilt.position_deg,
        pan.target_deg,
        tilt.target_deg,
        pan.velocity_dps,
        tilt.velocity_dps,
        (long)pan.encoder_count,
        (long)tilt.encoder_count,
        (unsigned long)s_loop_us);
#else
    // 发布固件：不逐帧刷屏，只发低频心跳，证明控制环仍在运行。
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - last_heartbeat_ms) >= TELEMETRY_HEARTBEAT_MS) {
        last_heartbeat_ms = now;
        Serial.printf("AIM,%lu,%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%lu\n",
                      (unsigned long)now,
                      (int)state,
                      obs.valid ? 1 : 0,
                      obs.centroid.x, obs.centroid.y, obs.centroid.confidence,
                      pan.position_deg, tilt.position_deg,
                      pan.target_deg, tilt.target_deg,
                      pan.velocity_dps, tilt.velocity_dps,
                      (long)pan.encoder_count, (long)tilt.encoder_count,
                      (unsigned long)s_loop_us);
    }
    (void)sol;
#endif
}

void telemetryEmitEvent(const char* tag, const char* msg) {
    // 事件行与数据行用不同前缀区分，PC 端可分别过滤。
    Serial.printf("EVT,%lu,%s,%s\n", (unsigned long)millis(),
                  tag ? tag : "-", msg ? msg : "-");
}
