/**
 * telemetry.cpp
 *
 * CSV 表头（§19，共 17 个字段，字段名与顺序不可改）：
 *   MST,timestamp,experiment_id,prototype_version,mechanism_version,mode,boundary_state,
 *       preload_state,membrane_id,payload_id,cycle_count,stage1_event,stage2_event,
 *       mechanism_recovered,magazine_position,magazine_index_ok,fault_code,operator_note
 */

#include "telemetry.h"

#include <Arduino.h>

#include "config.h"

void telemetryInit(uint32_t baud) {
    Serial.begin(baud);
    // 等串口就绪，但绝不阻塞过久：只等很短时间，且 loop 前调用一次无妨。
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 200) {
        delay(1);
    }
    Serial.println();
    Serial.println("# MST telemetry; header:");
    Serial.println("MST,timestamp,experiment_id,prototype_version,mechanism_version,mode,"
                   "boundary_state,preload_state,membrane_id,payload_id,cycle_count,"
                   "stage1_event,stage2_event,mechanism_recovered,magazine_position,"
                   "magazine_index_ok,fault_code,operator_note");
}

static void emitLine(const TelemetryRecord& rec) {
    Serial.printf("MST,%lu,%u,%s,%s,%u,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%u,%u,%s\n",
                  (unsigned long)rec.timestamp,
                  (unsigned)rec.experiment_id,
                  rec.prototype_version ? rec.prototype_version : "-",
                  rec.mechanism_version ? rec.mechanism_version : "-",
                  (unsigned)rec.mode,
                  (unsigned)rec.boundary_state,
                  (unsigned)rec.preload_state,
                  (unsigned)rec.membrane_id,
                  (unsigned)rec.payload_id,
                  (unsigned long)rec.cycle_count,
                  (unsigned)rec.stage1_event,
                  (unsigned)rec.stage2_event,
                  (unsigned)rec.mechanism_recovered,
                  (unsigned)rec.magazine_position,
                  (unsigned)rec.magazine_index_ok,
                  (unsigned)rec.fault_code,
                  rec.operator_note ? rec.operator_note : "-");
}

void telemetryEmit(const TelemetryRecord& rec) {
#if AIM_VERBOSE_TELEMETRY
    // 逐帧刷新时用记录里的时间戳；调用方未填则退回 millis()。
    TelemetryRecord out = rec;
    if (out.timestamp == 0) out.timestamp = millis();
    emitLine(out);
#else
    // 发布固件：不逐帧刷屏，只发低频心跳，证明控制环仍在运行。
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - last_heartbeat_ms) >= TELEMETRY_HEARTBEAT_MS) {
        last_heartbeat_ms = now;
        TelemetryRecord out = rec;
        out.timestamp = now;
        emitLine(out);
    }
#endif
}

void telemetryEmitEvent(const char* tag, const char* msg) {
    // 事件行与数据行用不同前缀区分，PC 端可分别过滤。
    Serial.printf("EVT,%lu,%s,%s\n", (unsigned long)millis(),
                  tag ? tag : "-", msg ? msg : "-");
}
