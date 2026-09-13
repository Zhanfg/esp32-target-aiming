#pragma once
/**
 * telemetry.h
 * 串口 CSV 遥测。数据行前缀 MST,，字段与顺序按规格书 §19 的 17 项，便于 PC 端脚本解析：
 *   MST,timestamp,experiment_id,prototype_version,mechanism_version,mode,boundary_state,
 *       preload_state,membrane_id,payload_id,cycle_count,stage1_event,stage2_event,
 *       mechanism_recovered,magazine_position,magazine_index_ok,fault_code,operator_note
 * 单位：时间毫秒；字符串字段不得含逗号。事件行 EVT,<t_ms>,<tag>,<msg>。
 * AIM_VERBOSE_TELEMETRY==0 时只留低频心跳。
 */

#include <cstdint>

struct TelemetryRecord {
    uint32_t timestamp = 0;
    uint16_t experiment_id = 0;
    const char* prototype_version = "";
    const char* mechanism_version = "";
    uint8_t mode = 0;               // FireMode 整数值
    uint8_t boundary_state = 0;
    uint8_t preload_state = 0;      // 0=空闲 1=预载中 2=已武装
    uint16_t membrane_id = 0;
    uint16_t payload_id = 0;
    uint32_t cycle_count = 0;
    uint8_t stage1_event = 0;
    uint8_t stage2_event = 0;
    uint8_t mechanism_recovered = 0;
    uint8_t magazine_position = 0;
    uint8_t magazine_index_ok = 0;
    uint16_t fault_code = 0;        // FaultCode 整数值
    const char* operator_note = "-";
};

void telemetryInit(uint32_t baud);
void telemetryEmit(const TelemetryRecord& rec);
void telemetryEmitEvent(const char* tag, const char* msg);
