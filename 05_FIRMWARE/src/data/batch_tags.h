#pragma once
// batch_tags.h：批次追溯编号（膜片、载荷）的持久化。
// 这两个编号不是运行期测量值，来源只能是操作者输入，串口 SET 命令写入后存 NVS，断电不丢。
// 两字段类型都是 uint16，取值 0..65535，0 表示"未设置"。

#include <cstdint>

struct BatchTags {
    uint16_t membrane_id = 0; // 0 = 未设置
    uint16_t payload_id = 0;  // 0 = 未设置
};

// NVS 读写。无记录时 load 返回 false，out 保持默认 0（未设置）。
// save 成功返回 true；任一字段写入失败返回 false。
bool batchTagsLoad(BatchTags& out);
bool batchTagsSave(const BatchTags& in);
