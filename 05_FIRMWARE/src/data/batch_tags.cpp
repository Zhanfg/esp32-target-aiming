/**
 * batch_tags.cpp
 * 用 Preferences(NVS) 存取膜片与载荷的批次编号，风格对齐 calibration.cpp。
 */

#include "batch_tags.h"

#include <Preferences.h>

namespace {
// NVS 命名空间与键名固定，首次烧录后不再变更。键名改动会让旧记录读不回。
// 命名空间：aim_batch
//   键 membrane：膜片批次/试样编号，uint16，0 = 未设置
//   键 payload ：载荷批次/编号，uint16，0 = 未设置
constexpr const char* kNamespace = "aim_batch";
constexpr const char* kKeyMembrane = "membrane";
constexpr const char* kKeyPayload = "payload";
} // namespace

bool batchTagsLoad(BatchTags& out) {
    out = BatchTags(); // 默认 0，0 即"未设置"
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/true)) {
        return false;
    }
    // 两个键任一缺失都当无记录处理，避免半写状态被当成有效配置。
    if (!prefs.isKey(kKeyMembrane) || !prefs.isKey(kKeyPayload)) {
        prefs.end();
        return false;
    }
    out.membrane_id = prefs.getUShort(kKeyMembrane, 0);
    out.payload_id = prefs.getUShort(kKeyPayload, 0);
    prefs.end();
    return true;
}

bool batchTagsSave(const BatchTags& in) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
        return false;
    }
    bool ok = true;
    ok &= prefs.putUShort(kKeyMembrane, in.membrane_id) > 0;
    ok &= prefs.putUShort(kKeyPayload, in.payload_id) > 0;
    prefs.end();
    return ok;
}
