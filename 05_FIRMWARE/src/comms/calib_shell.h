#pragma once
// 串口行命令标定外壳，与 telemetry 共用 Serial（telemetry 只写，本模块只读）。
// 命令每行一条，不分大小写；CAL JOG 走 turntable 保持路径，限速与软限位仍生效。
// calibShellPoll() 在 loop() 每拍非阻塞轮询，不做任何等待。
// 状态读写经 CalibShellHooks 注入，comms 层不反向依赖 main。

#include <cstdint>

#include "../aim_types.h"
#include "../calibration/calibration.h"

struct CalibShellHooks {
    void (*getObservation)(TargetObservation& out) = nullptr;
    void (*getAxes)(AxisState& pan, AxisState& tilt) = nullptr;
    AimState (*getState)() = nullptr;
    CalibrationData* (*calibration)() = nullptr;        // 指向 main 当前标定，SOLVE/LOAD 就地改写
    void (*applyCalibration)() = nullptr;               // 标定改写后刷新 cal_ok 并注入 turntable
    bool (*setCalibrationMode)(bool enable) = nullptr;  // true 进入，false 退出
    void (*jog)(float pan_deg, float tilt_deg) = nullptr;
    void (*emergencyStop)() = nullptr;
};

// 任一回调为空则本模块禁用。返回 false 表示注入不完整。
bool calibShellInit(const CalibShellHooks& hooks);

void calibShellPoll();
