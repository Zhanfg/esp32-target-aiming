#pragma once
// home.h：限幅速度驱向参考方向，等 PAN_HOME/TILT_HOME 稳定触发后退回已知角；
// 3 秒不触发或触发后丢失即失败，确认窗口内开关回跳判为矛盾。

#include <cstdint>

enum class HomeResult { RUNNING, DONE, TIMEOUT, CONFLICT };

void homeInit();
bool homeStart(uint32_t now_ms);
HomeResult homeUpdate(uint32_t now_ms);
bool homeActive();
void homeAbort();
