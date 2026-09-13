#pragma once
/**
 * home.h
 * 上电归零流程，五步对应架构文档 §6.10：
 *   1) 归零期间释放链处于禁止侧（由 main 保证，本模块只驱动两轴）；
 *   2) 以限幅速度驱向参考方向，等 PAN_HOME/TILT_HOME 触发；
 *   3) 触发后记为物理基准，退回已知角度作为 HOME；3 秒不触发或触发后丢失即失败；
 *   4) 全程区域 A = HOME、区域 B = CALIBRATING；
 *   5) 完成后由 main 校验标定数据再进 READY。
 * 无反馈舵机的"触发后丢失"判据是参考开关在确认窗口内又变回未触发。
 */

#include <cstdint>

enum class HomeResult { RUNNING, DONE, TIMEOUT, CONFLICT };

void homeInit();
bool homeStart(uint32_t now_ms);
HomeResult homeUpdate(uint32_t now_ms);
bool homeActive();
void homeAbort();
