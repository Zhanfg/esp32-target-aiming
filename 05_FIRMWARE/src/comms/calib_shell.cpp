/**
 * calib_shell.cpp
 * calib_shell.h 的实现。命令解析与点表管理在本文件，控制动作经 CalibShellHooks 转发。
 */

#include "calib_shell.h"

#include <Arduino.h>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"

namespace {

constexpr int kMaxPoints = 32;
constexpr size_t kMaxLineLen = 96;
constexpr size_t kMaxLabelLen = 16;

// 留出验证抽取步长：按记录顺序每第 3 个点进留出集，其余参与解算。
constexpr int kHoldoutStride = 3;

struct CalibPoint {
    float px;
    float py;
    float pan_deg;
    float tilt_deg;
    char label[kMaxLabelLen + 1];
};

CalibPoint s_points[kMaxPoints];
int s_count = 0;

CalibShellHooks s_hooks;
bool s_ready = false;

char s_line[kMaxLineLen + 1];
size_t s_line_len = 0;
bool s_overflow = false;

void replyOk(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.printf("OK,%s\n", buf);
}

void replyErr(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    Serial.printf("ERR,%s\n", buf);
}

const char* stateName(AimState s) {
    switch (s) {
        case AimState::IDLE:        return "IDLE";
        case AimState::CALIBRATING: return "CALIBRATING";
        case AimState::SEARCHING:   return "SEARCHING";
        case AimState::TRACKING:    return "TRACKING";
        case AimState::LOCKED:      return "LOCKED";
        case AimState::CALIB_MODE:  return "CALIB_MODE";
    }
    return "UNKNOWN";
}

const char* cycleName(CycleState s) {
    switch (s) {
        case CycleState::BOOT:        return "BOOT";
        case CycleState::SAFE:        return "SAFE";
        case CycleState::HOME:        return "HOME";
        case CycleState::READY:       return "READY";
        case CycleState::AIM_ALLOWED: return "AIM_ALLOWED";
        case CycleState::PRELOAD:     return "PRELOAD";
        case CycleState::ARMED:       return "ARMED";
        case CycleState::RELEASE:     return "RELEASE";
        case CycleState::RECOVER:     return "RECOVER";
        case CycleState::INDEX:       return "INDEX";
        case CycleState::FAULT:       return "FAULT";
    }
    return "UNKNOWN";
}

bool inCalMode() {
    return s_hooks.getState && s_hooks.getState() == AimState::CALIB_MODE;
}

// 就地取下一个空白分隔 token，写 '\0' 截断。返回 nullptr 表示已到行尾。
char* nextToken(char*& p) {
    while (*p == ' ' || *p == '\t') ++p;
    if (*p == '\0') return nullptr;
    char* start = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
    if (*p != '\0') {
        *p = '\0';
        ++p;
    }
    return start;
}

void toUpper(char* s) {
    for (; *s; ++s) *s = (char)toupper((unsigned char)*s);
}

bool parseFloat(const char* s, float& out) {
    if (!s || *s == '\0') return false;
    char* end = nullptr;
    errno = 0;
    double v = strtod(s, &end);
    if (end == s || *end != '\0') return false;  // 带非数字后缀也算畸形
    if (!std::isfinite(v)) return false;
    out = (float)v;
    return true;
}

bool parseInt(const char* s, long& out) {
    if (!s || *s == '\0') return false;
    char* end = nullptr;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    out = v;
    return true;
}

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void printHelp() {
    Serial.println("CMD,HELP,列出命令");
    Serial.println("CMD,STATUS,打印状态/内参/仿射");
    Serial.println("CMD,CAL START,进入标定模式（暂停自动跟踪，发射器强制 SAFE）");
    Serial.println("CMD,CAL JOG <pan_deg> <tilt_deg>,设两轴目标角，仍走双环 PID 与限位");
    Serial.println("CMD,CAL MARK [label],记录当前像素观测 + 当前实际角");
    Serial.println("CMD,CAL LIST,列出点表");
    Serial.println("CMD,CAL DEL <n>,删除第 n 个点（序号从 1 开始）");
    Serial.println("CMD,CAL CLEAR,清空点表");
    Serial.println("CMD,CAL SOLVE,最小二乘解算并报告训练/留出 RMSE");
    Serial.println("CMD,CAL SAVE,把当前仿射写入 NVS");
    Serial.println("CMD,CAL LOAD,从 NVS 读回并应用");
    Serial.println("CMD,CAL EXIT,退出标定模式回 SEARCHING");
    Serial.println("CMD,ESTOP,急停：切断舵机与释放链并锁存 FAULT");
    Serial.println("CMD,CLEAR,清除软故障并送 SAFE（硬故障无效，唯一恢复路径 SAFE->HOME）");
}

void printStatus() {
    AimState st = s_hooks.getState ? s_hooks.getState() : AimState::IDLE;
    CycleState cy = s_hooks.getCycleState ? s_hooks.getCycleState() : CycleState::FAULT;
    AxisState pan, tilt;
    if (s_hooks.getAxes) s_hooks.getAxes(pan, tilt);

    Serial.printf("ST,state,%d,%s\n", (int)st, stateName(st));
    Serial.printf("ST,cycle,%d,%s\n", (int)cy, cycleName(cy));
    if (s_hooks.getFaultCode) {
        Serial.printf("ST,fault_code,%d\n", (int)s_hooks.getFaultCode());
    }
    Serial.printf("ST,cal_mode,%d\n", inCalMode() ? 1 : 0);

    const CalibrationData* cal = s_hooks.calibration ? s_hooks.calibration() : nullptr;
    if (!cal) {
        Serial.println("ST,cal,none");
    } else {
        Serial.printf("ST,cal_valid,%d\n", cal->valid ? 1 : 0);
        Serial.printf("ST,image,%d,%d\n", cal->width, cal->height);
        Serial.printf("ST,intrinsic,%.3f,%.3f,%.3f,%.3f\n",
                      cal->fx, cal->fy, cal->cx, cal->cy);
        Serial.printf("ST,dist,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                      cal->dist[0], cal->dist[1], cal->dist[2], cal->dist[3], cal->dist[4]);
        Serial.printf("ST,affine,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                      cal->affine[0][0], cal->affine[0][1], cal->affine[0][2],
                      cal->affine[1][0], cal->affine[1][1], cal->affine[1][2]);
        Serial.printf("ST,offset,%.3f,%.3f\n", cal->pan_offset_deg, cal->tilt_offset_deg);
    }

    Serial.printf("ST,pan,pos=%.3f,tgt=%.3f,fb=%.3f\n",
                  pan.position_deg, pan.target_deg, pan.feedback_deg);
    Serial.printf("ST,tilt,pos=%.3f,tgt=%.3f,fb=%.3f\n",
                  tilt.position_deg, tilt.target_deg, tilt.feedback_deg);
    Serial.printf("ST,points,%d/%d\n", s_count, kMaxPoints);
}

void cmdStart() {
    if (!s_hooks.setCalibrationMode) {
        replyErr("标定接口未注入");
        return;
    }
    if (inCalMode()) {
        replyErr("已在标定模式");
        return;
    }
    // 标定只能在 READY 及以下发起；PRELOAD/ARMED 等含储能态先卸载残余能量。
    CycleState cy = s_hooks.getCycleState ? s_hooks.getCycleState() : CycleState::FAULT;
    if (cy != CycleState::READY) {
        replyErr("只能在 READY 进入标定模式（当前 %s）", cycleName(cy));
        return;
    }
    if (!s_hooks.setCalibrationMode(true)) {
        replyErr("进入标定模式被拒绝（FAULT 或未初始化）");
        return;
    }
    replyOk("CAL START 已进入标定模式：自动跟踪暂停，发射器已强制 SAFE");
}

void cmdExit() {
    if (!s_hooks.setCalibrationMode) {
        replyErr("标定接口未注入");
        return;
    }
    if (!inCalMode()) {
        replyErr("当前不在标定模式");
        return;
    }
    s_hooks.setCalibrationMode(false);
    replyOk("CAL EXIT 已退出标定模式");
}

void cmdJog(char* p) {
    if (!inCalMode()) {
        replyErr("先 CAL START 进入标定模式");
        return;
    }
    char* a = nextToken(p);
    char* b = nextToken(p);
    if (!a || !b) {
        replyErr("参数个数不对，用法 CAL JOG <pan_deg> <tilt_deg>");
        return;
    }
    if (nextToken(p)) {
        replyErr("参数过多，用法 CAL JOG <pan_deg> <tilt_deg>");
        return;
    }
    float pan, tilt;
    if (!parseFloat(a, pan) || !parseFloat(b, tilt)) {
        replyErr("参数非数字");
        return;
    }

    float pan_c = clampf(pan, PAN_MIN_DEG, PAN_MAX_DEG);
    float tilt_c = clampf(tilt, TILT_MIN_DEG, TILT_MAX_DEG);
    s_hooks.jog(pan_c, tilt_c);

    if (pan_c != pan || tilt_c != tilt) {
        replyOk("CAL JOG 目标已夹到限位: pan=%.3f tilt=%.3f", pan_c, tilt_c);
    } else {
        replyOk("CAL JOG pan=%.3f tilt=%.3f", pan_c, tilt_c);
    }
}

void cmdMark(char* p) {
    if (!inCalMode()) {
        replyErr("先 CAL START 进入标定模式");
        return;
    }
    if (s_count >= kMaxPoints) {
        replyErr("点表已满（%d），先 CAL DEL 或 CAL CLEAR", kMaxPoints);
        return;
    }
    TargetObservation obs;
    if (!s_hooks.getObservation) {
        replyErr("观测接口未注入");
        return;
    }
    s_hooks.getObservation(obs);
    if (!obs.valid) {
        replyErr("当前无有效观测（obs.valid=false），拒绝记录");
        return;
    }
    AxisState pan, tilt;
    s_hooks.getAxes(pan, tilt);

    CalibPoint& pt = s_points[s_count];
    pt.px = obs.centroid.x;
    pt.py = obs.centroid.y;
    pt.pan_deg = pan.position_deg;
    pt.tilt_deg = tilt.position_deg;
    pt.label[0] = '\0';
    char* label = nextToken(p);
    if (label) {
        strncpy(pt.label, label, kMaxLabelLen);
        pt.label[kMaxLabelLen] = '\0';
    }
    ++s_count;
    replyOk("CAL MARK #%d px=%.2f py=%.2f pan=%.3f tilt=%.3f conf=%.3f label=%s",
            s_count, pt.px, pt.py, pt.pan_deg, pt.tilt_deg,
            obs.centroid.confidence, pt.label[0] ? pt.label : "-");
}

void cmdList() {
    Serial.printf("LIST,count,%d/%d\n", s_count, kMaxPoints);
    for (int i = 0; i < s_count; ++i) {
        const CalibPoint& pt = s_points[i];
        Serial.printf("PT,%d,%.2f,%.2f,%.3f,%.3f,%s\n",
                      i + 1, pt.px, pt.py, pt.pan_deg, pt.tilt_deg,
                      pt.label[0] ? pt.label : "-");
    }
}

void cmdDel(char* p) {
    if (!inCalMode()) {
        replyErr("先 CAL START 进入标定模式");
        return;
    }
    char* a = nextToken(p);
    if (!a || nextToken(p)) {
        replyErr("参数个数不对，用法 CAL DEL <n>");
        return;
    }
    long n = 0;
    if (!parseInt(a, n)) {
        replyErr("序号非整数");
        return;
    }
    if (n < 1 || n > s_count) {
        replyErr("序号越界，有效范围 1~%d", s_count);
        return;
    }
    int idx = (int)n - 1;
    for (int i = idx; i < s_count - 1; ++i) {
        s_points[i] = s_points[i + 1];
    }
    --s_count;
    replyOk("已删除第 %ld 个，剩余 %d 个", n, s_count);
}

void cmdClear() {
    if (!inCalMode()) {
        replyErr("先 CAL START 进入标定模式");
        return;
    }
    s_count = 0;
    replyOk("点表已清空");
}

void cmdSolve() {
    if (!inCalMode()) {
        replyErr("先 CAL START 进入标定模式");
        return;
    }
    if (s_count < 3) {
        replyErr("点数不足，至少 3 个（当前 %d）", s_count);
        return;
    }

    float train_pix[2 * kMaxPoints];
    float train_ang[2 * kMaxPoints];
    float hold_pix[2 * kMaxPoints];
    float hold_ang[2 * kMaxPoints];
    int nt = 0, nh = 0;
    for (int i = 0; i < s_count; ++i) {
        const CalibPoint& pt = s_points[i];
        bool hold = (i % kHoldoutStride) == (kHoldoutStride - 1);
        float* dst_pix = hold ? hold_pix : train_pix;
        float* dst_ang = hold ? hold_ang : train_ang;
        int j = hold ? nh : nt;
        dst_pix[2 * j] = pt.px;
        dst_pix[2 * j + 1] = pt.py;
        dst_ang[2 * j] = pt.pan_deg;
        dst_ang[2 * j + 1] = pt.tilt_deg;
        if (hold) ++nh; else ++nt;
    }
    // 训练点不足 3 时退化为全点解算，此时没有独立留出集，不报留出 RMSE。
    if (nt < 3) {
        nt = s_count;
        nh = 0;
        for (int i = 0; i < s_count; ++i) {
            train_pix[2 * i] = s_points[i].px;
            train_pix[2 * i + 1] = s_points[i].py;
            train_ang[2 * i] = s_points[i].pan_deg;
            train_ang[2 * i + 1] = s_points[i].tilt_deg;
        }
    }

    // 点对里的角度是 MARK 时两轴的实际角，因此这里拟合的是像素 -> pan/tilt 直接映射，
    // 零点偏移保持 0，与 main 的指向跟踪把仿射输出当作相对修正量的用法一致。
    float affine[2][3];
    float rmse_train[2];
    if (!calibrationSolveAffine(train_pix, train_ang, nt, affine, rmse_train)) {
        replyErr("解算失败：点退化或共线，法方程奇异");
        return;
    }

    float rmse_hold[2] = {0.0f, 0.0f};
    if (nh > 0) {
        double se_p = 0.0, se_t = 0.0;
        for (int j = 0; j < nh; ++j) {
            double px = hold_pix[2 * j], py = hold_pix[2 * j + 1];
            double pred_pan = affine[0][0] * px + affine[0][1] * py + affine[0][2];
            double pred_tilt = affine[1][0] * px + affine[1][1] * py + affine[1][2];
            double dp = (double)hold_ang[2 * j] - pred_pan;
            double dt = (double)hold_ang[2 * j + 1] - pred_tilt;
            se_p += dp * dp;
            se_t += dt * dt;
        }
        rmse_hold[0] = (float)std::sqrt(se_p / nh);
        rmse_hold[1] = (float)std::sqrt(se_t / nh);
    }

    CalibrationData* cal = s_hooks.calibration ? s_hooks.calibration() : nullptr;
    if (cal) {
        cal->width = VISION_SRC_W;
        cal->height = VISION_SRC_H;
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 3; ++c) cal->affine[r][c] = affine[r][c];
        cal->pan_offset_deg = 0.0f;
        cal->tilt_offset_deg = 0.0f;
        cal->valid = true;
        if (s_hooks.applyCalibration) s_hooks.applyCalibration();
    }

    Serial.printf("SOLVE,points,train=%d,hold=%d\n", nt, nh);
    Serial.printf("SOLVE,affine,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                  affine[0][0], affine[0][1], affine[0][2],
                  affine[1][0], affine[1][1], affine[1][2]);
    Serial.printf("SOLVE,rmse_train,%.4f,%.4f\n", rmse_train[0], rmse_train[1]);
    if (nh > 0) {
        Serial.printf("SOLVE,rmse_hold,%.4f,%.4f\n", rmse_hold[0], rmse_hold[1]);
    } else {
        Serial.println("SOLVE,rmse_hold,NA（训练点不足 3，未留出）");
    }
    float worst = rmse_train[0] > rmse_train[1] ? rmse_train[0] : rmse_train[1];
    if (nh > 0) {
        if (rmse_hold[0] > worst) worst = rmse_hold[0];
        if (rmse_hold[1] > worst) worst = rmse_hold[1];
    }
    const char* grade = worst < 0.2f ? "优秀" :
                        worst < 0.5f ? "可用" :
                        worst < 1.5f ? "偏大" : "不可用";
    Serial.printf("SOLVE,grade,%.4f,%s\n", worst, grade);
    replyOk("CAL SOLVE 完成，仿射已应用（未写 NVS，需要则 CAL SAVE）");
}

void cmdSave() {
    CalibrationData* cal = s_hooks.calibration ? s_hooks.calibration() : nullptr;
    if (!cal) {
        replyErr("标定数据未注入");
        return;
    }
    if (!cal->valid) {
        replyErr("当前仿射无效（先 CAL SOLVE），拒绝写入 NVS");
        return;
    }
    if (!calibrationSave(*cal)) {
        replyErr("写入 NVS 失败");
        return;
    }
    replyOk("CAL SAVE 已写入 NVS");
}

void cmdLoad() {
    CalibrationData tmp;
    if (!calibrationLoad(tmp)) {
        replyErr("NVS 无标定记录");
        return;
    }
    CalibrationData* cal = s_hooks.calibration ? s_hooks.calibration() : nullptr;
    if (!cal) {
        replyErr("标定数据未注入");
        return;
    }
    *cal = tmp;
    if (s_hooks.applyCalibration) s_hooks.applyCalibration();
    Serial.printf("LOAD,valid=%d,image=%dx%d,intrinsic=%.3f,%.3f,%.3f,%.3f\n",
                  cal->valid ? 1 : 0, cal->width, cal->height,
                  cal->fx, cal->fy, cal->cx, cal->cy);
    replyOk("CAL LOAD 已读回并应用");
}

void cmdEstop() {
    if (s_hooks.emergencyStop) s_hooks.emergencyStop();
    replyOk("ESTOP 已触发：舵机与释放链切断，FAULT 锁存");
}

void cmdFaultClear() {
    if (!s_hooks.clearFault) {
        replyErr("清除接口未注入");
        return;
    }
    if (s_hooks.clearFault()) {
        replyOk("软故障已清除，系统送 SAFE，随后自动重新 HOME");
    } else {
        replyErr("清除被拒绝：无故障或属硬故障（软件清除无效，需断电/人工处理）");
    }
}

void dispatchCal(char* p) {
    char* sub = nextToken(p);
    if (!sub) {
        replyErr("CAL 缺少子命令，输入 HELP");
        return;
    }
    toUpper(sub);
    if (strcmp(sub, "START") == 0) cmdStart();
    else if (strcmp(sub, "JOG") == 0) cmdJog(p);
    else if (strcmp(sub, "MARK") == 0) cmdMark(p);
    else if (strcmp(sub, "LIST") == 0) cmdList();
    else if (strcmp(sub, "DEL") == 0) cmdDel(p);
    else if (strcmp(sub, "CLEAR") == 0) cmdClear();
    else if (strcmp(sub, "SOLVE") == 0) cmdSolve();
    else if (strcmp(sub, "SAVE") == 0) cmdSave();
    else if (strcmp(sub, "LOAD") == 0) cmdLoad();
    else if (strcmp(sub, "EXIT") == 0) cmdExit();
    else replyErr("未知子命令 \"%s\"，输入 HELP", sub);
}

void processLine(char* line) {
    char* p = line;
    char* cmd = nextToken(p);
    if (!cmd) return;
    toUpper(cmd);

    if (strcmp(cmd, "HELP") == 0) printHelp();
    else if (strcmp(cmd, "STATUS") == 0) printStatus();
    else if (strcmp(cmd, "ESTOP") == 0) cmdEstop();
    else if (strcmp(cmd, "CLEAR") == 0) cmdFaultClear();
    else if (strcmp(cmd, "CAL") == 0) dispatchCal(p);
    else replyErr("未知命令 \"%s\"，输入 HELP", cmd);
}

void readSerial() {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        char ch = (char)c;

        if (ch == '\n' || ch == '\r') {
            if (s_overflow) {
                s_overflow = false;
                s_line_len = 0;
                replyErr("行过长已丢弃（上限 %u 字符）", (unsigned)kMaxLineLen);
                continue;
            }
            if (s_line_len == 0) continue;  // 空行与 CRLF 的第二个字符
            s_line[s_line_len] = '\0';
            processLine(s_line);
            s_line_len = 0;
            continue;
        }

        if (s_overflow) continue;
        if (s_line_len >= kMaxLineLen) {
            s_overflow = true;  // 继续吃到行尾再报错，避免把残行当新命令
            continue;
        }
        s_line[s_line_len++] = ch;
    }
}

} // namespace

bool calibShellInit(const CalibShellHooks& hooks) {
    s_hooks = hooks;
    s_ready = hooks.getObservation && hooks.getAxes && hooks.getState &&
              hooks.getCycleState && hooks.calibration && hooks.applyCalibration &&
              hooks.setCalibrationMode && hooks.jog && hooks.emergencyStop &&
              hooks.clearFault;
    s_count = 0;
    s_line_len = 0;
    s_overflow = false;
    return s_ready;
}

void calibShellPoll() {
    if (!s_ready) return;
    readSerial();
}
