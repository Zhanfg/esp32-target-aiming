/**
 * motor_driver.cpp
 * TB6612FNG 驱动实现：LEDC 出 PWM，GPIO 出方向。引脚与上限来自 config.h。
 *
 * TB6612 真值表（PWM 为高时有效）：
 *   IN1=1 IN2=0  正转（CW）
 *   IN1=0 IN2=1  反转（CCW）
 *   IN1=1 IN2=1  短路制动
 *   IN1=0 IN2=0  滑行/停止
 *   STBY=0       两路同时关闭
 */

#include "motor_driver.h"

#include <Arduino.h>
#include <driver/ledc.h>
#include <cmath>

#include "config.h"

// LEDC 参考时钟上限（ESP32-S3 APB 为 80MHz）。用于推算给定频率下可用的最高分辨率：
// 需要满足 2^res <= clk / freq（分频系数 >= 1）。
static const uint32_t LEDC_SRC_CLK_HZ = 80000000UL;

// 每轴硬件参数表：集中描述引脚、LEDC 资源与极性，避免 if/else 散落。
struct MotorChannel {
    int in1_pin;
    int in2_pin;
    int pwm_pin;
    ledc_channel_t ledc_channel;
    ledc_timer_t ledc_timer;
    bool invert; // true 表示软件反向逻辑方向
};

static const MotorChannel kMotors[AXIS_COUNT] = {
    // pan（A 路）
    { PAN_AIN1_PIN, PAN_AIN2_PIN, PAN_PWM_PIN,
      (ledc_channel_t)PAN_LEDC_CHANNEL, (ledc_timer_t)PAN_LEDC_TIMER,
      MOTOR_INVERT_PAN != 0 },
    // tilt（B 路）
    { TILT_AIN1_PIN, TILT_AIN2_PIN, TILT_PWM_PIN,
      (ledc_channel_t)TILT_LEDC_CHANNEL, (ledc_timer_t)TILT_LEDC_TIMER,
      MOTOR_INVERT_TILT != 0 },
};

static bool s_initialized = false;
static uint8_t s_resolution_bits = MOTOR_PWM_RESOLUTION_BITS; // 实际生效的分辨率

// 计算给定 PWM 频率下 LEDC 真正支持的最高分辨率，并受 config 目标值封顶。
static uint8_t computeResolutionBits(uint32_t freq_hz) {
    if (freq_hz == 0) return 1;
    uint32_t limit = LEDC_SRC_CLK_HZ / freq_hz; // 允许的最大计数步数 = 2^res
    uint8_t bits = 1;
    while (bits < MOTOR_PWM_RESOLUTION_BITS &&
           (uint32_t)(1UL << (bits + 1)) <= limit) {
        bits++;
    }
    return bits;
}

// 写入某轴 PWM 占空比（已是 0~1 的正幅度，方向另行处理）。
static inline void writePwm(const MotorChannel& m, float magnitude) {
    uint32_t max_duty = (1UL << s_resolution_bits) - 1UL;
    uint32_t value = (uint32_t)(magnitude * (float)max_duty + 0.5f);
    if (value > max_duty) value = max_duty;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m.ledc_channel, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m.ledc_channel);
}

bool motorDriverInit() {
    pinMode(TB6612_STBY_PIN, OUTPUT);
    // 初始先关闭输出，等所有配置到位后再由 motorDriverSetStandby(true) 使能。
    motorDriverSetStandby(false);

    for (int i = 0; i < AXIS_COUNT; ++i) {
        const MotorChannel& m = kMotors[i];
        pinMode(m.in1_pin, OUTPUT);
        pinMode(m.in2_pin, OUTPUT);
        digitalWrite(m.in1_pin, LOW);
        digitalWrite(m.in2_pin, LOW);
    }

    // 分辨率回退：16 位 @20kHz 需要 80e6/2^16=1220Hz，物理上做不到。
    // 这里按频率推算实际可用分辨率，保证 PWM 频率优先（避开可听频段）。
    uint8_t res = computeResolutionBits(MOTOR_PWM_FREQ_HZ);
    s_resolution_bits = res;

    for (int i = 0; i < AXIS_COUNT; ++i) {
        const MotorChannel& m = kMotors[i];
        ledc_timer_config_t tcfg = {};
        tcfg.speed_mode = LEDC_LOW_SPEED_MODE;
        tcfg.duty_resolution = (ledc_timer_bit_t)res;
        tcfg.timer_num = m.ledc_timer;
        tcfg.freq_hz = MOTOR_PWM_FREQ_HZ;
        tcfg.clk_cfg = LEDC_AUTO_CLK;
        if (ledc_timer_config(&tcfg) != ESP_OK) {
            Serial.printf("[motor] LEDC 定时器配置失败 (axis=%d)\n", i);
            return false;
        }

        ledc_channel_config_t ccfg = {};
        ccfg.gpio_num = m.pwm_pin;
        ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
        ccfg.channel = m.ledc_channel;
        ccfg.timer_sel = m.ledc_timer;
        ccfg.duty = 0;
        ccfg.hpoint = 0;
        if (ledc_channel_config(&ccfg) != ESP_OK) {
            Serial.printf("[motor] LEDC 通道配置失败 (axis=%d)\n", i);
            return false;
        }
    }

    if (res != MOTOR_PWM_RESOLUTION_BITS) {
        Serial.printf("[motor] PWM %uHz 下无法达到 %u 位分辨率，实际 %u 位"
                      "（LEDC 时钟上限 %luHz）\n",
                      (unsigned)MOTOR_PWM_FREQ_HZ,
                      (unsigned)MOTOR_PWM_RESOLUTION_BITS,
                      (unsigned)res,
                      (unsigned long)LEDC_SRC_CLK_HZ);
    }

    s_initialized = true;
    motorDriverSetStandby(true);
    for (int i = 0; i < AXIS_COUNT; ++i) {
        motorCoast(i);
    }
    return true;
}

void motorSetDuty(uint8_t axis, float duty) {
    if (!s_initialized || axis >= AXIS_COUNT) {
        return;
    }
    const MotorChannel& m = kMotors[axis];

    // 1) 上限保护：占空比过高会让电机与 TB6612 过流、过热。这里引用全局上限。
    float mag = std::fabs(duty);
    if (mag > MOTOR_PWM_MAX_DUTY) {
        mag = MOTOR_PWM_MAX_DUTY;
    }

    // 2) 零附近处理：完全停止时滑行，避免持续小电流堵转发热。
    if (mag <= MOTOR_PWM_MIN_DUTY) {
        motorCoast(axis);
        return;
    }

    // 3) 死区补偿：低于静态摩擦阈值的占空比驱动不了电机，若确实要动就补到阈值。
    //    目的：小误差时也能起步，减少"给指令却不动"的静差。
    if (mag < MOTOR_DUTY_DEADBAND) {
        mag = MOTOR_DUTY_DEADBAND;
    }

    // 4) 方向判定：先按逻辑符号，再套用该轴极性反转。
    bool forward = (duty >= 0.0f);
    if (m.invert) {
        forward = !forward;
    }
    digitalWrite(m.in1_pin, forward ? HIGH : LOW);
    digitalWrite(m.in2_pin, forward ? LOW : HIGH);

    // 5) 幅度 → 寄存器值：(1<<res)-1 对应 100%。
    writePwm(m, mag);
}

void motorBrake(uint8_t axis) {
    if (!s_initialized || axis >= AXIS_COUNT) {
        motorCoast(axis);
        return;
    }
    const MotorChannel& m = kMotors[axis];
    digitalWrite(m.in1_pin, HIGH);
    digitalWrite(m.in2_pin, HIGH);
    writePwm(m, 0.0f);
}

void motorCoast(uint8_t axis) {
    if (!s_initialized || axis >= AXIS_COUNT) {
        return;
    }
    const MotorChannel& m = kMotors[axis];
    digitalWrite(m.in1_pin, LOW);
    digitalWrite(m.in2_pin, LOW);
    writePwm(m, 0.0f);
}

void motorDriverSetStandby(bool enable) {
    // 通过 AXIS 宏支持正/负逻辑，不同底板 STBY 可能经反相器。
    bool level = (TB6612_STBY_ACTIVE_LEVEL != 0) ? enable : !enable;
    digitalWrite(TB6612_STBY_PIN, level ? HIGH : LOW);
}
