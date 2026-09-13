#pragma once
/**
 * config.h
 * 全局编译期配置：引脚、机械参数、控制器增益、通信参数。
 * 标注 "待实测标定" 的都是占位默认值，首次上电前按实物核对或整定。
 *
 * 全工程单向依赖本文件；硬件或整定值变化只改这里，代码里不再散落这些数字。
 */

// 构建版本：烧进镜像，启动横幅与遥测首行会打印，用于确认板上跑的是哪一版。
#define AIM_BUILD_VERSION "0.1.0"

// 轴索引：全工程统一用它做数组下标/接口参数，避免散落 0/1 字面量。
#define AXIS_PAN   0
#define AXIS_TILT  1
#define AXIS_COUNT 2

// ===== CAMERA PINS: OV2640/OV3660 等 DVP 摄像头并口 =====
// Y2..Y9 对应 esp32-camera 的 D0..D7，顺序不可打乱。不同 S3-CAM 板的线序差异极大，
// 接错不会烧板，但会黑屏/花屏，首次烧录前对照原理图确认。
#define CAM_PIN_PWDN        (-1)   // 待实测标定：-1 表示未接
#define CAM_PIN_RESET       (-1)   // 待实测标定：-1 表示未接
#define CAM_PIN_XCLK        15     // 待实测标定
#define CAM_PIN_SIOD        4      // 待实测标定
#define CAM_PIN_SIOC        5      // 待实测标定
#define CAM_PIN_Y9          16     // 待实测标定
#define CAM_PIN_Y8          17     // 待实测标定
#define CAM_PIN_Y7          18     // 待实测标定
#define CAM_PIN_Y6          12     // 待实测标定
#define CAM_PIN_Y5          10     // 待实测标定
#define CAM_PIN_Y4          8      // 待实测标定
#define CAM_PIN_Y3          9      // 待实测标定
#define CAM_PIN_Y2          11     // 待实测标定
#define CAM_PIN_VSYNC       6      // 待实测标定
#define CAM_PIN_HREF        7      // 待实测标定
#define CAM_PIN_PCLK        13     // 待实测标定

// 20MHz 是 OV2640 常用稳定值，提高会增大 EMI 与走线要求。
#define CAM_XCLK_FREQ_HZ    20000000

// XCLK 走 LEDC timer2/channel2。电机 PWM 已占 timer0/1 的 channel0/1，分开避免抢占。
#define CAM_LEDC_CHANNEL    2
#define CAM_LEDC_TIMER      2

// ===== MOTOR PAN: TB6612FNG A 路驱动 pan（水平） =====
#define PAN_AIN1_PIN        1
#define PAN_AIN2_PIN        2
#define PAN_PWM_PIN         14
#define PAN_LEDC_CHANNEL    0
#define PAN_LEDC_TIMER      0

// ===== MOTOR TILT: TB6612FNG B 路驱动 tilt（俯仰） =====
#define TILT_AIN1_PIN       47
#define TILT_AIN2_PIN       48
#define TILT_PWM_PIN        21
#define TILT_LEDC_CHANNEL   1
#define TILT_LEDC_TIMER     1

// 方向极性：若"正指令 → 负方向运动"就把对应宏置 1 软件反向，不必改接线。
// MOTOR_INVERT_* 与 ENCODER_INVERT_* 只能改其一，合起来保证"正占空比使编码器角度增大"。
#define MOTOR_INVERT_PAN    0      // 待实测标定
#define MOTOR_INVERT_TILT   0      // 待实测标定

// ===== MOTOR COMMON: TB6612FNG 公共与 PWM 参数 =====
#define TB6612_STBY_PIN     38     // 拉高才允许 A/B 输出，兼作硬件级急停
#define TB6612_STBY_ACTIVE_LEVEL 1 // 正逻辑：高电平使能

// 20kHz 高于人耳上限（约 16kHz）且远离电机机械谐振带，电机不啸叫；代价是分辨率受限。
#define MOTOR_PWM_FREQ_HZ   20000
// 目标 16 位，但 LEDC 时钟最高 80MHz，步数 = 80e6/20e3 = 4000 < 2^16，16 位 @20kHz
// 硬件上无法同时满足；motor_driver 初始化时会退到该频率下可用最高分辨率并告警。
#define MOTOR_PWM_RESOLUTION_BITS 16

#define MOTOR_PWM_MAX_DUTY      0.95f  // 保护电机与 TB6612 发热
#define MOTOR_DUTY_DEADBAND     0.05f  // 静摩擦死区补偿：|duty| 低于此值补到该值
#define MOTOR_PWM_MIN_DUTY      0.0f   // 0 即停止

// ===== ENCODER PAN: 霍尔增量编码器，PCNT 硬件四倍频 =====
#define PAN_ENC_A_PIN       39     // 待实测标定
#define PAN_ENC_B_PIN       40     // 待实测标定
#define PAN_PCNT_UNIT       0      // 单元内用 channel0/1 组合四倍频

// ===== ENCODER TILT =====
#define TILT_ENC_A_PIN      41     // 待实测标定
#define TILT_ENC_B_PIN      42     // 待实测标定
#define TILT_PCNT_UNIT      1

// ===== ENCODER COMMON: 线数、减速比与换算 =====
// PPR 为电机轴（减速箱输入侧）每转 A/B 单相脉冲数。JGB37-520 常见 11 或 13，
// 必须按铭牌/实测确认，填错会让所有角度按比例偏（如 90° 实际只转约 76°）。
#define ENCODER_PPR         11     // 待实测标定：常见 11 或 13

// JGB37-520 减速比有 30/56/90 等规格。
#define GEAR_RATIO_PAN      30     // 待实测标定
#define GEAR_RATIO_TILT     30     // 待实测标定

// 输出轴每转计数 = 单相脉冲 × 四倍频 × 减速比：
//   COUNTS_PER_OUTPUT_REV = ENCODER_PPR * 4 * GEAR_RATIO
#define COUNTS_PER_OUTPUT_REV_PAN  (ENCODER_PPR * 4 * GEAR_RATIO_PAN)
#define COUNTS_PER_OUTPUT_REV_TILT (ENCODER_PPR * 4 * GEAR_RATIO_TILT)

// 计数方向与 MOTOR_INVERT_* 联合标定，使正占空比 → 角度增大；两者只改其一。
#define ENCODER_INVERT_PAN  0      // 待实测标定
#define ENCODER_INVERT_TILT 0      // 待实测标定

// 速度估计一阶低通 α ∈ (0,1]：越小越平滑、相位滞后越大。
#define ENCODER_VEL_LPF_ALPHA 0.30f

// ===== CONTROL: 限位、双环 PID 与安全约束 =====
// 机械限位（度）：pan ±180°，tilt -30~+60。
#define PAN_MIN_DEG        (-180.0f)
#define PAN_MAX_DEG        (180.0f)
#define TILT_MIN_DEG       (-30.0f)
#define TILT_MAX_DEG       (60.0f)

// 软限位提前量：进入限位前该角度内禁止继续朝限位运动，给减速留缓冲。
#define SOFT_LIMIT_MARGIN_DEG  2.0f

// 360° 环绕预留开关。默认 0（限位模式），角差按线性直接相减。置 1 进入环绕模式后
// 角度按 360° 周期处理，必须配滑环/无限位机械结构，切换前确认硬件允许连续旋转。
#define ANGLE_WRAP_360_ENABLED  0

// 控制节拍（Hz）。
#define CONTROL_LOOP_HZ         100
#define CONTROL_LOOP_BUDGET_MS  (1000 / CONTROL_LOOP_HZ)

// 以下死区、限速、占空比、归零位与 PID 增益均为占位默认值，须实测整定。
// 位置环（外环）输入 deg、输出 dps；速度环（内环）输入 dps、输出 duty。
#define PAN_POS_KP          4.0f
#define PAN_POS_KI          0.0f
#define PAN_POS_KD          0.05f
#define PAN_POS_I_MIN      (-60.0f) // 积分限幅（dps）
#define PAN_POS_I_MAX      (60.0f)

#define PAN_VEL_KP          0.02f   // duty / dps
#define PAN_VEL_KI          0.50f
#define PAN_VEL_KD          0.0f
#define PAN_VEL_I_MIN      (-0.30f) // 积分限幅（duty）
#define PAN_VEL_I_MAX      (0.30f)

#define TILT_POS_KP         4.0f
#define TILT_POS_KI         0.0f
#define TILT_POS_KD         0.05f
#define TILT_POS_I_MIN     (-60.0f)
#define TILT_POS_I_MAX     (60.0f)

#define TILT_VEL_KP         0.02f
#define TILT_VEL_KI         0.50f
#define TILT_VEL_KD         0.0f
#define TILT_VEL_I_MIN     (-0.30f)
#define TILT_VEL_I_MAX     (0.30f)

// 进入该误差带且连续 LOCK_FRAMES_N 帧即 LOCKED。
#define PAN_DEADBAND_DEG    0.5f
#define TILT_DEADBAND_DEG   0.5f

// 位置环输出的速度设定上限（度/秒），防过冲与机械冲击。
#define PAN_MAX_SLEW_DPS    180.0f
#define TILT_MAX_SLEW_DPS   120.0f

// 单轴占空比上限与 duty 变化率上限（每秒）。变化率限制抑制电流与齿隙冲击。
#define PAN_MAX_DUTY        0.95f
#define TILT_MAX_DUTY       0.85f
#define PAN_DUTY_SLEW_PER_S 5.0f
#define TILT_DUTY_SLEW_PER_S 5.0f

// 上电归零安全位：进入闭环前先运动到此位，避开限位与遮挡。
#define CONTROL_HOME_PAN_DEG   0.0f
#define CONTROL_HOME_TILT_DEG  0.0f

// 连续满足死区的帧数。
#define LOCK_FRAMES_N       8

// 观测超时：超过则判定目标丢失并转 SEARCHING。
#define OBS_TIMEOUT_MS      500

// ===== TRANSMITTER: 发射器抽象层参数（当前用继电器/电机脉冲占位） =====
#define TRANSMITTER_PIN             3      // 待实测标定
#define TRANSMITTER_ACTIVE_LEVEL    1      // 1=高电平触发
#define TRANSMITTER_FIRE_PULSE_MS   50     // 占位脉冲宽度
#define TRANSMITTER_COOLDOWN_MS     500    // 两次发射强制冷却

// ===== VISION: 图像采集与目标分割 =====
// QVGA(320x240) 在算力、带宽与精度间折中；提分辨率需同步评估帧率。
#define VISION_FRAME_SIZE   FRAMESIZE_QVGA
#define VISION_SRC_W        320    // 与 VISION_FRAME_SIZE 对应，标定默认值用
#define VISION_SRC_H        240
#define VISION_TARGET_FPS   30

// 降采样工作缓冲尺寸上限，越界检测与连通域都在这个小图上做。
#define VISION_WORK_W       160
#define VISION_WORK_H       120

// HSV 阈值初值（H:0~180，S/V:0~255），默认偏"高亮标记"目标。
// 必须用现场光照与目标颜色重标定，否则分割过宽/过窄。
#define VISION_HSV_H_MIN    0
#define VISION_HSV_H_MAX    180
#define VISION_HSV_S_MIN    80
#define VISION_HSV_S_MAX    255
#define VISION_HSV_V_MIN    180
#define VISION_HSV_V_MAX    255

// 连通域达到该面积置信度为 1.0，按比例递减。
#define VISION_CONF_AREA_REF 300.0f

// ===== COMMS: 串口遥测 =====
#define TELEMETRY_BAUD      115200
// 关闭逐帧遥测时仍以此频率输出心跳，证明固件存活。
#define TELEMETRY_HEARTBEAT_MS 1000

// ===== 引脚与资源预留（扩展用，只有注释，不新增宏） =====
// 完整表格见 docs/引脚分配与扩展预留.md。改动任一 *_PIN 后同步本段。
//
// 已占用 GPIO：
//   1,2 pan AIN1/2；3 发射器；4,5 摄像头 SCCB；6,7 VSYNC/HREF；8,9,10 D2/D1/D3；
//   11,12 D0/D4；13 PCLK；14 pan PWM；15 XCLK；16,17,18 D7/D6/D5；21 tilt PWM；
//   38 STBY；39,40 pan 编码器；41,42 tilt 编码器；47,48 tilt BIN1/2。
//
// 系统占用不可作普通 IO：26-32 Flash/PSRAM SPI；33-37 N16R8 八线 OPI PSRAM 数据线；
//   19,20 原生 USB（调试/烧录时不可占用）；43,44 UART0（默认串口监视器）；22-25 不引出。
// 空闲：0（BOOT/strapping）、45、46（strapping）、19/20（放弃原生 USB 后）。
// 三者都必须在复位瞬间保持规定电平，接外设时注意上下拉别拉反。
//
// 预留建议：I2C 用空闲的 I2C0，GPIO45=SDA、46=SCL（I2C1 已被摄像头 SCCB 占用）；
//   第二路 UART 复用 UART0 并释放 19/20；ADC 无空闲脚，走 I2C ADC 或把发射器从
//   GPIO3 挪到 45/46 腾出 ADC1_CH2；SPI 无空闲脚，优先用 I2C；通用 IO 用 PCF8574。
//
// 外设余量：LEDC 已用 ch0/timer0、ch1/timer1、ch2/timer2，剩 ch3-ch7 与 timer3；
//   PCNT 已用 unit0/unit1，剩 unit2/unit3；UART 剩 1/2；I2C 剩 I2C0；
//   SPI 剩 SPI2/SPI3（需引脚）；ADC 无空闲通道（ADC1=GPIO1-10，ADC2=GPIO11-20 已占满）。
