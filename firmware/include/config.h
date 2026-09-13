#pragma once
/**
 * config.h
 * 全局编译期配置：引脚、机械参数、控制器增益、通信参数。
 *
 * 不依赖其它模块，所有模块单向依赖它。硬件或整定值变化只改这里，
 * 代码里不要再散落这些数字。
 *
 * 标注 "待实测标定" 的都是占位默认值，首次上电前按实物核对或整定，
 * 不要当成已验证的实测值。
 */

// ===========================================================================
// 构建信息
// ===========================================================================
#define AIM_BUILD_VERSION "0.1.0"

// 轴索引：全工程统一用这两个常量做数组下标/接口参数，避免散落 0/1 字面量。
#define AXIS_PAN   0
#define AXIS_TILT  1
#define AXIS_COUNT 2

// ===========================================================================
// CAMERA PINS: OV2640/OV3660 等 DVP 摄像头与 ESP32-S3 的并口连线
// ===========================================================================
// 待核对：首次烧录前必须对照你自己开发板的原理图确认这些引脚。
// 不同厂家的 S3-CAM 板 DVP 数据线顺序差异极大，接错不会烧板，但会黑屏/花屏。
// 下表的 Y2..Y9 与 esp32-camera 的 D0..D7 一一对应，顺序不可随意打乱。
#define CAM_PIN_PWDN        (-1)   // 待实测标定：-1 表示未接（传感器掉电脚常被固定拉低）
#define CAM_PIN_RESET       (-1)   // 待实测标定：-1 表示未接（由摄像头模块内部复位）
#define CAM_PIN_XCLK        15     // 待实测标定：主时钟输出脚
#define CAM_PIN_SIOD        4      // 待实测标定：SCCB 数据
#define CAM_PIN_SIOC        5      // 待实测标定：SCCB 时钟
#define CAM_PIN_Y9          16     // 待实测标定：D7
#define CAM_PIN_Y8          17     // 待实测标定：D6
#define CAM_PIN_Y7          18     // 待实测标定：D5
#define CAM_PIN_Y6          12     // 待实测标定：D4
#define CAM_PIN_Y5          10     // 待实测标定：D3
#define CAM_PIN_Y4          8      // 待实测标定：D2
#define CAM_PIN_Y3          9      // 待实测标定：D1
#define CAM_PIN_Y2          11     // 待实测标定：D0
#define CAM_PIN_VSYNC       6      // 待实测标定：场同步
#define CAM_PIN_HREF        7      // 待实测标定：行有效
#define CAM_PIN_PCLK        13     // 待实测标定：像素时钟

// 摄像头主时钟：20MHz 是 OV2640 的常用稳定值；提高会增加 EMI 与走线要求。
#define CAM_XCLK_FREQ_HZ    20000000

// 摄像头 XCLK 由 LEDC 产生。电机 PWM 已占用低速模式 timer0/timer1 与 channel0/1，
// 这里给摄像头单独分配 timer2/channel2，避免与电机 PWM 抢占 LEDC 资源。
#define CAM_LEDC_CHANNEL    2
#define CAM_LEDC_TIMER      2

// ===========================================================================
// MOTOR PAN: TB6612FNG A 路驱动 pan（水平）电机
// ===========================================================================
// 待实测标定：以下 3 个引脚必须与接线表一致，且不能与摄像头引脚冲突。
#define PAN_AIN1_PIN        1
#define PAN_AIN2_PIN        2
#define PAN_PWM_PIN         14
#define PAN_LEDC_CHANNEL    0      // LEDC 通道 0（ESP32-S3 共 8 路）
#define PAN_LEDC_TIMER      0      // LEDC 定时器 0

// ===========================================================================
// MOTOR TILT: TB6612FNG B 路驱动 tilt（俯仰）电机
// ===========================================================================
#define TILT_AIN1_PIN       47
#define TILT_AIN2_PIN       48
#define TILT_PWM_PIN        21
#define TILT_LEDC_CHANNEL   1
#define TILT_LEDC_TIMER     1

// 方向极性：若某轴"正指令 -> 负方向运动"，把对应宏改为 1 即可软件反向，不必改接线。
// 要和 ENCODER_INVERT_* 一起标定，最终保证"正占空比使编码器角度增大"。
#define MOTOR_INVERT_PAN    0      // 待实测标定
#define MOTOR_INVERT_TILT   0      // 待实测标定

// ===========================================================================
// MOTOR COMMON: TB6612FNG 公共与 PWM 参数
// ===========================================================================
#define TB6612_STBY_PIN     38     // STBY 拉高才允许 A/B 两路输出，用于硬件级急停
#define TB6612_STBY_ACTIVE_LEVEL 1 // 本设计用正逻辑：高电平使能

// PWM 频率：20kHz 高于人耳可听上限（约 16kHz），也远离电机绕组的机械谐振带，
// 电机不会啸叫。代价是分辨率拉不满，见下面一行。
#define MOTOR_PWM_FREQ_HZ   20000
// PWM 分辨率：目标 16 位。注意 LEDC 时钟最高 80MHz，占空比步数 = 时钟/频率，
// 即 80e6/20e3 = 4000 < 2^16，因此 16 位 @20kHz 在硬件上无法同时满足；
// motor_driver 会在初始化时退回到该频率下实际可用的最高分辨率并打印告警。
#define MOTOR_PWM_RESOLUTION_BITS 16

#define MOTOR_PWM_MAX_DUTY      0.95f  // 占空比上限：保护电机与 TB6612 发热
#define MOTOR_DUTY_DEADBAND     0.05f  // 静态摩擦死区补偿量：|duty| 低于此值补到该值
#define MOTOR_PWM_MIN_DUTY      0.0f   // 允许的占空比下限（0 = 停止）

// ===========================================================================
// ENCODER PAN: pan 轴霍尔增量编码器，PCNT 硬件四倍频解码
// ===========================================================================
#define PAN_ENC_A_PIN       39     // 待实测标定：编码器 A 相
#define PAN_ENC_B_PIN       40     // 待实测标定：编码器 B 相
#define PAN_PCNT_UNIT       0      // 使用 PCNT 单元 0（单元内用 channel 0/1 组合实现四倍频）

// ===========================================================================
// ENCODER TILT
// ===========================================================================
#define TILT_ENC_A_PIN      41     // 待实测标定
#define TILT_ENC_B_PIN      42     // 待实测标定
#define TILT_PCNT_UNIT      1      // 使用 PCNT 单元 1

// ===========================================================================
// ENCODER COMMON: 线数、减速比与换算
// ===========================================================================
// ENCODER_PPR：电机轴（减速箱输入侧）每转的 A/B 单相脉冲数。
// JGB37-520 常见的霍尔编码器为 11 或 13 线，必须按实物铭牌/实测确认，
// 取值错误会让所有角度按比例偏大或偏小（表现为"转 90° 实际只转约 76°"这类）。
#define ENCODER_PPR         11     // 待实测标定：常见 11 或 13

// 减速箱减速比：JGB37-520 有 30 / 56 / 90 等规格。
#define GEAR_RATIO_PAN      30     // 待实测标定
#define GEAR_RATIO_TILT     30     // 待实测标定

// 输出轴每转计数值推导：
//   电机轴每转 A 相脉冲 = ENCODER_PPR
//   正交解码四倍频   → ×4（A/B 的上升沿与下降沿都计数）
//   经减速箱再放大    → ×GEAR_RATIO
//   即  COUNTS_PER_OUTPUT_REV = ENCODER_PPR * 4 * GEAR_RATIO
#define COUNTS_PER_OUTPUT_REV_PAN  (ENCODER_PPR * 4 * GEAR_RATIO_PAN)
#define COUNTS_PER_OUTPUT_REV_TILT (ENCODER_PPR * 4 * GEAR_RATIO_TILT)

// 编码器计数方向：与 MOTOR_INVERT_* 联合标定，使正占空比 → 角度增大。
// 二者只需反转其中一个即可改变闭环整体极性。
#define ENCODER_INVERT_PAN  0      // 待实测标定
#define ENCODER_INVERT_TILT 0      // 待实测标定

// 速度估计一阶低通系数 α ∈ (0,1]：越小越平滑、相位滞后越大。
#define ENCODER_VEL_LPF_ALPHA 0.30f

// ===========================================================================
// CONTROL: 限位、双环 PID 与安全约束
// ===========================================================================
// 机械限位（度）。pan ±180°，tilt -30°~+60°。
#define PAN_MIN_DEG        (-180.0f)
#define PAN_MAX_DEG        (180.0f)
#define TILT_MIN_DEG       (-30.0f)
#define TILT_MAX_DEG       (60.0f)

// 软限位提前量：进入限位前该角度内开始禁止继续朝限位方向运动，
// 给减速与机械余量留缓冲，避免硬撞。
#define SOFT_LIMIT_MARGIN_DEG  2.0f

// 360° 环绕预留开关。默认 0（限位模式）：所有角度按线性区间处理，角差直接相减。
// 置 1 后进入环绕模式：角度按 360° 周期处理，必须安装滑环/无限位机械结构，
// angle_utils 的角度解算会自动走最短路径环绕分支。切换前请确认硬件允许连续旋转。
#define ANGLE_WRAP_360_ENABLED  0

// 控制节拍（Hz）。100Hz 给速度环足够带宽，同时为视觉留出算力。
#define CONTROL_LOOP_HZ         100
#define CONTROL_LOOP_BUDGET_MS  (1000 / CONTROL_LOOP_HZ)

// 位置环（外环）与速度环（内环）PID 增益，全部 "待实测标定"。
// 单位说明：位置环输入 deg、输出 dps；速度环输入 dps、输出 duty。
#define PAN_POS_KP          4.0f    // 待实测标定
#define PAN_POS_KI          0.0f    // 待实测标定
#define PAN_POS_KD          0.05f   // 待实测标定
#define PAN_POS_I_MIN      (-60.0f) // 待实测标定：积分限幅（dps 量纲）
#define PAN_POS_I_MAX      (60.0f)  // 待实测标定

#define PAN_VEL_KP          0.02f   // 待实测标定：duty / dps
#define PAN_VEL_KI          0.50f   // 待实测标定
#define PAN_VEL_KD          0.0f    // 待实测标定
#define PAN_VEL_I_MIN      (-0.30f) // 待实测标定：积分限幅（duty 量纲）
#define PAN_VEL_I_MAX      (0.30f)  // 待实测标定

#define TILT_POS_KP         4.0f    // 待实测标定
#define TILT_POS_KI         0.0f    // 待实测标定
#define TILT_POS_KD         0.05f   // 待实测标定
#define TILT_POS_I_MIN     (-60.0f) // 待实测标定
#define TILT_POS_I_MAX     (60.0f)  // 待实测标定

#define TILT_VEL_KP         0.02f   // 待实测标定
#define TILT_VEL_KI         0.50f   // 待实测标定
#define TILT_VEL_KD         0.0f    // 待实测标定
#define TILT_VEL_I_MIN     (-0.30f) // 待实测标定
#define TILT_VEL_I_MAX     (0.30f)  // 待实测标定

// "已对准"判定死区（度）：进入该误差带且连续若干帧即视为 LOCKED。
#define PAN_DEADBAND_DEG    0.5f    // 待实测标定
#define TILT_DEADBAND_DEG   0.5f    // 待实测标定

// 指令角速度上限（度/秒）：限制位置环输出的速度设定，防止过冲与机械冲击。
#define PAN_MAX_SLEW_DPS    180.0f  // 待实测标定
#define TILT_MAX_SLEW_DPS   120.0f  // 待实测标定

// 单轴占空比上限（0~1）与 duty 变化率上限（每秒）。
// 变化率限制用于抑制电流冲击与齿轮背隙冲击，延长减速箱寿命。
#define PAN_MAX_DUTY        0.95f   // 待实测标定
#define TILT_MAX_DUTY       0.85f   // 待实测标定
#define PAN_DUTY_SLEW_PER_S 5.0f    // 待实测标定
#define TILT_DUTY_SLEW_PER_S 5.0f   // 待实测标定

// 上电归零目标（安全位）。舵盘在进入闭环前先运动到此位，避开限位与遮挡。
#define CONTROL_HOME_PAN_DEG   0.0f    // 待实测标定
#define CONTROL_HOME_TILT_DEG  0.0f    // 待实测标定

// 连续满足死区的帧数达到该值才进入 LOCKED。
#define LOCK_FRAMES_N       8

// 观测（视觉目标）超时时间：超过则判定目标丢失并转入 SEARCHING。
#define OBS_TIMEOUT_MS      500

// ===========================================================================
// TRANSMITTER: 发射器抽象层参数（当前用继电器/电机脉冲占位）
// ===========================================================================
#define TRANSMITTER_PIN             3      // 待实测标定：控制发射器/继电器的 GPIO
#define TRANSMITTER_ACTIVE_LEVEL    1      // 有效电平：1=高电平触发
#define TRANSMITTER_FIRE_PULSE_MS   50     // 占位发射脉冲宽度
#define TRANSMITTER_COOLDOWN_MS     500    // 两次发射之间的强制冷却时间

// ===========================================================================
// VISION: 图像采集与目标分割
// ===========================================================================
// 采集分辨率。QVGA(320x240) 在算力、带宽与精度间折中；提高分辨率需同步评估帧率。
#define VISION_FRAME_SIZE   FRAMESIZE_QVGA
#define VISION_SRC_W        320    // 与 VISION_FRAME_SIZE 对应的源图像宽（标定默认值用）
#define VISION_SRC_H        240    // 与 VISION_FRAME_SIZE 对应的源图像高
#define VISION_TARGET_FPS   30

// 降采样后的工作缓冲尺寸上限。越界检测与连通域都在这个小图上做，控制算力。
#define VISION_WORK_W       160
#define VISION_WORK_H       120

// HSV 阈值初值（H:0~180, S/V:0~255）。默认偏向"高亮标记"目标。
// 待实测标定：必须用现场光照与目标颜色重新标定，否则分割会过宽/过窄。
#define VISION_HSV_H_MIN    0
#define VISION_HSV_H_MAX    180
#define VISION_HSV_S_MIN    80
#define VISION_HSV_S_MAX    255
#define VISION_HSV_V_MIN    180
#define VISION_HSV_V_MAX    255

// 置信度参考面积（工作缓冲像素数）：连通域达到该面积置信度为 1.0，按比例递减。
#define VISION_CONF_AREA_REF 300.0f

// ===========================================================================
// COMMS: 串口遥测
// ===========================================================================
#define TELEMETRY_BAUD      115200
// 心跳周期：AIM_VERBOSE_TELEMETRY=0 时仍以此频率输出一行，证明固件还活着。
#define TELEMETRY_HEARTBEAT_MS 1000
