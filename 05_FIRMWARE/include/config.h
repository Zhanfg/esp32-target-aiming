#pragma once
/**
 * config.h
 * 全局编译期配置：引脚、机械参数、控制器参数、安全阈值、通信参数。
 * 标注 "待实测标定" 的都是占位默认值，首次上电前按实物核对或整定。
 *
 * 第二期的执行层是舵机指向 + 两级脉冲核心 + 旋转供给盘，第一期的电机闭环、
 * TB6612、编码器层已整体删除，相关宏随之移除。全工程单向依赖本文件，
 * 硬件或整定值变化只改这里。
 */

// 构建版本：烧进镜像，启动横幅与遥测首行会打印，用于确认板上跑的是哪一版。
#define AIM_BUILD_VERSION "0.2.0"

// ===== 机构变体常量与研究门控（规格书 §30 STOP-01/02/03）=====
// STOP-01/02 由实验结论决定是否需要回退单级：触发时把 STAGES 改回 1 并用构建
// 开关关掉双级相关功能，不进 FAULT。STOP-03 同理关闭边界研究分支。
#define MST_MECHANISM_STAGES             2      // 1=单级回退，2=双级串联
#define MST_PROTOTYPE_VERSION            "MST-01C"
#define MST_MECHANISM_VERSION            "MST-01C"
#define MST_RESEARCH_DUAL_STAGE_ENABLED  1
#define MST_RESEARCH_BOUNDARY_ENABLED    1
#define MST_BOUNDARY_RESEARCH_ACTIVE     0      // 运行期是否置起 BOUNDARY_CMD
#define MST_EXPERIMENT_ID                0

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

// XCLK 走 LEDC timer2/channel2。舵机 PWM 用 timer3 的 channel3/4，分开避免抢占。
#define CAM_LEDC_CHANNEL    2
#define CAM_LEDC_TIMER      2

// ===== CONTROL: 机械限位与指向约束 =====
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

// 进入该误差带且满足锁定窗口即 LOCKED。
#define PAN_DEADBAND_DEG    0.5f
#define TILT_DEADBAND_DEG   0.5f

// 指向角速度上限（度/秒）：单轴指令变化率限制，抑制舵机冲击与过冲。
#define PAN_MAX_SLEW_DPS    180.0f
#define TILT_MAX_SLEW_DPS   120.0f

// 有反馈舵机的位置校正比例。无反馈实现不使用。
#define SERVO_POS_CORR_KP   0.90f  // 待实测标定

// 上电归零安全位：进入闭环前先运动到此位，避开限位与遮挡。
#define CONTROL_HOME_PAN_DEG   0.0f
#define CONTROL_HOME_TILT_DEG  0.0f

// LOCKED 作发射门：改用时窗 + 多视觉帧确认，替代原来纯控制拍计数，
// 使判定口径与规格书 §11.1 的 ≤500ms 一致（对应 §6.9 第 14 条冲突）。
#define LOCK_WINDOW_MS          500
#define LOCK_MIN_VISION_FRAMES  3
// 保留的旧宏：纯控制拍计数已不作为发射门判据，供诊断参考。
#define LOCK_FRAMES_N       8

// 观测超时：超过则判定目标丢失并转 SEARCHING。
#define OBS_TIMEOUT_MS      500

// ===== SERVO: 两轴位置舵机 =====
// 可替换实现由 SERVO_DRIVE_TYPE 编译期选择，采购前不必冻结选型。
#define SERVO_DRIVE_PWM 0
#define SERVO_DRIVE_BUS 1
#define SERVO_DRIVE_TYPE SERVO_DRIVE_PWM   // 待实测标定

// PAN_CMD/TILT_CMD 直连 PWM，LEDC channel3/4 共用 timer3。
#define SERVO_PAN_PIN     14     // 待实测标定
#define SERVO_TILT_PIN    21     // 待实测标定
#define SERVO_PAN_LEDC_CHANNEL   3
#define SERVO_TILT_LEDC_CHANNEL  4
#define SERVO_LEDC_TIMER   3
#define SERVO_PWM_FREQ_HZ  50
#define SERVO_PWM_RESOLUTION_BITS 16
#define SERVO_PULSE_MIN_US 500.0f    // 待实测标定
#define SERVO_PULSE_MAX_US 2500.0f   // 待实测标定

// 脉宽端点对应的舵机轴角，用于角度↔脉宽换算。没有实测行程前先令其等于机械限位，
// 安装后按舵机铭牌与实测重复定位结果重标。
#define SERVO_PAN_MIN_DEG   PAN_MIN_DEG    // 待实测标定
#define SERVO_PAN_MAX_DEG   PAN_MAX_DEG    // 待实测标定
#define SERVO_TILT_MIN_DEG  TILT_MIN_DEG   // 待实测标定
#define SERVO_TILT_MAX_DEG  TILT_MAX_DEG   // 待实测标定

// 方向极性：正指令应使轴朝正方向运动，反了就置 1，不必改接线。
#define SERVO_INVERT_PAN    0    // 待实测标定
#define SERVO_INVERT_TILT   0    // 待实测标定

// 总线舵机（ServoBus）串口参数。协议未定，接入选型后补齐帧格式与命令字。
#define SERVO_BUS_UART_NUM  1        // 待实测标定
#define SERVO_BUS_TX_PIN    (-1)     // 待实测标定：与 UART1 可用脚一并确定
#define SERVO_BUS_RX_PIN    (-1)     // 待实测标定
#define SERVO_BUS_BAUD      1000000  // 待实测标定

// ===== HOME: 上电归零（架构文档 §6.10）=====
#define HOME_TIMEOUT_MS      3000   // 步骤 3：3 秒内不触发即失败
#define HOME_SEEK_STEP_DEG   1.0f   // 每拍朝参考方向推进的角度
#define HOME_SEEK_PERIOD_MS  20
#define HOME_CONFIRM_MS      20     // 参考开关需稳定闭合的确认时间
#define HOME_SETTLE_MS       200    // 触发后退回已知角度的稳定等待
#define HOME_PAN_DIR        (-1)    // 待实测标定：参考开关所在方向，-1 朝最小角
#define HOME_TILT_DIR       (-1)    // 待实测标定
#define HOME_PAN_BACKOFF_DEG   5.0f // 待实测标定
#define HOME_TILT_BACKOFF_DEG  5.0f // 待实测标定

// ===== FIRING CYCLE: 两级脉冲核心与供给盘 =====
#define RELEASE_CMD_PIN     3        // 待实测标定，直连，strapping 脚
#define RELEASE_PULSE_MS    30       // 软件脉冲宽度；硬件单稳态另设上限（§6.6）
#define STAGE1_STATE_PIN    40       // 待实测标定，直连中断
#define STAGE2_STATE_PIN    41       // 待实测标定，直连中断

#define PRELOAD_TIME_MS      300     // 待实测标定：预载到位判定窗口
#define PRELOAD_ABORT_MS     200     // 预载中丢失 LOCKED 的中止窗口
#define RECOVER_TIMEOUT_MS   500     // 待实测标定
#define INDEX_TIMEOUT_MS     500     // 待实测标定：单次索引到位超时
#define INDEX_MAX_RETRY      3       // 连续失败上限，超过升级故障
#define ARMED_MIN_DWELL_MS   50      // ARMED 到允许释放的最小停留
#define RELEASE_AUTO_ENABLED 0       // 1=锁定后自动释放；0=等 EXTERNAL_ALLOW 上升沿
#define MAG_POSITIONS        6       // 供给盘工位数
#define FIRE_MODE_DEFAULT    0       // 0=Mode A 空气，1=Mode B 软载荷；待实测标定

// ===== MCP23017: I2C0 慢速离散 I/O =====
#define I2C0_SDA_PIN        45       // 待实测标定（strapping 脚，注意上电电平）
#define I2C0_SCL_PIN        46       // 待实测标定（strapping 脚，注意上电电平）
#define I2C0_FREQ_HZ        400000
#define MCP23017_ADDR       0x20     // A2A1A0 全低

// 扩展器位编号：0-7 = GPA0-GPA7，8-15 = GPB0-GPB7。
#define EXP_BOUNDARY_CMD_BIT    0
#define EXP_PRELOAD_CMD_BIT     1
#define EXP_MAG_INDEX_CMD_BIT   2
#define EXP_STATUS_LED_BIT      3
#define EXP_MAG_HOME_BIT        4
#define EXP_MAG_INDEX_OK_BIT    5
#define EXP_MECH_RECOVERED_BIT  6
#define EXP_PAN_HOME_BIT        7
#define EXP_TILT_HOME_BIT       8
#define EXP_FIRE_INHIBIT_BIT    9
#define EXP_EXTERNAL_ALLOW_BIT  10

// 输出位掩码：这些位配置为输出，其余为输入并启用内部上拉。
#define EXP_OUTPUT_MASK ((1u << EXP_BOUNDARY_CMD_BIT) | \
                         (1u << EXP_PRELOAD_CMD_BIT) | \
                         (1u << EXP_MAG_INDEX_CMD_BIT) | \
                         (1u << EXP_STATUS_LED_BIT))

// ===== WATCHDOG: 控制环超预算保护 =====
// 连续超预算达到该拍数，或单拍超过硬上限，直接进 FAULT（对应 §6.9 第 9 条）。
#define WATCHDOG_STREAK_N        100
#define WATCHDOG_HARD_OVERRUN_MS 100
#define OBS_LINK_FAIL_N          100  // 连续取帧失败上限，超过判观测链路异常

// ===== TRANSMITTER: 第一期发射器抽象层参数（第二期由四路命令层取代，保留占位）=====
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
// 完整表格见 05_FIRMWARE/io-map.md。改动任一 *_PIN 后同步本段。
//
// 已占用 GPIO：
//   3 RELEASE_CMD；4,5 摄像头 SCCB；6,7 VSYNC/HREF；8,9,10 D2/D1/D3；11,12 D0/D4；
//   13 PCLK；14 PAN_CMD；15 XCLK；16,17,18 D7/D6/D5；21 TILT_CMD；40,41 STAGE1/2；
//   45,46 I2C0 到 MCP23017（strapping 脚）。
//
// 系统占用不可作普通 IO：26-32 Flash/PSRAM SPI；33-37 N16R8 八线 OPI PSRAM 数据线；
//   19,20 原生 USB（调试/烧录时不可占用）；43,44 UART0（默认串口监视器）；22-25 不引出。
// 空闲：0（BOOT/strapping）、38、39、42、47、48（后三者部分开发板兼 RGB LED，
//   用于通用 IO 前必须核对原理图）。
//
// 预留建议：I2C1 已被摄像头 SCCB 占用，I2C0 接 MCP23017；第二路 UART 复用于
//   总线舵机；ADC 无空闲脚，电流采样走 I2C。
//
// 外设余量：LEDC 已用 ch2/timer2（XCLK）、ch3/ch4/timer3（两舵机），
//   剩 ch0/1/ch5-ch7 与 timer0/1；PCNT 全部空闲；UART 剩 1/2；I2C 剩 I2C0；
//   SPI 剩 SPI2/SPI3（需引脚）；ADC 无空闲通道（ADC1=GPIO1-10，ADC2=GPIO11-20 已占满）。
