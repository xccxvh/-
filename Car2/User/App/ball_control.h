#ifndef APP_BALL_CONTROL_H
#define APP_BALL_CONTROL_H

/*
 * 滚球自动稳定控制器（应用层）。
 *
 * 控制模型：
 *   θ = Kp * (target - x) - Kd * v
 *
 * 其中：
 * - target：钢珠目标位置，单位 cm；
 * - x：Jetson 测得的钢珠位置，单位 cm；
 * - v：由相邻有效视觉帧差分并低通滤波得到的钢珠速度，单位 cm/s；
 * - θ：摆杆目标倾角，单位 °。正方向定义为“使钢珠沿 +x 方向加速”，
 *   实际机械方向由 Servo 模块的方向标定决定。
 *
 * 分层：
 * - 本模块只做状态机、速度估计和 PD，不直接操作 PWM 寄存器；
 * - 倾角输出统一调用 Servo_SetBeamAngle()，由 Servo 模块完成映射、限幅和
 *   速率限制；
 * - 视觉帧由应用层调用 BallControl_OnMeasurement() 喂入，便于模拟数据和
 *   真实 UART 数据共用同一套控制逻辑。
 *
 * 安全策略：
 * - 上电进入 IDLE，必须显式 Enable 才进入 AUTO；
 * - AUTO 期间视觉无效或超过 150 ms 未更新，立即转 VISION_LOST，并以限速
 *   方式回中，不会继续使用旧位置；
 * - 视觉恢复后不会自动重新进入 AUTO，必须重新使能；
 * - 故障进入 FAULT 后只能显式 Clear，期间摆杆限速回中。
 *
 * 头文件位置：User/App，属于应用层；只依赖 BSP 的 Servo 与 Vision 接口。
 */
#include "bsp_common.h"
#include "VISION/vision_link.h"

/* 控制器状态机；数值会写入遥测帧，修改顺序时需同步协议文档。 */
typedef enum {
    BALL_STATE_IDLE = 0,        /* 不执行自动滚球，摆杆回中 */
    BALL_STATE_MANUAL = 1,      /* 手动调整摆杆，本模块不干预 */
    BALL_STATE_AUTO = 2,        /* 自动 PD 稳球 */
    BALL_STATE_VISION_LOST = 3, /* 视觉丢失/无效，已退出闭环并限速回中 */
    BALL_STATE_FAULT = 4        /* 通信或执行机构故障，锁存等待清除 */
} ball_control_state_t;

/*
 * 默认 PD 增益（可在工程宏里覆盖，也可运行时调用 BallControl_SetGains）。
 *
 * 单位：
 * - Kp：°/cm；
 * - Kd：°/(cm/s)。
 * 这两个值只是仿真基准，实车必须重新标定。
 */
#ifndef BALL_CONTROL_DEFAULT_KP
#define BALL_CONTROL_DEFAULT_KP         (0.60f)
#endif
#ifndef BALL_CONTROL_DEFAULT_KD
#define BALL_CONTROL_DEFAULT_KD         (0.30f)
#endif

/* 故障码，写入调试信息；0 表示无故障。 */
#define BALL_FAULT_NONE                 (0U)
#define BALL_FAULT_VISION_LINK          (1U)  /* 视觉链路连续解码错误 */
#define BALL_FAULT_SERVO                (2U)  /* 舵机模块未就绪/异常 */
#define BALL_FAULT_CONFIG               (3U)  /* 参数配置非法 */

/*
 * 调试快照，供 LCD、遥测帧和上位机日志使用。
 *
 * 字段含义：
 * - error_cm：target - x，正表示钢珠在目标左侧（需要 +θ 加速）；
 * - beam_angle_cmd_deg：控制器输出的摆杆目标倾角；
 * - servo_pwm_us：Servo 模块当前实际脉宽；
 * - vision_age_ms：距最近一帧的本地毫秒数；
 * - frames_used/frames_rejected：本控制器接受/拒绝的测量帧计数。
 */
typedef struct {
    uint32_t time_ms;
    float    target_cm;
    float    ball_x_cm;
    float    ball_v_cm_s;
    float    error_cm;
    float    beam_angle_cmd_deg;
    uint16_t servo_pwm_us;
    uint8_t  control_state;
    uint8_t  fault_code;
    bool     vision_valid;
    bool     auto_enabled;
    uint32_t vision_age_ms;
    uint32_t frames_used;
    uint32_t frames_rejected;
    uint16_t last_seq;
} ball_debug_t;

/*
 * 初始化控制器状态。
 *
 * 注意：
 * - 不初始化舵机和视觉硬件；调用顺序应为：
 *   SYSCFG_DL_init -> system_time_init -> Servo_Init -> Vision_Init ->
 *   BallControl_Init。
 * - 初始化后状态为 IDLE，目标 0 cm，默认 Kp/Kd 见下方宏。
 */
void BallControl_Init(void);

/*
 * 设置钢珠目标位置，单位 cm。
 *
 * 参数：
 * - target_cm：会被限制在 ±BALL_CONTROL_TARGET_LIMIT_CM 内；
 *   NaN/Inf 会被忽略。
 */
void BallControl_SetTarget(float target_cm);

/*
 * 设置 PD 增益。
 *
 * 参数：
 * - kp：单位 °/cm；
 * - kd：单位 °/(cm/s)。
 *
 * 约束：
 * - 只接受有限且非负的值；非法参数直接忽略，保留原增益；
 * - 建议先调 Kp 到能收敛但略有振荡，再增加 Kd 抑制超调。
 */
void BallControl_SetGains(float kp, float kd);

/*
 * 喂入一帧视觉测量（真实 UART 或模拟数据均由应用层调用）。
 *
 * 处理内容：
 * - 检查 valid、置信度、坐标范围；
 * - 用本地接收时刻计算帧间 Δt（不使用 Jetson 绝对时钟）；
 * - 对位置跳变做剔除，对速度做低通滤波；
 * - 置“有新测量”标志，等待 BallControl_Update() 计算 PD。
 *
 * 参数：
 * - measurement：测量帧，不能为 NULL。
 */
void BallControl_OnMeasurement(const BallMeasurement *measurement);

/*
 * 周期调用：状态机推进、超时检测、PD 计算和舵机输出。
 *
 * 调用位置：
 * - 主循环每圈调用；内部只在收到新有效测量时重算 PD，
 *   其余时间保持上一次倾角或执行限速回中。
 * - 不使用阻塞延时，不影响电机/灰度任务。
 */
void BallControl_Update(void);

/*
 * 使能/关闭自动控制。
 *
 * 参数：
 * - true：请求进入 AUTO。只有存在新鲜有效测量时才会成功，
 *   否则保持 IDLE（可通过 BallControl_GetState() 判断结果）；
 * - false：退出自动控制，进入 IDLE 并限速回中。
 */
void BallControl_Enable(bool enable);

/*
 * 进入手动模式（测试 A：手动微倾角）。
 *
 * 行为：
 * - 退出 AUTO，速度估计清零；
 * - 之后由应用层调用 BallControl_SetManualBeamAngle() 控制摆杆。
 */
void BallControl_EnterManual(void);

/*
 * 手动模式下设置摆杆倾角。
 *
 * 参数：
 * - angle_deg：目标倾角，仍受 Servo 模块的摆杆限幅和速率限制。
 *
 * 说明：
 * - 仅在 MANUAL 状态生效，其他状态调用会被忽略，避免手动值意外
 *   覆盖自动控制输出。
 */
void BallControl_SetManualBeamAngle(float angle_deg);

/*
 * 通知控制器发生故障（由应用层根据视觉链路状态或硬件自检调用）。
 *
 * 参数：
 * - fault_code：BALL_FAULT_xxx。
 *
 * 行为：
 * - 立即退出 AUTO，进入 FAULT，摆杆限速回中；
 * - 故障锁存，必须调用 BallControl_ClearFault() 才能恢复 IDLE。
 */
void BallControl_NotifyFault(uint8_t fault_code);

/*
 * 清除故障锁存并回到 IDLE。
 *
 * 说明：
 * - 如果根因（如视觉链路持续错误）仍然存在，应用层应再次 NotifyFault，
 *   控制器会重新进入 FAULT。
 */
void BallControl_ClearFault(void);

/* 当前是否处于 AUTO。 */
bool BallControl_IsEnabled(void);

/* 当前状态机状态。 */
ball_control_state_t BallControl_GetState(void);

/* 当前目标位置，单位 cm。 */
float BallControl_GetTarget(void);

/*
 * 读取当前 PD 增益，便于 LCD/日志显示实际生效值。
 *
 * 参数：
 * - kp/kd：输出指针，可为 NULL；为 NULL 时忽略对应项。
 */
void BallControl_GetGains(float *kp, float *kd);

/* 最近一次 PD 输出的摆杆倾角，单位 °；IDLE/MANUAL 时可能为 0。 */
float BallControl_GetBeamAngleCmd(void);

/* 最近一次有效测量的钢珠位置，单位 cm。 */
float BallControl_GetBallX(void);

/* 最近一次滤波后的钢珠速度，单位 cm/s。 */
float BallControl_GetBallV(void);

/* 当前是否满足进入 AUTO 的条件（视觉新鲜且有效）。 */
bool BallControl_CanEnable(void);

/* 填充调试快照。debug 为 NULL 时直接返回。 */
void BallControl_GetDebug(ball_debug_t *debug);

/* 状态对应的短字符串，用于 LCD 和日志。 */
const char *BallControl_StateText(ball_control_state_t state);

#endif
