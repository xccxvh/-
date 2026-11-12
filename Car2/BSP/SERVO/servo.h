#ifndef BSP_SERVO_SERVO_H
#define BSP_SERVO_SERVO_H

/*
 * 摆杆舵机 BSP 驱动（MG995/MG996R 类 50 Hz 舵机）。
 *
 * 硬件资源：
 * - 信号：PA9，TIMA0 CCP1，对应 SysConfig 中的 PWM_0 通道 1。
 * - 电源：板上 PA9/V+/GND 排针的 V+（独立 5 V），必须与 G3519 共地。
 * - 定时器：TIMA0 由 80 MHz 经预分频 100 得到 800 kHz 计数时钟。
 * - 周期：16000 计数 = 20 ms = 50 Hz。
 * - 脉宽：400~2000 计数 = 0.5~2.5 ms，对应舵机 0~180°。
 *
 * 实测要点（来自 ServoTest 副工程）：
 * - 本板 TIMA0 的 CC1 高电平时间 = “周期 - 比较值”，因此写比较值时要用
 *   16000 - 脉宽计数，不能直接写脉宽。
 * - 舵机 104° 时摆杆水平，104° 即零位 SERVO_NEUTRAL_ANGLE_DEG。
 *
 * 分层约束：
 * - 本模块只负责“摆杆目标倾角 -> 舵机角度 -> PWM 比较值”的映射、限幅和
 *   速率限制，不包含任何 PD 或视觉逻辑。
 * - 所有接口均为非阻塞：只做浮点运算和寄存器写入，不调用 delay_ms()。
 * - 速率限制依赖 system_time 的 1 ms 时基，调用前必须先 system_time_init()。
 *
 * 头文件规则：
 * - 只包含 BSP 公共基础头 bsp_common.h，不包含聚合头 bsp.h。
 */
#include "bsp_common.h"

/*
 * 舵机模块运行状态快照，供 LCD、串口遥测和上层调试读取。
 *
 * 字段说明：
 * - beam_angle_cmd_deg：最近一次下发的摆杆目标倾角，单位 °，正方向由
 *   SERVO_BEAM_DIR_SIGN 决定。
 * - servo_angle_cmd_deg：换算并限幅后的舵机目标角度，单位 °（0~180）。
 * - pulse_us：当前 PWM 高电平脉宽，单位 us，用于验证映射表。
 * - ready：Servo_Init() 是否已经完成。
 * - rate_limited：最近一次调用是否因为斜率限制而没有到达目标值。
 */
typedef struct {
    float    beam_angle_cmd_deg;
    float    servo_angle_cmd_deg;
    uint16_t pulse_us;
    bool     ready;
    bool     rate_limited;
} servo_status_t;

/*
 * 初始化舵机 PWM。
 *
 * 行为：
 * - 把 TIMA0 从 SysConfig 默认的 80 kHz 重新配置为 50 Hz 舵机时序；
 * - 只保留 CC1（PA9）输出，避免 PA8 继续输出无关脉冲；
 * - 立即把舵机写到零位 104°（摆杆水平），然后启动计数器。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 和 system_time_init() 之后调用。
 * - 上电只调用一次；重复调用会重新拉起定时器并强制回零位。
 */
void Servo_Init(void);

/*
 * 设置摆杆目标倾角（自动控制的主接口）。
 *
 * 参数：
 * - angle_deg：摆杆目标倾角，单位 °。正值表示“使钢珠沿位置坐标正方向加速”，
 *   实际机械方向由 SERVO_BEAM_DIR_SIGN 标定。
 *
 * 处理流程：
 * 1. 过滤 NaN/Inf，并把倾角限制在 SERVO_BEAM_LIMIT_DEG 内。
 * 2. 按 SERVO_BEAM_TO_SERVO_RATIO 换算成舵机角度，再叠加零位 104°。
 * 3. 限制到机械安全范围 SERVO_MECH_MIN_DEG ~ SERVO_MECH_MAX_DEG。
 * 4. 按 SERVO_MAX_SLEW_DPS 做变化率限制，避免舵机瞬间大幅摆动。
 * 5. 写入 TIMA0 CC1 比较值。
 *
 * 使用约束：
 * - 可以在 30 Hz 视觉节拍调用，也可以在更高频率的主循环调用；
 *   函数不阻塞，单次执行只做少量浮点运算。
 */
void Servo_SetBeamAngle(float angle_deg);

/*
 * 让摆杆以受限速率返回水平位置（摆杆倾角 0°）。
 *
 * 用途：
 * - 视觉丢失、退出 AUTO、故障保护时的“安全回中”动作。
 * - 与直接写 104° 不同，本函数会受斜率限制，不会突然大幅摆动。
 */
void Servo_SetNeutral(void);

/*
 * 直接设置舵机角度（绕过摆杆倾角映射），仅用于零位/方向标定。
 *
 * 参数：
 * - angle_deg：舵机机械角度，单位 °，0~180。
 *
 * 说明：
 * - 仍然受机械限幅和斜率限制，避免标定过程中超行程。
 * - 正常自动控制请使用 Servo_SetBeamAngle()。
 */
void Servo_SetManualServoAngle(float angle_deg);

/*
 * 运行时修改摆杆最大倾角，单位 °。
 *
 * 用途：
 * - 不同机械结构需要不同安全限幅时，可在初始化后调整。
 * - 参数会被限制在 0.1~10°；PD 控制器本身还有一层 ±2° 限幅。
 */
void Servo_SetBeamLimit(float limit_deg);

/*
 * 运行时修改摆杆正方向对应的舵机旋转方向。
 *
 * 参数：
 * - sign：>0 表示摆杆正倾角对应舵机角度增大；<0 表示相反。
 *
 * 标定方法：
 * - 手动给 +1° 倾角，观察钢珠是否向位置坐标正方向运动；
 * - 若方向相反，调用本函数传 -1 或修改 SERVO_BEAM_DIR_SIGN。
 */
void Servo_SetDirection(float sign);

/*
 * 读取模块状态快照。
 *
 * 参数：
 * - status：输出缓冲区，不能为 NULL。
 *
 * 返回值：
 * - 无。status 为 NULL 时直接返回。
 */
void Servo_GetStatus(servo_status_t *status);

/* 返回最近一次下发的摆杆目标倾角，单位 °。 */
float Servo_GetBeamAngle(void);

/* 返回最近一次换算后的舵机角度，单位 °。 */
float Servo_GetServoAngle(void);

/*
 * 返回经过斜率限制后的“实际摆杆倾角”估计，单位 °。
 *
 * 说明：
 * - Servo_GetBeamAngle() 返回的是目标值，本函数返回限速后的实际值；
 * - 钢珠动力学仿真和 LCD 调试应使用本函数，才能反映舵机逐步转动过程。
 */
float Servo_GetBeamAngleActual(void);

/* 返回当前 PWM 高电平脉宽，单位 us。 */
uint16_t Servo_GetPulseUs(void);

/* 返回 Servo_Init() 是否已完成。 */
bool Servo_IsReady(void);

#endif
