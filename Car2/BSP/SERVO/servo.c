#include "SERVO/servo.h"
#include "SystemTime/system_time.h"

/*
 * 摆杆舵机 BSP 实现。
 *
 * 实现要点：
 * 1. 复用 SysConfig 已经分配好的 PWM_0/TIMA0/PA9，不新增定时器，避免和
 *    电机 PWM（TIMG14）、蜂鸣器/RGB（TIMG12）发生资源冲突。
 * 2. ServoTest 副工程实测：本板 TIMA0 的 CC1 输出高电平时间是
 *    “周期 - 比较值”，所以比较值必须写 16000 - 脉宽计数。
 * 3. 所有动作都在函数内完成，不阻塞；斜率限制基于 1 ms 系统时基。
 *
 * 标定基准（来自 ServoTest）：
 * - 舵机 104° 时摆杆水平；
 * - 0° 对应 400 计数（0.5 ms），180° 对应 2000 计数（2.5 ms）。
 */

/* PWM 时基：80 MHz / (0 + 1) / (99 + 1) = 800 kHz，1 计数 = 1.25 us。 */
#define SERVO_PWM_PRESCALE              (99U)
#define SERVO_PWM_PERIOD_TICKS          (16000U)   /* 20 ms -> 50 Hz */
#define SERVO_PULSE_MIN_TICKS           (400U)     /* 500 us -> 0°  */
#define SERVO_PULSE_MAX_TICKS           (2000U)    /* 2500 us -> 180° */

/*
 * 零位与机械限位。
 * - 104° 是实测“摆杆水平、钢珠在 O 点附近稳定”的舵机角度。
 * - 10°~170° 避开 MG995/MG996R 的机械端点，防止堵转发热。
 * - 拆装舵盘、摆杆或改变配重后零位会漂移，需要重新标定并修改本宏。
 */
#define SERVO_NEUTRAL_ANGLE_DEG         (104.0f)
#define SERVO_MECH_MIN_DEG              (10.0f)
#define SERVO_MECH_MAX_DEG              (170.0f)

/*
 * 摆杆倾角 -> 舵机角度映射。
 *
 * SERVO_BEAM_TO_SERVO_RATIO 表示“摆杆倾角 1° 需要舵机转多少度”。
 * 当前按直连结构默认 1.0；若实测同样倾角需要更大舵机行程，可改大该值。
 *
 * SERVO_BEAM_DIR_SIGN 决定正倾角方向：
 * - +1：正倾角 -> 舵机角度增大；
 * - -1：正倾角 -> 舵机角度减小。
 * 必须用“钢珠向 +x 方向加速”为判据实测标定，不能仅凭直觉假定。
 */
#define SERVO_BEAM_TO_SERVO_RATIO       (1.0f)
#define SERVO_BEAM_DIR_SIGN             (1.0f)

/*
 * 安全限幅：
 * - 摆杆最大倾角默认 2.5°，PD 控制器还会再限制到 ±2.0°。
 * - 舵机最大角速度 20°/s，对应 30 Hz 更新时每帧最多 0.67°，
 *   避免视觉噪声直接变成舵机抖动。
 * - 单次斜率计算的 dt 上限 50 ms，防止长时间未调用后一次性跳很大角度。
 */
#define SERVO_BEAM_LIMIT_DEG_DEFAULT    (2.5f)
#define SERVO_MAX_SLEW_DPS              (20.0f)
#define SERVO_SLEW_DT_CAP_MS            (50U)

/* SysConfig 默认 TIMA0 为 80 MHz、预分频 0、周期 1000、计数器停止。 */
static const DL_TimerA_ClockConfig s_servo_clock = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = SERVO_PWM_PRESCALE
};

static servo_status_t s_status;
static float    s_servo_angle_deg = SERVO_NEUTRAL_ANGLE_DEG;
static float    s_beam_limit_deg  = SERVO_BEAM_LIMIT_DEG_DEFAULT;
static float    s_beam_dir_sign   = SERVO_BEAM_DIR_SIGN;
static uint32_t s_last_update_ms  = 0U;

/*
 * 浮点有效性检查。
 *
 * 说明：不使用 isfinite()/isnan()，避免链接额外的 libm 符号。
 * 对 NaN 来说所有比较都为 false，因此下面的写法会返回 false；
 * 对 ±Inf 则会超出阈值范围，同样返回 false。
 */
static bool servo_float_is_finite(float value)
{
    return (value >= -1.0e30f) && (value <= 1.0e30f);
}

static float servo_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/*
 * 舵机角度 -> PWM 比较值计数。
 *
 * 映射关系：0° -> 400，180° -> 2000，线性插值后四舍五入。
 * 返回值是“脉宽计数”，写寄存器时还要用 16000 - 该值。
 */
static uint16_t servo_angle_to_pulse_ticks(float angle_deg)
{
    float span = (float)(SERVO_PULSE_MAX_TICKS - SERVO_PULSE_MIN_TICKS);
    float ticks;

    angle_deg = servo_clampf(angle_deg, 0.0f, 180.0f);
    ticks = (float)SERVO_PULSE_MIN_TICKS + (span * angle_deg / 180.0f) + 0.5f;

    if (ticks < (float)SERVO_PULSE_MIN_TICKS) {
        ticks = (float)SERVO_PULSE_MIN_TICKS;
    }
    if (ticks > (float)SERVO_PULSE_MAX_TICKS) {
        ticks = (float)SERVO_PULSE_MAX_TICKS;
    }

    return (uint16_t)ticks;
}

/* 800 kHz 计数换算成 us：1 计数 = 1.25 us = 5/4 us。 */
static uint16_t servo_ticks_to_us(uint16_t ticks)
{
    return (uint16_t)(((uint32_t)ticks * 5U) / 4U);
}

/*
 * 写入 CC1 比较值并更新状态快照。
 *
 * 硬件约束：本板 CC1 高电平 = 周期 - 比较值，故此处必须取反。
 */
static void servo_write_pulse_ticks(uint16_t pulse_ticks)
{
    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
                                     SERVO_PWM_PERIOD_TICKS - pulse_ticks,
                                     DL_TIMER_CC_1_INDEX);

    s_status.pulse_us = servo_ticks_to_us(pulse_ticks);
}

/*
 * 斜率限制：把 current 朝 target 移动，单次最多移动 max_slew * dt。
 *
 * dt 的计算：
 * - 取距上次调用的毫秒差，最小 1 ms、最大 50 ms；
 * - 上限 50 ms 可防止主循环长时间停顿后一帧跳出很大角度。
 *
 * 返回值：
 * - 限制后的新角度；*limited 置 true 表示尚未到达 target。
 */
static float servo_slew_limit(float current, float target, bool *limited)
{
    uint32_t now = system_time_get_tick_ms();
    uint32_t dt_ms = (uint32_t)(now - s_last_update_ms);
    float max_step;
    float delta;

    if (dt_ms < 1U) {
        dt_ms = 1U;
    }
    if (dt_ms > SERVO_SLEW_DT_CAP_MS) {
        dt_ms = SERVO_SLEW_DT_CAP_MS;
    }
    s_last_update_ms = now;

    max_step = SERVO_MAX_SLEW_DPS * (float)dt_ms / 1000.0f;
    delta = target - current;

    if (delta > max_step) {
        float next = current + max_step;

        /* 目标可能比“当前 + 单步最大量”更近，不能越过目标。 */
        *limited = true;
        return (next > target) ? target : next;
    }
    if (delta < -max_step) {
        float next = current - max_step;

        *limited = true;
        return (next < target) ? target : next;
    }

    *limited = false;
    return target;
}

void Servo_Init(void)
{
    uint16_t neutral_ticks;

    /*
     * 重新配置 TIMA0：
     * - SysConfig 已生成 80 MHz、周期 1000、计数器停止的默认配置；
     * - 这里把预分频改为 99，得到 800 kHz 计数时钟，并改写周期为 16000。
     */
    DL_TimerA_setClockConfig(PWM_0_INST, (DL_TimerA_ClockConfig *)&s_servo_clock);
    DL_TimerA_setLoadValue(PWM_0_INST, SERVO_PWM_PERIOD_TICKS);

    /*
     * 只让 CC1（PA9）输出舵机信号。
     * CC0（PA8）在本工程没有用途，关闭输出避免它继续产生 50 Hz 脉冲。
     */
    DL_TimerA_setCCPDirection(PWM_0_INST, DL_TIMER_CC1_OUTPUT);

    /* 初始化时直接回零位，不经过斜率限制，保证上电姿态确定。 */
    s_servo_angle_deg = SERVO_NEUTRAL_ANGLE_DEG;
    neutral_ticks = servo_angle_to_pulse_ticks(s_servo_angle_deg);
    servo_write_pulse_ticks(neutral_ticks);

    s_status.beam_angle_cmd_deg  = 0.0f;
    s_status.servo_angle_cmd_deg = s_servo_angle_deg;
    s_status.ready               = true;
    s_status.rate_limited        = false;
    s_last_update_ms             = system_time_get_tick_ms();

    DL_TimerA_startCounter(PWM_0_INST);
}

void Servo_SetBeamAngle(float angle_deg)
{
    float servo_target;
    bool limited = false;

    if (!s_status.ready) {
        return;
    }

    if (!servo_float_is_finite(angle_deg)) {
        angle_deg = 0.0f;
    }

    /* 第一层限幅：摆杆倾角安全范围。 */
    angle_deg = servo_clampf(angle_deg, -s_beam_limit_deg, s_beam_limit_deg);

    /*
     * 第二层映射：摆杆倾角 -> 舵机角度。
     * 正方向由 s_beam_dir_sign 标定，角度比例由 SERVO_BEAM_TO_SERVO_RATIO 决定。
     */
    servo_target = SERVO_NEUTRAL_ANGLE_DEG +
                   (s_beam_dir_sign * SERVO_BEAM_TO_SERVO_RATIO * angle_deg);

    /* 第三层限幅：舵机机械安全范围。 */
    servo_target = servo_clampf(servo_target, SERVO_MECH_MIN_DEG, SERVO_MECH_MAX_DEG);

    /* 第四层限幅：角度变化速率。 */
    s_servo_angle_deg = servo_slew_limit(s_servo_angle_deg, servo_target, &limited);

    servo_write_pulse_ticks(servo_angle_to_pulse_ticks(s_servo_angle_deg));

    s_status.beam_angle_cmd_deg  = angle_deg;
    s_status.servo_angle_cmd_deg = s_servo_angle_deg;
    s_status.rate_limited        = limited;
}

void Servo_SetNeutral(void)
{
    Servo_SetBeamAngle(0.0f);
}

void Servo_SetManualServoAngle(float angle_deg)
{
    float clamped;
    bool limited = false;

    if (!s_status.ready) {
        return;
    }

    if (!servo_float_is_finite(angle_deg)) {
        return;
    }

    clamped = servo_clampf(angle_deg, SERVO_MECH_MIN_DEG, SERVO_MECH_MAX_DEG);
    s_servo_angle_deg = servo_slew_limit(s_servo_angle_deg, clamped, &limited);

    servo_write_pulse_ticks(servo_angle_to_pulse_ticks(s_servo_angle_deg));

    /* 手动模式下反推等效摆杆倾角，便于 LCD 同时显示两种角度。 */
    s_status.beam_angle_cmd_deg = (s_servo_angle_deg - SERVO_NEUTRAL_ANGLE_DEG) /
                                  (s_beam_dir_sign * SERVO_BEAM_TO_SERVO_RATIO);
    s_status.servo_angle_cmd_deg = s_servo_angle_deg;
    s_status.rate_limited        = limited;
}

void Servo_SetBeamLimit(float limit_deg)
{
    if (!servo_float_is_finite(limit_deg)) {
        return;
    }

    s_beam_limit_deg = servo_clampf(limit_deg, 0.1f, 10.0f);
}

void Servo_SetDirection(float sign)
{
    if (!servo_float_is_finite(sign) || (sign == 0.0f)) {
        return;
    }

    s_beam_dir_sign = (sign > 0.0f) ? 1.0f : -1.0f;
}

void Servo_GetStatus(servo_status_t *status)
{
    if (status == NULL) {
        return;
    }

    *status = s_status;
}

float Servo_GetBeamAngle(void)
{
    return s_status.beam_angle_cmd_deg;
}

float Servo_GetServoAngle(void)
{
    return s_status.servo_angle_cmd_deg;
}

float Servo_GetBeamAngleActual(void)
{
    /*
     * 反推限速后的等效摆杆倾角：
     * 舵机实际角度先减零位，再除以“方向 × 比例”，得到和 SetBeamAngle
     * 输入同一坐标系的倾角。
     */
    return (s_servo_angle_deg - SERVO_NEUTRAL_ANGLE_DEG) /
           (s_beam_dir_sign * SERVO_BEAM_TO_SERVO_RATIO);
}

uint16_t Servo_GetPulseUs(void)
{
    return s_status.pulse_us;
}

bool Servo_IsReady(void)
{
    return s_status.ready;
}
