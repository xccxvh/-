#include "App/ball_control.h"
#include "SERVO/servo.h"
#include "SystemTime/system_time.h"

/*
 * 滚球 PD 控制器实现。
 *
 * 数据流：
 *   BallMeasurement -> OnMeasurement（有效性检查、速度估计）
 *   -> Update（状态机、超时判断、PD）
 *   -> Servo_SetBeamAngle（映射、限幅、速率限制、PWM）。
 *
 * 关键约定：
 * 1. 速度只用“相邻两帧本地接收时刻”的差值，Jetson 的 frame_time_ms 不参与
 *    任何运算，避免两个绝对时钟混用；
 * 2. 视觉无效或超时后不保留旧测量继续控制，而是退出 AUTO 并限速回中；
 * 3. 所有浮点输出在 Servo 模块里还有一层机械限幅和斜率限制，本模块的
 *    ±2° 限幅是第二道保险。
 */

/* ===================== 可调参数 ===================== */

/*
 * 默认增益的数值和单位定义在 ball_control.h：
 * - Kp 0.60 °/cm：2 cm 偏差对应 1.2° 倾角；
 * - Kd 0.30 °/(cm/s)：抑制钢珠接近目标时的速度超调。
 * 调试建议：先 Kd=0 逐步加 Kp 到能收敛，再加入 Kd 消除振荡。
 */

/* PD 输出的摆杆倾角硬限幅；题目初期建议 ±2°，机械安全由 Servo 模块再限一层。 */
#define BALL_CONTROL_MAX_BEAM_DEG           (2.0f)

/* 视觉有效期：与 VISION_LINK_TIMEOUT_MS 保持一致，便于两处同时解释。 */
#define BALL_CONTROL_VISION_TIMEOUT_MS      (150U)

/* 帧间时间窗口：小于 10 ms 视为突发帧，大于 200 ms 视为断流，不参与速度计算。 */
#define BALL_CONTROL_DT_MIN_MS              (10U)
#define BALL_CONTROL_DT_MAX_MS              (200U)

/* 单帧位置跳变上限：33 ms 内超过 3 cm 视为误检测，整帧剔除。 */
#define BALL_CONTROL_POS_JUMP_MAX_CM        (3.0f)

/* 速度物理上限，超过即判为异常帧。 */
#define BALL_CONTROL_V_MAX_CM_S             (200.0f)

/* 速度一阶低通系数：越大越平滑，代价是相位滞后。 */
#define BALL_CONTROL_VEL_LPF_ALPHA          (0.60f)

/* 目标位置和坐标的物理范围。 */
#define BALL_CONTROL_TARGET_LIMIT_CM        (25.0f)
#define BALL_CONTROL_X_LIMIT_CM             (30.0f)
#define BALL_CONTROL_CONFIDENCE_MIN         (20.0f)

/* 连续位置跳变达到该次数后，认为视觉已不可信，置 vision_valid=false。 */
#define BALL_CONTROL_OUTLIER_LIMIT          (5U)

/* ===================== 内部状态 ===================== */

static ball_control_state_t s_state = BALL_STATE_IDLE;

static float    s_target_cm = 0.0f;
static float    s_kp = BALL_CONTROL_DEFAULT_KP;
static float    s_kd = BALL_CONTROL_DEFAULT_KD;

static float    s_ball_x_cm = 0.0f;
static float    s_ball_v_cm_s = 0.0f;
static float    s_error_cm = 0.0f;
static float    s_beam_cmd_deg = 0.0f;

static bool     s_have_measurement = false;
static bool     s_vision_valid = false;
static bool     s_new_measurement = false;
static uint32_t s_last_rx_ms = 0U;
static uint16_t s_last_seq = 0U;
static uint32_t s_frames_used = 0U;
static uint32_t s_frames_rejected = 0U;

/* 速度估计状态：上一有效帧的位置/时刻，以及滤波后的速度。 */
static bool     s_have_prev = false;
static float    s_prev_x_cm = 0.0f;
static uint32_t s_prev_tick_ms = 0U;
static uint8_t  s_outlier_count = 0U;

static uint8_t  s_fault_code = BALL_FAULT_NONE;

/* 手动模式目标倾角：由 SetManualBeamAngle 写入，由 Update 周期性重发。 */
static float    s_manual_beam_deg = 0.0f;

/* 回中/手动保持动作的调用节流：主循环可能跑到几十 kHz，没必要每圈都重算 PWM。 */
#define BALL_CONTROL_NEUTRAL_PERIOD_MS      (10U)
static uint32_t s_last_neutral_ms = 0U;
static uint32_t s_last_manual_ms = 0U;

/*
 * 限速回中调度。
 *
 * 每 10 ms 最多调用一次 Servo_SetNeutral()：
 * - 舵机本身有 20°/s 斜率限制，10 ms 更新一次足以平滑回中；
 * - 避免主循环高速重复写比较值造成不必要的抖动和 CPU 浪费。
 */
static void ball_control_hold_neutral(void)
{
    uint32_t now = system_time_get_tick_ms();

    if ((uint32_t)(now - s_last_neutral_ms) >= BALL_CONTROL_NEUTRAL_PERIOD_MS) {
        s_last_neutral_ms = now;
        Servo_SetNeutral();
    }
}

/* 避免 libm 依赖的有限值检查：NaN 会使比较失败并返回 false。 */
static bool ball_control_float_is_finite(float value)
{
    return (value >= -1.0e30f) && (value <= 1.0e30f);
}

static float ball_control_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ball_control_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/* 清空速度估计，状态切换时必须调用，防止把断流前后的两帧硬算成速度。 */
static void ball_control_reset_estimator(void)
{
    s_have_prev = false;
    s_prev_x_cm = 0.0f;
    s_prev_tick_ms = 0U;
    s_ball_v_cm_s = 0.0f;
    s_outlier_count = 0U;
}

/*
 * 计算并下发 PD 倾角。
 *
 * θ = Kp * (target - x) - Kd * v
 *
 * 说明：
 * - 位置误差正方向与 x 坐标正方向一致；
 * - 速度项取负号，起阻尼作用；
 * - 输出在 ±BALL_CONTROL_MAX_BEAM_DEG 内，Servo 模块还会做机械限幅和速率限制。
 */
static void ball_control_apply_pd(void)
{
    float theta;

    s_error_cm = s_target_cm - s_ball_x_cm;
    theta = (s_kp * s_error_cm) - (s_kd * s_ball_v_cm_s);

    if (!ball_control_float_is_finite(theta)) {
        theta = 0.0f;
    }

    theta = ball_control_clampf(theta,
                                -BALL_CONTROL_MAX_BEAM_DEG,
                                BALL_CONTROL_MAX_BEAM_DEG);

    s_beam_cmd_deg = theta;
    Servo_SetBeamAngle(theta);
}

void BallControl_Init(void)
{
    s_state = BALL_STATE_IDLE;
    s_target_cm = 0.0f;
    s_kp = BALL_CONTROL_DEFAULT_KP;
    s_kd = BALL_CONTROL_DEFAULT_KD;

    s_ball_x_cm = 0.0f;
    s_ball_v_cm_s = 0.0f;
    s_error_cm = 0.0f;
    s_beam_cmd_deg = 0.0f;

    s_have_measurement = false;
    s_vision_valid = false;
    s_new_measurement = false;
    s_last_rx_ms = 0U;
    s_last_seq = 0U;
    s_frames_used = 0U;
    s_frames_rejected = 0U;
    s_fault_code = BALL_FAULT_NONE;
    s_manual_beam_deg = 0.0f;
    s_last_manual_ms = 0U;
    s_last_neutral_ms = 0U;

    ball_control_reset_estimator();
}

void BallControl_SetTarget(float target_cm)
{
    if (!ball_control_float_is_finite(target_cm)) {
        return;
    }

    s_target_cm = ball_control_clampf(target_cm,
                                      -BALL_CONTROL_TARGET_LIMIT_CM,
                                      BALL_CONTROL_TARGET_LIMIT_CM);

    /*
     * 目标改变后不需要等下一帧视觉数据才动作：下一次 Update 用最近一次
     * 有效位置重算一次 PD，响应更快。
     */
    if (s_have_measurement && s_vision_valid) {
        s_new_measurement = true;
    }
}

void BallControl_SetGains(float kp, float kd)
{
    if (!ball_control_float_is_finite(kp) || !ball_control_float_is_finite(kd)) {
        return;
    }
    if ((kp < 0.0f) || (kd < 0.0f)) {
        return;
    }
    if ((kp > 100.0f) || (kd > 100.0f)) {
        return;
    }

    s_kp = kp;
    s_kd = kd;
}

void BallControl_OnMeasurement(const BallMeasurement *measurement)
{
    uint32_t now;
    float x_cm;
    bool structurally_valid;

    if (measurement == NULL) {
        return;
    }

    now = system_time_get_tick_ms();
    x_cm = measurement->x_cm;

    /*
     * 重复帧防御：
     * 视觉链路层已经按 seq 去重，但应用层（例如测试注入）也可能重复调用，
     * 这里再判一次，避免同一位置被当成两帧参与速度差分。
     */
    if (s_have_measurement && (measurement->seq == s_last_seq)) {
        s_frames_rejected++;
        return;
    }
    s_last_seq = measurement->seq;

    /*
     * 第一层有效性：Jetson 标志位、置信度、坐标范围。
     * 任一不满足都立即标记视觉无效，让 AUTO 在本次 Update 中退出。
     */
    structurally_valid = measurement->valid &&
                         (measurement->confidence >= BALL_CONTROL_CONFIDENCE_MIN) &&
                         (x_cm >= -BALL_CONTROL_X_LIMIT_CM) &&
                         (x_cm <= BALL_CONTROL_X_LIMIT_CM);

    if (!structurally_valid) {
        s_frames_rejected++;
        s_vision_valid = false;
        s_new_measurement = false;
        return;
    }

    /*
     * 第二层：位置跳变剔除。
     * 单帧位移超过 3 cm 通常是误检测或跟踪 ID 跳变，整帧丢弃，
     * 不更新 prev，也不产生速度尖峰。
     */
    if (s_have_prev) {
        float dx = x_cm - s_prev_x_cm;

        if (ball_control_absf(dx) > BALL_CONTROL_POS_JUMP_MAX_CM) {
            s_frames_rejected++;
            if (s_outlier_count < 0xFFU) {
                s_outlier_count++;
            }
            if (s_outlier_count >= BALL_CONTROL_OUTLIER_LIMIT) {
                s_vision_valid = false;
            }
            return;
        }
    }

    /*
     * 第三层：用本地接收时刻计算速度。
     *
     * 不使用 Jetson 的 frame_time_ms：两端时钟没有共同基准，直接相减会得到
     * 错误的 Δt。帧时间戳只保留在测量结构里供日志排查。
     */
    if (s_have_prev) {
        uint32_t dt_ms = (uint32_t)(now - s_prev_tick_ms);

        if ((dt_ms >= BALL_CONTROL_DT_MIN_MS) && (dt_ms <= BALL_CONTROL_DT_MAX_MS)) {
            float v_raw = ((x_cm - s_prev_x_cm) * 1000.0f) / (float)dt_ms;

            if (ball_control_absf(v_raw) <= BALL_CONTROL_V_MAX_CM_S) {
                /* 一阶低通，抑制 30 Hz 视觉的量化抖动。 */
                s_ball_v_cm_s = (BALL_CONTROL_VEL_LPF_ALPHA * s_ball_v_cm_s) +
                                ((1.0f - BALL_CONTROL_VEL_LPF_ALPHA) * v_raw);
            } else {
                s_ball_v_cm_s = 0.0f;
            }
        } else {
            /* 断流或突发帧：不允许把跨越大间隔的两帧硬算速度。 */
            s_ball_v_cm_s = 0.0f;
        }
    } else {
        s_ball_v_cm_s = 0.0f;
    }

    s_prev_x_cm = x_cm;
    s_prev_tick_ms = now;
    s_have_prev = true;
    s_outlier_count = 0U;

    s_ball_x_cm = x_cm;
    s_have_measurement = true;
    s_vision_valid = true;
    s_new_measurement = true;
    s_last_rx_ms = now;
    s_frames_used++;

    /* 不在本函数里算 PD：统一由 Update 决定当前状态是否允许输出。 */
}

void BallControl_Update(void)
{
    uint32_t now = system_time_get_tick_ms();
    uint32_t age_ms;

    switch (s_state) {
    case BALL_STATE_AUTO:
        age_ms = (uint32_t)(now - s_last_rx_ms);

        /*
         * 视觉无效或超时：立即退出闭环，转入 VISION_LOST 并限速回中。
         * 这里绝不继续使用旧位置，也不做状态预测。
         */
        if ((!s_vision_valid) || (age_ms > BALL_CONTROL_VISION_TIMEOUT_MS)) {
            s_state = BALL_STATE_VISION_LOST;
            s_new_measurement = false;
            s_beam_cmd_deg = 0.0f;
            /*
             * 清空速度估计：视觉中断期间钢珠可能已经滚了很远，
             * 恢复后的第一帧必须作为新的基线，不能拿断流前的位置硬算跳变/速度。
             */
            ball_control_reset_estimator();
            ball_control_hold_neutral();
            return;
        }

        /* 只有收到新的有效测量才重算 PD，避免同一帧重复积分/重复动作。 */
        if (s_new_measurement) {
            s_new_measurement = false;
            ball_control_apply_pd();
        }
        return;

    case BALL_STATE_VISION_LOST:
        /*
         * 视觉恢复后不自动重新闭环：必须由操作者显式 Enable，
         * 防止钢珠位置未知时突然恢复自动控制。
         */
        ball_control_hold_neutral();
        return;

    case BALL_STATE_IDLE:
        ball_control_hold_neutral();
        return;

    case BALL_STATE_FAULT:
        /* 故障期间只允许限速回中，不执行任何 PD 输出。 */
        ball_control_hold_neutral();
        return;

    case BALL_STATE_MANUAL:
    default:
        /*
         * 手动模式也要周期性重发目标角度：
         * Servo 模块按“每次调用最多转动固定角度”做斜率限制，只调用一次
         * SetBeamAngle 会停在半路。这里每 10 ms 重发一次，直到到达目标。
         */
        if ((uint32_t)(now - s_last_manual_ms) >= BALL_CONTROL_NEUTRAL_PERIOD_MS) {
            s_last_manual_ms = now;
            Servo_SetBeamAngle(s_manual_beam_deg);
        }
        return;
    }
}

bool BallControl_CanEnable(void)
{
    uint32_t now = system_time_get_tick_ms();
    uint32_t age_ms = (uint32_t)(now - s_last_rx_ms);

    return s_have_measurement &&
           s_vision_valid &&
           (age_ms <= BALL_CONTROL_VISION_TIMEOUT_MS);
}

void BallControl_Enable(bool enable)
{
    /* FAULT 必须显式 ClearFault，Enable(false) 也不允许绕过故障锁存。 */
    if (s_state == BALL_STATE_FAULT) {
        return;
    }

    if (!enable) {
        s_state = BALL_STATE_IDLE;
        s_new_measurement = false;
        s_beam_cmd_deg = 0.0f;
        ball_control_reset_estimator();
        Servo_SetNeutral();
        return;
    }

    if (!Servo_IsReady()) {
        s_fault_code = BALL_FAULT_SERVO;
        s_state = BALL_STATE_FAULT;
        Servo_SetNeutral();
        return;
    }

    if (!BallControl_CanEnable()) {
        /*
         * 没有新鲜有效的视觉数据时拒绝进入 AUTO：
         * - 若之前是 MANUAL，则退回 IDLE；
         * - 不输出任何 PD 动作。
         */
        s_state = BALL_STATE_IDLE;
        s_new_measurement = false;
        Servo_SetNeutral();
        return;
    }

    s_state = BALL_STATE_AUTO;
    s_fault_code = BALL_FAULT_NONE;

    /*
     * 以最近一次有效位置作为速度差分的起点，速度从 0 开始，
     * 避免把使能前不确定的历史帧算进速度。
     */
    s_have_prev = true;
    s_prev_x_cm = s_ball_x_cm;
    s_prev_tick_ms = system_time_get_tick_ms();
    s_ball_v_cm_s = 0.0f;
    s_outlier_count = 0U;

    /* 下一次 Update 立即用已有测量算一次，缩短使能后的空窗。 */
    s_new_measurement = true;
}

void BallControl_EnterManual(void)
{
    s_state = BALL_STATE_MANUAL;
    s_new_measurement = false;
    s_manual_beam_deg = 0.0f;
    s_beam_cmd_deg = 0.0f;
    s_last_manual_ms = system_time_get_tick_ms() - BALL_CONTROL_NEUTRAL_PERIOD_MS;
    ball_control_reset_estimator();
    Servo_SetNeutral();
}

void BallControl_SetManualBeamAngle(float angle_deg)
{
    if (s_state != BALL_STATE_MANUAL) {
        return;
    }
    if (!ball_control_float_is_finite(angle_deg)) {
        return;
    }

    s_manual_beam_deg = angle_deg;
    s_beam_cmd_deg = angle_deg;

    /* 立即执行本次调用，并让 Update 的 10 ms 保持节拍马上重新计时。 */
    s_last_manual_ms = system_time_get_tick_ms() - BALL_CONTROL_NEUTRAL_PERIOD_MS;
    Servo_SetBeamAngle(angle_deg);
}

void BallControl_NotifyFault(uint8_t fault_code)
{
    if (fault_code == BALL_FAULT_NONE) {
        return;
    }

    /*
     * 应用层可能在链路持续故障时每圈都调用本函数；
     * 已经处于同一故障码的 FAULT 状态时直接返回，避免重复回中和清速度。
     */
    if ((s_state == BALL_STATE_FAULT) && (s_fault_code == fault_code)) {
        return;
    }

    s_fault_code = fault_code;
    s_state = BALL_STATE_FAULT;
    s_new_measurement = false;
    s_beam_cmd_deg = 0.0f;
    ball_control_reset_estimator();
    Servo_SetNeutral();
}

void BallControl_ClearFault(void)
{
    s_fault_code = BALL_FAULT_NONE;
    if (s_state == BALL_STATE_FAULT) {
        s_state = BALL_STATE_IDLE;
    }
    s_new_measurement = false;
    s_beam_cmd_deg = 0.0f;
    ball_control_reset_estimator();
    Servo_SetNeutral();
}

bool BallControl_IsEnabled(void)
{
    return s_state == BALL_STATE_AUTO;
}

ball_control_state_t BallControl_GetState(void)
{
    return s_state;
}

float BallControl_GetTarget(void)
{
    return s_target_cm;
}

void BallControl_GetGains(float *kp, float *kd)
{
    if (kp != NULL) {
        *kp = s_kp;
    }
    if (kd != NULL) {
        *kd = s_kd;
    }
}

float BallControl_GetBeamAngleCmd(void)
{
    return s_beam_cmd_deg;
}

float BallControl_GetBallX(void)
{
    return s_ball_x_cm;
}

float BallControl_GetBallV(void)
{
    return s_ball_v_cm_s;
}

void BallControl_GetDebug(ball_debug_t *debug)
{
    servo_status_t servo;
    uint32_t now;

    if (debug == NULL) {
        return;
    }

    now = system_time_get_tick_ms();
    Servo_GetStatus(&servo);

    debug->time_ms = now;
    debug->target_cm = s_target_cm;
    debug->ball_x_cm = s_ball_x_cm;
    debug->ball_v_cm_s = s_ball_v_cm_s;
    debug->error_cm = s_target_cm - s_ball_x_cm;
    debug->beam_angle_cmd_deg = s_beam_cmd_deg;
    debug->servo_pwm_us = servo.pulse_us;
    debug->control_state = (uint8_t)s_state;
    debug->fault_code = s_fault_code;
    debug->vision_valid = s_vision_valid &&
                          ((uint32_t)(now - s_last_rx_ms) <= BALL_CONTROL_VISION_TIMEOUT_MS);
    debug->auto_enabled = (s_state == BALL_STATE_AUTO);
    debug->vision_age_ms = now - s_last_rx_ms;
    debug->frames_used = s_frames_used;
    debug->frames_rejected = s_frames_rejected;
    debug->last_seq = s_last_seq;
}

const char *BallControl_StateText(ball_control_state_t state)
{
    switch (state) {
    case BALL_STATE_IDLE:
        return "IDLE";
    case BALL_STATE_MANUAL:
        return "MANUAL";
    case BALL_STATE_AUTO:
        return "AUTO";
    case BALL_STATE_VISION_LOST:
        return "VLOST";
    case BALL_STATE_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}
