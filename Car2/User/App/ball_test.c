#include "App/ball_test.h"
#include "App/ball_control.h"
#include "SERVO/servo.h"
#include "VISION/vision_link.h"
#include "bsp.h"

/*
 * 滚球控制测试应用实现。
 *
 * 本文件是“整机入口”，只负责：
 * - 初始化 LCD、按键、舵机、视觉链路和 PD 控制器；
 * - 按当前测试模式提供输入（手动微调 / 自由舵机 ±30° / 开环 5/7.5/10° / 模拟视觉 / 真实 UART）；
 * - 处理按键、刷新 LCD、发送遥测。
 *
 * 真正的 PD 算法在 App/ball_control.c，底层 PWM 在 BSP/SERVO/servo.c，
 * 三者严格分层，便于单独修改和测试。
 *
 * 开环滚动（ROLL）是纯时间驱动的状态机，K1 启动后循环
 * 执行左右倾斜，直到再次按 K1 停止。
 * SERVO 模式用于不依赖视觉的自由舵机角度检查：K1 +3°、K2 -3°，范围 ±30°。
 *
 * 时序：
 * - 模拟视觉 33 ms 注入一次，接近 Jetson 30 FPS；
 * - 遥测 100 ms 一帧，非阻塞发送；
 * - LCD 逐行增量刷新，每行最多 50 ms 一次，避免整屏阻塞。
 */

/* ===================== 测试场景定义 ===================== */

#define BALL_TEST_SIM_PERIOD_MS         (33U)   /* 约 30 Hz */
#define BALL_TEST_TELEMETRY_MS          (100U)

/*
 * LCD 采用“逐行缓存 + 每次最多刷新一行”的策略：
 * - SPI0 为 8 MHz，一屏 7 行全量重绘约 60 ms，会破坏 30 Hz 控制节拍；
 * - 每次只写一行（约 8 ms），且仅在内容变化时写；
 * - 两行之间至少间隔 50 ms，把写屏开销摊平到多个主循环周期。
 */
#define BALL_TEST_LCD_LINE_CHARS        (34U)
#define BALL_TEST_LCD_LINE_HEIGHT       (20U)
#define BALL_TEST_LCD_LINES             (7U)
#define BALL_TEST_LCD_LINE_INTERVAL_MS  (50U)

/*
 * 开环滚动的摆杆目标倾角（°，相对标定后的平衡点）。
 * 当前档位：5° / 10° / 15°，由 K2 循环选择。
 */
static const float s_roll_angles[BALL_TEST_ROLL_LEVEL_COUNT] = {
    5.0f, 7.50f, 10.0f
};

/* 测试 B 的模拟场景，顺序与题目表格一致，另加一个闭环动力学场景。 */
typedef enum {
    BALL_TEST_SIM_X_NEG = 0,   /* 目标 0，实际 -2，期望正倾角 */
    BALL_TEST_SIM_X_POS,       /* 目标 0，实际 +2，期望负倾角 */
    BALL_TEST_SIM_CENTER,      /* 目标 0，实际 0、速度 0，期望水平 */
    BALL_TEST_SIM_TARGET_POS,  /* 目标 +5，实际 +3，期望正倾角 */
    BALL_TEST_SIM_INVALID,     /* 数据失效，期望退出 AUTO */
    BALL_TEST_SIM_DYNAMIC,     /* 钢珠动力学仿真闭环 */
    BALL_TEST_SIM_COUNT
} ball_test_sim_case_t;

/* 模拟钢珠动力学参数：滚动球体等效加速度 a = (5/7) g sinθ，cm/s²。 */
#define BALL_TEST_SIM_DT_S              (0.0333f)
#define BALL_TEST_SIM_G_TERM_CM_S2      (981.0f * 5.0f / 7.0f)
#define BALL_TEST_SIM_DEG_TO_RAD        (0.0174533f)
#define BALL_TEST_SIM_TRAVEL_CM         (18.0f)
#define BALL_TEST_SIM_RAIL_RESTITUTION  (0.25f)

/* ===================== 开环滚动状态机 ===================== */

/*
 * 阶段说明：
 *   NEUTRAL -> POSITIVE -> RETURN_POS -> NEGATIVE -> RETURN_NEG -> POSITIVE ...
 * 只有到位超时才进入 FINISH（ABRT）。
 *
 * 枚举定义放在 ball_test.h，便于 LCD、串口日志和主机测试读取当前阶段。
 * 每个阶段都先等舵机实际到达目标倾角（SETTLE_TOL_DEG），
 * 再按配置的保持时间停留，避免“目标还没到位就开始计时”造成倾角保持不足。
 */

/* ===================== 内部状态 ===================== */

static ball_test_mode_t s_mode = BALL_TEST_MODE_UART;
static float    s_manual_target_deg = 0.0f;  /* MANUAL 0.1° / SERVO 3° 微调目标 */
static uint8_t  s_sim_case = 0U;

/* 开环滚动状态：只在主循环更新，不需要 volatile。 */
static ball_test_roll_state_t s_roll_state = BALL_ROLL_IDLE;
static bool     s_roll_running = false;
static bool     s_roll_aborted = false;
static uint8_t  s_roll_level = 0U;           /* 0..2 -> 5/7.5/10° */
static bool     s_roll_settled = false;
static uint32_t s_roll_settle_tick_ms = 0U;
static uint32_t s_roll_hold_tick_ms = 0U;

static uint16_t s_sim_seq = 0U;
static float    s_sim_x_cm = 0.0f;
static float    s_sim_v_cm_s = 0.0f;

static uint32_t s_sim_tick_ms = 0U;
static uint32_t s_telemetry_tick_ms = 0U;

/* LCD 行缓存：只有内容变化时才重写该行，避免无谓的 SPI 阻塞。 */
static char     s_lcd_lines[BALL_TEST_LCD_LINES][BALL_TEST_LCD_LINE_CHARS + 1U];
static char     s_lcd_cache[BALL_TEST_LCD_LINES][BALL_TEST_LCD_LINE_CHARS + 1U];
static bool     s_lcd_cache_valid[BALL_TEST_LCD_LINES];
static uint8_t  s_lcd_scan_index = 0U;
static uint32_t s_lcd_line_tick_ms = 0U;
static bool     s_lcd_vision_ok = false;   /* 供第 4 行颜色判断使用 */

static bool     s_have_forwarded = false;
static uint16_t s_last_forwarded_seq = 0U;

/* REMOTE 模式状态：Nano 0x02 目标、使能、超时和 0x82 回执。 */
static bool     s_remote_enabled = false;
static uint16_t s_remote_seq = 0U;
static uint16_t s_remote_target_ticks = 0U;
static uint8_t  s_remote_result = BALL_REMOTE_RESULT_STOP;
static uint32_t s_remote_apply_tick_ms = 0U;
static uint32_t s_remote_telemetry_tick_ms = 0U;
static uint32_t s_remote_enable_tick_ms = 0U;

/* 0x03 控制指令去重和最近一次回执结果，供 LCD 显示拒绝原因。 */
static bool     s_have_control_seq = false;
static uint16_t s_last_control_seq = 0U;
static uint8_t  s_last_cmd_result = BALL_CMD_RESULT_OK;

/* ===================== 小工具 ===================== */

/*
 * 把浮点格式化成“符号 + 整数.两位小数”，避免使用 printf 的 %f。
 *
 * Keil 默认的 printf 浮点支持开销较大，且本项目已有大量整型打印；
 * 这里手工放大 100 倍后按整数输出，精度足够 LCD 和日志使用。
 */
static void ball_test_fmt2(char *buf, float value)
{
    int32_t scaled;
    char sign = '+';

    if (value >= 0.0f) {
        scaled = (int32_t)(value * 100.0f + 0.5f);
    } else {
        scaled = (int32_t)(value * 100.0f - 0.5f);
    }

    if (scaled < 0) {
        sign = '-';
        scaled = -scaled;
    }

    sprintf(buf, "%c%ld.%02ld",
            sign,
            (long)(scaled / 100),
            (long)(scaled % 100));
}

/*
 * 固定宽度补齐空格，保证 LCD 覆盖写入时不会留下上一帧的长字符串。
 *
 * 调用者必须保证 line 缓冲区至少 BALL_TEST_LCD_LINE_CHARS + 1 字节；
 * 本文件内所有行缓冲区均为 48 字节，满足该约束。
 */
static void ball_test_pad_line(char *line)
{
    uint16_t len = (uint16_t)strlen(line);

    while (len < BALL_TEST_LCD_LINE_CHARS) {
        line[len] = ' ';
        len++;
    }
    line[len] = '\0';
}

static const char *ball_test_mode_name(ball_test_mode_t mode)
{
    switch (mode) {
    case BALL_TEST_MODE_MANUAL:
        return "MANUAL";
    case BALL_TEST_MODE_SIM:
        return "SIM";
    case BALL_TEST_MODE_UART:
        return "UART";
    case BALL_TEST_MODE_ROLL:
        return "ROLL";
    case BALL_TEST_MODE_SERVO:
        return "SERVO";
    case BALL_TEST_MODE_REMOTE:
        return "REMOTE";
    default:
        return "?";
    }
}

/* 开环滚动阶段的短名称，用于 LCD。 */
static const char *ball_test_roll_state_name(ball_test_roll_state_t state)
{
    switch (state) {
    case BALL_ROLL_NEUTRAL:
        return "CENTER";
    case BALL_ROLL_POSITIVE:
        return "POS";
    case BALL_ROLL_RETURN_POS:
        return "BACK+";
    case BALL_ROLL_NEGATIVE:
        return "NEG";
    case BALL_ROLL_RETURN_NEG:
        return "BACK-";
    case BALL_ROLL_FINISH:
        return "DONE";
    case BALL_ROLL_IDLE:
    default:
        return "IDLE";
    }
}

/* 0x83 控制回执结果的短字符串。 */
static const char *ball_test_cmd_result_text(uint8_t result)
{
    switch (result) {
    case BALL_CMD_RESULT_OK:
        return "OK";
    case BALL_CMD_RESULT_BAD_MODE:
        return "BADM";
    case BALL_CMD_RESULT_VISION_INVALID:
        return "VISN";
    case BALL_CMD_RESULT_PARAM_RANGE:
        return "RNG";
    case BALL_CMD_RESULT_FAULT:
        return "FLT";
    case BALL_CMD_RESULT_REJECTED:
    default:
        return "REJ";
    }
}

/* 0x82 远程执行结果的短字符串。 */
static const char *ball_test_remote_result_text(uint8_t result)
{
    switch (result) {
    case BALL_REMOTE_RESULT_OK:
        return "OK";
    case BALL_REMOTE_RESULT_STOP:
        return "STOP";
    case BALL_REMOTE_RESULT_TIMEOUT:
        return "TMO";
    case BALL_REMOTE_RESULT_NO_CMD:
        return "NOCMD";
    case BALL_REMOTE_RESULT_CLAMP:
        return "CLAMP";
    default:
        return "?";
    }
}

static const char *ball_test_sim_case_name(uint8_t sim_case)
{
    switch (sim_case) {
    case BALL_TEST_SIM_X_NEG:
        return "X=-2";
    case BALL_TEST_SIM_X_POS:
        return "X=+2";
    case BALL_TEST_SIM_CENTER:
        return "CENTER";
    case BALL_TEST_SIM_TARGET_POS:
        return "TGT+5";
    case BALL_TEST_SIM_INVALID:
        return "INVALID";
    case BALL_TEST_SIM_DYNAMIC:
        return "DYNAMIC";
    default:
        return "?";
    }
}

static uint16_t ball_test_lcd_color_for_state(ball_control_state_t state)
{
    switch (state) {
    case BALL_STATE_AUTO:
        return GREEN;
    case BALL_STATE_VISION_LOST:
        return YELLOW;
    case BALL_STATE_FAULT:
        return RED;
    case BALL_STATE_MANUAL:
        return CYAN;
    case BALL_STATE_IDLE:
    default:
        return WHITE;
    }
}

/* ===================== 模拟视觉 ===================== */

/*
 * 计算一帧模拟测量并喂给控制器。
 *
 * 静态场景直接给出题目表格中的位置；DYNAMIC 场景用滚球动力学模型
 * 根据“实际摆杆倾角”推进钢珠状态，形成真正的闭环仿真。
 */
static void ball_test_inject_simulated(void)
{
    BallMeasurement measurement;
    float target_cm = 0.0f;
    float x_cm = 0.0f;
    bool valid = true;

    switch (s_sim_case) {
    case BALL_TEST_SIM_X_NEG:
        x_cm = -2.0f;
        break;

    case BALL_TEST_SIM_X_POS:
        x_cm = 2.0f;
        break;

    case BALL_TEST_SIM_CENTER:
        x_cm = 0.0f;
        break;

    case BALL_TEST_SIM_TARGET_POS:
        target_cm = 5.0f;
        x_cm = 3.0f;
        break;

    case BALL_TEST_SIM_INVALID:
        x_cm = 0.0f;
        valid = false;
        break;

    case BALL_TEST_SIM_DYNAMIC:
    default:
    {
        /*
         * 滚动钢珠：x'' = (5/7) g sinθ。
         * PD 输出不超过 ±2°，sinθ 与 θ 的误差小于 0.03%，直接用弧度近似。
         */
        float theta_rad = Servo_GetBeamAngleActual() * BALL_TEST_SIM_DEG_TO_RAD;
        float accel = BALL_TEST_SIM_G_TERM_CM_S2 * theta_rad;

        s_sim_v_cm_s += accel * BALL_TEST_SIM_DT_S;
        s_sim_x_cm += s_sim_v_cm_s * BALL_TEST_SIM_DT_S;

        /* 摆杆两端当成挡板，碰撞后损失大部分动能，便于观察长期有界性。 */
        if (s_sim_x_cm > BALL_TEST_SIM_TRAVEL_CM) {
            s_sim_x_cm = BALL_TEST_SIM_TRAVEL_CM;
            s_sim_v_cm_s = -s_sim_v_cm_s * BALL_TEST_SIM_RAIL_RESTITUTION;
        } else if (s_sim_x_cm < -BALL_TEST_SIM_TRAVEL_CM) {
            s_sim_x_cm = -BALL_TEST_SIM_TRAVEL_CM;
            s_sim_v_cm_s = -s_sim_v_cm_s * BALL_TEST_SIM_RAIL_RESTITUTION;
        }

        x_cm = s_sim_x_cm;
        break;
    }
    }

    measurement.x_cm = x_cm;
    measurement.frame_time_ms = system_time_get_tick_ms();
    measurement.seq = ++s_sim_seq;
    measurement.confidence = 95.0f;
    measurement.valid = valid;

    BallControl_SetTarget(target_cm);
    BallControl_OnMeasurement(&measurement);
}

/* ===================== 真实 UART 数据 ===================== */

/*
 * 读取一帧新的真实测量并转发给控制器。
 *
 * 说明：
 * - Vision_GetLatest() 返回 false 可能是“超时”或“帧本身无效”，
 *   两种情况都要让控制器知道；
 * - 用 seq 去重，避免同一帧在主循环里被重复喂入；
 * - 只处理 1 s 内收到过的帧，更早的旧帧没有意义。
 */
static void ball_test_poll_uart(void)
{
    BallMeasurement measurement;
    vision_link_status_t status;
    uint32_t age_ms = 0U;

    /* 上电后还没收到任何完整帧时，不要用零初始化的 seq 参与去重。 */
    Vision_GetStatus(&status);
    if (!status.have_frame) {
        return;
    }

    (void)Vision_GetLatest(&measurement, &age_ms);

    /*
     * 只转发仍在视觉有效期内的帧：
     * - 明确无效但刚收到的帧（age 很小）会转发，让控制器立即退出 AUTO；
     * - 已经超龄的旧帧不再转发，避免旧数据把控制器的超时计时搅乱；
     * - 完全没有新帧时，BallControl 自己按最后一次有效测量计时 150 ms。
     */
    if (age_ms > VISION_LINK_TIMEOUT_MS) {
        return;
    }

    if (s_have_forwarded && (measurement.seq == s_last_forwarded_seq)) {
        return;
    }

    s_have_forwarded = true;
    s_last_forwarded_seq = measurement.seq;

    BallControl_OnMeasurement(&measurement);
}

/* 链路连续解码错误时映射为 FAULT，防止错误数据继续驱动舵机。 */
static void ball_test_check_link_fault(void)
{
    vision_link_status_t status;

    Vision_GetStatus(&status);
    if (status.link_fault) {
        BallControl_NotifyFault(BALL_FAULT_VISION_LINK);
    }
}

/* ===================== 开环滚动 ===================== */

/* 当前档位对应的摆杆倾角，单位 °（当前配置 5/7.5/10）。 */
static float ball_test_roll_level_angle(void)
{
    if (s_roll_level >= BALL_TEST_ROLL_LEVEL_COUNT) {
        s_roll_level = 0U;
    }
    return s_roll_angles[s_roll_level];
}

/* 各阶段的目标倾角：正向 +θ，反向 -θ，其余阶段回平衡点 0°。 */
static float ball_test_roll_target_for_state(ball_test_roll_state_t state)
{
    float theta = ball_test_roll_level_angle();

    switch (state) {
    case BALL_ROLL_POSITIVE:
        return theta;
    case BALL_ROLL_NEGATIVE:
        return -theta;
    case BALL_ROLL_IDLE:
    case BALL_ROLL_NEUTRAL:
    case BALL_ROLL_RETURN_POS:
    case BALL_ROLL_RETURN_NEG:
    case BALL_ROLL_FINISH:
    default:
        return 0.0f;
    }
}

/* 各阶段的保持时间，单位 ms；0 表示该阶段不额外停留。 */
static uint32_t ball_test_roll_hold_for_state(ball_test_roll_state_t state)
{
    switch (state) {
    case BALL_ROLL_POSITIVE:
        return BALL_TEST_ROLL_POSITIVE_HOLD_MS;
    case BALL_ROLL_NEGATIVE:
        return BALL_TEST_ROLL_NEGATIVE_HOLD_MS;
    case BALL_ROLL_NEUTRAL:
    case BALL_ROLL_RETURN_POS:
    case BALL_ROLL_RETURN_NEG:
        return BALL_TEST_ROLL_CENTER_HOLD_MS;
    default:
        return 0U;
    }
}

/*
 * 进入某个阶段：
 * - 记录阶段开始时刻，等待舵机实际到达；
 * - 通过 BallControl 的手动接口下发目标（仍受 Servo 斜率/机械限幅）。
 */
static void ball_test_roll_enter(ball_test_roll_state_t state)
{
    char target_text[12];

    s_roll_state = state;
    s_roll_settled = false;
    s_roll_settle_tick_ms = system_time_get_tick_ms();
    BallControl_SetManualBeamAngle(ball_test_roll_target_for_state(state));

    /*
     * 阶段切换只在按键/计时事件发生，打印开销可以忽略；
     * 输出目标角便于记录每档的实际动作时间线。
     */
    ball_test_fmt2(target_text, ball_test_roll_target_for_state(state));
    printf("[ROLL] stage=%s target=%s deg\r\n",
           ball_test_roll_state_name(state), target_text);
}

/* 中止测试：停止后续动作，摆杆限速回平衡点，并显示 ABORT。 */
static void ball_test_roll_abort(void)
{
    s_roll_running = false;
    s_roll_aborted = true;
    s_roll_state = BALL_ROLL_FINISH;
    s_roll_settled = false;

    /* 中止即退出测试，立刻恢复默认 5.0° 安全限幅，再限速回平衡点。 */
    Servo_SetBeamLimit(SERVO_BEAM_LIMIT_DEFAULT_DEG);
    BallControl_SetManualBeamAngle(0.0f);

    printf("[ROLL] ABORT: target not reached in %u ms\r\n",
           (unsigned)BALL_TEST_ROLL_SETTLE_TIMEOUT_MS);
}

/* 启动连续左右滚动（K1 再按停止）。 */
static void ball_test_roll_start(void)
{
    char theta_text[12];

    ball_test_fmt2(theta_text, ball_test_roll_level_angle());

    /*
     * 只有测试真正开始时才把摆杆上限放宽到 5°；
     * 空闲、停止和结束后都保持默认 5.0°，避免非测试状态下也能大幅摆动。
     */
    Servo_SetBeamLimit(BALL_TEST_ROLL_BEAM_LIMIT_DEG);

    s_roll_running = true;
    s_roll_aborted = false;
    ball_test_roll_enter(BALL_ROLL_NEUTRAL);

    printf("[ROLL] start level=%u theta=%s deg\r\n",
           (unsigned)(s_roll_level + 1U), theta_text);
}

/*
 * 停止自动测试（K1 再按一次）。
 *
 * 只做两件事：清除运行标志 + 目标回 0°。
 * 不清空 s_roll_state 的显示值，方便 LCD 看到是从哪个阶段退出的；
 * 但后续阶段不会再被执行。
 */
static void ball_test_roll_stop(void)
{
    s_roll_running = false;
    s_roll_state = BALL_ROLL_IDLE;
    s_roll_settled = false;

    /* 退出测试立即恢复默认限幅，然后限速回平衡点。 */
    Servo_SetBeamLimit(SERVO_BEAM_LIMIT_DEFAULT_DEG);
    BallControl_SetManualBeamAngle(0.0f);

    printf("[ROLL] stopped by key, returning to balance point\r\n");
}

/*
 * 阶段推进：正负方向循环执行，直到操作者按 K1 停止。
 *
 * 循环路径：
 *   NEUTRAL -> POSITIVE -> RETURN_POS -> NEGATIVE -> RETURN_NEG
 *           -> POSITIVE -> ...（保持运行标志为 true）
 *
 * 只有在以下情况才退出循环：
 * - 操作者按 K1 调用 ball_test_roll_stop()；
 * - 到位超时调用 ball_test_roll_abort()；
 * - 切换模式调用 BallTest_SetMode()。
 *
 * 退出时统一恢复默认 5.0° 限幅并限速回平衡点。
 */
static void ball_test_roll_advance(void)
{
    switch (s_roll_state) {
    case BALL_ROLL_NEUTRAL:
        ball_test_roll_enter(BALL_ROLL_POSITIVE);
        break;
    case BALL_ROLL_POSITIVE:
        ball_test_roll_enter(BALL_ROLL_RETURN_POS);
        break;
    case BALL_ROLL_RETURN_POS:
        ball_test_roll_enter(BALL_ROLL_NEGATIVE);
        break;
    case BALL_ROLL_NEGATIVE:
        ball_test_roll_enter(BALL_ROLL_RETURN_NEG);
        break;
    case BALL_ROLL_RETURN_NEG:
        /*
         * 不结束，直接进入下一轮 +θ。
         * RETURN_NEG 阶段本身已经等待摆杆回到平衡点并保持 CENTER_HOLD_MS，
         * 因此这里不再额外插入一次 NEUTRAL，避免两轮之间停顿过长。
         */
        ball_test_roll_enter(BALL_ROLL_POSITIVE);
        break;
    case BALL_ROLL_FINISH:
    case BALL_ROLL_IDLE:
    default:
        break;
    }
}

/*
 * 开环滚动任务，由主循环每圈调用。
 *
 * 时间逻辑：
 * 1. 先等实际倾角进入 ±0.2° 到达窗口；
 * 2. 到达后才开始计算该阶段保持时间；
 * 3. 保持时间到则推进到下一阶段；
 * 4. 超过 3 s 仍未到位则中止并回平衡点。
 *
 * 不使用 delay，不阻塞其他任务。
 */
static void ball_test_roll_task(void)
{
    uint32_t now;
    float target_deg;
    float actual_deg;
    uint32_t hold_ms;

    if (!s_roll_running) {
        return;
    }

    now = system_time_get_tick_ms();
    target_deg = ball_test_roll_target_for_state(s_roll_state);
    actual_deg = Servo_GetBeamAngleActual();

    if (!s_roll_settled) {
        float error = actual_deg - target_deg;

        if (error < 0.0f) {
            error = -error;
        }

        if (error <= BALL_TEST_ROLL_SETTLE_TOL_DEG) {
            s_roll_settled = true;
            s_roll_hold_tick_ms = now;
            return;
        }

        if ((uint32_t)(now - s_roll_settle_tick_ms) >=
            BALL_TEST_ROLL_SETTLE_TIMEOUT_MS) {
            ball_test_roll_abort();
        }
        return;
    }

    hold_ms = ball_test_roll_hold_for_state(s_roll_state);
    if (hold_ms == 0U) {
        ball_test_roll_advance();
        return;
    }

    if ((uint32_t)(now - s_roll_hold_tick_ms) >= hold_ms) {
        ball_test_roll_advance();
    }
}

/* ===================== 按键与模式 ===================== */

/*
 * 手动微调摆杆倾角，单位 °。
 *
 * 受 Servo 当前摆杆限幅约束（MANUAL 默认 5°，SERVO 模式 30°），
 * 底层还有 20°/s 斜率限制。
 * MANUAL 用 0.1° 步长逼近平衡点；SERVO 用 3° 步长做自由舵机角度测试。
 */
static void ball_test_manual_adjust(float delta_deg)
{
    float limit = Servo_GetBeamLimit();
    char beam_text[12];
    char servo_target_text[12];
    char servo_current_text[12];

    s_manual_target_deg += delta_deg;

    if (s_manual_target_deg > limit) {
        s_manual_target_deg = limit;
    } else if (s_manual_target_deg < -limit) {
        s_manual_target_deg = -limit;
    }

    BallControl_SetManualBeamAngle(s_manual_target_deg);

    /*
     * 标定日志：同时给出“相对平衡点的倾角”和“绝对舵机角度”。
     * 找到摆杆水平的位置后，把这里的 servo 值写回
     * SERVO_NEUTRAL_ANGLE_DEG 即可把该位置定义为新的 0°。
     */
    ball_test_fmt2(beam_text, s_manual_target_deg);
    ball_test_fmt2(servo_target_text, Servo_GetServoAngleTarget());
    ball_test_fmt2(servo_current_text, Servo_GetServoAngle());
    printf("[CAL] beam=%s -> servo target=%s current=%s deg\r\n",
           beam_text, servo_target_text, servo_current_text);
}

static void ball_test_advance_sim_case(void)
{
    s_sim_case = (uint8_t)((s_sim_case + 1U) % BALL_TEST_SIM_COUNT);

    /* 切换场景时清空仿真状态，避免带着上一种场景的速度进入下一种。 */
    s_sim_x_cm = 0.0f;
    s_sim_v_cm_s = 0.0f;
}

/*
 * ROLL 模式切换倾角档位。
 *
 * 运行中忽略，避免测试进行到一半目标角突然变化。
 * 切换后回到 IDLE/0°，等操作者重新按 K1 启动。
 */
static void ball_test_advance_roll_level(void)
{
    char theta_text[12];

    if (s_roll_running) {
        return;
    }

    s_roll_level = (uint8_t)((s_roll_level + 1U) % BALL_TEST_ROLL_LEVEL_COUNT);
    s_roll_state = BALL_ROLL_IDLE;
    s_roll_aborted = false;
    s_roll_settled = false;
    BallControl_SetManualBeamAngle(0.0f);

    ball_test_fmt2(theta_text, ball_test_roll_level_angle());
    printf("[ROLL] level=%u theta=%s deg\r\n",
           (unsigned)(s_roll_level + 1U), theta_text);
}

static void ball_test_next_mode(void)
{
    ball_test_mode_t next;

    switch (s_mode) {
    case BALL_TEST_MODE_MANUAL:
        next = BALL_TEST_MODE_REMOTE;
        break;
    case BALL_TEST_MODE_REMOTE:
        next = BALL_TEST_MODE_UART;
        break;
    case BALL_TEST_MODE_UART:
        next = BALL_TEST_MODE_MANUAL;
        break;
    case BALL_TEST_MODE_SIM:
    case BALL_TEST_MODE_ROLL:
    case BALL_TEST_MODE_SERVO:
    default:
        next = BALL_TEST_MODE_MANUAL;
        break;
    }

    BallTest_SetMode(next);
}

/*
 * REMOTE 目标 PWM 计数限幅：
 * - 先限制在零位 ±BALL_TEST_REMOTE_LIMIT_DEG（初始 ±5°）；
 * - 再交给 Servo_ClampPulseTicks() 做 10°~170° 机械限幅。
 */
static uint16_t ball_test_remote_clamp_ticks(uint16_t raw_ticks)
{
    int32_t neutral = (int32_t)Servo_GetNeutralPulseTicks();
    int32_t limit_ticks = (int32_t)(BALL_TEST_REMOTE_LIMIT_DEG * 1600.0f /
                                    180.0f + 0.5f);
    int32_t ticks = (int32_t)raw_ticks;

    if (ticks < (neutral - limit_ticks)) {
        ticks = neutral - limit_ticks;
    } else if (ticks > (neutral + limit_ticks)) {
        ticks = neutral + limit_ticks;
    }

    return Servo_ClampPulseTicks((uint16_t)ticks);
}

/* 发送 0x82 远程舵机回执（12 字节，非阻塞）。 */
static void ball_test_remote_send_telemetry(void)
{
    uint8_t payload[VISION_LINK_REMOTE_TELEMETRY_PAYLOAD_LEN];
    uint16_t output_ticks = Servo_GetPulseTicks();

    payload[0] = (uint8_t)(s_remote_seq & 0xFFU);
    payload[1] = (uint8_t)(s_remote_seq >> 8);
    Vision_WriteU32LE(&payload[2], system_time_get_tick_ms());
    payload[6] = (uint8_t)(s_remote_target_ticks & 0xFFU);
    payload[7] = (uint8_t)(s_remote_target_ticks >> 8);
    payload[8] = (uint8_t)(output_ticks & 0xFFU);
    payload[9] = (uint8_t)(output_ticks >> 8);
    payload[10] = s_remote_enabled ? 1U : 0U;
    payload[11] = s_remote_result;

    (void)Vision_SendFrame(VISION_LINK_TYPE_REMOTE_TELEMETRY,
                           payload, sizeof(payload));
}

/*
 * REMOTE 任务：
 * - 已使能时，最近一次 0x02 超过 300 ms 未更新则退出远程控制并回零；
 * - 目标 PWM 计数先经 Servo 机械限幅，再每 10 ms 下发一次；
 * - 未使能时保持安全零位；
 * - 每 100 ms 发送 0x82 回执。
 */
static void ball_test_remote_task(void)
{
    ServoCommand cmd;
    uint32_t age_ms = 0U;
    uint32_t now = system_time_get_tick_ms();
    bool have_cmd = Vision_GetServoCommand(&cmd, &age_ms);

    if (have_cmd) {
        s_remote_seq = cmd.seq;
    }

    if (s_remote_enabled) {
        /*
         * 超时基准：
         * - 已收到过 0x02：按最近一帧的年龄；
         * - 从未收到 0x02：按 START_REMOTE 使能时刻。
         */
        uint32_t hold_ms = have_cmd ?
                           age_ms : (uint32_t)(now - s_remote_enable_tick_ms);

        if (hold_ms > BALL_TEST_REMOTE_TIMEOUT_MS) {
            s_remote_enabled = false;
            s_remote_result = BALL_REMOTE_RESULT_TIMEOUT;
            s_remote_target_ticks = Servo_GetNeutralPulseTicks();
            Servo_SetNeutral();
        } else if (have_cmd) {
            uint16_t clamped = ball_test_remote_clamp_ticks(cmd.target_pwm_ticks);

            s_remote_target_ticks = clamped;
            s_remote_result = (clamped != cmd.target_pwm_ticks) ?
                              BALL_REMOTE_RESULT_CLAMP : BALL_REMOTE_RESULT_OK;

            if ((uint32_t)(now - s_remote_apply_tick_ms) >=
                BALL_TEST_REMOTE_APPLY_MS) {
                s_remote_apply_tick_ms = now;
                Servo_SetTargetPulseTicks(clamped);
            }
        } else {
            /* START_REMOTE 后尚未收到 0x02：保持零位等待。 */
            if ((uint32_t)(now - s_remote_apply_tick_ms) >=
                BALL_TEST_REMOTE_APPLY_MS) {
                s_remote_apply_tick_ms = now;
                s_remote_target_ticks = Servo_GetNeutralPulseTicks();
                Servo_SetNeutral();
            }
        }
    } else {
        if ((uint32_t)(now - s_remote_apply_tick_ms) >=
            BALL_TEST_REMOTE_APPLY_MS) {
            s_remote_apply_tick_ms = now;
            s_remote_target_ticks = Servo_GetNeutralPulseTicks();
            Servo_SetNeutral();
        }
    }

    if ((uint32_t)(now - s_remote_telemetry_tick_ms) >=
        BALL_TEST_REMOTE_TELEMETRY_MS) {
        s_remote_telemetry_tick_ms = now;
        ball_test_remote_send_telemetry();
    }
}

/* 停止远程控制：退出远程、回安全零位，不自动恢复。 */
static void ball_test_remote_stop(void)
{
    s_remote_enabled = false;
    s_remote_result = BALL_REMOTE_RESULT_STOP;
    s_remote_target_ticks = Servo_GetNeutralPulseTicks();
    Servo_SetNeutral();
}

/*
 * 允许远程控制：由 Nano 的 0x03 START_REMOTE 调用，不要求本地 K1。
 *
 * 使能后先保持零位；后续 0x02 到达时再按目标执行。
 * 如果一直没有 0x02，远程任务会在 300 ms 后自动超时退出。
 */
static bool ball_test_remote_start(void)
{
    ServoCommand cmd;
    uint32_t age_ms = 0U;
    uint32_t now = system_time_get_tick_ms();

    if (s_remote_enabled) {
        return true;
    }

    if (!Servo_IsReady()) {
        return false;
    }

    s_remote_enabled = true;
    s_remote_enable_tick_ms = now;
    s_remote_target_ticks = Servo_GetNeutralPulseTicks();
    s_remote_apply_tick_ms = now - BALL_TEST_REMOTE_APPLY_MS;

    if (Vision_GetServoCommand(&cmd, &age_ms) &&
        (age_ms <= BALL_TEST_REMOTE_TIMEOUT_MS)) {
        uint16_t clamped = ball_test_remote_clamp_ticks(cmd.target_pwm_ticks);

        s_remote_seq = cmd.seq;
        s_remote_target_ticks = clamped;
        s_remote_result = (clamped != cmd.target_pwm_ticks) ?
                          BALL_REMOTE_RESULT_CLAMP : BALL_REMOTE_RESULT_OK;
    } else {
        s_remote_result = BALL_REMOTE_RESULT_NO_CMD;
    }

    return true;
}

/* 浮点有限值检查，避免 NaN/Inf 进入目标或 PD 参数。 */
static bool ball_test_float_is_finite(float value)
{
    return (value >= -1.0e30f) && (value <= 1.0e30f);
}

/* 发送 0x83 控制回执（6 字节，非阻塞）。 */
static void ball_test_send_control_ack(uint16_t seq, uint8_t opcode, uint8_t result)
{
    uint8_t payload[VISION_LINK_CONTROL_ACK_PAYLOAD_LEN];

    payload[0] = (uint8_t)(seq & 0xFFU);
    payload[1] = (uint8_t)(seq >> 8);
    payload[2] = opcode;
    payload[3] = result;
    payload[4] = (uint8_t)s_mode;
    payload[5] = (uint8_t)BallControl_GetState();

    (void)Vision_SendFrame(VISION_LINK_TYPE_CONTROL_ACK,
                           payload, sizeof(payload));
}

/*
 * 执行一条 0x03 控制指令。
 *
 * 结果码：
 * - OK：执行成功；
 * - BAD_MODE：当前模式不允许该指令；
 * - VISION_INVALID：START_AUTO 时没有新鲜有效视觉；
 * - PARAM_RANGE：目标或 PD 参数越界；
 * - FAULT：舵机/控制器故障；
 * - REJECTED：未知指令或 REMOTE 没有新鲜 0x02。
 */
static uint8_t ball_test_process_control(const ControlCommand *cmd)
{
    switch (cmd->opcode) {
    case BALL_CMD_SET_MODE:
        if ((cmd->mode == BALL_TEST_MODE_MANUAL) ||
            (cmd->mode == BALL_TEST_MODE_UART) ||
            (cmd->mode == BALL_TEST_MODE_REMOTE)) {
            BallTest_SetMode((ball_test_mode_t)cmd->mode);
            return BALL_CMD_RESULT_OK;
        }
        return BALL_CMD_RESULT_BAD_MODE;

    case BALL_CMD_START_AUTO:
        if (s_mode != BALL_TEST_MODE_UART) {
            return BALL_CMD_RESULT_BAD_MODE;
        }
        if (BallControl_GetState() == BALL_STATE_FAULT) {
            return BALL_CMD_RESULT_FAULT;
        }
        if (!BallControl_CanEnable()) {
            return BALL_CMD_RESULT_VISION_INVALID;
        }
        BallControl_Enable(true);
        if (BallControl_GetState() == BALL_STATE_AUTO) {
            return BALL_CMD_RESULT_OK;
        }
        return (BallControl_GetState() == BALL_STATE_FAULT) ?
               BALL_CMD_RESULT_FAULT : BALL_CMD_RESULT_REJECTED;

    case BALL_CMD_STOP:
        if (s_mode == BALL_TEST_MODE_REMOTE) {
            ball_test_remote_stop();
        } else if (s_mode == BALL_TEST_MODE_UART) {
            BallControl_Enable(false);
        } else if (s_mode == BALL_TEST_MODE_MANUAL) {
            BallControl_EnterManual();
            BallControl_SetManualBeamAngle(0.0f);
        } else {
            BallControl_Enable(false);
        }
        return BALL_CMD_RESULT_OK;

    case BALL_CMD_SET_TARGET:
        if (s_mode != BALL_TEST_MODE_UART) {
            return BALL_CMD_RESULT_BAD_MODE;
        }
        if ((!ball_test_float_is_finite(cmd->arg0)) ||
            (cmd->arg0 < -25.0f) || (cmd->arg0 > 25.0f)) {
            return BALL_CMD_RESULT_PARAM_RANGE;
        }
        BallControl_SetTarget(cmd->arg0);
        return BALL_CMD_RESULT_OK;

    case BALL_CMD_SET_PD:
        if (s_mode != BALL_TEST_MODE_UART) {
            return BALL_CMD_RESULT_BAD_MODE;
        }
        if ((!ball_test_float_is_finite(cmd->arg0)) ||
            (!ball_test_float_is_finite(cmd->arg1)) ||
            (cmd->arg0 < 0.0f) || (cmd->arg0 > 100.0f) ||
            (cmd->arg1 < 0.0f) || (cmd->arg1 > 100.0f)) {
            return BALL_CMD_RESULT_PARAM_RANGE;
        }
        BallControl_SetGains(cmd->arg0, cmd->arg1);
        return BALL_CMD_RESULT_OK;

    case BALL_CMD_START_REMOTE:
        if (s_mode != BALL_TEST_MODE_REMOTE) {
            return BALL_CMD_RESULT_BAD_MODE;
        }
        if (!Servo_IsReady()) {
            return BALL_CMD_RESULT_FAULT;
        }
        return ball_test_remote_start() ?
               BALL_CMD_RESULT_OK : BALL_CMD_RESULT_REJECTED;

    default:
        return BALL_CMD_RESULT_REJECTED;
    }
}

/*
 * 处理最近一条 0x03 控制指令。
 *
 * 用 seq 去重，保证同一帧只执行一次；处理完成后发送 0x83 回执。
 */
static void ball_test_handle_control(void)
{
    ControlCommand cmd;

    if (!Vision_GetControlCommand(&cmd, NULL)) {
        return;
    }
    if (s_have_control_seq && (cmd.seq == s_last_control_seq)) {
        return;
    }

    s_have_control_seq = true;
    s_last_control_seq = cmd.seq;
    s_last_cmd_result = ball_test_process_control(&cmd);
    ball_test_send_control_ack(cmd.seq, cmd.opcode, s_last_cmd_result);
}

/*
 * 按键处理。
 *
 * 组合键判定：
 * - 先取走两路边沿，再判断“按下 K2 时 K1 是否仍被按住”；
 * - 组合键触发模式切换后直接返回，避免同一次操作又切换场景/使能 AUTO。
 */
static void ball_test_handle_keys(void)
{
    bool k1_edge = user_key_take_edge(USER_KEY_K1);
    bool k2_edge = user_key_take_edge(USER_KEY_K2);

    if (k2_edge && user_key_is_pressed(USER_KEY_K1)) {
        ball_test_next_mode();
        return;
    }

    if (k2_edge) {
        if (s_mode == BALL_TEST_MODE_SIM) {
            ball_test_advance_sim_case();
        } else if (s_mode == BALL_TEST_MODE_UART) {
            /* UART：K2 暂不分配，目标由 0x03 SET_TARGET 设置。 */
        } else if (s_mode == BALL_TEST_MODE_ROLL) {
            ball_test_advance_roll_level();
        } else if (s_mode == BALL_TEST_MODE_SERVO) {
            /* SERVO：K2 每次 -3°。 */
            ball_test_manual_adjust(-BALL_TEST_SERVO_STEP_DEG);
        } else if (s_mode == BALL_TEST_MODE_REMOTE) {
            /* REMOTE：K2 暂不分配。 */
        } else {
            /* MANUAL：K2 反向微调，便于零位标定时来回试探。 */
            ball_test_manual_adjust(-BALL_TEST_MANUAL_STEP_DEG);
        }
    }

    if (k1_edge) {
        if (s_mode == BALL_TEST_MODE_MANUAL) {
            /* MANUAL：K1 正向微调 0.1°/0.2°。 */
            ball_test_manual_adjust(BALL_TEST_MANUAL_STEP_DEG);
        } else if (s_mode == BALL_TEST_MODE_SERVO) {
            /* SERVO：K1 每次 +3°。 */
            ball_test_manual_adjust(BALL_TEST_SERVO_STEP_DEG);
        } else if (s_mode == BALL_TEST_MODE_REMOTE) {
            /* REMOTE：K1 仅本地停止，启动由 0x03 START_REMOTE 完成。 */
            ball_test_remote_stop();
        } else if (s_mode == BALL_TEST_MODE_ROLL) {
            /* ROLL：K1 启动连续滚动；运行中按下立即停止并回平衡点。 */
            if (s_roll_running) {
                ball_test_roll_stop();
            } else {
                ball_test_roll_start();
            }
        } else if (s_mode == BALL_TEST_MODE_UART) {
            /* UART：K1 仅本地停止，启动由 0x03 START_AUTO 完成。 */
            BallControl_Enable(false);
        } else if (s_mode == BALL_TEST_MODE_SIM) {
            /* SIM 历史模式保留本地使能/停止。 */
            if (BallControl_IsEnabled()) {
                BallControl_Enable(false);
            } else {
                BallControl_Enable(true);
            }
        }
    }
}

/* ===================== 遥测 ===================== */

/*
 * 组包并发送 32 字节遥测负载（非阻塞，缓冲满时整帧丢弃）。
 *
 * 字段布局与 vision_link.h 顶部的协议说明一一对应；
 * 末尾额外附带“实际摆杆倾角”，供 PC 做半实物闭环仿真。
 */
static void ball_test_send_telemetry(void)
{
    ball_debug_t debug;
    uint8_t payload[32];
    uint8_t flags = 0U;

    BallControl_GetDebug(&debug);

    if (debug.vision_valid) {
        flags |= 0x01U;
    }
    if (debug.auto_enabled) {
        flags |= 0x02U;
    }
    if (debug.control_state == (uint8_t)BALL_STATE_FAULT) {
        flags |= 0x04U;
    }

    Vision_WriteU32LE(&payload[0], debug.time_ms);
    Vision_WriteF32LE(&payload[4], debug.target_cm);
    Vision_WriteF32LE(&payload[8], debug.ball_x_cm);
    Vision_WriteF32LE(&payload[12], debug.ball_v_cm_s);
    Vision_WriteF32LE(&payload[16], debug.error_cm);
    Vision_WriteF32LE(&payload[20], debug.beam_angle_cmd_deg);
    payload[24] = (uint8_t)(debug.servo_pwm_us & 0xFFU);
    payload[25] = (uint8_t)((debug.servo_pwm_us >> 8) & 0xFFU);
    payload[26] = debug.control_state;
    payload[27] = flags;

    /*
     * 附加实际摆杆倾角（限速后的值），用于 PC 半实物闭环仿真：
     * PC 收到该角度后积分钢珠动力学，再把位置帧发回 G3519。
     */
    Vision_WriteF32LE(&payload[28], Servo_GetBeamAngleActual());

    (void)Vision_SendFrame(VISION_LINK_TYPE_TELEMETRY, payload, sizeof(payload));
}

/* ===================== LCD ===================== */

/* 模式切换后强制刷新所有行，避免旧模式的文字残留。 */
static void ball_test_lcd_invalidate(void)
{
    uint8_t i;

    for (i = 0U; i < BALL_TEST_LCD_LINES; i++) {
        s_lcd_cache_valid[i] = false;
        s_lcd_cache[i][0] = '\0';
    }
    s_lcd_scan_index = 0U;
    s_lcd_line_tick_ms = 0U;
}

/*
 * 把当前调试快照格式化成 7 行固定宽度文本。
 *
 * 只做字符串拼接，不访问 SPI；耗时在微秒级，可以每圈调用。
 */
/* 无符号两位小数格式，用于舵机/摆杆角度等非负量。 */
static void ball_test_fmt2_abs(char *buf, float value)
{
    int32_t scaled;

    if (value < 0.0f) {
        value = -value;
    }
    scaled = (int32_t)(value * 100.0f + 0.5f);

    sprintf(buf, "%ld.%02ld", (long)(scaled / 100), (long)(scaled % 100));
}

/* PWM 计数换算成绝对舵机角度，仅用于 LCD/回执显示。 */
static float ball_test_ticks_to_angle(uint16_t ticks)
{
    return ((float)ticks - 400.0f) * 180.0f / 1600.0f;
}

static void ball_test_lcd_build(void)
{
    ball_debug_t debug;
    vision_link_status_t link;
    ball_control_state_t state;
    const char *state_text;
    char t[12];
    char x[12];
    char v[12];
    char b[12];
    char servo[12];
    char actual[12];
    char kp_s[12];
    char kd_s[12];
    float kp;
    float kd;

    BallControl_GetDebug(&debug);
    Vision_GetStatus(&link);
    state = BallControl_GetState();
    s_lcd_vision_ok = debug.vision_valid;

    if (s_mode == BALL_TEST_MODE_MANUAL) {
        /* MANUAL：本地零位标定界面。 */
        ball_test_fmt2_abs(servo, debug.servo_angle_cmd_deg);
        ball_test_fmt2(b, debug.beam_angle_cmd_deg);

        snprintf(s_lcd_lines[0], sizeof(s_lcd_lines[0]), "MODE: MANUAL");
        snprintf(s_lcd_lines[1], sizeof(s_lcd_lines[1]), "Servo: %s deg", servo);
        snprintf(s_lcd_lines[2], sizeof(s_lcd_lines[2]), "PWM:   %4u us",
                 (unsigned)debug.servo_pwm_us);
        snprintf(s_lcd_lines[3], sizeof(s_lcd_lines[3]), "Ticks: %4u",
                 (unsigned)Servo_GetPulseTicks());
        snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "Offset:%s deg", b);
        snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]), "K1:+0.1 K2:-0.1");
        s_lcd_lines[6][0] = '\0';
    } else if (s_mode == BALL_TEST_MODE_REMOTE) {
        /* REMOTE：Nano 目标 PWM 与 G3519 实际输出。 */
        ball_remote_status_t remote;
        float target_angle;

        BallTest_GetRemoteStatus(&remote);
        target_angle = ball_test_ticks_to_angle(remote.target_ticks);
        ball_test_fmt2_abs(t, target_angle);
        ball_test_fmt2_abs(servo, debug.servo_angle_cmd_deg);

        snprintf(s_lcd_lines[0], sizeof(s_lcd_lines[0]), "MODE: REMOTE");
        snprintf(s_lcd_lines[1], sizeof(s_lcd_lines[1]), "State: %s",
                 remote.enabled ? "RUN" : "STOP");
        snprintf(s_lcd_lines[2], sizeof(s_lcd_lines[2]), "Target: %s deg", t);
        snprintf(s_lcd_lines[3], sizeof(s_lcd_lines[3]), "Output: %s deg", servo);
        snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "PWM: %4u us",
                 (unsigned)debug.servo_pwm_us);
        snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]), "Ticks: %4u",
                 (unsigned)remote.output_ticks);
        snprintf(s_lcd_lines[6], sizeof(s_lcd_lines[6]), "Seq:%05u R:%s",
                 (unsigned)remote.seq,
                 ball_test_remote_result_text(remote.result));
    } else if (s_mode == BALL_TEST_MODE_UART) {
        /* UART：钢珠位置、PD 输出和视觉状态。 */
        state_text = BallControl_StateText(state);
        ball_test_fmt2(t, debug.target_cm);
        ball_test_fmt2(x, debug.ball_x_cm);
        ball_test_fmt2(v, debug.ball_v_cm_s);
        ball_test_fmt2(b, debug.beam_angle_cmd_deg);
        BallControl_GetGains(&kp, &kd);
        ball_test_fmt2_abs(kp_s, kp);
        ball_test_fmt2_abs(kd_s, kd);

        snprintf(s_lcd_lines[0], sizeof(s_lcd_lines[0]), "MODE: UART %s",
                 state_text);
        snprintf(s_lcd_lines[1], sizeof(s_lcd_lines[1]), "Target: %s cm", t);
        snprintf(s_lcd_lines[2], sizeof(s_lcd_lines[2]), "X:%s V:%s", x, v);
        snprintf(s_lcd_lines[3], sizeof(s_lcd_lines[3]), "Beam:%s deg", b);
        snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "PWM:%4u us Vis:%u",
                 (unsigned)debug.servo_pwm_us,
                 debug.vision_valid ? 1U : 0U);
        snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]), "Kp:%s Kd:%s",
                 kp_s, kd_s);
        snprintf(s_lcd_lines[6], sizeof(s_lcd_lines[6]), "Stop:%s Cmd:%s",
                 BallControl_ExitReasonText((ball_exit_reason_t)debug.exit_reason),
                 ball_test_cmd_result_text(s_last_cmd_result));
    } else {
        /* SIM / ROLL / SERVO 历史功能保留原有调试界面。 */
        state_text = BallControl_StateText(state);
        if (s_mode == BALL_TEST_MODE_ROLL) {
            if (s_roll_running) {
                state_text = "RUN";
            } else if (s_roll_state == BALL_ROLL_FINISH) {
                state_text = s_roll_aborted ? "ABRT" : "DONE";
            } else {
                state_text = "STOP";
            }
        }

        ball_test_fmt2(t, debug.target_cm);
        ball_test_fmt2(x, debug.ball_x_cm);
        ball_test_fmt2(v, debug.ball_v_cm_s);
        ball_test_fmt2(b, debug.beam_angle_cmd_deg);
        ball_test_fmt2(servo, debug.servo_angle_cmd_deg);
        ball_test_fmt2(actual, debug.beam_angle_actual_deg);

        snprintf(s_lcd_lines[0], sizeof(s_lcd_lines[0]), "M:%-6s S:%-5s T:%s",
                 ball_test_mode_name(s_mode), state_text, b);
        snprintf(s_lcd_lines[1], sizeof(s_lcd_lines[1]), "Servo:%s Act:%s",
                 servo, actual);
        snprintf(s_lcd_lines[2], sizeof(s_lcd_lines[2]), "Beam:%s Pulse:%4u",
                 b, (unsigned)debug.servo_pwm_us);

        if (s_mode == BALL_TEST_MODE_ROLL) {
            snprintf(s_lcd_lines[3], sizeof(s_lcd_lines[3]),
                     "Stage:%-6s Run:%u Ab:%u",
                     ball_test_roll_state_name(s_roll_state),
                     s_roll_running ? 1U : 0U,
                     s_roll_aborted ? 1U : 0U);
        } else {
            snprintf(s_lcd_lines[3], sizeof(s_lcd_lines[3]),
                     "Vis:%u Age:%3lu Seq:%5u",
                     debug.vision_valid ? 1U : 0U,
                     (unsigned long)debug.vision_age_ms,
                     (unsigned)link.last_seq);
        }

        if (s_mode == BALL_TEST_MODE_SIM) {
            snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "Case:%-8s",
                     ball_test_sim_case_name(s_sim_case));
        } else if (s_mode == BALL_TEST_MODE_SERVO) {
            ball_test_fmt2(t, s_manual_target_deg);
            snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "SrvAng:%s", t);
        } else {
            ball_test_fmt2(t, ball_test_roll_level_angle());
            snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "Lvl:%u/%u %s",
                     (unsigned)(s_roll_level + 1U),
                     (unsigned)BALL_TEST_ROLL_LEVEL_COUNT, t);
        }

        if (s_mode == BALL_TEST_MODE_SERVO) {
            snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]),
                     "Rng:-%d/+%d Step:%d",
                     (int)BALL_TEST_SERVO_LIMIT_DEG,
                     (int)BALL_TEST_SERVO_LIMIT_DEG,
                     (int)BALL_TEST_SERVO_STEP_DEG);
        } else if (s_mode == BALL_TEST_MODE_ROLL) {
            snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]),
                     "Pos:%lums Neg:%lums",
                     (unsigned long)BALL_TEST_ROLL_POSITIVE_HOLD_MS,
                     (unsigned long)BALL_TEST_ROLL_NEGATIVE_HOLD_MS);
        } else {
            snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]),
                     "Ok:%lu Bad:%lu Ovf:%lu",
                     (unsigned long)debug.frames_used,
                     (unsigned long)debug.frames_rejected,
                     (unsigned long)link.rx_overflow);
        }

        BallControl_GetGains(&kp, &kd);
        ball_test_fmt2(t, kp);
        ball_test_fmt2(x, kd);
        snprintf(s_lcd_lines[6], sizeof(s_lcd_lines[6]),
                 "K1+K2:MODE Kp:%s Kd:%s", t, x);
    }

    /* 固定宽度补空格，覆盖写入时不会留下上一帧的尾巴。 */
    {
        uint8_t i;

        for (i = 0U; i < BALL_TEST_LCD_LINES; i++) {
            ball_test_pad_line(s_lcd_lines[i]);
        }
    }
}

/*
 * 增量刷新任务，由主循环每圈调用。
 *
 * 行为：
 * - 距上一次写屏不足 50 ms 时直接返回，避免频繁占用 SPI；
 * - 轮询 7 行，找到第一行“内容变化或缓存无效”的行并只写这一行；
 * - 本轮没有变化则不写屏，仅推进计时。
 *
 * 最坏情况下每 50 ms 阻塞约 8 ms（一行 16 px 字符），
 * 相比整屏 60 ms 的阻塞，对 30 Hz 控制节拍影响小得多。
 */
static void ball_test_lcd_task(void)
{
    uint32_t now = system_time_get_tick_ms();
    uint8_t checked;
    static const uint16_t line_colors[BALL_TEST_LCD_LINES] = {
        WHITE, WHITE, WHITE, GREEN, CYAN, LGRAY, GRAY
    };

    if ((uint32_t)(now - s_lcd_line_tick_ms) < BALL_TEST_LCD_LINE_INTERVAL_MS) {
        return;
    }

    ball_test_lcd_build();

    for (checked = 0U; checked < BALL_TEST_LCD_LINES; checked++) {
        uint8_t i = s_lcd_scan_index;
        uint16_t color;

        s_lcd_scan_index = (uint8_t)((i + 1U) % BALL_TEST_LCD_LINES);

        if (s_lcd_cache_valid[i] &&
            (strcmp(s_lcd_lines[i], s_lcd_cache[i]) == 0)) {
            continue;
        }

        color = line_colors[i];
        if (i == 0U) {
            if (s_mode == BALL_TEST_MODE_ROLL) {
                if (s_roll_running) {
                    color = GREEN;
                } else if ((s_roll_state == BALL_ROLL_FINISH) && s_roll_aborted) {
                    color = RED;
                } else {
                    color = CYAN;
                }
            } else if (s_mode == BALL_TEST_MODE_REMOTE) {
                color = s_remote_enabled ? GREEN : YELLOW;
            } else {
                color = ball_test_lcd_color_for_state(BallControl_GetState());
            }
        } else if (i == 3U) {
            if (s_mode == BALL_TEST_MODE_ROLL) {
                color = (s_roll_state == BALL_ROLL_FINISH && s_roll_aborted) ? RED : YELLOW;
            } else if (s_mode == BALL_TEST_MODE_UART) {
                color = s_lcd_vision_ok ? GREEN : YELLOW;
            }
        }

        LCD_ShowString(0U,
                       (u16)(i * BALL_TEST_LCD_LINE_HEIGHT),
                       (const u8 *)s_lcd_lines[i],
                       color, BLACK, 16U, 0U);

        strcpy(s_lcd_cache[i], s_lcd_lines[i]);
        s_lcd_cache_valid[i] = true;
        s_lcd_line_tick_ms = now;
        return;
    }

    /* 全部行都与缓存一致，本轮不写屏。 */
    s_lcd_line_tick_ms = now;
}

/* ===================== 对外接口 ===================== */

void BallTest_SetMode(ball_test_mode_t mode)
{
    /*
     * 第一优先级：停止正在运行的开环滚动。
     * 切换模式后绝不能再执行上一次测试剩余的预设倾角动作。
     */
    s_roll_running = false;
    s_roll_state = BALL_ROLL_IDLE;
    s_roll_aborted = false;
    s_roll_settled = false;

    /* 任何模式切换都先退出远程控制，防止旧目标继续驱动舵机。 */
    s_remote_enabled = false;
    s_remote_seq = 0U;
    s_remote_result = BALL_REMOTE_RESULT_STOP;
    s_remote_target_ticks = Servo_GetNeutralPulseTicks();
    s_remote_apply_tick_ms = system_time_get_tick_ms() - BALL_TEST_REMOTE_APPLY_MS;
    s_remote_telemetry_tick_ms =
        system_time_get_tick_ms() - BALL_TEST_REMOTE_TELEMETRY_MS;
    s_remote_enable_tick_ms = system_time_get_tick_ms();

    /*
     * 先清故障再退出自动：ClearFault 本身也是显式操作，
     * 用于允许操作者在故障后重新选择测试模式。
     */
    BallControl_ClearFault();
    BallControl_Enable(false);

    /*
     * 自由舵机模式需要 ±30°，其余模式恢复默认 5° 安全限幅。
     * ROLL 仍在 K1 启动时单独放宽到 15°，停止/结束后恢复默认。
     */
    Servo_SetBeamLimit((mode == BALL_TEST_MODE_SERVO) ?
                       BALL_TEST_SERVO_LIMIT_DEG :
                       SERVO_BEAM_LIMIT_DEFAULT_DEG);

    s_mode = mode;
    s_manual_target_deg = 0.0f;
    s_sim_case = 0U;
    s_roll_level = 0U;
    s_sim_seq = 0U;
    s_sim_x_cm = 0.0f;
    s_sim_v_cm_s = 0.0f;
    s_have_forwarded = false;
    s_last_forwarded_seq = 0U;

    if (mode == BALL_TEST_MODE_REMOTE) {
        /*
         * REMOTE 直接由 Servo_SetTargetPulseTicks() 驱动，
         * 不进入 BallControl 手动保持，也不调用 BallControl_Update()。
         * 上电/切换模式后先回到安全零位，等待 K1 使能。
         */
        Servo_SetNeutral();
    } else if ((mode == BALL_TEST_MODE_MANUAL) ||
        (mode == BALL_TEST_MODE_ROLL) ||
        (mode == BALL_TEST_MODE_SERVO)) {
        /*
         * MANUAL / SERVO / ROLL 都复用 BallControl 的手动保持通道：
         * - 摆杆回平衡点 0°；
         * - 后续由手动微调或开环状态机周期性下发目标；
         * - 仍然受 Servo 的斜率和机械限幅保护。
         */
        BallControl_EnterManual();
        BallControl_SetManualBeamAngle(0.0f);
    } else {
        BallControl_SetTarget(0.0f);
        /* 让第一次任务循环立刻产生一帧数据，减少使能前的等待。 */
        s_sim_tick_ms = system_time_get_tick_ms() - BALL_TEST_SIM_PERIOD_MS;
    }

    s_telemetry_tick_ms = system_time_get_tick_ms() - BALL_TEST_TELEMETRY_MS;

    /* 模式切换后立即整屏重画，避免旧模式残留文字。 */
    ball_test_lcd_invalidate();

    /*
     * 不用 printf 的 %f：本项目未启用浮点格式化，统一走整数格式，
     * 避免在不同 Keil/MicroLIB 配置下打印出乱码。
     */
    {
        char kp_text[12];
        char kd_text[12];

        ball_test_fmt2(kp_text, BALL_CONTROL_DEFAULT_KP);
        ball_test_fmt2(kd_text, BALL_CONTROL_DEFAULT_KD);
        printf("[BALL] mode=%s target=0.00 Kp=%s Kd=%s\r\n",
               ball_test_mode_name(mode), kp_text, kd_text);
    }
}

void BallTest_GetRollStatus(ball_roll_status_t *status)
{
    if (status == NULL) {
        return;
    }

    status->running = s_roll_running;
    status->aborted = s_roll_aborted;
    status->level = s_roll_level;
    status->theta_deg = ball_test_roll_level_angle();
    status->state = s_roll_state;
}

void BallTest_GetRemoteStatus(ball_remote_status_t *status)
{
    if (status == NULL) {
        return;
    }

    status->enabled = s_remote_enabled;
    status->seq = s_remote_seq;
    status->target_ticks = s_remote_target_ticks;
    status->output_ticks = Servo_GetPulseTicks();
    status->result = s_remote_result;
}

uint8_t BallTest_GetLastCommandResult(void)
{
    return s_last_cmd_result;
}

void BallTest_AppInit(void)
{
    printf("\r\n[BALL] G3519 ball balance control init\r\n");

    LCD_Init();
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
    LCD_ShowString(0U, 0U, (const u8 *)"Ball control init...", WHITE, BLACK, 16U, 0U);

    /*
     * 初始化顺序不可交换：
     * - user_key 的 20 ms 消抖依赖 system_time；
     * - Servo_Init 的斜率限制依赖 system_time；
     * - Vision_Init 会重新配置 UART0（PA10/PA11）并打开中断；
     * - BallControl_Init 只清状态，不访问硬件。
     */
    user_key_init();
    Servo_Init();
    Vision_Init(VISION_LINK_DEFAULT_BAUD);
    BallControl_Init();
    BallControl_SetGains(BALL_CONTROL_DEFAULT_KP, BALL_CONTROL_DEFAULT_KD);

    BallTest_SetMode((ball_test_mode_t)BALL_TEST_DEFAULT_MODE);

    printf("[BALL] servo PA9/TIMA0 50Hz, vision UART0 PA10/PA11 115200\r\n");
    printf("[BALL] K1=enable/step/run, K2=case/level, K1+K2=mode\r\n");
}

void BallTest_AppTask(void)
{
    /* 先解析串口，保证本圈能拿到最新的位置帧和控制指令。 */
    Vision_Update();

    /*
     * UART 模式先喂入最新钢珠位置，再处理 0x03；
     * 这样 START_AUTO 同圈就能看到新鲜有效测量。
     */
    if (s_mode == BALL_TEST_MODE_UART) {
        ball_test_poll_uart();
        ball_test_check_link_fault();
    }

    /* Nano 控制指令优先于本地按键处理。 */
    ball_test_handle_control();
    ball_test_handle_keys();

    if (s_mode == BALL_TEST_MODE_SIM) {
        if (system_time_elapsed_ms(&s_sim_tick_ms, BALL_TEST_SIM_PERIOD_MS)) {
            ball_test_inject_simulated();
        }
    } else if (s_mode == BALL_TEST_MODE_ROLL) {
        /* 开环状态机只依赖按键和系统时基，不使用视觉数据。 */
        ball_test_roll_task();
    } else if (s_mode == BALL_TEST_MODE_REMOTE) {
        /* 远程舵机控制：直接下发 PWM 目标并发送 0x82 回执。 */
        ball_test_remote_task();
    }

    /*
     * REMOTE 直接控制 Servo，不使用 BallControl；
     * 其他模式继续推进 PD/手动/滚动状态机。
     */
    if (s_mode != BALL_TEST_MODE_REMOTE) {
        BallControl_Update();
    }

    if (system_time_elapsed_ms(&s_telemetry_tick_ms, BALL_TEST_TELEMETRY_MS)) {
        ball_test_send_telemetry();
    }

    ball_test_lcd_task();
}
