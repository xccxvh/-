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
 * - 按当前测试模式提供测量数据（真实 UART / 模拟 / 手动）；
 * - 处理按键、刷新 LCD、发送遥测。
 *
 * 真正的控制算法在 App/ball_control.c，底层 PWM 在 BSP/SERVO/servo.c，
 * 三者严格分层，便于单独修改和测试。
 *
 * 时序：
 * - 模拟视觉 33 ms 注入一次，接近 Jetson 30 FPS；
 * - 遥测 100 ms 一帧，非阻塞发送；
 * - LCD 200 ms 刷新一次，使用固定宽度行覆盖，避免残影。
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

/* 手动测试 A 的倾角档位：水平 -> 正向 -> 反向。 */
static const float s_manual_steps[] = { 0.0f, 1.0f, -1.0f };
#define BALL_TEST_MANUAL_STEP_COUNT     (sizeof(s_manual_steps) / sizeof(s_manual_steps[0]))

/* 测试 D 的指定位置序列：0 cm -> +5 cm -> -5 cm。 */
static const float s_targets_cm[] = { 0.0f, 5.0f, -5.0f };
#define BALL_TEST_TARGET_COUNT          (sizeof(s_targets_cm) / sizeof(s_targets_cm[0]))

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

/* ===================== 内部状态 ===================== */

static ball_test_mode_t s_mode = BALL_TEST_MODE_UART;
static uint8_t  s_manual_step = 0U;
static uint8_t  s_sim_case = 0U;
static uint8_t  s_target_index = 0U;

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
         * 摆杆限幅 ±2.5°，sinθ 与 θ 的误差小于 0.03%，直接用弧度近似。
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

    if (age_ms > 1000U) {
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

/* ===================== 按键与模式 ===================== */

static void ball_test_advance_manual_step(void)
{
    s_manual_step = (uint8_t)((s_manual_step + 1U) % BALL_TEST_MANUAL_STEP_COUNT);
    BallControl_SetManualBeamAngle(s_manual_steps[s_manual_step]);
}

static void ball_test_advance_sim_case(void)
{
    s_sim_case = (uint8_t)((s_sim_case + 1U) % BALL_TEST_SIM_COUNT);

    /* 切换场景时清空仿真状态，避免带着上一种场景的速度进入下一种。 */
    s_sim_x_cm = 0.0f;
    s_sim_v_cm_s = 0.0f;
}

static void ball_test_advance_target(void)
{
    s_target_index = (uint8_t)((s_target_index + 1U) % BALL_TEST_TARGET_COUNT);
    BallControl_SetTarget(s_targets_cm[s_target_index]);
}

static void ball_test_next_mode(void)
{
    ball_test_mode_t next;

    switch (s_mode) {
    case BALL_TEST_MODE_MANUAL:
        next = BALL_TEST_MODE_SIM;
        break;
    case BALL_TEST_MODE_SIM:
        next = BALL_TEST_MODE_UART;
        break;
    case BALL_TEST_MODE_UART:
    default:
        next = BALL_TEST_MODE_MANUAL;
        break;
    }

    BallTest_SetMode(next);
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
            ball_test_advance_target();
        } else {
            ball_test_advance_manual_step();
        }
    }

    if (k1_edge) {
        if (s_mode == BALL_TEST_MODE_MANUAL) {
            /* 手动测试 A：K1 逐档切换水平/正倾/反倾。 */
            ball_test_advance_manual_step();
        } else if (BallControl_IsEnabled()) {
            BallControl_Enable(false);
        } else {
            /*
             * 使能 AUTO。若当前没有新鲜视觉，控制器会保持 IDLE，
             * LCD 状态栏会显示 IDLE，可据此判断使能是否成功。
             */
            BallControl_Enable(true);
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
static void ball_test_lcd_build(void)
{
    ball_debug_t debug;
    vision_link_status_t link;
    ball_control_state_t state;
    char t[12];
    char x[12];
    char v[12];
    char e[12];
    char b[12];
    float kp;
    float kd;

    BallControl_GetDebug(&debug);
    Vision_GetStatus(&link);
    state = BallControl_GetState();
    s_lcd_vision_ok = debug.vision_valid;

    ball_test_fmt2(t, debug.target_cm);
    ball_test_fmt2(x, debug.ball_x_cm);
    ball_test_fmt2(v, debug.ball_v_cm_s);
    ball_test_fmt2(e, debug.error_cm);
    ball_test_fmt2(b, debug.beam_angle_cmd_deg);

    snprintf(s_lcd_lines[0], sizeof(s_lcd_lines[0]), "M:%-6s S:%-5s T:%s", ball_test_mode_name(s_mode),
            BallControl_StateText(state), t);
    snprintf(s_lcd_lines[1], sizeof(s_lcd_lines[1]), "X:%s V:%s E:%s", x, v, e);
    snprintf(s_lcd_lines[2], sizeof(s_lcd_lines[2]), "Beam:%s Pulse:%4u", b, (unsigned)debug.servo_pwm_us);
    snprintf(s_lcd_lines[3], sizeof(s_lcd_lines[3]), "Vis:%u Age:%3lu Seq:%5u",
            debug.vision_valid ? 1U : 0U,
            (unsigned long)debug.vision_age_ms,
            (unsigned)link.last_seq);

    if (s_mode == BALL_TEST_MODE_SIM) {
        snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "Case:%-8s", ball_test_sim_case_name(s_sim_case));
    } else if (s_mode == BALL_TEST_MODE_UART) {
        ball_test_fmt2(t, s_targets_cm[s_target_index]);
        snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "Target:%s", t);
    } else {
        ball_test_fmt2(t, s_manual_steps[s_manual_step]);
        snprintf(s_lcd_lines[4], sizeof(s_lcd_lines[4]), "Manual:%s", t);
    }

    snprintf(s_lcd_lines[5], sizeof(s_lcd_lines[5]), "Ok:%lu Bad:%lu Ovf:%lu",
            (unsigned long)debug.frames_used,
            (unsigned long)debug.frames_rejected,
            (unsigned long)link.rx_overflow);

    BallControl_GetGains(&kp, &kd);
    ball_test_fmt2(t, kp);
    ball_test_fmt2(x, kd);
    snprintf(s_lcd_lines[6], sizeof(s_lcd_lines[6]), "K1+K2:MODE Kp:%s Kd:%s", t, x);

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
            color = ball_test_lcd_color_for_state(BallControl_GetState());
        } else if (i == 3U) {
            color = s_lcd_vision_ok ? GREEN : YELLOW;
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
     * 先清故障再退出自动：ClearFault 本身也是显式操作，
     * 用于允许操作者在故障后重新选择测试模式。
     */
    BallControl_ClearFault();
    BallControl_Enable(false);

    s_mode = mode;
    s_manual_step = 0U;
    s_sim_case = 0U;
    s_target_index = 0U;
    s_sim_seq = 0U;
    s_sim_x_cm = 0.0f;
    s_sim_v_cm_s = 0.0f;
    s_have_forwarded = false;
    s_last_forwarded_seq = 0U;

    if (mode == BALL_TEST_MODE_MANUAL) {
        BallControl_EnterManual();
        BallControl_SetManualBeamAngle(s_manual_steps[0]);
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
     * - Vision_Init 会重新配置 UART4 并打开中断；
     * - BallControl_Init 只清状态，不访问硬件。
     */
    user_key_init();
    Servo_Init();
    Vision_Init(VISION_LINK_DEFAULT_BAUD);
    BallControl_Init();
    BallControl_SetGains(BALL_CONTROL_DEFAULT_KP, BALL_CONTROL_DEFAULT_KD);

    BallTest_SetMode((ball_test_mode_t)BALL_TEST_DEFAULT_MODE);

    printf("[BALL] servo PA9/TIMA0 50Hz, vision UART4 PB10/PB11 115200\r\n");
    printf("[BALL] K1=enable/step, K2=case/target, K1+K2=mode\r\n");
}

void BallTest_AppTask(void)
{
    /* 先解析串口，保证本圈能拿到最新帧。 */
    Vision_Update();

    ball_test_handle_keys();

    if (s_mode == BALL_TEST_MODE_SIM) {
        if (system_time_elapsed_ms(&s_sim_tick_ms, BALL_TEST_SIM_PERIOD_MS)) {
            ball_test_inject_simulated();
        }
    } else if (s_mode == BALL_TEST_MODE_UART) {
        ball_test_poll_uart();
        ball_test_check_link_fault();
    }

    /*
     * 状态机推进放在喂数据之后：
     * 本圈收到的新测量可以在同一圈转成舵机输出，降低控制延迟。
     */
    BallControl_Update();

    if (system_time_elapsed_ms(&s_telemetry_tick_ms, BALL_TEST_TELEMETRY_MS)) {
        ball_test_send_telemetry();
    }

    /*
     * LCD 每次最多刷新一行，写屏开销被摊平；
     * 放在控制更新之后，保证视觉/PD 优先执行。
     */
    ball_test_lcd_task();
}
