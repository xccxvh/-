/*
 * 滚球 PD 控制器与舵机映射的主机端测试。
 *
 * 目标：
 * - 在没有 MSPM0 硬件的情况下验证 Servo 角度/脉宽映射、斜率限制；
 * - 验证 PD 符号、视觉失效/超时保护、位置跳变剔除、速度滤波；
 * - 用滚球动力学模型做闭环仿真，给出“模拟视觉数据测试”的实际结果。
 *
 * 编译/运行见同目录 run_tests.sh。
 */

#include "App/ball_control.h"
#include "SERVO/servo.h"
#include "SystemTime/system_time.h"

#include <stdio.h>
#include <math.h>

#define TEST_PI                     (3.14159265f)
#define TEST_DEG_TO_RAD             (TEST_PI / 180.0f)
#define TEST_SIM_DT_S               (0.0333f)
#define TEST_SIM_DT_MS              (33U)
#define TEST_G_TERM_CM_S2           (981.0f * 5.0f / 7.0f)
#define TEST_RAIL_CM                (18.0f)

static int g_checks = 0;
static int g_failures = 0;
static uint16_t g_seq = 0U;

/* 非 NULL 时，闭环仿真的每一步都会写成 CSV，作为调参记录留档。 */
static FILE *g_csv = NULL;

static void check_true(bool condition, const char *name)
{
    g_checks++;
    if (condition) {
        printf("[PASS] %s\n", name);
    } else {
        g_failures++;
        printf("[FAIL] %s\n", name);
    }
}

static void check_near(float actual, float expect, float tolerance, const char *name)
{
    g_checks++;
    if (fabsf(actual - expect) <= tolerance) {
        printf("[PASS] %s (actual=%.4f expect=%.4f)\n", name, actual, expect);
    } else {
        g_failures++;
        printf("[FAIL] %s (actual=%.4f expect=%.4f tol=%.4f)\n",
               name, actual, expect, tolerance);
    }
}

/* 每个用例开始前恢复到确定的初始状态。 */
static void reset_all(void)
{
    host_time_reset();
    Servo_Init();
    BallControl_Init();
    g_seq = 0U;
}

static void feed_seq(float x_cm, bool valid, float confidence, uint16_t seq)
{
    BallMeasurement measurement;

    measurement.x_cm = x_cm;
    measurement.frame_time_ms = system_time_get_tick_ms();
    measurement.seq = seq;
    measurement.confidence = confidence;
    measurement.valid = valid;

    BallControl_OnMeasurement(&measurement);
    g_seq = seq;
}

static void feed(float x_cm, bool valid, float confidence)
{
    feed_seq(x_cm, valid, confidence, (uint16_t)(g_seq + 1U));
}

/* ===================== 舵机映射 ===================== */

static void test_servo_mapping(void)
{
    uint16_t neutral_us;

    reset_all();
    check_true(Servo_IsReady(), "servo init ready");
    check_true(host_servo_started, "servo timer started");
    check_true(host_servo_load_ticks == 16000U, "servo period = 16000 ticks (50 Hz)");

    neutral_us = Servo_GetPulseUs();
    /* 104° -> 400 + 1600*104/180 = 1324 计数 -> 1655 us */
    check_true((neutral_us >= 1653U) && (neutral_us <= 1657U),
               "neutral 104 deg -> ~1655 us");

    /*
     * 单次调用间隔 1 s：dt 被限制到 50 ms，只能转 20°/s × 0.05 s = 1°。
     * 这是防止主循环停顿后一帧跳出大角度的安全设计。
     */
    host_time_advance_ms(1000);
    Servo_SetBeamAngle(2.0f);
    check_near(Servo_GetBeamAngleActual(), 1.0f, 0.02f,
               "slew limited to 1.0 deg per call after long gap");

    /* 以 10 ms 节拍重复下发 +2°，让斜率限制逐步到位。 */
    for (int i = 0; i < 10; i++) {
        host_time_advance_ms(10);
        Servo_SetBeamAngle(2.0f);
    }
    check_near(Servo_GetBeamAngleActual(), 2.0f, 0.01f, "servo reaches +2.0 deg");
    /* 106° -> 1342 计数 -> 1677 us */
    check_true((Servo_GetPulseUs() >= 1675U) && (Servo_GetPulseUs() <= 1679U),
               "beam +2 deg -> ~1677 us");

    /* 反向回到 0°，同样受限速。 */
    for (int i = 0; i < 30; i++) {
        host_time_advance_ms(10);
        Servo_SetBeamAngle(0.0f);
    }
    check_near(Servo_GetBeamAngleActual(), 0.0f, 0.01f, "servo returns to neutral");

    /* 超范围输入必须被摆杆限幅挡住（默认 2.5°）。 */
    for (int i = 0; i < 60; i++) {
        host_time_advance_ms(10);
        Servo_SetBeamAngle(30.0f);
    }
    check_near(Servo_GetBeamAngleActual(), 2.5f, 0.01f,
               "beam hard limit 2.5 deg");
}

/* ===================== PD 符号与静态误差 ===================== */

static float run_static_case(float target_cm, float x_cm)
{
    reset_all();
    BallControl_SetTarget(target_cm);

    feed(x_cm, true, 95.0f);
    BallControl_Update();          /* IDLE：先记录测量 */
    BallControl_Enable(true);      /* 有新鲜测量，应成功进入 AUTO */
    BallControl_Update();          /* 立即用该测量算一次 PD */

    return BallControl_GetBeamAngleCmd();
}

static void test_pd_signs(void)
{
    float theta;

    theta = run_static_case(0.0f, -2.0f);
    check_true(BallControl_GetState() == BALL_STATE_AUTO, "enable AUTO with fresh vision");
    check_near(theta, 1.2f, 0.05f, "x=-2 cm -> +1.2 deg (accelerate +x)");

    theta = run_static_case(0.0f, 2.0f);
    check_near(theta, -1.2f, 0.05f, "x=+2 cm -> -1.2 deg (accelerate -x)");

    theta = run_static_case(0.0f, 0.0f);
    check_near(theta, 0.0f, 0.02f, "x=0, v=0 -> level");

    theta = run_static_case(5.0f, 3.0f);
    check_near(theta, 1.2f, 0.05f, "target +5, x=+3 -> +1.2 deg");

    /* 误差过大时必须被 ±2° 限幅。 */
    theta = run_static_case(0.0f, -20.0f);
    check_near(theta, 2.0f, 0.01f, "PD output clamped to +2.0 deg");
}

/* ===================== 安全保护 ===================== */

static void test_vision_invalid_exits_auto(void)
{
    reset_all();

    feed(0.0f, true, 95.0f);
    BallControl_Update();
    BallControl_Enable(true);
    BallControl_Update();
    check_true(BallControl_GetState() == BALL_STATE_AUTO, "AUTO before invalid frame");

    feed(0.0f, false, 95.0f);
    BallControl_Update();
    check_true(BallControl_GetState() == BALL_STATE_VISION_LOST,
               "invalid frame -> VISION_LOST immediately");

    /* 停止喂数据 1 s，摆杆应已经限速回到 0° 附近。 */
    for (int i = 0; i < 100; i++) {
        host_time_advance_ms(10);
        BallControl_Update();
    }
    check_near(Servo_GetBeamAngleActual(), 0.0f, 0.05f,
               "VISION_LOST returns beam to neutral with slew limit");

    /* 视觉恢复也不能自动回到 AUTO。 */
    feed(1.0f, true, 95.0f);
    BallControl_Update();
    check_true(BallControl_GetState() == BALL_STATE_VISION_LOST,
               "vision recovery does not auto-resume AUTO");
}

static void test_vision_timeout(void)
{
    reset_all();

    feed(0.0f, true, 95.0f);
    BallControl_Update();
    BallControl_Enable(true);
    BallControl_Update();
    check_true(BallControl_GetState() == BALL_STATE_AUTO, "AUTO before timeout");

    host_time_advance_ms(151U);
    BallControl_Update();
    check_true(BallControl_GetState() == BALL_STATE_VISION_LOST,
               "151 ms without frame -> VISION_LOST");
}

static void test_position_jump_rejected(void)
{
    ball_debug_t debug;
    uint32_t rejected_before;

    reset_all();

    feed(0.0f, true, 95.0f);
    BallControl_Update();
    BallControl_Enable(true);
    BallControl_Update();

    BallControl_GetDebug(&debug);
    rejected_before = debug.frames_rejected;

    host_time_advance_ms(33U);
    feed(10.0f, true, 95.0f);      /* 33 ms 跳 10 cm，必须剔除 */
    BallControl_Update();

    BallControl_GetDebug(&debug);
    check_true(debug.frames_rejected > rejected_before, "10 cm jump rejected");
    check_near(debug.ball_x_cm, 0.0f, 0.001f, "position unchanged after jump");
    check_near(debug.ball_v_cm_s, 0.0f, 0.001f, "no velocity spike after jump");

    host_time_advance_ms(33U);
    feed(0.05f, true, 95.0f);      /* 从旧位置继续，小幅移动应接受 */
    BallControl_Update();
    BallControl_GetDebug(&debug);
    check_near(debug.ball_x_cm, 0.05f, 0.001f, "normal frame accepted after jump");
}

static void test_velocity_filter(void)
{
    ball_debug_t debug;

    reset_all();

    feed(0.0f, true, 95.0f);
    host_time_advance_ms(33U);
    feed(1.0f, true, 95.0f);
    host_time_advance_ms(33U);
    feed(2.0f, true, 95.0f);

    BallControl_GetDebug(&debug);
    /* 原始速度约 30.3 cm/s，一阶滤波后应明显小于原始值但大于 0。 */
    check_true((debug.ball_v_cm_s > 10.0f) && (debug.ball_v_cm_s < 25.0f),
               "velocity low-pass filtered");
}

static void test_duplicate_frame_ignored(void)
{
    ball_debug_t debug;

    reset_all();

    feed(0.0f, true, 95.0f);
    BallControl_Update();
    feed_seq(1.0f, true, 95.0f, g_seq);   /* 相同 seq，应被忽略 */
    BallControl_Update();

    BallControl_GetDebug(&debug);
    check_near(debug.ball_x_cm, 0.0f, 0.001f, "duplicate seq ignored");
}

/* ===================== 手动模式 ===================== */

static void test_manual_hold(void)
{
    reset_all();

    BallControl_EnterManual();
    BallControl_SetManualBeamAngle(1.0f);
    for (int i = 0; i < 100; i++) {
        host_time_advance_ms(10);
        BallControl_Update();
    }

    check_near(Servo_GetBeamAngleActual(), 1.0f, 0.05f,
               "manual +1.0 deg reached via periodic re-send");

    BallControl_SetManualBeamAngle(-1.0f);
    for (int i = 0; i < 150; i++) {
        host_time_advance_ms(10);
        BallControl_Update();
    }
    check_near(Servo_GetBeamAngleActual(), -1.0f, 0.05f,
               "manual -1.0 deg reached");
}

/* ===================== 闭环动力学仿真 ===================== */

/*
 * 用滚球模型验证闭环：
 *   a = (5/7) g sinθ ≈ 700.7 θ(rad)  cm/s²
 *   v += a·dt, x += v·dt
 * 视觉帧按 30 Hz 注入，控制器输出经舵机斜率限制后作用到钢珠。
 */
static void simulate_ms(float target_cm, uint32_t duration_ms,
                        float *x_io, float *v_io,
                        float *max_abs_x, uint32_t *settle_ms)
{
    uint32_t elapsed = 0U;
    uint32_t inside_ms = 0U;
    uint32_t settle = duration_ms;
    bool settled = false;

    BallControl_SetTarget(target_cm);

    while (elapsed < duration_ms) {
        float theta_rad;
        float accel;
        float x = *x_io;
        float v = *v_io;

        host_time_advance_ms(TEST_SIM_DT_MS);
        elapsed += TEST_SIM_DT_MS;

        /* 物理部分：摆杆实际角度受舵机斜率限制。 */
        theta_rad = Servo_GetBeamAngleActual() * TEST_DEG_TO_RAD;
        accel = TEST_G_TERM_CM_S2 * theta_rad;

        v += accel * TEST_SIM_DT_S;
        x += v * TEST_SIM_DT_S;

        if (x > TEST_RAIL_CM) {
            x = TEST_RAIL_CM;
            v = -0.25f * v;
        } else if (x < -TEST_RAIL_CM) {
            x = -TEST_RAIL_CM;
            v = -0.25f * v;
        }

        *x_io = x;
        *v_io = v;

        feed(x, true, 95.0f);
        BallControl_Update();

        if (g_csv != NULL) {
            fprintf(g_csv, "%u,%.3f,%.3f,%.3f,%.3f\n",
                    elapsed,
                    target_cm,
                    x,
                    v,
                    Servo_GetBeamAngleActual());
        }

        if (fabsf(x) > *max_abs_x) {
            *max_abs_x = fabsf(x);
        }

        /*
         * 稳定判据：误差绝对值小于 1 cm 连续保持 300 ms。
         * 记录首次满足的时刻，用于对照题目“5 s 内到位”的要求。
         */
        if (fabsf(x - target_cm) < 1.0f) {
            inside_ms += TEST_SIM_DT_MS;
            if (!settled && (inside_ms >= 300U)) {
                settled = true;
                settle = elapsed;
            }
        } else {
            inside_ms = 0U;
        }
    }

    *settle_ms = settle;
}

static void test_closed_loop_simulation(void)
{
    float x = 5.0f;
    float v = 0.0f;
    float max_abs_x = 0.0f;
    uint32_t settle = 0U;

    reset_all();
    BallControl_SetGains(0.60f, 0.30f);

    feed(x, true, 95.0f);
    BallControl_Update();
    BallControl_Enable(true);
    BallControl_Update();
    check_true(BallControl_GetState() == BALL_STATE_AUTO, "AUTO enabled for simulation");

    /* 从 +5 cm 收敛到 0 cm。 */
    simulate_ms(0.0f, 6000U, &x, &v, &max_abs_x, &settle);
    printf("[SIM] center hold: x=%.3f cm v=%.3f cm/s settle=%u ms max|x|=%.3f cm\n",
           x, v, settle, max_abs_x);
    check_true(fabsf(x) < 1.0f, "closed loop settles within +/-1 cm at 0 cm");
    check_true(settle <= 5000U, "settle time <= 5 s for 0 cm");
    check_true(max_abs_x <= 6.0f, "overshoot bounded");

    /* 指定位置 +5 cm，再回 -5 cm，各给 5 s。 */
    max_abs_x = 0.0f;
    settle = 0U;
    simulate_ms(5.0f, 5000U, &x, &v, &max_abs_x, &settle);
    printf("[SIM] target +5 cm: x=%.3f cm v=%.3f cm/s settle=%u ms max|x|=%.3f cm\n",
           x, v, settle, max_abs_x);
    check_true(fabsf(x - 5.0f) < 1.0f, "closed loop settles within +/-1 cm at +5 cm");
    check_true(settle <= 5000U, "settle time <= 5 s for +5 cm");

    max_abs_x = 0.0f;
    settle = 0U;
    simulate_ms(-5.0f, 5000U, &x, &v, &max_abs_x, &settle);
    printf("[SIM] target -5 cm: x=%.3f cm v=%.3f cm/s settle=%u ms max|x|=%.3f cm\n",
           x, v, settle, max_abs_x);
    check_true(fabsf(x + 5.0f) < 1.0f, "closed loop settles within +/-1 cm at -5 cm");
    check_true(settle <= 5000U, "settle time <= 5 s for -5 cm");
}

/*
 * 生成一份可留档的阶跃响应日志：+5 cm -> 0 cm，3 s。
 * 文件位置固定为 tests/host/results/closed_loop_step_response.csv，
 * 可用 Excel/Python 画图检查超调、稳态误差和调节时间。
 */
static void run_logged_simulation(void)
{
    float x = 5.0f;
    float v = 0.0f;
    float max_abs_x = 0.0f;
    uint32_t settle = 0U;

    reset_all();
    BallControl_SetGains(0.60f, 0.30f);

    feed(x, true, 95.0f);
    BallControl_Update();
    BallControl_Enable(true);
    BallControl_Update();

    g_csv = fopen("tests/host/results/closed_loop_step_response.csv", "w");
    if (g_csv == NULL) {
        printf("[WARN] cannot open results CSV, skip logging\n");
        return;
    }

    fprintf(g_csv, "time_ms,target_cm,x_cm,v_cm_s,beam_actual_deg\n");
    simulate_ms(0.0f, 3000U, &x, &v, &max_abs_x, &settle);
    fclose(g_csv);
    g_csv = NULL;

    printf("[SIM] logged step response: settle=%u ms final_x=%.3f cm max|x|=%.3f cm\n",
           settle, x, max_abs_x);
}

/* ===================== 入口 ===================== */

int main(void)
{
    printf("=== G3519 ball control host tests ===\n");

    test_servo_mapping();
    test_pd_signs();
    test_vision_invalid_exits_auto();
    test_vision_timeout();
    test_position_jump_rejected();
    test_velocity_filter();
    test_duplicate_frame_ignored();
    test_manual_hold();
    test_closed_loop_simulation();
    run_logged_simulation();

    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
