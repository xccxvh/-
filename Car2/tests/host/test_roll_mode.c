/*
 * 开环 5°/10°/15° 自动左右滚动的主机端测试。
 *
 * 使用真实的 BSP/SERVO/servo.c、User/App/ball_control.c 和
 * User/App/ball_test.c，只把 LCD、按键和视觉链路替换成脚本/空实现，
 * 因此可以直接验证：
 * - ROLL 运行期间把摆杆上限临时放宽到 15°，退出后恢复默认 5°；
 * - K2 在 5/10/15° 之间切换；
 * - 状态机按 NEUTRAL -> POSITIVE -> RETURN -> NEGATIVE -> RETURN -> FINISH 执行一次；
 * - K1 在运行中能立即停止，且不会再执行后续反向动作；
 * - 每个阶段会等待实际倾角到位后再计时。
 *
 * 编译/运行见 run_tests.sh。
 */

#include "App/ball_test.h"
#include "App/ball_control.h"
#include "SERVO/servo.h"
#include "SystemTime/system_time.h"
#include "bsp.h"

#include <stdio.h>

void host_key_press(user_key_id_t key);
void host_vision_set_servo_command(uint16_t ticks, uint16_t seq);
void host_vision_set_control_command(uint8_t opcode, uint8_t mode,
                                     float arg0, float arg1, uint16_t seq);
void host_key_release(user_key_id_t key);

static int g_checks = 0;
static int g_failures = 0;

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

static void check_near(float actual, float expect, float tol, const char *name)
{
    g_checks++;
    if ((actual >= expect - tol) && (actual <= expect + tol)) {
        printf("[PASS] %s (actual=%.2f)\n", name, actual);
    } else {
        g_failures++;
        printf("[FAIL] %s (actual=%.2f expect=%.2f)\n", name, actual, expect);
    }
}

/* 推进 10 ms 并跑一次真实的应用任务。 */
static void step_ms(uint32_t ms)
{
    host_time_advance_ms(ms);
    BallTest_AppTask();
}

/* 模拟一次“按下并立即松开”的有效按键边沿。 */
static void tap_key(user_key_id_t key)
{
    host_key_press(key);
    BallTest_AppTask();
    host_key_release(key);
}

static void setup_roll_mode(void)
{
    host_time_reset();
    BallTest_AppInit();
    BallTest_SetMode(BALL_TEST_MODE_ROLL);
}

/* ===================== 模式与档位 ===================== */

static void test_mode_and_levels(void)
{
    ball_roll_status_t status;
    float default_theta;
    uint8_t i;

    setup_roll_mode();

    BallTest_GetRollStatus(&status);
    default_theta = status.theta_deg;

    check_true(status.state == BALL_ROLL_IDLE, "ROLL starts in IDLE");
    check_true(!status.running, "ROLL not running after mode switch");
    check_true(status.level == 0U, "default level index = 0");
    check_true(default_theta > 0.01f, "default level angle is positive");
    check_near(Servo_GetBeamLimit(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.001f,
               "ROLL idle keeps default beam limit");

    /*
     * 不写死具体角度：只要 K2 能依次切到不同的正角度，
     * 并且按满一圈后回到默认档位即可。
     * 具体 5/7.5/10/... 由 s_roll_angles[] 决定，改参数不会误报。
     */
    for (i = 1U; i < BALL_TEST_ROLL_LEVEL_COUNT; i++) {
        float previous = status.theta_deg;
        float delta;

        tap_key(USER_KEY_K2);
        BallTest_GetRollStatus(&status);
        delta = status.theta_deg - previous;
        if (delta < 0.0f) {
            delta = -delta;
        }
        check_true(delta > 0.01f, "K2 advances to another level");
        check_true(status.theta_deg > 0.01f, "level angle stays positive");
    }

    check_true(status.level == (BALL_TEST_ROLL_LEVEL_COUNT - 1U),
               "K2 reaches last level");

    tap_key(USER_KEY_K2);
    BallTest_GetRollStatus(&status);
    check_near(status.theta_deg, default_theta, 0.001f,
               "K2 wraps back to default level");
}

/* ===================== 完整一次左右滚动 ===================== */

static void test_continuous_cycle(void)
{
    ball_roll_status_t status;
    ball_test_roll_state_t last_state = BALL_ROLL_IDLE;
    uint8_t positive_entries = 0U;
    bool seen_negative = false;
    bool seen_return_pos = false;
    bool seen_return_neg = false;
    float default_theta;
    float max_actual = -100.0f;
    float min_actual = 100.0f;
    uint32_t elapsed = 0U;

    setup_roll_mode();
    BallTest_GetRollStatus(&status);
    default_theta = status.theta_deg;

    tap_key(USER_KEY_K1);   /* 启动连续循环 */

    BallTest_GetRollStatus(&status);
    check_true(status.running, "K1 starts continuous roll");
    check_near(Servo_GetBeamLimit(), BALL_TEST_ROLL_BEAM_LIMIT_DEG, 0.001f,
               "running test raises beam limit to configured max");

    /*
     * 跑到第二次进入 POSITIVE，证明 RETURN_NEG 之后会重新开始下一轮，
     * 而不是停在 FINISH。角度断言跟随当前默认档位，不写死 5°。
     */
    while (status.running && (elapsed < 10000U) && (positive_entries < 2U)) {
        float actual;

        step_ms(10U);
        elapsed += 10U;
        BallTest_GetRollStatus(&status);
        actual = Servo_GetBeamAngleActual();

        if ((status.state == BALL_ROLL_POSITIVE) &&
            (last_state != BALL_ROLL_POSITIVE)) {
            positive_entries++;
        }
        if (status.state == BALL_ROLL_RETURN_POS) {
            seen_return_pos = true;
        } else if (status.state == BALL_ROLL_NEGATIVE) {
            seen_negative = true;
        } else if (status.state == BALL_ROLL_RETURN_NEG) {
            seen_return_neg = true;
        }
        last_state = status.state;

        if (actual > max_actual) {
            max_actual = actual;
        }
        if (actual < min_actual) {
            min_actual = actual;
        }
    }

    check_true(positive_entries >= 2U, "POSITIVE repeats after RETURN_NEG");
    check_true(status.running, "sequence keeps running until K1 stop");
    check_true(seen_return_pos, "RETURN_POS stage executed");
    check_true(seen_negative, "NEGATIVE stage executed");
    check_true(seen_return_neg, "RETURN_NEG stage executed");
    check_near(max_actual, default_theta, 0.05f,
               "positive stage reaches configured +theta");
    check_near(min_actual, -default_theta, 0.05f,
               "negative stage reaches configured -theta");

    /* 用 K1 停止，确认恢复默认限幅并回到平衡点。 */
    tap_key(USER_KEY_K1);
    BallTest_GetRollStatus(&status);
    check_true(!status.running, "K1 stops continuous roll");
    check_true(status.state == BALL_ROLL_IDLE, "stop returns to IDLE");
    check_near(Servo_GetBeamLimit(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.001f,
               "stop restores default beam limit");

    for (int i = 0; i < 50; i++) {
        step_ms(10U);
    }
    check_near(Servo_GetBeamAngleActual(), 0.0f, 0.05f,
               "beam returns to balance point after stop");
}

/* ===================== 运行中停止 ===================== */

static void test_stop_during_positive(void)
{
    ball_roll_status_t status;
    uint32_t elapsed = 0U;
    float min_after_stop = 100.0f;
    float default_theta;
    bool saw_negative_after_stop = false;

    setup_roll_mode();
    BallTest_GetRollStatus(&status);
    default_theta = status.theta_deg;

    tap_key(USER_KEY_K1);

    /* 等到进入正倾角阶段并且真的到达当前档位角度。 */
    do {
        step_ms(10U);
        elapsed += 10U;
        BallTest_GetRollStatus(&status);
    } while ((elapsed < 3000U) &&
             !((status.state == BALL_ROLL_POSITIVE) &&
               (Servo_GetBeamAngleActual() >= (default_theta - 0.1f))));

    check_true(status.running, "still running before stop");
    check_near(Servo_GetBeamAngleActual(), default_theta, 0.1f,
               "positive angle reached before stop");

    tap_key(USER_KEY_K1);   /* 停止 */

    BallTest_GetRollStatus(&status);
    check_true(!status.running, "K1 stops running sequence");
    check_true(status.state == BALL_ROLL_IDLE, "state returns to IDLE on stop");
    check_near(Servo_GetBeamLimit(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.001f,
               "stop restores default beam limit");

    /* 停止后 1 s 内不允许出现反向阶段，摆杆应回 0°。 */
    for (int i = 0; i < 100; i++) {
        float actual;

        step_ms(10U);
        BallTest_GetRollStatus(&status);
        actual = Servo_GetBeamAngleActual();
        if (actual < min_after_stop) {
            min_after_stop = actual;
        }
        if (status.state == BALL_ROLL_NEGATIVE) {
            saw_negative_after_stop = true;
        }
    }

    check_true(!saw_negative_after_stop, "no NEGATIVE stage after stop");
    check_true(min_after_stop >= -0.05f, "no reverse motion after stop");
    check_near(Servo_GetBeamAngleActual(), 0.0f, 0.05f,
               "beam returns to balance point after stop");
}

/* ===================== 最大档位与限幅隔离 ===================== */

static void test_max_level(void)
{
    ball_roll_status_t status;
    uint32_t elapsed = 0U;
    float max_actual = -100.0f;
    float max_theta;
    uint8_t i;

    setup_roll_mode();

    /* 从默认档位按满一圈之前的次数，到达数组最后一档。 */
    for (i = 1U; i < BALL_TEST_ROLL_LEVEL_COUNT; i++) {
        tap_key(USER_KEY_K2);
    }

    BallTest_GetRollStatus(&status);
    max_theta = status.theta_deg;
    check_true(status.level == (BALL_TEST_ROLL_LEVEL_COUNT - 1U),
               "selected last configured level");

    tap_key(USER_KEY_K1);
    BallTest_GetRollStatus(&status);
    check_true(status.running, "max level sequence started");
    check_near(Servo_GetBeamLimit(), BALL_TEST_ROLL_BEAM_LIMIT_DEG, 0.001f,
               "max level run raises limit to configured max");

    while (status.running && (elapsed < 3000U)) {
        float actual;

        step_ms(10U);
        elapsed += 10U;
        BallTest_GetRollStatus(&status);
        actual = Servo_GetBeamAngleActual();
        if (actual > max_actual) {
            max_actual = actual;
        }
    }

    check_near(max_actual, max_theta, 0.1f, "max level reaches configured theta");

    tap_key(USER_KEY_K1);   /* 停止，避免跑完整周期 */
    BallTest_GetRollStatus(&status);
    check_true(!status.running, "max level test stopped by K1");
    check_near(Servo_GetBeamLimit(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.001f,
               "max level stop restores default beam limit");

    /* 回到 MANUAL 后必须恢复默认限幅（当前工程配置为 5°）。 */
    BallTest_SetMode(BALL_TEST_MODE_MANUAL);
    check_near(Servo_GetBeamLimit(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.001f,
               "leaving ROLL restores default beam limit");

    for (int i = 0; i < 100; i++) {
        host_time_advance_ms(10U);
        Servo_SetBeamAngle(20.0f);
    }
    check_near(Servo_GetBeamAngleActual(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.02f,
               "MANUAL still clamps beam to default limit");
}

/* ===================== 自由舵机角度模式（SERVO） ===================== */

static void test_servo_mode(void)
{
    uint8_t i;

    host_time_reset();
    BallTest_AppInit();
    BallTest_SetMode(BALL_TEST_MODE_SERVO);

    check_near(Servo_GetBeamLimit(), BALL_TEST_SERVO_LIMIT_DEG, 0.001f,
               "SERVO mode allows +/-30 deg");

    tap_key(USER_KEY_K1);
    check_near(Servo_GetBeamAngle(), BALL_TEST_SERVO_STEP_DEG, 0.01f,
               "SERVO K1 +3 deg");

    tap_key(USER_KEY_K2);
    check_near(Servo_GetBeamAngle(), 0.0f, 0.01f, "SERVO K2 back to 0");

    tap_key(USER_KEY_K2);
    check_near(Servo_GetBeamAngle(), -BALL_TEST_SERVO_STEP_DEG, 0.01f,
               "SERVO K2 -3 deg");

    /* 从 -3° 连续按 K1，最多停在 +30°。 */
    for (i = 0U; i < 11U; i++) {
        tap_key(USER_KEY_K1);
    }
    check_near(Servo_GetBeamAngle(), BALL_TEST_SERVO_LIMIT_DEG, 0.01f,
               "SERVO clamps at +30 deg");

    tap_key(USER_KEY_K1);
    check_near(Servo_GetBeamAngle(), BALL_TEST_SERVO_LIMIT_DEG, 0.01f,
               "SERVO stays at +30 deg");

    tap_key(USER_KEY_K2);
    check_near(Servo_GetBeamAngle(), BALL_TEST_SERVO_LIMIT_DEG -
               BALL_TEST_SERVO_STEP_DEG, 0.01f,
               "SERVO K2 steps down from limit");

    BallTest_SetMode(BALL_TEST_MODE_UART);
    check_near(Servo_GetBeamLimit(), SERVO_BEAM_LIMIT_DEFAULT_DEG, 0.001f,
               "leaving SERVO restores default limit");
}

/* ===================== REMOTE 远程舵机控制 ===================== */

static void test_remote_mode(void)
{
    ball_remote_status_t rs;
    uint16_t neutral = Servo_GetNeutralPulseTicks();

    host_time_reset();
    BallTest_AppInit();
    BallTest_SetMode(BALL_TEST_MODE_REMOTE);

    BallTest_GetRemoteStatus(&rs);
    check_true(!rs.enabled, "REMOTE starts disarmed");
    check_near((float)rs.target_ticks, (float)neutral, 1.0f,
               "REMOTE starts at neutral target");
    check_true(rs.result == BALL_REMOTE_RESULT_STOP, "REMOTE starts STOP");

    /* K1 现在只做本地停止，没有 0x02/START_REMOTE 时不会使能。 */
    tap_key(USER_KEY_K1);
    BallTest_GetRemoteStatus(&rs);
    check_true(!rs.enabled, "K1 only stops remote");

    /* START_REMOTE 不再要求先收到 0x02：先使能并保持零位。 */
    host_vision_set_control_command(BALL_CMD_START_REMOTE,
                                    BALL_TEST_MODE_REMOTE, 0.0f, 0.0f, 0U);
    BallTest_AppTask();
    BallTest_GetRemoteStatus(&rs);
    check_true(rs.enabled, "START_REMOTE enables without prior 0x02");
    check_true(rs.result == BALL_REMOTE_RESULT_NO_CMD,
               "remote starts in NO_CMD waiting state");

    host_time_advance_ms(301U);
    BallTest_AppTask();
    BallTest_GetRemoteStatus(&rs);
    check_true(!rs.enabled, "remote without 0x02 times out");
    check_true(rs.result == BALL_REMOTE_RESULT_TIMEOUT,
               "no-command timeout recorded");

    /* 注入 0x02（零位 +20 ticks，处于 ±5° 安全范围内），
     * 再由 0x03 START_REMOTE 使能。 */
    host_vision_set_servo_command((uint16_t)(neutral + 20U), 7U);
    host_vision_set_control_command(BALL_CMD_START_REMOTE,
                                    BALL_TEST_MODE_REMOTE, 0.0f, 0.0f, 1U);
    BallTest_AppTask();
    BallTest_GetRemoteStatus(&rs);
    check_true(rs.enabled, "START_REMOTE arms remote without K1");
    check_true(BallTest_GetLastCommandResult() == BALL_CMD_RESULT_OK,
               "START_REMOTE ack OK");
    check_true(rs.seq == 7U, "remote command seq recorded");
    check_near((float)rs.target_ticks, (float)(neutral + 20U), 1.0f,
               "remote target is neutral +20 ticks");

    /* 200 ms 后输出应向 1400 ticks 靠近，但不超过目标。 */
    for (int i = 0; i < 20; i++) {
        step_ms(10U);
    }
    BallTest_GetRemoteStatus(&rs);
    check_true(rs.enabled, "remote stays armed while commands are fresh");
    check_true(rs.output_ticks > neutral, "remote output moves up");
    check_true(rs.output_ticks <= (uint16_t)(neutral + 20U),
               "remote output does not overshoot");

    /* 超过 300 ms 没有新 0x02：退出远程并回零。 */
    host_time_advance_ms(301U);
    BallTest_AppTask();
    BallTest_GetRemoteStatus(&rs);
    check_true(!rs.enabled, "remote timeout disarms");
    check_true(rs.result == BALL_REMOTE_RESULT_TIMEOUT,
               "timeout result recorded");
    check_near((float)rs.target_ticks, (float)neutral, 1.0f,
               "timeout target returns to neutral");

    /* 新 0x02 + START_REMOTE 可重新使能。 */
    host_vision_set_servo_command(1360U, 8U);
    host_vision_set_control_command(BALL_CMD_START_REMOTE,
                                    BALL_TEST_MODE_REMOTE, 0.0f, 0.0f, 2U);
    BallTest_AppTask();
    BallTest_GetRemoteStatus(&rs);
    check_true(rs.enabled, "START_REMOTE re-arms remote");

    /* 超出零位 ±5° 的目标应被限幅并回执 CLAMP。 */
    host_vision_set_servo_command(1600U, 9U);
    step_ms(10U);
    BallTest_GetRemoteStatus(&rs);
    check_true(rs.enabled, "remote stays armed after clamp");
    check_near((float)rs.target_ticks,
               (float)Servo_GetNeutralPulseTicks() + 44.0f, 1.0f,
               "remote target clamped to neutral +5 deg");
    check_true(rs.result == BALL_REMOTE_RESULT_CLAMP,
               "clamp result recorded");

    tap_key(USER_KEY_K1);
    BallTest_GetRemoteStatus(&rs);
    check_true(!rs.enabled, "K1 local stop disarms remote");
    check_true(rs.result == BALL_REMOTE_RESULT_STOP, "stop result recorded");
}


/* ===================== 0x03 控制指令 ===================== */

static void test_control_commands(void)
{
    BallMeasurement measurement;
    float kp = 0.0f;
    float kd = 0.0f;

    host_time_reset();
    BallTest_AppInit();
    BallTest_SetMode(BALL_TEST_MODE_UART);

    /* 没有视觉数据时 START_AUTO 必须拒绝。 */
    host_vision_set_control_command(BALL_CMD_START_AUTO,
                                    BALL_TEST_MODE_UART, 0.0f, 0.0f, 1U);
    BallTest_AppTask();
    check_true(BallTest_GetLastCommandResult() ==
               BALL_CMD_RESULT_VISION_INVALID,
               "START_AUTO without vision -> VISN");
    check_true(BallControl_GetState() != BALL_STATE_AUTO,
               "START_AUTO without vision stays out of AUTO");

    /* 注入一帧有效钢珠位置后，START_AUTO 应成功。 */
    measurement.x_cm = 1.0f;
    measurement.frame_time_ms = 0U;
    measurement.seq = 1U;
    measurement.confidence = 95.0f;
    measurement.valid = true;
    BallControl_OnMeasurement(&measurement);

    host_vision_set_control_command(BALL_CMD_START_AUTO,
                                    BALL_TEST_MODE_UART, 0.0f, 0.0f, 2U);
    BallTest_AppTask();
    check_true(BallTest_GetLastCommandResult() == BALL_CMD_RESULT_OK,
               "START_AUTO ack OK");
    check_true(BallControl_GetState() == BALL_STATE_AUTO,
               "START_AUTO enters AUTO");

    host_vision_set_control_command(BALL_CMD_SET_TARGET,
                                    BALL_TEST_MODE_UART, 2.5f, 0.0f, 3U);
    BallTest_AppTask();
    check_true(BallTest_GetLastCommandResult() == BALL_CMD_RESULT_OK,
               "SET_TARGET ack OK");
    check_near(BallControl_GetTarget(), 2.5f, 0.01f,
               "SET_TARGET updates target");

    host_vision_set_control_command(BALL_CMD_SET_PD,
                                    BALL_TEST_MODE_UART, 0.5f, 0.2f, 4U);
    BallTest_AppTask();
    check_true(BallTest_GetLastCommandResult() == BALL_CMD_RESULT_OK,
               "SET_PD ack OK");
    BallControl_GetGains(&kp, &kd);
    check_near(kp, 0.5f, 0.01f, "SET_PD updates Kp");
    check_near(kd, 0.2f, 0.01f, "SET_PD updates Kd");

    host_vision_set_control_command(BALL_CMD_STOP,
                                    BALL_TEST_MODE_UART, 0.0f, 0.0f, 5U);
    BallTest_AppTask();
    check_true(BallTest_GetLastCommandResult() == BALL_CMD_RESULT_OK,
               "STOP ack OK");
    check_true(BallControl_GetState() == BALL_STATE_IDLE,
               "STOP leaves AUTO");

    host_vision_set_control_command(BALL_CMD_SET_MODE,
                                    BALL_TEST_MODE_MANUAL, 0.0f, 0.0f, 6U);
    BallTest_AppTask();
    check_true(BallTest_GetLastCommandResult() == BALL_CMD_RESULT_OK,
               "SET_MODE MANUAL ack OK");
    check_true(BallControl_GetState() == BALL_STATE_MANUAL,
               "SET_MODE MANUAL enters manual");
}

int main(void)
{
    printf("=== roll mode host tests ===\n");

    test_mode_and_levels();
    test_continuous_cycle();
    test_stop_during_positive();
    test_max_level();
    test_servo_mode();
    test_remote_mode();
    test_control_commands();

    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
