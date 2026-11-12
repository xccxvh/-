/*------------------------------------------------------------------------------
 * Project: core_test
 * Target : TI MSPM0G3519
 *
 * Contest task 1:
 * Put the car at point A. The car drives straight to point B, stops, then gives
 * a sound and light indication. The target distance is controlled by the two
 * wheel encoder counts. Straight sections use encoder feedback to trim PWM.
 *----------------------------------------------------------------------------*/
#include "ti_msp_dl_config.h"
#include "bsp.h"

#if defined(__clang__)
#define NOINLINE __attribute__((noinline))
#else
#define NOINLINE
#endif

#define TASK1_STOP_PWM 1
#define TASK1_BASE_PWM 260
#define MOTOR_TEST_PWM 300
#define TASK1_MIN_PWM 140
#define TASK1_MAX_PWM 520
#define TASK1_LOOP_DELAY_MS 5
#define STRAIGHT_ENCODER_KP_DIV 80
#define STRAIGHT_ENCODER_MAX_CORRECTION 45
#define STRAIGHT_LEFT_QEI_INDEX 1U
#define STRAIGHT_RIGHT_QEI_INDEX 0U

/*
 * 任务1直行到 B 点的目标编码器计数，需要根据实车轮径、编码器分辨率和场地距离校准。
 * 小车提前停车时增大该值，越过 B 点时减小该值；任务2直线路段复用同一距离基准。
 */
#define TASK1_TARGET_COUNT 280000U

#define TASK1_SLOWDOWN_COUNT 2500U
#define TASK1_SLOWDOWN_PWM 190
#define TASK1_SIGNAL_HZ 2000
#define TASK1_SIGNAL_DUTY 50
#define TASK1_SIGNAL_MS 180

#define TASK3_STRAIGHT_COUNT 400000U
#define STRAIGHT_LINE_MIN_COUNT 80000U
/* 任务2直线找线的最大距离直接复用任务1目标，避免维护等价的中间别名。 */
#define STRAIGHT_LINE_MAX_COUNT TASK1_TARGET_COUNT
#define STRAIGHT_LINE_HIT_LIMIT 4
#define TASK2_ARC_BASE_PWM 230
#define TASK2_ARC_MIN_COUNT 31000U
#define TASK3_ARC_MIN_COUNT 31000U
#define TASK2_LINE_KP 28
#define TASK2_LINE_LOST_VALUE 100
/*
 * 循迹结束采用“连续丢线次数”消抖：B 键的 B→C 左转需要更早结束，
 * 因此使用较小阈值；其他弧线保留较大阈值，避免传感器瞬时漏检导致误停车。
 * 循迹循环周期约为 TASK2_ARC_LOOP_DELAY_MS，减小阈值会缩短离开黑线后的转动距离。
 */
#define TASK2_BC_LINE_LOST_LIMIT 12U
#define TASK_OTHER_LINE_LOST_LIMIT 20U
#define TASK2_LINE_SEEN_LIMIT 5U
#define TASK2_ARC_LOOP_DELAY_MS 2
#define TASK2_ARC_EXTRA_COUNT 0U
/*
 * B→C 提前识别终点后，仅让左轮继续向前补偿一小段，使车头在 C 点按实车方向回正。
 * COUNT 控制补偿距离，PWM 需要克服单轮静摩擦；左转不足时增大 COUNT，
 * 左转过多时减小 COUNT。提高驱动力后先使用较小计数，避免产生明显过转。
 */
/* 初始补偿按约 1～2°的小角度设置，实车可每次按 50U 小步调整。 */
#define TASK2_C_LEFT_FORWARD_COUNT 100U
#define TASK2_C_LEFT_FORWARD_PWM 100
/*
 * 编码器可能因循迹结束时的车轮惯性迅速超过较小目标值，因此额外规定最短驱动时间。
 * 只有编码器距离和最短时间都满足后才停车，确保补偿动作发生在 C 点停车之前。
 */
#define TASK2_C_LEFT_FORWARD_MIN_MS 150U
#define TASK3_ARC_EXTRA_TURN_COUNT 25000U
/* C 键到达 B 点后的左转编码器目标值；增大该值可加大进入 B→D 前的左转角度。 */
#define TASK3_ARC_END_TURN_COUNT 25000U
#define TASK3_C_TURN_COUNT 25000U
#define TASK3_D_TURN_COUNT 25000U
#define TASK3_EXTRA_TURN_PWM 220
#define TASK3_POINT_FORWARD_MS 100U

/*
 * 第四问陀螺仪航向约定：小车上电时的前进方向是全局 Y 轴负方向，定义为 0°；
 * 从车顶向下观察，顺时针（右转）为正角度。GY-6050 正面朝上安装时，芯片 Z 轴
 * 正方向对应逆时针，因此积分符号取 -1；若实物倒置，只需把该常量改为 +1。
 */
#define TASK4_GYRO_Z_CLOCKWISE_SIGN       (-1)
#define TASK4_GYRO_CALIBRATION_SAMPLES    (400U)
#define TASK4_GYRO_CALIBRATION_PERIOD_MS  (2U)
#define TASK4_GYRO_SAMPLE_MAX_TICKS       (300U)
#define TASK4_GYRO_SCALE_DIVISOR           (655LL)

/*
 * 原编码器转向计数均相同，按 90°直角转向替换。角度采用毫度，避免在 100 us
 * 控制周期内使用浮点运算；实车若机械结构导致路线角度不是直角，可单独修改各值。
 */
#define TASK4_INITIAL_RIGHT_ANGLE_MD       (90000L)
#define TASK4_C_LEFT_ANGLE_MD             (-90000L)
#define TASK4_B_LEFT_ANGLE_MD             (-90000L)
#define TASK4_D_RIGHT_ANGLE_MD             (90000L)
#define TASK4_TURN_TOLERANCE_MD            (1500L)
#define TASK4_TURN_STABLE_SAMPLES          (4U)
#define TASK4_TURN_MIN_PWM                 (150)
#define TASK4_TURN_MAX_PWM                 (260)
#define TASK4_TURN_KP_DIV                  (300L)
#define TASK4_TURN_LOOP_DELAY_MS           (5U)
#define TASK4_TURN_TIMEOUT_TICKS           (50000UL)

/* 直线段以陀螺航向误差修正左右轮 PWM，编码器仍负责距离上限和点位搜索窗口。 */
#define TASK4_STRAIGHT_KP_DIV              (250L)
#define TASK4_STRAIGHT_MAX_CORRECTION      (70)

/*
 * 弧线循迹退出诊断信息：记录退出原因和当时两轮较小的累计编码器计数。
 * 该信息保留给前三问和调试使用，第四问的弧线同时更新陀螺仪累计航向。
 */
#define ARC_EXIT_NONE 0U
#define ARC_EXIT_LINE_LOST 1U
#define ARC_EXIT_MAX_COUNT 2U

static uint32_t qei_cnt[2] = {0U, 0U};
static uint32_t qei_base[2] = {0U, 0U};
static uint32_t qei_prev[2] = {0U, 0U};
static uint32_t qei_total[2] = {0U, 0U};
static uint32_t arc_exit_count = 0U;
static uint8_t arc_exit_reason = ARC_EXIT_NONE;

/*
 * 第四问航向状态。积分分子保留 65.5 LSB/(°/s) 换算产生的小数余量；TimerA1
 * 每个 tick 为 100 us，因此 raw_delta*tick/655 的结果正好是毫度。
 */
static int32_t task4_gyro_z_bias_raw = 0;
static int64_t task4_yaw_integral_numerator = 0LL;
static uint32_t task4_gyro_last_tick = 0U;
static uint8_t task4_gyro_fault = 0U;

static int limit_pwm(int pwm)
{
    if (pwm > TASK1_MAX_PWM)
    {
        return TASK1_MAX_PWM;
    }

    if (pwm < -TASK1_MAX_PWM)
    {
        return -TASK1_MAX_PWM;
    }

    if ((pwm > 0) && (pwm < TASK1_MIN_PWM))
    {
        return TASK1_MIN_PWM;
    }

    if ((pwm < 0) && (pwm > -TASK1_MIN_PWM))
    {
        return -TASK1_MIN_PWM;
    }

    return pwm;
}

static int32_t qei_signed_delta(uint32_t now, uint32_t base)
{
    int32_t delta = (int32_t)now - (int32_t)base;

    if (delta > 32767)
    {
        delta -= 65536;
    }
    else if (delta < -32768)
    {
        delta += 65536;
    }

    return delta;
}

static uint32_t qei_abs_delta(uint32_t now, uint32_t base)
{
    int32_t delta = qei_signed_delta(now, base);

    return (delta >= 0) ? (uint32_t)delta : (uint32_t)(-delta);
}

static void qei_zero_soft(void)
{
    get_motor_qei_cnt(qei_base);
    qei_prev[0] = qei_base[0];
    qei_prev[1] = qei_base[1];
    qei_total[0] = 0U;
    qei_total[1] = 0U;
}

static void show_task1_status(uint32_t m1_delta, uint32_t m2_delta)
{
    OLED_ShowNum(64, 0, m1_delta, 6, 16);
    OLED_ShowNum(64, 2, m2_delta, 6, 16);
}

static void stop_motor(void)
{
    set_motor_speed(TASK1_STOP_PWM, TASK1_STOP_PWM);
}

/* 将任意航向误差折叠到 [-180°, 180°]，保证转向总是选择较短方向。 */
static int32_t task4_normalize_angle_md(int32_t angle_md)
{
    while (angle_md > 180000L)
    {
        angle_md -= 360000L;
    }

    while (angle_md < -180000L)
    {
        angle_md += 360000L;
    }

    return angle_md;
}

/* 根据积分分子换算当前航向，返回值采用“0°=-Y、顺时针为正”的毫度坐标。 */
static int32_t task4_get_heading_md(void)
{
    return task4_normalize_angle_md(
        (int32_t)(task4_yaw_integral_numerator / TASK4_GYRO_SCALE_DIVISOR));
}

/*
 * 丢弃停车提示等非控制阶段经过的时间，只同步时间戳而不积分。若把一次新的速率
 * 样本乘到很长的旧时间间隔上，会产生虚假角度，因此每段运动开始前都调用本函数。
 */
static void task4_gyro_sync_time(void)
{
    task4_gyro_last_tick = TimerA1_get_tick_100us();
}

/*
 * 读取一次 Z 轴角速度并按真实 100 us 节拍积分。正常控制周期远小于 30 ms；若因
 * OLED、红外采样或中断阻塞造成更长间隔，则丢弃该段积分，防止单点速率外推过远。
 */
static bool task4_gyro_update(void)
{
    int16_t gyro_z_raw;
    int32_t corrected_raw;
    uint32_t now_tick;
    uint32_t elapsed_ticks;

    if (!mpu6050_read_gyro_z_raw(&gyro_z_raw))
    {
        task4_gyro_fault = 1U;
        return false;
    }

    now_tick = TimerA1_get_tick_100us();
    elapsed_ticks = now_tick - task4_gyro_last_tick;
    task4_gyro_last_tick = now_tick;

    if (elapsed_ticks > TASK4_GYRO_SAMPLE_MAX_TICKS)
    {
        return true;
    }

    corrected_raw = (int32_t)gyro_z_raw - task4_gyro_z_bias_raw;
    task4_yaw_integral_numerator +=
        (int64_t)TASK4_GYRO_Z_CLOCKWISE_SIGN *
        (int64_t)corrected_raw * (int64_t)elapsed_ticks;
    return true;
}

/*
 * 初始化第四问航向：校准时电机保持停止，完成后把当前车头方向置为 0°，即用户
 * 指定的全局 -Y。MPU6050 没有磁力计，运行中只能保持相对该启动方向的航向。
 */
static bool task4_gyro_prepare(void)
{
    stop_motor();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MPU6050 init");

    if (!mpu6050_init())
    {
        return false;
    }

    OLED_ShowString(0, 2, (u8 *)"Keep still...");
    if (!mpu6050_calibrate_gyro_z(TASK4_GYRO_CALIBRATION_SAMPLES,
                                  TASK4_GYRO_CALIBRATION_PERIOD_MS,
                                  &task4_gyro_z_bias_raw))
    {
        return false;
    }

    task4_yaw_integral_numerator = 0LL;
    task4_gyro_fault = 0U;
    task4_gyro_sync_time();
    return true;
}

/*
 * 按绝对航向原地闭环转向。大误差时提高 PWM，接近目标时降速，并要求连续多个
 * 样本进入容差带后才完成，以抑制车轮惯性造成的越界；超时或 I2C 错误立即停车。
 */
static bool task4_gyro_turn_to(int32_t target_heading_md)
{
    int32_t error_md;
    int32_t abs_error_md;
    int turn_pwm;
    uint8_t stable_samples = 0U;
    uint32_t start_tick = TimerA1_get_tick_100us();

    target_heading_md = task4_normalize_angle_md(target_heading_md);
    task4_gyro_sync_time();

    while ((TimerA1_get_tick_100us() - start_tick) < TASK4_TURN_TIMEOUT_TICKS)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        error_md = task4_normalize_angle_md(target_heading_md -
                                             task4_get_heading_md());
        abs_error_md = (error_md >= 0L) ? error_md : -error_md;

        if (abs_error_md <= TASK4_TURN_TOLERANCE_MD)
        {
            stop_motor();
            stable_samples++;
            if (stable_samples >= TASK4_TURN_STABLE_SAMPLES)
            {
                return true;
            }
        }
        else
        {
            stable_samples = 0U;
            turn_pwm = (int)(abs_error_md / TASK4_TURN_KP_DIV);
            if (turn_pwm < TASK4_TURN_MIN_PWM)
            {
                turn_pwm = TASK4_TURN_MIN_PWM;
            }
            else if (turn_pwm > TASK4_TURN_MAX_PWM)
            {
                turn_pwm = TASK4_TURN_MAX_PWM;
            }

            /* 正误差表示需要顺时针右转：左轮前进、右轮后退。 */
            if (error_md > 0L)
            {
                set_motor_speed(turn_pwm, -turn_pwm);
            }
            else
            {
                set_motor_speed(-turn_pwm, turn_pwm);
            }
        }

        delay_ms(TASK4_TURN_LOOP_DELAY_MS);
    }

    stop_motor();
    task4_gyro_fault = 1U;
    return false;
}

/* 以当前累计航向为基准执行相对转向，正角度右转、负角度左转。 */
static bool task4_gyro_turn_relative(int32_t delta_heading_md)
{
    return task4_gyro_turn_to(task4_get_heading_md() + delta_heading_md);
}

static void set_straight_speed_closed_loop(int base_pwm)
{
    int32_t left_count = (int32_t)qei_total[STRAIGHT_LEFT_QEI_INDEX];
    int32_t right_count = (int32_t)qei_total[STRAIGHT_RIGHT_QEI_INDEX];
    int correction = (int)((right_count - left_count) / STRAIGHT_ENCODER_KP_DIV);

    if (correction > STRAIGHT_ENCODER_MAX_CORRECTION)
    {
        correction = STRAIGHT_ENCODER_MAX_CORRECTION;
    }
    else if (correction < -STRAIGHT_ENCODER_MAX_CORRECTION)
    {
        correction = -STRAIGHT_ENCODER_MAX_CORRECTION;
    }

    set_motor_speed(limit_pwm(base_pwm + correction),
                    limit_pwm(base_pwm - correction));
}

static void point_signal_once(void)
{
    LED1_toggle;
    LED2_toggle;
    set_RGB(0x00, 0xff, 0x00);
    set_led_buzzer(TASK1_SIGNAL_HZ, TASK1_SIGNAL_DUTY, TASK1_SIGNAL_DUTY);
    delay_ms(TASK1_SIGNAL_MS);

    set_RGB(0x00, 0x00, 0x00);
    set_led_buzzer(TASK1_SIGNAL_HZ, 100, 100);
}

static void buzzer_off(void)
{
    set_led_buzzer(TASK1_SIGNAL_HZ, 100, 100);
}

#define INF_COLLECT_SAMPLE_COUNT 12U
#define INF_COLLECT_THRESHOLD 7U

static void collect_inf_counts(uint8_t *hit_count)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        hit_count[i] = 0U;
    }

    for (i = 0U; i < INF_COLLECT_SAMPLE_COUNT; i++)
    {
        if (INF_1 != 0)
        {
            hit_count[0]++;
        }
        if (INF_2 != 0)
        {
            hit_count[1]++;
        }
        if (INF_3 != 0)
        {
            hit_count[2]++;
        }
        if (INF_4 != 0)
        {
            hit_count[3]++;
        }
        if (INF_5 != 0)
        {
            hit_count[4]++;
        }
        if (INF_6 != 0)
        {
            hit_count[5]++;
        }
        if (INF_7 != 0)
        {
            hit_count[6]++;
        }
        if (INF_8 != 0)
        {
            hit_count[7]++;
        }
        delay_ms(1);
    }
}

static int inf_error_from_mask(uint8_t mask)
{
    int sum = 0;
    int count = 0;

    if ((mask & 0x01U) != 0U)
    {
        sum -= 7;
        count++;
    }
    if ((mask & 0x02U) != 0U)
    {
        sum -= 5;
        count++;
    }
    if ((mask & 0x04U) != 0U)
    {
        sum -= 3;
        count++;
    }
    if ((mask & 0x08U) != 0U)
    {
        sum -= 1;
        count++;
    }
    if ((mask & 0x10U) != 0U)
    {
        sum += 1;
        count++;
    }
    if ((mask & 0x20U) != 0U)
    {
        sum += 3;
        count++;
    }
    if ((mask & 0x40U) != 0U)
    {
        sum += 5;
        count++;
    }
    if ((mask & 0x80U) != 0U)
    {
        sum += 7;
        count++;
    }

    if (count == 0)
    {
        return TASK2_LINE_LOST_VALUE;
    }

    return -(sum / count);
}

static int read_inf_delayed(void)
{
    uint8_t hit_count[8];
    uint8_t mask = 0U;
    uint8_t i;

    collect_inf_counts(hit_count);

    for (i = 0U; i < 8U; i++)
    {
        if (hit_count[i] >= INF_COLLECT_THRESHOLD)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return inf_error_from_mask(mask);
}

static void show_signed_num(uint8_t x, uint8_t y, int value)
{
    if (value < 0)
    {
        OLED_ShowString(x, y, (u8 *)"-");
        OLED_ShowNum(x + 8U, y, (uint32_t)(-value), 3, 16);
    }
    else
    {
        OLED_ShowString(x, y, (u8 *)" ");
        OLED_ShowNum(x + 8U, y, (uint32_t)value, 3, 16);
    }
}

static NOINLINE void sensor_collect_mode(void)
{
    uint8_t hit_count[8];
    uint8_t mask;
    int error;

    stop_motor();

    while (1)
    {
        collect_inf_counts(hit_count);
        mask = read_inf_mask();
        error = inf_error_from_mask(mask);

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"1-4:");
        OLED_ShowNum(40, 0, hit_count[0], 2, 16);
        OLED_ShowNum(64, 0, hit_count[1], 2, 16);
        OLED_ShowNum(88, 0, hit_count[2], 2, 16);
        OLED_ShowNum(112, 0, hit_count[3], 2, 16);

        OLED_ShowString(0, 2, (u8 *)"5-8:");
        OLED_ShowNum(40, 2, hit_count[4], 2, 16);
        OLED_ShowNum(64, 2, hit_count[5], 2, 16);
        OLED_ShowNum(88, 2, hit_count[6], 2, 16);
        OLED_ShowNum(112, 2, hit_count[7], 2, 16);

        OLED_ShowString(0, 4, (u8 *)"M:");
        OLED_ShowNum(16, 4, mask, 3, 16);
        OLED_ShowString(64, 4, (u8 *)"E:");
        show_signed_num(80, 4, error);

        OLED_ShowString(0, 6, (u8 *)"# exit");
        buzzer_off();

        if (get_keyboard_value() == '#')
        {
            while (get_keyboard_value() == '#')
            {
                buzzer_off();
                delay_ms(20);
            }
            break;
        }

        delay_ms(80);
    }

    stop_motor();
}
static void show_motor_test_menu(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A:L+ B:L-");
    OLED_ShowString(0, 2, (u8 *)"C:R+ D:R-");
    OLED_ShowString(0, 4, (u8 *)"1:++ 2:--");
    OLED_ShowString(0, 6, (u8 *)"#:Stop *:Exit");
}

static NOINLINE void motor_direction_test_mode(void)
{
    char key;
    char last_key = 0;

    stop_motor();
    show_motor_test_menu();

    while (1)
    {
        key = get_keyboard_value();

        if ((key != 0) && (key != last_key))
        {
            last_key = key;

            switch (key)
            {
            case 'A':
                set_motor_speed(MOTOR_TEST_PWM, TASK1_STOP_PWM);
                OLED_ShowString(72, 0, (u8 *)"L+");
                break;
            case 'B':
                set_motor_speed(-MOTOR_TEST_PWM, TASK1_STOP_PWM);
                OLED_ShowString(72, 0, (u8 *)"L-");
                break;
            case 'C':
                set_motor_speed(TASK1_STOP_PWM, MOTOR_TEST_PWM);
                OLED_ShowString(72, 2, (u8 *)"R+");
                break;
            case 'D':
                set_motor_speed(TASK1_STOP_PWM, -MOTOR_TEST_PWM);
                OLED_ShowString(72, 2, (u8 *)"R-");
                break;
            case '1':
                set_motor_speed(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
                OLED_ShowString(72, 4, (u8 *)"++");
                break;
            case '2':
                set_motor_speed(-MOTOR_TEST_PWM, -MOTOR_TEST_PWM);
                OLED_ShowString(72, 4, (u8 *)"--");
                break;
            case '#':
                stop_motor();
                show_motor_test_menu();
                break;
            case '*':
                stop_motor();
                while (get_keyboard_value() == '*')
                {
                    buzzer_off();
                    delay_ms(20);
                }
                return;
            default:
                break;
            }
        }
        else if (key == 0)
        {
            last_key = 0;
        }

        buzzer_off();
        delay_ms(40);
    }
}
static void show_task_menu(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A: Task1");
    OLED_ShowString(0, 2, (u8 *)"B: Task2");
    OLED_ShowString(0, 4, (u8 *)"C: Task3");
    OLED_ShowString(0, 6, (u8 *)"D:T4 #:S *:M");
}

static NOINLINE char wait_task_key(void)
{
    char key;
    char last_key = 0;
    uint8_t stable_count = 0U;
    uint8_t release_count = 0U;

    while (release_count < 10U)
    {
        if (get_keyboard_value() == 0)
        {
            release_count++;
        }
        else
        {
            release_count = 0U;
        }

        buzzer_off();
        delay_ms(20);
    }

    while (1)
    {
        key = get_keyboard_value();

        if (((key >= 'A') && (key <= 'D')) || (key == '#') || (key == '*'))
        {
            if (key == last_key)
            {
                stable_count++;
            }
            else
            {
                last_key = key;
                stable_count = 1U;
            }

            if (stable_count >= 6U)
            {
                while (get_keyboard_value() == key)
                {
                    buzzer_off();
                    delay_ms(20);
                }

                buzzer_off();
                return key;
            }
        }
        else
        {
            last_key = 0;
            stable_count = 0U;
        }

        buzzer_off();
        delay_ms(20);
    }
}

static void task_not_ready(char key)
{
    stop_motor();
    buzzer_off();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Task not ready");
    OLED_ShowString(0, 2, (u8 *)"Key:");
    OLED_ShowString(48, 2, (key == 'B') ? (u8 *)"B" : ((key == 'C') ? (u8 *)"C" : (u8 *)"D"));
    point_signal_once();
    delay_ms(500);
}

static uint32_t update_qei_total(void)
{
    int32_t m1_step;
    int32_t m2_step;

    get_motor_qei_cnt(qei_cnt);
    m1_step = qei_signed_delta(qei_cnt[0], qei_prev[0]);
    m2_step = qei_signed_delta(qei_cnt[1], qei_prev[1]);
    qei_prev[0] = qei_cnt[0];
    qei_prev[1] = qei_cnt[1];

    qei_total[0] += (m1_step >= 0) ? (uint32_t)m1_step : (uint32_t)(-m1_step);
    qei_total[1] += (m2_step >= 0) ? (uint32_t)m2_step : (uint32_t)(-m2_step);

    return (qei_total[0] + qei_total[1]) / 2U;
}

static uint32_t qei_min_total(void)
{
    return (qei_total[0] < qei_total[1]) ? qei_total[0] : qei_total[1];
}

static void drive_straight_count(uint32_t target_count)
{
    uint32_t average_delta = 0U;
    int base_pwm = TASK1_BASE_PWM;

    qei_zero_soft();

    while (average_delta < target_count)
    {
        average_delta = update_qei_total();

        if ((target_count - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        set_straight_speed_closed_loop(base_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}

static void drive_straight_until_line(uint32_t min_count, uint32_t max_count)
{
    uint32_t average_delta = 0U;
    int base_pwm = TASK1_BASE_PWM;
    uint8_t hit_count = 0U;

    qei_zero_soft();

    while (average_delta < max_count)
    {
        average_delta = update_qei_total();

        if ((max_count - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        if (average_delta > min_count)
        {
            if (read_inf() != TASK2_LINE_LOST_VALUE)
            {
                hit_count++;
                if (hit_count >= STRAIGHT_LINE_HIT_LIMIT)
                {
                    break;
                }
            }
            else
            {
                hit_count = 0U;
            }
        }

        set_straight_speed_closed_loop(base_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}

/*
 * 第四问直线路段。进入函数时锁定当前绝对航向，后续用陀螺仪误差差速修正车轮；
 * 红外传感器仍确认到达场地横线，编码器最大计数用于传感器异常时的安全退出。
 */
static bool task4_drive_straight_until_line(uint32_t min_count, uint32_t max_count)
{
    uint32_t average_delta = 0U;
    uint8_t hit_count = 0U;
    int32_t target_heading_md = task4_get_heading_md();
    int32_t heading_error_md;
    int correction;
    int base_pwm = TASK1_BASE_PWM;

    qei_zero_soft();
    task4_gyro_sync_time();

    while (average_delta < max_count)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        average_delta = update_qei_total();
        if ((max_count - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        if (average_delta > min_count)
        {
            if (read_inf() != TASK2_LINE_LOST_VALUE)
            {
                hit_count++;
                if (hit_count >= STRAIGHT_LINE_HIT_LIMIT)
                {
                    break;
                }
            }
            else
            {
                hit_count = 0U;
            }
        }

        heading_error_md = task4_normalize_angle_md(target_heading_md -
                                                     task4_get_heading_md());
        correction = (int)(heading_error_md / TASK4_STRAIGHT_KP_DIV);
        if (correction > TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = TASK4_STRAIGHT_MAX_CORRECTION;
        }
        else if (correction < -TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = -TASK4_STRAIGHT_MAX_CORRECTION;
        }

        /* 正误差需要向右修正，因此提高左轮、降低右轮的前进 PWM。 */
        set_motor_speed(limit_pwm(base_pwm + correction),
                        limit_pwm(base_pwm - correction));
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    return true;
}

static void task3_turn_right_by_count(uint32_t turn_count)
{
    uint32_t turn_delta = 0U;

    if (turn_count == 0U)
    {
        return;
    }

    qei_zero_soft();
    while (turn_delta < turn_count)
    {
        update_qei_total();
        turn_delta = qei_total[0] + qei_total[1];
        set_motor_speed(TASK3_EXTRA_TURN_PWM, -TASK3_EXTRA_TURN_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}
static void task3_turn_opposite_by_count(uint32_t turn_count)
{
    uint32_t turn_delta = 0U;

    if (turn_count == 0U)
    {
        return;
    }

    qei_zero_soft();
    while (turn_delta < turn_count)
    {
        update_qei_total();
        turn_delta = qei_total[0] + qei_total[1];
        set_motor_speed(-TASK3_EXTRA_TURN_PWM, TASK3_EXTRA_TURN_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}
static void task3_back_by_count(uint32_t back_count)
{
    uint32_t back_delta = 0U;

    if (back_count == 0U)
    {
        return;
    }

    qei_zero_soft();
    while (back_delta < back_count)
    {
        update_qei_total();
        back_delta = qei_total[0] + qei_total[1];
        set_motor_speed(-TASK3_EXTRA_TURN_PWM, -TASK3_EXTRA_TURN_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}
static void task3_forward_by_time(uint32_t forward_ms)
{
    if (forward_ms == 0U)
    {
        return;
    }

    set_motor_speed(TASK1_BASE_PWM, TASK1_BASE_PWM);
    delay_ms(forward_ms);
    stop_motor();
}

/*
 * 仅驱动左轮向前指定的编码器距离，右轮保持停止，用于 B→C 提前结束循迹后的
 * 车身回正。函数进入时重新建立软件编码器零点，达到目标后立即停车，随后才允许
 * C 点的声光提示开始；根据实车测试选择左轮作为补偿轮，较高 PWM 用于可靠克服
 * 单轮驱动时的静摩擦，编码器目标值和最短时间共同保证动作有效。
 */
static void task2_c_left_wheel_forward(uint32_t target_count)
{
    uint32_t left_delta = 0U;
    uint32_t elapsed_ms = 0U;

    if (target_count == 0U)
    {
        stop_motor();
        return;
    }

    qei_zero_soft();
    /* 状态计数占用第 0、2 行，因此将补偿标记放在第 4 行，便于实车观察。 */
    OLED_ShowString(0, 4, (u8 *)"C Align");
    /*
     * 循迹函数以不停机方式退出，因此必须先把右轮置为停止、左轮置为向前驱动，
     * 再开始读取补偿编码器，避免进入循环后的第一拍仍沿用上一组循迹 PWM。
     */
    set_motor_speed(TASK2_C_LEFT_FORWARD_PWM, TASK1_STOP_PWM);
    /* 距离或最短持续时间任一未满足时都继续驱动，停车只能发生在循环结束之后。 */
    while ((left_delta < target_count) ||
           (elapsed_ms < TASK2_C_LEFT_FORWARD_MIN_MS))
    {
        update_qei_total();
        left_delta = qei_total[STRAIGHT_LEFT_QEI_INDEX];
        set_motor_speed(TASK2_C_LEFT_FORWARD_PWM, TASK1_STOP_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
        elapsed_ms += TASK1_LOOP_DELAY_MS;
    }

    stop_motor();
}

/*
 * 沿黑线行驶至线段末端。lost_limit 是连续丢线确认次数，允许每个路段
 * 独立调整停止时刻，避免校准某一处转弯时改变其他循迹路段的动作。gyro_tracking
 * 仅由第四问置 1，使弧线运动产生的实际转角也进入绝对航向积分。
 */
static void follow_arc_to_line_end_config(uint32_t min_count, uint32_t extra_count,
                                          uint32_t extra_turn_count, uint8_t stop_after,
                                          uint8_t lost_limit, uint32_t slow_count,
                                          uint32_t max_count, int slow_pwm,
                                          int lost_left_pwm, int lost_right_pwm,
                                          uint8_t gyro_tracking)
{
    uint32_t average_delta = 0U;
    uint32_t min_delta = 0U;
    uint32_t extra_target = 0U;
    int line_error = 0;
    int last_error = 0;
    int correction;
    int base_pwm = TASK2_ARC_BASE_PWM;
    int left_pwm = TASK2_ARC_BASE_PWM;
    int right_pwm = TASK2_ARC_BASE_PWM;
    uint8_t lost_count = 0U;
    uint8_t seen_count = 0U;
    uint8_t line_confirmed = 0U;

    qei_zero_soft();
    arc_exit_reason = ARC_EXIT_NONE;
    arc_exit_count = 0U;

    while (1)
    {
        if ((gyro_tracking != 0U) && !task4_gyro_update())
        {
            stop_motor();
            return;
        }

        average_delta = update_qei_total();
        min_delta = qei_min_total();

        /*
         * slow_count 和 max_count 为 0 时关闭对应功能，保持 B、C 键原有行为。
         * D 键进入 A 点附近后先降速，以缩短传感器确认和电机惯性造成的越界距离。
         */
        if ((slow_count > 0U) && (min_delta >= slow_count))
        {
            base_pwm = slow_pwm;
        }

        /* 红外异常时以编码器最大距离强制结束，防止 D 键越过 A 点后继续前进。 */
        if ((max_count > 0U) && (min_delta >= max_count))
        {
            arc_exit_reason = ARC_EXIT_MAX_COUNT;
            arc_exit_count = min_delta;
            break;
        }

        line_error = read_inf_delayed();

        if (line_error == TASK2_LINE_LOST_VALUE)
        {
            /*
             * 在尚未进入终点窗口时可能长时间检测不到黑线，计数必须饱和而不能溢出。
             * 进入窗口后若仍处于丢线状态，可立即按设定阈值结束循迹。
             */
            if (lost_count < lost_limit)
            {
                lost_count++;
            }
            if ((line_confirmed != 0U) &&
                (min_delta >= min_count) &&
                (lost_count >= lost_limit))
            {
                arc_exit_reason = ARC_EXIT_LINE_LOST;
                arc_exit_count = min_delta;
                break;
            }

            line_error = last_error;

            /*
             * D 键在终点前短暂丢线时，以两个正 PWM 低速向右搜索黑线；两个参数
             * 为 0 时保持普通 B、C 键的原有 PID 丢线处理，不改变其他任务行为。
             */
            if ((lost_left_pwm > 0) && (lost_right_pwm > 0))
            {
                set_motor_speed(lost_left_pwm, lost_right_pwm);
                show_task1_status(qei_total[0], qei_total[1]);
                delay_ms(TASK2_ARC_LOOP_DELAY_MS);
                continue;
            }
        }
        else
        {
            lost_count = 0U;
            if (seen_count < TASK2_LINE_SEEN_LIMIT)
            {
                seen_count++;
                if (seen_count >= TASK2_LINE_SEEN_LIMIT)
                {
                    line_confirmed = 1U;
                }
            }
            last_error = line_error;
        }

        correction = line_error * TASK2_LINE_KP;
        left_pwm = limit_pwm(base_pwm + correction);
        right_pwm = limit_pwm(base_pwm - correction);
        set_motor_speed(left_pwm, right_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK2_ARC_LOOP_DELAY_MS);
    }

    if (extra_count > 0U)
    {
        extra_target = average_delta + extra_count;
        while (average_delta < extra_target)
        {
            if ((gyro_tracking != 0U) && !task4_gyro_update())
            {
                stop_motor();
                return;
            }

            average_delta = update_qei_total();
            set_motor_speed(left_pwm, right_pwm);
            show_task1_status(qei_total[0], qei_total[1]);
            delay_ms(TASK2_ARC_LOOP_DELAY_MS);
        }
    }

    task3_turn_opposite_by_count(extra_turn_count);

    if (stop_after != 0U)
    {
        stop_motor();
    }
}

/* 普通循迹路段不启用末端减速和最大距离限制，维持 B、C 键原来的运行方式。 */
static void follow_arc_to_line_end(uint32_t min_count, uint32_t extra_count,
                                   uint32_t extra_turn_count, uint8_t stop_after,
                                   uint8_t lost_limit)
{
    follow_arc_to_line_end_config(min_count, extra_count, extra_turn_count, stop_after,
                                  lost_limit, 0U, 0U, TASK2_ARC_BASE_PWM, 0, 0, 0U);
}

/*
 * 第四问弧线继续使用红外闭环识别黑线，但同步读取 MPU6050，使 C→B、D→A
 * 两段弧线的旋转也计入全局航向。退出时停车，随后再执行对应的陀螺仪转向。
 */
static bool task4_follow_arc_to_line_end(void)
{
    task4_gyro_sync_time();
    follow_arc_to_line_end_config(TASK3_ARC_MIN_COUNT, TASK2_ARC_EXTRA_COUNT,
                                  0U, 1U, TASK_OTHER_LINE_LOST_LIMIT,
                                  0U, 0U, TASK2_ARC_BASE_PWM, 0, 0, 1U);
    return (task4_gyro_fault == 0U);
}
static void task2_point_prompt(const char *label)
{
    stop_motor();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)label);
    point_signal_once();
    delay_ms(150);
}

static NOINLINE void task2_drive_first_loop(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Run Task2");
    delay_ms(300);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> B");
    drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT, STRAIGHT_LINE_MAX_COUNT);
    task2_point_prompt("Point B");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"B -> C");
    /*
     * B→C 提前确认黑线结束后暂不停车，立即让左轮单独前进一小段完成实车回正，
     * 然后再进入 C 点停车与声光提示，避免识别过快造成车头仍然偏左。
     */
    follow_arc_to_line_end(TASK2_ARC_MIN_COUNT, TASK2_ARC_EXTRA_COUNT, 0U, 0U,
                           TASK2_BC_LINE_LOST_LIMIT);
    task2_c_left_wheel_forward(TASK2_C_LEFT_FORWARD_COUNT);
    task2_point_prompt("Point C");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"C -> D");
    drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT, STRAIGHT_LINE_MAX_COUNT);
    task2_point_prompt("Point D");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"D -> A");
    follow_arc_to_line_end(TASK2_ARC_MIN_COUNT, TASK2_ARC_EXTRA_COUNT, 0U, 1U,
                           TASK_OTHER_LINE_LOST_LIMIT);
    task2_point_prompt("Point A");
    stop_motor();
}

static NOINLINE void task3_drive_second_loop_internal(uint8_t final_stop, uint8_t show_start)
{
    if (show_start != 0U)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"Run Task3");
        delay_ms(300);
    }

    task3_turn_right_by_count(TASK3_ARC_EXTRA_TURN_COUNT);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> C");
    drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT, TASK3_STRAIGHT_COUNT);
    task2_point_prompt("Point C");
    task3_forward_by_time(TASK3_POINT_FORWARD_MS);
    task3_turn_opposite_by_count(TASK3_C_TURN_COUNT);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"C -> B");
    follow_arc_to_line_end(TASK3_ARC_MIN_COUNT, TASK2_ARC_EXTRA_COUNT, 0U, 1U,
                           TASK_OTHER_LINE_LOST_LIMIT);
    task2_point_prompt("Point B");
    /* 到达 B 点后按编码器计数左转，使车头对准 B→D 直线路段。 */
    task3_turn_opposite_by_count(TASK3_ARC_END_TURN_COUNT);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"B -> D");
    drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT, TASK3_STRAIGHT_COUNT);
    task2_point_prompt("Point D");
    task3_forward_by_time(TASK3_POINT_FORWARD_MS);
    task3_turn_right_by_count(TASK3_D_TURN_COUNT);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"D -> A");
    /* 第三问保留原有编码器转向和红外循迹，不受第四问陀螺仪控制改造影响。 */
    follow_arc_to_line_end(TASK3_ARC_MIN_COUNT, TASK2_ARC_EXTRA_COUNT, 0U, 1U,
                           TASK_OTHER_LINE_LOST_LIMIT);
    task2_point_prompt("Point A");

    if (final_stop != 0U)
    {
        stop_motor();
    }
}

static NOINLINE void task3_drive_second_loop(void)
{
    task3_drive_second_loop_internal(1U, 1U);
}

/*
 * 第四问点位附近的短距离前进也保持进入时的航向，防止固定时间开环动作在连续四圈
 * 中逐步积累偏差。该动作只按时间结束，不参与红外点位判断。
 */
static bool task4_forward_by_time(uint32_t forward_ms)
{
    int32_t target_heading_md = task4_get_heading_md();
    int32_t heading_error_md;
    int correction;
    uint32_t start_tick = TimerA1_get_tick_100us();
    uint32_t duration_ticks = forward_ms * 10U;

    task4_gyro_sync_time();
    while ((TimerA1_get_tick_100us() - start_tick) < duration_ticks)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        heading_error_md = task4_normalize_angle_md(target_heading_md -
                                                     task4_get_heading_md());
        correction = (int)(heading_error_md / TASK4_STRAIGHT_KP_DIV);
        if (correction > TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = TASK4_STRAIGHT_MAX_CORRECTION;
        }
        else if (correction < -TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = -TASK4_STRAIGHT_MAX_CORRECTION;
        }

        set_motor_speed(limit_pwm(TASK1_BASE_PWM + correction),
                        limit_pwm(TASK1_BASE_PWM - correction));
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    return true;
}

/* 显示第四问陀螺仪故障并保持停车，提示用户检查 3.3V、共地及 PA0/PA1 接线。 */
static void task4_show_gyro_error(void)
{
    stop_motor();
    buzzer_off();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MPU6050 ERROR");
    OLED_ShowString(0, 2, (u8 *)"SDA:PA0");
    OLED_ShowString(0, 4, (u8 *)"SCL:PA1");
    printf("task4 MPU6050 error, check 3V3/GND/PA0/PA1\r\n");
    point_signal_once();
    delay_ms(500U);
}

/*
 * 第四问完整流程：在 A 点把 -Y 方向建立为 0°，每圈的直线和原地转向采用陀螺仪
 * 闭环，C→B、D→A 弧线保留红外循迹并同步积分航向；回到 A 后主动校正到 0°，
 * 从而阻止四圈之间累积航向误差。任何 I2C/转向超时都会立即中止并停车。
 */
static NOINLINE void task4_drive_four_loops(void)
{
    uint8_t loop;

    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return;
    }

    for (loop = 0U; loop < 4U; loop++)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"Task4 Loop:");
        OLED_ShowNum(96, 0, (uint32_t)loop + 1U, 1, 16);
        delay_ms(100U);

        if (!task4_gyro_turn_relative(TASK4_INITIAL_RIGHT_ANGLE_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"A -> C Gyro");
        if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                              TASK3_STRAIGHT_COUNT))
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point C");

        if (!task4_forward_by_time(TASK3_POINT_FORWARD_MS) ||
            !task4_gyro_turn_relative(TASK4_C_LEFT_ANGLE_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"C -> B Track");
        if (!task4_follow_arc_to_line_end())
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point B");

        if (!task4_gyro_turn_relative(TASK4_B_LEFT_ANGLE_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"B -> D Gyro");
        if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                              TASK3_STRAIGHT_COUNT))
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point D");

        if (!task4_forward_by_time(TASK3_POINT_FORWARD_MS) ||
            !task4_gyro_turn_relative(TASK4_D_RIGHT_ANGLE_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"D -> A Track");
        if (!task4_follow_arc_to_line_end())
        {
            task4_show_gyro_error();
            return;
        }

        /* 每圈回到 A 后对准初始 -Y（0°），再进入下一圈，最后一圈同样保证停车朝向。 */
        OLED_ShowString(0, 4, (u8 *)"Align -Y");
        if (!task4_gyro_turn_to(0L))
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point A");
    }

    stop_motor();
}

static NOINLINE void task1_drive_a_to_b(void);

static NOINLINE void run_selected_task(char key)
{
    OLED_Clear();

    switch (key)
    {
    case 'A':
        OLED_ShowString(0, 0, (u8 *)"Run Task1");
        OLED_ShowString(0, 2, (u8 *)"A -> B");
        delay_ms(300);
        task1_drive_a_to_b();
        point_signal_once();
        break;
    case 'B':
        task2_drive_first_loop();
        break;
    case 'C':
        task3_drive_second_loop();
        break;
    case 'D':
        task4_drive_four_loops();
        break;
    case '#':
        sensor_collect_mode();
        break;
    case '*':
        motor_direction_test_mode();
        break;
    default:
        stop_motor();
        break;
    }
}
static NOINLINE void task1_drive_a_to_b(void)
{
    uint32_t m1_delta = 0U;
    uint32_t m2_delta = 0U;
    uint32_t average_delta = 0U;
    int32_t m1_step = 0;
    int32_t m2_step = 0;
    int base_pwm = TASK1_BASE_PWM;

    qei_zero_soft();

    while (average_delta < TASK1_TARGET_COUNT)
    {
        get_motor_qei_cnt(qei_cnt);
        m1_step = qei_signed_delta(qei_cnt[0], qei_prev[0]);
        m2_step = qei_signed_delta(qei_cnt[1], qei_prev[1]);
        qei_prev[0] = qei_cnt[0];
        qei_prev[1] = qei_cnt[1];

        qei_total[0] += (m1_step >= 0) ? (uint32_t)m1_step : (uint32_t)(-m1_step);
        qei_total[1] += (m2_step >= 0) ? (uint32_t)m2_step : (uint32_t)(-m2_step);
        m1_delta = qei_total[0];
        m2_delta = qei_total[1];
        average_delta = (m1_delta + m2_delta) / 2U;

        if ((TASK1_TARGET_COUNT - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        set_straight_speed_closed_loop(base_pwm);
        show_task1_status(m1_delta, m2_delta);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}

int main(void)
{
    char task_key;

    SYSCFG_DL_init();
    uart0_init(115200);
    timerg6_pwm_rgb_init();
    TimerA1_init();
    keyboard_init();

    OLED_Init();
    OLED_Clear();

    stop_motor();
    buzzer_off();
    set_RGB(0x00, 0x00, 0x20);
    printf("task menu ready\r\n");

    while (1)
    {
        show_task_menu();
        task_key = wait_task_key();
        printf("run task %c\r\n", task_key);
        run_selected_task(task_key);
        stop_motor();
        buzzer_off();
        delay_ms(300);
    }
}
