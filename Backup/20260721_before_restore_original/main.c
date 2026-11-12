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
/*
 * 当前200 PWM下车速过高，依靠增加循环延时会降低纠偏频率并扩大终点越界距离。
 * 因此基础PWM降至150，比例修正按相同比例由25降至19，保持原有转弯强度比例。
 */
#define TASK2_ARC_BASE_PWM 150
#define TASK2_LINE_KP 19
#define TASK2_LINE_LOST_VALUE 100
/*
 * 弧线结束只按灰度全丢失的真实持续时间判断。八路传感器全部稳定识别不到黑线
 * 达到500ms才退出；任意一路重新识别到黑线就重新计时。使用100us系统节拍，
 * 因此500ms对应5000个tick，避免循环执行时间变化影响实际确认时长。
 */
#define TASK2_LINE_SEEN_LIMIT 5U
#define ARC_ALL_LOST_EXIT_MS 500U
#define ARC_ALL_LOST_EXIT_TICKS (ARC_ALL_LOST_EXIT_MS * 10U)
/* 测试2到达C点后的右转补偿由5°增加约3°，以顺时针为正闭环原地右转8°。 */
#define TASK2_C_RIGHT_TURN_ANGLE_MD 8000L
/* 降速后把附加延时缩短至5ms，提高灰度纠偏频率并减少丢线确认期间的前冲距离。 */
#define TASK2_ARC_LOOP_DELAY_MS 5
/* 测试3按实车反馈单独增大至8ms，不改变测试2和测试4的循迹响应周期。 */
#define TASK3_ARC_LOOP_DELAY_MS 8
/*
 * 测试4的 C→B、D→A均把纠偏延时恢复为5ms，避免10ms控制间隔造成中途丢线。
 * D→A另外使用700ms全丢确认，其余弧线继续使用公共500ms退出时间。
 */
#define TASK4_CB_ARC_LOOP_DELAY_MS 5
#define TASK4_DA_ARC_LOOP_DELAY_MS 5
#define TASK4_DA_ALL_LOST_EXIT_MS 700U
#define TASK4_DA_ALL_LOST_EXIT_TICKS (TASK4_DA_ALL_LOST_EXIT_MS * 10U)
#define TASK3_ARC_EXTRA_TURN_COUNT 25000U
/* C 键到达 B 点后的左转编码器目标值；增大该值可加大进入 B→D 前的左转角度。 */
#define TASK3_ARC_END_TURN_COUNT 25000U
#define TASK3_C_TURN_COUNT 25000U
#define TASK3_D_TURN_COUNT 25000U
#define TASK3_EXTRA_TURN_PWM 220
#define TASK3_POINT_FORWARD_MS 100U

/*
 * 四个任务共用的陀螺仪航向约定：小车放在 A 点时的前进方向是全局 Y 轴负方向，
 * 定义为 0°；
 * 从车顶向下观察，顺时针（右转）为正角度。GY-6050 正面朝上安装时，芯片 Z 轴
 * 正方向对应逆时针，因此积分符号取 -1；若实物倒置，只需把该常量改为 +1。
 */
#define TASK4_GYRO_Z_CLOCKWISE_SIGN       (-1)
#define TASK4_GYRO_CALIBRATION_SAMPLES    (400U)
#define TASK4_GYRO_CALIBRATION_PERIOD_MS  (2U)
#define TASK4_GYRO_SAMPLE_MAX_TICKS       (300U)
#define TASK4_GYRO_SCALE_DIVISOR           (655LL)

/*
 * 场地 AB=100、AD=80，A→C/B→D 对角线相对 AB 的夹角为 atan(80/100)，
 * 即 38.6598°。角度统一采用毫度，避免控制循环使用浮点运算；C→B、D→A
 * 是半圆弧线，弧线自身完成约 180°航向变化，不应把原地转向误算为 128.66°。
 */
#define FIELD_DIAGONAL_ANGLE_MD            (38660L)
#define FIELD_HEADING_AB_MD                (0L)
#define FIELD_HEADING_AC_MD                (FIELD_DIAGONAL_ANGLE_MD)
#define FIELD_HEADING_CD_MD                (180000L)
#define FIELD_HEADING_BD_MD                (180000L - FIELD_DIAGONAL_ANGLE_MD)
#define FIELD_HEADING_DA_ENTRY_MD          (180000L)
/*
 * 测试3到达 B 后的左转由35°增加2°至37°，目标航向因此由145°变为143°。
 * 测试4的 A→C 右转在几何角38.66°上增加2°，绝对航向为40.66°；B→D
 * 补偿量为-2°时，目标绝对航向为143.34°，对应从180°向左转36.66°。
 * A→C和B→D使用独立补偿量，避免调整其中一段时连带改变另一段。
 */
#define TASK3_B_LEFT_TURN_ANGLE_MD          (37000L)
#define TASK3_HEADING_BD_MD                 (180000L - TASK3_B_LEFT_TURN_ANGLE_MD)
#define TASK4_AC_TURN_COMPENSATE_MD         (2000L)
#define TASK4_BD_TURN_COMPENSATE_MD         (-2000L)
#define TASK4_HEADING_AC_MD                 (FIELD_HEADING_AC_MD + \
                                             TASK4_AC_TURN_COMPENSATE_MD)
#define TASK4_HEADING_BD_MD                 (FIELD_HEADING_BD_MD - \
                                             TASK4_BD_TURN_COMPENSATE_MD)
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

static uint32_t qei_cnt[2] = {0U, 0U};
static uint32_t qei_base[2] = {0U, 0U};
static uint32_t qei_prev[2] = {0U, 0U};
static uint32_t qei_total[2] = {0U, 0U};

/*
 * 公共航向状态。积分分子保留 65.5 LSB/(°/s) 换算产生的小数余量；TimerA1
 * 每个 tick 为 100 us，因此 raw_delta*tick/655 的结果正好是毫度。
 */
static int32_t task4_gyro_z_bias_raw = 0;
static int64_t task4_yaw_integral_numerator = 0LL;
static uint32_t task4_gyro_last_tick = 0U;
static uint8_t task4_gyro_fault = 0U;

/* 多个任务都可能在 MPU6050 初始化、读取或转向超时时进入同一安全停车界面。 */
static void task4_show_gyro_error(void);
static bool task4_forward_by_time(uint32_t forward_ms);

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

/*
 * 循迹修正时较慢一侧可能低于普通运动使用的140最小值，因此不能调用limit_pwm，
 * 否则差速指令会被强制抬高并改变转弯比例。该函数只保留硬件安全上限，让降速后
 * 的左右轮差速保持连续，具体低PWM能否克服静摩擦由实车测试确认。
 */
static int limit_arc_pwm(int pwm)
{
    if (pwm > TASK1_MAX_PWM)
    {
        return TASK1_MAX_PWM;
    }

    if (pwm < -TASK1_MAX_PWM)
    {
        return -TASK1_MAX_PWM;
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
 * 返回不折叠到 ±180°范围的连续累计航向，专门用于判断本段弧线已经转过的角度。
 * 使用连续值可避免跨越 180°边界时从 +179°跳到 -179°而误算转角。
 */
static int32_t task4_get_unwrapped_heading_md(void)
{
    return (int32_t)(task4_yaw_integral_numerator /
                     TASK4_GYRO_SCALE_DIVISOR);
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
 * 测试3在纯灰度循迹期间暂停陀螺仪读取，弧线结束后依据赛道已知切线方向重建
 * 航向状态。同步时间戳可丢弃暂停期间的时间差，避免下一次采样把整段弧线时间
 * 乘到单个角速度样本上；输入和内部状态均使用毫度。
 */
static void task4_set_known_heading_md(int32_t heading_md)
{
    heading_md = task4_normalize_angle_md(heading_md);
    task4_yaw_integral_numerator =
        (int64_t)heading_md * TASK4_GYRO_SCALE_DIVISOR;
    task4_gyro_sync_time();
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
 * 初始化当前任务的航向：校准时电机保持停止，完成后把当前车头方向置为 0°，
 * 即用户指定的全局 -Y。MPU6050 没有磁力计，只能保持相对该启动方向的航向。
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
/*
 * 恢复最开始未使用陀螺仪时的灰度采样参数：每个窗口采样12次，某一路至少
 * 命中7次才认作黑线。只使用一个门槛，不再把短暂反光按弱线二次放行，避免
 * 到达 C 点后零星电平不断清除连续丢线计数。
 */
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

    /* 按最初版本的单一7/12门槛生成稳定灰度位图。 */
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

/*
 * 弧线退出窗口使用两轮中较小的累计计数，只有两侧车轮都完成最低运动距离后
 * 才允许把持续丢线解释为到达弧线末端，避免单轮打滑导致过早退出。
 */
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
 * 各任务共用的直线路段。进入函数时锁定当前绝对航向，后续用陀螺仪误差差速修正车轮；
 * 红外传感器仍确认到达场地横线，编码器最大计数用于传感器异常时的安全退出。
 */
static bool task4_drive_straight_until_line(uint32_t min_count,
                                             uint32_t max_count,
                                             uint8_t stop_after)
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

    if (stop_after != 0U)
    {
        stop_motor();
    }
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
 * 所有弧线共用最开始未引入陀螺仪时的循迹逻辑：灰度有效时按比例差速，丢线时
 * 完整保持最后一次有效偏差。曾识别到5个有效窗口后开放结束检测，只有八路传感器
 * 全部持续识别不到黑线达到 all_lost_exit_ticks 指定时间才退出；任何一路恢复都会
 * 取消本次计时。gyro_tracking
 * 非零时仅更新全局航向，始终不参与电机修正或退出；stop_after 决定是否停车，
 * loop_delay_ms 允许各任务独立设置电机响应等待时间。
 */
static bool follow_arc_to_line_end_original(uint8_t stop_after,
                                             uint8_t gyro_tracking,
                                             uint8_t loop_delay_ms,
                                             uint32_t all_lost_exit_ticks)
{
    uint32_t all_lost_start_tick = 0U;
    int line_error = 0;
    int last_error = 0;
    int correction;
    int left_pwm;
    int right_pwm;
    uint8_t seen_count = 0U;
    uint8_t line_confirmed = 0U;
    uint8_t all_lost_timing = 0U;

    qei_zero_soft();
    task4_gyro_sync_time();

    while (1)
    {
        /* 测试3传入0时完全跳过MPU6050读取，电机始终只由灰度偏差控制。 */
        if ((gyro_tracking != 0U) && !task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        update_qei_total();
        line_error = read_inf_delayed();

        if (line_error == TASK2_LINE_LOST_VALUE)
        {
            /*
             * read_inf_delayed 只有在八路稳定掩码全部为0时才返回丢线值。首次全丢
             * 记录真实系统节拍；计时期间不依赖循环次数，因此调节采样或延时不会
             * 改变调用方指定的真实退出时间。尚未确认进入过黑线时不开放结束计时。
             */
            if (line_confirmed != 0U)
            {
                if (all_lost_timing == 0U)
                {
                    all_lost_start_tick = TimerA1_get_tick_100us();
                    all_lost_timing = 1U;
                }
                else if ((TimerA1_get_tick_100us() - all_lost_start_tick) >=
                         all_lost_exit_ticks)
                {
                    break;
                }
            }

            /* 原始版本不限制偏差幅度，也不左右扫线，直接沿用最后一次有效偏差。 */
            line_error = last_error;
        }
        else
        {
            /* 任意一路重新稳定识别到黑线，连续全丢时间立即作废并从下次重新计算。 */
            all_lost_timing = 0U;
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
        left_pwm = limit_arc_pwm(TASK2_ARC_BASE_PWM + correction);
        right_pwm = limit_arc_pwm(TASK2_ARC_BASE_PWM - correction);
        set_motor_speed(left_pwm, right_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(loop_delay_ms);
    }

    if (stop_after != 0U)
    {
        stop_motor();
    }
    return true;
}

/*
 * 所有点位统一短暂停车：先停止电机，执行180ms声光提示，再额外等待150ms。
 * 总停留时间约330ms，确保下一段直行、循迹或原地转向从静止状态开始。
 */
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
    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Run Task2 Gyro");
    delay_ms(300);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> B");
    if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                          STRAIGHT_LINE_MAX_COUNT, 0U))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point B");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"B -> C");
    /*
     * B→C 按八路全丢持续时间退出后，用 MPU6050 闭环原地右转8°；转向函数
     * 以顺时针为正并在到达目标后停车，随后执行 C 点声光提示。
     */
    if (!follow_arc_to_line_end_original(1U, 1U,
                                          TASK2_ARC_LOOP_DELAY_MS,
                                          ARC_ALL_LOST_EXIT_TICKS) ||
        !task4_gyro_turn_relative(TASK2_C_RIGHT_TURN_ANGLE_MD))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point C");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"C -> D");
    if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                          STRAIGHT_LINE_MAX_COUNT, 0U))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point D");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"D -> A");
    if (!follow_arc_to_line_end_original(1U, 1U,
                                          TASK2_ARC_LOOP_DELAY_MS,
                                          ARC_ALL_LOST_EXIT_TICKS) ||
        !task4_gyro_turn_to(FIELD_HEADING_AB_MD))
    {
        task4_show_gyro_error();
        return;
    }
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

    /* A→C 相对 A→B 向右偏 atan(80/100)=38.66°。 */
    if (!task4_gyro_turn_to(FIELD_HEADING_AC_MD))
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> C Gyro");
    if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                          TASK3_STRAIGHT_COUNT, 0U))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point C");

    /* C 点左转 38.66°，从对角线恢复到 -Y，作为 C→B 半圆的切线方向。 */
    if (!task4_forward_by_time(TASK3_POINT_FORWARD_MS) ||
        !task4_gyro_turn_to(FIELD_HEADING_AB_MD))
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"C -> B Track");
    if (!follow_arc_to_line_end_original(0U, 0U,
                                          TASK3_ARC_LOOP_DELAY_MS,
                                          ARC_ALL_LOST_EXIT_TICKS))
    {
        task4_show_gyro_error();
        return;
    }
    /* C→B 全程只使用灰度；到 B 后按半圆出口切线重建180°航向，再执行对准。 */
    task4_set_known_heading_md(FIELD_HEADING_CD_MD);
    if (!task4_gyro_turn_to(FIELD_HEADING_CD_MD))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point B");

    /* 测试3的 B 点按实车结果向左转 35°，对准调整后的 B→D 绝对航向 145°。 */
    if (!task4_gyro_turn_to(TASK3_HEADING_BD_MD))
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"B -> D Gyro");
    if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                          TASK3_STRAIGHT_COUNT, 0U))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point D");

    /* D 点右转 38.66°恢复到 +Y，作为 D→A 半圆的切线方向。 */
    if (!task4_forward_by_time(TASK3_POINT_FORWARD_MS) ||
        !task4_gyro_turn_to(FIELD_HEADING_DA_ENTRY_MD))
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"D -> A Track");
    if (!follow_arc_to_line_end_original(1U, 0U,
                                          TASK3_ARC_LOOP_DELAY_MS,
                                          ARC_ALL_LOST_EXIT_TICKS))
    {
        task4_show_gyro_error();
        return;
    }
    /* D→A 同样纯灰度循迹；到 A 后依据半圆出口切线恢复初始-Y航向。 */
    task4_set_known_heading_md(FIELD_HEADING_AB_MD);
    if (!task4_gyro_turn_to(FIELD_HEADING_AB_MD))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point A");

    if (final_stop != 0U)
    {
        stop_motor();
    }
}

static NOINLINE void task3_drive_second_loop(void)
{
    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return;
    }

    task3_drive_second_loop_internal(1U, 1U);
}

/*
 * 点位附近的短距离前进也保持进入时的航向，防止固定时间开环动作在连续运行中
 * 逐步积累偏差。该动作只按时间结束，不参与红外点位判断。
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

/* 显示公共陀螺仪故障并保持停车，提示用户检查 3.3V、共地及 PA0/PA1 接线。 */
static void task4_show_gyro_error(void)
{
    stop_motor();
    buzzer_off();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MPU6050 ERROR");
    OLED_ShowString(0, 2, (u8 *)"SDA:PA0");
    OLED_ShowString(0, 4, (u8 *)"SCL:PA1");
    printf("MPU6050 error, check 3V3/GND/PA0/PA1\r\n");
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

        if (!task4_gyro_turn_to(TASK4_HEADING_AC_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"A -> C Gyro");
        if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                              TASK3_STRAIGHT_COUNT, 0U))
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point C");

        if (!task4_forward_by_time(TASK3_POINT_FORWARD_MS) ||
            !task4_gyro_turn_to(FIELD_HEADING_AB_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"C -> B Track");
        if (!follow_arc_to_line_end_original(0U, 1U,
                                              TASK4_CB_ARC_LOOP_DELAY_MS,
                                              ARC_ALL_LOST_EXIT_TICKS) ||
            !task4_gyro_turn_to(FIELD_HEADING_CD_MD))
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point B");

        if (!task4_gyro_turn_to(TASK4_HEADING_BD_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"B -> D Gyro");
        if (!task4_drive_straight_until_line(STRAIGHT_LINE_MIN_COUNT,
                                              TASK3_STRAIGHT_COUNT, 0U))
        {
            task4_show_gyro_error();
            return;
        }
        task2_point_prompt("Point D");

        if (!task4_forward_by_time(TASK3_POINT_FORWARD_MS) ||
            !task4_gyro_turn_to(FIELD_HEADING_DA_ENTRY_MD))
        {
            task4_show_gyro_error();
            return;
        }

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"D -> A Track");
        if (!follow_arc_to_line_end_original(1U, 1U,
                                              TASK4_DA_ARC_LOOP_DELAY_MS,
                                              TASK4_DA_ALL_LOST_EXIT_TICKS))
        {
            task4_show_gyro_error();
            return;
        }

        /* 每圈回到 A 后对准初始 -Y（0°），再进入下一圈，最后一圈同样保证停车朝向。 */
        OLED_ShowString(0, 4, (u8 *)"Align -Y");
        if (!task4_gyro_turn_to(FIELD_HEADING_AB_MD))
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
    int32_t heading_error_md;
    int correction;
    int base_pwm = TASK1_BASE_PWM;

    /*
     * 第一问没有原地转向，但仍初始化 MPU6050，并把放车时的 -Y 建立为 0°；
     * 直行期间用航向闭环替代仅依赖两轮编码器差值的修正，距离终点仍由编码器决定。
     */
    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return;
    }

    qei_zero_soft();
    task4_gyro_sync_time();

    while (average_delta < TASK1_TARGET_COUNT)
    {
        if (!task4_gyro_update())
        {
            task4_show_gyro_error();
            return;
        }

        average_delta = update_qei_total();
        m1_delta = qei_total[0];
        m2_delta = qei_total[1];

        if ((TASK1_TARGET_COUNT - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        heading_error_md = task4_normalize_angle_md(FIELD_HEADING_AB_MD -
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

        set_motor_speed(limit_pwm(base_pwm + correction),
                        limit_pwm(base_pwm - correction));
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
