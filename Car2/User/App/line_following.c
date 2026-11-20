#include "App/line_following.h"
#include "bsp.h"
#include <stdarg.h>

/*
 * 小车循迹主应用。
 *
 * 控制思路：
 * - 启动时将阈值写入灰度模块 0x13/0x14；compare_bits 由模块按该阈值生成。
 * - G1~G8 对应 adc[0]~adc[7]；理想轨迹中心在 G4/G5 之间，加权平均得到 line_error。
 *   error > 0 表示黑线偏左（靠近 G1~G4），error < 0 表示偏右（靠近 G5~G8）。
 * - TIMA1 分频 10 ms 中断：读 compare_bits 做循迹；每 100 ms 另读 0x20 ADC 供 LCD 显示 G1~G8。
 * - read_status 不更新 adc[]，合并状态时必须保留已有 ADC，避免栈垃圾导致 G1~G8 乱跳。
 * - 当前为匀速模式：全程基准速度固定 150，直到 A 点停车。
 * - 线偏 G1~G4（error>0）：M1 加速、M2 减速甚至反转；线偏 G5~G8（error<0）：M2 加速、M1 减速甚至反转。
 * - 保留可选的“第二弯后降速”分段代码：把 LINE_FOLLOW_SPEED_PROFILE_ENABLE 改为 1 后，
 *   第二弯前 250，识别到第二弯后 3 s 内线性降到 100，之后保持 100。
 * - 转弯识别（仅分段模式使用）：|error| >= 30 连续 100 ms 记一次弯；
 *   回到 |error| <= 12 并保持 200 ms 才允许识别下一个弯。
 *
 * 安全约束：
 * - M1/M2 nFAULT/nSLEEP 异常时停车。
 * - 串口可输出 VOFA+ FireWater 格式 G1~G8 ADC，供上位机波形显示。
 */

#define LINE_FOLLOW_MOTOR_STATUS_MS         (50U)
#define LINE_FOLLOW_LCD_MS                  (100U)
#define LINE_FOLLOW_IMU_MS                  (50U)
#define LINE_FOLLOW_UART_MS                 (1000U)

/*
 * VOFA+ FireWater 灰度波形输出（UART0）。
 * 1：TIMA1 每 20 ms 快照 G1~G8 并置待发标志；0：改发 [LINE] 文字调试日志。
 * 发送仍在主循环完成（避免 ISR 内阻塞 UART），快照时刻由硬件 1 ms 时基保证固定 20 ms。
 */
#define LINE_FOLLOW_VOFA_GRAY_ENABLE        (1U)
#define LINE_FOLLOW_VOFA_GRAY_MS            (20U)

/*
 * 每 N 次 10 ms 采样补读一次 G1~G8 ADC（100 ms），写入 snapshot.adc[] 供 LCD 显示。
 * 循迹控制仍只用 compare_bits，不依赖本周期 ADC 读。
 */
#define LINE_FOLLOW_GRAY_ADC_PERIOD         (10U)

/*
 * 循迹灰度阈值：与 GRAY_TRACK_DEFAULT_LED_THRESHOLD 一致，启动时写入模块 0x13/0x14。
 */
#define LINE_FOLLOW_GRAY_THRESHOLD          GRAY_TRACK_DEFAULT_LED_THRESHOLD

/*
 * 速度策略开关：
 * - 0：匀速模式（当前使用），全程使用 K2 选中的档位速度；
 * - 1：启用“第二弯后降速”分段：第二弯前 250，识别到第二弯后 3 s 内
 *      线性降到 100，之后保持 100。
 *
 * 关闭时转弯计数代码仍保留，但不会改变速度，方便以后重新开启。
 */
#define LINE_FOLLOW_SPEED_PROFILE_ENABLE         (0U)

/*
 * 速度档位（单位：电机千分比），K2 在停车/结束状态循环选择：
 *   第 1 档 100、第 2 档 150、第 3 档 200、第 4 档 250。
 *
 * 默认从 100 开始，便于先验证 A 点识别；差速修正叠加在档位速度上，
 * 上限/下限仍保持 ±650。
 */
#define LINE_FOLLOW_SPEED_GEAR_COUNT             (4U)

static const int16_t s_line_speed_gears[LINE_FOLLOW_SPEED_GEAR_COUNT] = {
    100, 150, 200, 250
};
static volatile uint8_t s_line_speed_gear = 0U;
#define LINE_FOLLOW_R2_TURN2_START_SPEED_PERMILLE (250)
#define LINE_FOLLOW_R2_TURN2_SPEED_PERMILLE      (100)
#define LINE_FOLLOW_R2_TURN2_RAMP_MS             (3000U)
#define LINE_FOLLOW_MAX_SPEED_PERMILLE           (650)
#define LINE_FOLLOW_MIN_SPEED_PERMILLE           (-650)

/*
 * 转弯识别（用于确定“第二个转弯”的入口）。
 *
 * 判定方式：
 * - 8 路加权误差 |error| >= 30，连续保持 100 ms，记为进入一个弯；
 * - 之后必须回到 |error| <= 12 并保持 200 ms，才允许识别下一个弯，
 *   避免同一个弯的左右抖动被重复计数；
 * - 第 2 次识别到转弯时，启动 3 s 降速斜坡。
 *
 * 现场调整：
 * - 第二个弯识别不到：把 ENTER_ERROR 从 30 降到 25；
 * - 普通弯被误判成第二个弯：提高到 35~40，或增大 REARM_MS；
 * - 两个弯之间没有回中直道：减小 REARM_MS。
 */
#define LINE_FOLLOW_TURN_ENTER_ERROR             (30)
#define LINE_FOLLOW_TURN_CONFIRM_MS              (100U)
#define LINE_FOLLOW_TURN_REARM_ERROR             (12)
#define LINE_FOLLOW_TURN_REARM_MS                (200U)
#define LINE_FOLLOW_CONTROL_PERIOD_MS            (10U)
#define LINE_FOLLOW_TURN_CONFIRM_CYCLES          \
    (LINE_FOLLOW_TURN_CONFIRM_MS / LINE_FOLLOW_CONTROL_PERIOD_MS)

/*
 * 循迹转向增益。
 *
 * 误差范围：8 路加权平均后，单点 G1/G8 有效时约 ±50，中心 G4/G5 附近约 0。
 * 原来的 Kp=1.2（12/10）在 300 基准速度下最大差速只有 ±60，
 * 弯道外侧传感器已经压线但车头来不及转，表现为“拐弯不跟线”。
 *
 * 现在改成两段式：
 * - 基础段：|error| <= 20 时 Kp=1.6（16/10），保证直道不因离散跳变而摆动；
 * - 急弯段：|error| > 20 时，超出部分再叠加 1.4 倍增益，
 *   error=50 时总修正约 122（原来只有 60），单轮差速明显增大。
 *
 * 调参建议：
 * - 仍然转不过来：先增大 LINE_FOLLOW_TURN_BOOST_NUM；
 * - 直道左右摆动：减小 LINE_FOLLOW_KP_NUM 或提高 BOOST_START；
 * - 不要把单轮修正直接拉到接近 650 的饱和值，否则会变成原地打转。
 */
#define LINE_FOLLOW_KP_NUM                  (16)
#define LINE_FOLLOW_KP_DEN                  (10)

/* 急弯补偿起点与额外增益：error 超过 20 后，每多 1 点误差再增加 1.4 点修正。 */
#define LINE_FOLLOW_TURN_BOOST_START        (20)
#define LINE_FOLLOW_TURN_BOOST_NUM          (24)
#define LINE_FOLLOW_TURN_BOOST_DEN          (10)

#define LINE_FOLLOW_LCD_LINE_HEIGHT         (16U)
#define LINE_FOLLOW_LCD_MAX_LINES           (15U)
#define LINE_FOLLOW_LCD_TEXT_CHARS          (35U)

/* ===================== 要求 2：按键启动 + 计时 + A 点停车 ===================== */

/* 不再限制最短运行时间：只要离开起点并满足终点传感器条件就停车。 */

/* 题目要求整圈不超过 20 s，超过即本次测试失败并主动停车。 */
#define LINE_FOLLOW_R2_TIMEOUT_MS               (20000U)

/* 起点脱离：连续 100 ms 压黑路数不超过 2 路，才允许识别 A 点终点线。 */
#define LINE_FOLLOW_R2_START_CLEAR_MAX_SENSORS  (2U)
#define LINE_FOLLOW_R2_START_CLEAR_MS           (100U)

/*
 * A 点 5 cm 横向启停线判定。
 *
 * 条件：同时压黑路数 >= LINE_FOLLOW_R2_FINISH_SENSOR_MIN，
 * 并连续保持 LINE_FOLLOW_R2_FINISH_HOLD_MS 后停车。
 *
 * 识别数量就在这里改：
 *   4U -> 至少 4 路（当前）
 *   5U -> 至少 5 路
 *   3U -> 至少 3 路（更容易触发，也更容易在弯道误判）
 */
#define LINE_FOLLOW_R2_FINISH_SENSOR_MIN        (4U)

/* 终点模式需要连续保持的时间，按 10 ms 控制周期换算成周期数。 */
#define LINE_FOLLOW_R2_FINISH_HOLD_MS           (150U)
#define LINE_FOLLOW_R2_FINISH_CONFIRM_COUNT         (LINE_FOLLOW_R2_FINISH_HOLD_MS / LINE_FOLLOW_CONTROL_PERIOD_MS)

/* 连续丢线 1 s 仍未找回黑线则判失败并停车。 */
#define LINE_FOLLOW_R2_LOST_STOP_MS             (1000U)

/* 终点/故障后的短时反向制动，配合测试点位置满足 2 cm 停车偏差。 */
#define LINE_FOLLOW_R2_BRAKE_SPEED              (300)
#define LINE_FOLLOW_R2_BRAKE_MS                 (50U)

typedef enum {
    LINE_FOLLOW_STATE_BOOT = 0,
    LINE_FOLLOW_STATE_RUN,
    LINE_FOLLOW_STATE_LOST,
    LINE_FOLLOW_STATE_I2C_ERROR,
    LINE_FOLLOW_STATE_MOTOR_BLOCKED
} line_follow_state_t;

typedef struct {
    char text[LINE_FOLLOW_LCD_TEXT_CHARS + 1U];
    uint16_t color;
    bool valid;
} line_follow_lcd_cache_t;

/*
 * 要求 2 的运行状态机。
 * 控制 ISR 只负责实时循迹和终点/超时/丢线判定；按键、制动、显示和串口打印
 * 放在主循环，避免在 10 ms 中断里执行阻塞操作。
 */
typedef enum {
    LINE_FOLLOW_R2_WAIT_KEY = 0,   /* 等待 K1 按下启动 */
    LINE_FOLLOW_R2_RUN,            /* 循迹运行并计时 */
    LINE_FOLLOW_R2_FINISHED,       /* 识别到 A 点，正常结束 */
    LINE_FOLLOW_R2_TIMEOUT,        /* 超过 20 s */
    LINE_FOLLOW_R2_LOST            /* 连续丢线超时 */
} line_follow_r2_state_t;

static volatile line_follow_r2_state_t s_r2_state = LINE_FOLLOW_R2_WAIT_KEY;
static volatile bool s_r2_drive_enabled = false;    /* ISR 是否允许驱动电机 */
static volatile bool s_r2_result_done = true;       /* 制动/结果显示是否已处理 */
static volatile bool s_r2_finish_armed = false;     /* 是否已驶离起点，允许识别终点 */
static volatile uint32_t s_r2_start_ms = 0U;        /* 按键启动时刻 */
static volatile uint32_t s_r2_final_ms = 0U;        /* 锁存的最终成绩 */
static volatile uint32_t s_r2_start_clear_tick = 0U;
static volatile uint32_t s_r2_lost_tick = 0U;
static volatile uint8_t s_r2_start_clear_count = 0U;
static volatile uint8_t s_r2_finish_count = 0U;
static volatile uint8_t s_r2_lost_count = 0U;

/*
 * 转弯计数与第二弯降速状态。
 * 10 ms ISR 更新，主循环/LCD 只读，因此统一声明为 volatile。
 */
static volatile uint8_t  s_r2_turn_count = 0U;
static volatile bool     s_r2_turn_rearmed = true;
static volatile uint8_t  s_r2_turn_confirm_count = 0U;
static volatile bool     s_r2_turn_rearm_active = false;
static volatile uint32_t s_r2_turn_rearm_tick = 0U;
static volatile bool     s_r2_turn2_ramp_active = false;
static volatile uint32_t s_r2_turn2_ramp_start_ms = 0U;

/*
 * 传感器横向权重（加权平均 line_error）：
 * - 原厂约定 G1~G4 为正、G5~G8 为负，使线在 G1~G4 时 Err>0（左转）。
 * - 实测本车传感器安装方向与原厂相反：灰线在 G1~G4 时原权重算出的 Err 为负，
 *   控制会往反方向转，导致丢线后绕圈。
 * - 因此这里把 8 个权重整体取反，保证“线在 G1~G4 → Err>0 → M1 加速左转”，
 *   控制方向和显示符号都与原厂约定一致。
 * - G4/G5 仍对称（-22 / +22）代表中心。
 */
static const int16_t s_sensor_weight[GRAY_TRACK_CHANNEL_COUNT] = {
    -50, -38, -28, -22, 22, 28, 38, 50
};

static gray_track_snapshot_t s_gray_snapshot;
static motor_status_t s_motor_m1;
static motor_status_t s_motor_m2;
static float s_imu_ypr[3];
static line_follow_state_t s_state = LINE_FOLLOW_STATE_BOOT;
static line_follow_lcd_cache_t s_lcd_cache[LINE_FOLLOW_LCD_MAX_LINES];

static uint32_t s_motor_status_tick;
static uint32_t s_lcd_tick;
static uint32_t s_imu_tick;
static uint32_t s_uart_tick;

static int16_t s_line_error;
static int16_t s_last_line_error;
static int16_t s_last_m1_forward;
static int16_t s_last_m2_forward;
static int16_t s_m1_command;
static int16_t s_m2_command;
static uint8_t s_active_bits;
static uint8_t s_active_count;
static bool s_ever_had_line;
static bool s_have_valid_gray;
static bool s_gray_using_stale;
static bool s_gray_ok;
static bool s_imu_ready;
static uint8_t s_gray_adc_counter;

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
static volatile bool s_vofa_send_pending;
static char s_vofa_line[96];
#endif

static gray_track_snapshot_t s_last_valid_gray;

static int16_t line_follow_clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < (int32_t)min_value) {
        return min_value;
    }

    if (value > (int32_t)max_value) {
        return max_value;
    }

    return (int16_t)value;
}

static const char *line_follow_state_text(line_follow_state_t state)
{
    switch (state) {
    case LINE_FOLLOW_STATE_RUN:
        return "RUN";
    case LINE_FOLLOW_STATE_LOST:
        return "LOST";
    case LINE_FOLLOW_STATE_I2C_ERROR:
        return "I2CERR";
    case LINE_FOLLOW_STATE_MOTOR_BLOCKED:
        return "MOTOR";
    default:
        return "BOOT";
    }
}

/* 要求 2 状态机对应的 LCD/串口文字。 */
static const char *line_follow_r2_state_text(line_follow_r2_state_t state)
{
    switch (state) {
    case LINE_FOLLOW_R2_WAIT_KEY:
        return "WAIT";
    case LINE_FOLLOW_R2_RUN:
        return "RUN";
    case LINE_FOLLOW_R2_FINISHED:
        return "FINISH";
    case LINE_FOLLOW_R2_TIMEOUT:
        return "TIMEOUT";
    case LINE_FOLLOW_R2_LOST:
        return "LOST";
    default:
        return "?";
    }
}

static int16_t line_follow_m1_forward_to_cmd(int16_t forward_permille)
{
    /*
     * 实测本车两台电机的实际转向都与原厂代码的假设相反：
     * 原厂 M1 用负速度表示前进，但本车上电时两轮都向后。
     * 因此这里把 M1 的符号取反，逻辑“向前”直接下发正速度。
     * 例：forward=+300 -> cmd=+300，实车向前。
     */
    return forward_permille;
}

static int16_t line_follow_m2_forward_to_cmd(int16_t forward_permille)
{
    /*
     * M2 同样需要取反才能让实车向前。
     * 另外实测左轮（M2）比右轮略快，这里按 95% 做轻微补偿；
     * 如果补偿后仍偏快或偏慢，调整下面的 95 即可（例如 90/105）。
     */
    return (int16_t)(-(forward_permille * 95) / 100);
}

/*
 * 根据有效位计算加权平均 line_error。
 *
 * 返回值：
 * - true：至少 1 路有效，*error 为加权平均结果。
 * - false：无有效位，*error 置 0（上层按直行处理）。
 */
static bool line_follow_compute_error(uint8_t active_bits,
                                      int16_t *error,
                                      uint8_t *active_count)
{
    int32_t weighted_sum = 0;
    uint8_t count = 0U;
    uint32_t i;

    if ((error == NULL) || (active_count == NULL)) {
        return false;
    }

    for (i = 0U; i < GRAY_TRACK_CHANNEL_COUNT; i++) {
        if ((active_bits & (uint8_t)(1U << i)) != 0U) {
            weighted_sum += s_sensor_weight[i];
            count++;
        }
    }

    *active_count = count;
    if (count == 0U) {
        *error = 0;
        return false;
    }

    *error = (int16_t)(weighted_sum / (int32_t)count);
    return true;
}

/*
 * 判断当前压黑路数是否达到终点识别阈值。
 *
 * 阈值由 LINE_FOLLOW_R2_FINISH_SENSOR_MIN 决定（当前 4）。
 * 保持时间由调用方用 s_r2_finish_count 累计，
 * 连续 FINISH_CONFIRM_COUNT 个 10 ms 周期满足后才停车。
 */
static bool line_follow_is_finish_count(uint8_t active_count)
{
    return (active_count >= LINE_FOLLOW_R2_FINISH_SENSOR_MIN);
}

static bool line_follow_m1_m2_available(void)
{
    /*
     * 只检查参与循迹的 M1/M2。
     *
     * M3/M4 暂时不用，有些硬件版本可能未接电机或 nSLEEP/nFAULT 状态不具备参考意义；
     * 因此不让未使用通道阻塞循迹，但会在初始化和停止流程里确保 M3/M4 无 PWM 输出。
     */
    if (motor_is_fault(MOTOR_M1) || motor_is_fault(MOTOR_M2)) {
        return false;
    }

    if (motor_is_asleep(MOTOR_M1) || motor_is_asleep(MOTOR_M2)) {
        return false;
    }

    return true;
}

static void line_follow_stop_drive(line_follow_state_t state)
{
    s_m1_command = 0;
    s_m2_command = 0;
    s_state = state;
    motor_set_speed(MOTOR_M1, 0);
    motor_set_speed(MOTOR_M2, 0);
    motor_set_speed(MOTOR_M3, 0);
    motor_set_speed(MOTOR_M4, 0);
}

static void line_follow_apply_forward_speed(int16_t m1_forward,
                                            int16_t m2_forward,
                                            line_follow_state_t drive_state)
{
    s_m1_command = line_follow_m1_forward_to_cmd(m1_forward);
    s_m2_command = line_follow_m2_forward_to_cmd(m2_forward);
    s_state = drive_state;
    motor_set_speed(MOTOR_M1, s_m1_command);
    motor_set_speed(MOTOR_M2, s_m2_command);
}

/*
 * 当前基准速度。
 *
 * LINE_FOLLOW_SPEED_PROFILE_ENABLE = 0（当前）：
 *   返回 K2 当前选中的档位速度（100/150/200/250），实现匀速循迹。
 *
 * LINE_FOLLOW_SPEED_PROFILE_ENABLE = 1：
 *   - 未识别到第二弯：LINE_FOLLOW_R2_TURN2_START_SPEED_PERMILLE（250）；
 *   - 识别到第二弯后：3 s 内线性降到 100；
 *   - 之后保持 100。
 *
 * 函数在 10 ms 中断和主循环（LCD 显示）中都会被调用，
 * 只做整数运算，不访问硬件，执行时间可以忽略。
 */
static int16_t line_follow_get_base_speed(void)
{
    uint32_t elapsed_ms;
    int32_t speed;
    uint8_t gear = s_line_speed_gear;
    int16_t gear_speed;

    if (gear >= LINE_FOLLOW_SPEED_GEAR_COUNT) {
        gear = 0U;
    }
    gear_speed = s_line_speed_gears[gear];

    /* 匀速模式：忽略转弯状态，速度恒定为当前档位。 */
    if (LINE_FOLLOW_SPEED_PROFILE_ENABLE == 0U) {
        return gear_speed;
    }

    if (!s_r2_turn2_ramp_active) {
        return gear_speed;
    }

    elapsed_ms = (uint32_t)(system_time_get_tick_ms() - s_r2_turn2_ramp_start_ms);
    if (elapsed_ms >= LINE_FOLLOW_R2_TURN2_RAMP_MS) {
        return (int16_t)LINE_FOLLOW_R2_TURN2_SPEED_PERMILLE;
    }

    speed = (int32_t)gear_speed -
            (((int32_t)(gear_speed - LINE_FOLLOW_R2_TURN2_SPEED_PERMILLE) *
              (int32_t)elapsed_ms) /
             (int32_t)LINE_FOLLOW_R2_TURN2_RAMP_MS);

    return (int16_t)speed;
}

/* 每次 K1 启动/重新初始化时清空转弯计数和降速斜坡。 */
static void line_follow_reset_turn_profile(void)
{
    s_r2_turn_count = 0U;
    s_r2_turn_rearmed = true;
    s_r2_turn_confirm_count = 0U;
    s_r2_turn_rearm_active = false;
    s_r2_turn_rearm_tick = 0U;
    s_r2_turn2_ramp_active = false;
    s_r2_turn2_ramp_start_ms = 0U;
}

/*
 * 转弯识别（10 ms ISR 调用）。
 *
 * 逻辑：
 * 1. |error| >= ENTER_ERROR 连续 CONFIRM_CYCLES 个周期，记为一次转弯；
 * 2. 计到第 2 次时启动 3 s 降速斜坡；
 * 3. 每次计数后必须回到中心区 |error| <= REARM_ERROR 并保持 REARM_MS，
 *    才允许识别下一次，防止同一个弯被重复计数；
 * 4. 只有按键启动且本周期识别到黑线时才计数。
 *
 * error 的符号不参与计数：不管是左弯还是右弯都算一次。
 */
static void line_follow_update_turn_profile(int16_t error, bool has_line, uint32_t now_ms)
{
    int32_t magnitude;

    /*
     * 匀速模式下不统计转弯、也不启动降速斜坡。
     * 保留函数和参数，重新开启分段速度时无需改调用点。
     */
    if (LINE_FOLLOW_SPEED_PROFILE_ENABLE == 0U) {
        return;
    }

    /*
     * 必须已经离开起点横线（finish_armed）才开始计弯，
     * 避免起步姿态压在 A 点横线上时被误判成第一个弯。
     */
    if ((!s_r2_drive_enabled) || (!s_r2_finish_armed) ||
        (!has_line) || s_r2_turn2_ramp_active) {
        return;
    }

    magnitude = (error < 0) ? -(int32_t)error : (int32_t)error;

    /* 回中重新武装：离开弯道后要稳定在中心区一段时间。 */
    if (magnitude <= LINE_FOLLOW_TURN_REARM_ERROR) {
        if (!s_r2_turn_rearm_active) {
            s_r2_turn_rearm_active = true;
            s_r2_turn_rearm_tick = now_ms;
        } else if ((uint32_t)(now_ms - s_r2_turn_rearm_tick) >=
                   LINE_FOLLOW_TURN_REARM_MS) {
            s_r2_turn_rearmed = true;
        }
    } else {
        s_r2_turn_rearm_active = false;
    }

    if (magnitude >= LINE_FOLLOW_TURN_ENTER_ERROR) {
        if (s_r2_turn_confirm_count < 0xFFU) {
            s_r2_turn_confirm_count++;
        }

        if (s_r2_turn_rearmed &&
            (s_r2_turn_confirm_count >= LINE_FOLLOW_TURN_CONFIRM_CYCLES)) {
            s_r2_turn_confirm_count = 0U;
            s_r2_turn_rearmed = false;
            s_r2_turn_count++;

            /*
             * 第 2 个弯：记录起点，后续每个 10 ms 周期由
             * line_follow_get_base_speed() 计算 250 -> 100 的斜坡。
             */
            if (s_r2_turn_count == 2U) {
                s_r2_turn2_ramp_active = true;
                s_r2_turn2_ramp_start_ms = now_ms;
            }
        }
    } else {
        s_r2_turn_confirm_count = 0U;
    }
}

static void line_follow_apply_straight_forward(line_follow_state_t drive_state)
{
    int16_t base_speed = line_follow_get_base_speed();

    line_follow_apply_forward_speed(base_speed, base_speed, drive_state);
}

/*
 * 计算转向差速修正量（正数 = 向左修正，负数 = 向右修正）。
 *
 * 两段式比例：
 * 1. 基础段对所有误差生效，保证直道中心附近仍有连续修正；
 * 2. 急弯段只在 |error| > BOOST_START 时叠加，用来解决高速下外侧传感器
 *    已经压线、但基础比例不足以及时转车头的问题。
 *
 * 实现约束：
 * - 全部使用 int32 整数运算，避免在 10 ms 控制中断里引入浮点；
 * - error 由加权平均得到，最大约 ±50，不会溢出；
 * - 输出仍由后面的 clamp 限制在 ±650，不会超过电机允许范围。
 */
static int32_t line_follow_compute_correction(int16_t error)
{
    int32_t correction;
    int32_t magnitude;
    int32_t boost;

    correction = ((int32_t)error * LINE_FOLLOW_KP_NUM) / LINE_FOLLOW_KP_DEN;

    magnitude = (error < 0) ? -(int32_t)error : (int32_t)error;
    if (magnitude > LINE_FOLLOW_TURN_BOOST_START) {
        boost = ((magnitude - LINE_FOLLOW_TURN_BOOST_START) *
                 LINE_FOLLOW_TURN_BOOST_NUM) / LINE_FOLLOW_TURN_BOOST_DEN;
        correction += (error > 0) ? boost : -boost;
    }

    return correction;
}

static void line_follow_apply_drive_from_error(int16_t error,
                                               line_follow_state_t drive_state)
{
    int32_t correction;
    int16_t base_speed;
    int16_t m1_forward;
    int16_t m2_forward;

    correction = line_follow_compute_correction(error);
    base_speed = line_follow_get_base_speed();

    /*
     * 差速输出：左转时 M1 加速、M2 减速，右转反之。
     * 基准速度由第二弯降速逻辑动态给出（250 -> 100），
     * correction 仍按同样的转向增益叠加。
     */
    m1_forward = line_follow_clamp_i16((int32_t)base_speed + correction,
                                       LINE_FOLLOW_MIN_SPEED_PERMILLE,
                                       LINE_FOLLOW_MAX_SPEED_PERMILLE);
    m2_forward = line_follow_clamp_i16((int32_t)base_speed - correction,
                                       LINE_FOLLOW_MIN_SPEED_PERMILLE,
                                       LINE_FOLLOW_MAX_SPEED_PERMILLE);

    s_last_m1_forward = m1_forward;
    s_last_m2_forward = m2_forward;
    line_follow_apply_forward_speed(m1_forward, m2_forward, drive_state);
}

/*
 * 读取灰度状态；失败则再读一次，仍失败则返回 false。
 */
static bool line_follow_read_gray_status(gray_track_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    if (gray_track_read_status(snapshot)) {
        return true;
    }

    return gray_track_read_status(snapshot);
}

/*
 * 合并 gray_track_read_status() 结果到 dst，仅更新 0x00~0x04 对应字段。
 *
 * read_status 不填充 adc[]；若整结构体赋值会把栈上垃圾写入 G1~G8 显示区。
 */
static void line_follow_merge_gray_status(gray_track_snapshot_t *dst,
                                          const gray_track_snapshot_t *status)
{
    if ((dst == NULL) || (status == NULL)) {
        return;
    }

    dst->proto_version  = status->proto_version;
    dst->device_id      = status->device_id;
    dst->heartbeat      = status->heartbeat;
    dst->sample_counter = status->sample_counter;
    dst->compare_bits   = status->compare_bits;
    dst->online         = status->online;
    dst->result         = status->result;
}

/*
 * 从模块 0x20~0x2F 读取 8 路 12 位 ADC 原始值到 dst->adc[]。
 * 与 10 ms 状态快读分周期执行，单次 I2C 多读 16 字节。
 */
static bool line_follow_refresh_gray_adc(gray_track_snapshot_t *dst)
{
    if (dst == NULL) {
        return false;
    }

    return gray_track_read_adc(dst->adc);
}

void line_following_control_isr(void)
{
    gray_track_snapshot_t snapshot;
    int16_t error = 0;
    uint8_t active_count = 0U;
    bool has_line;
    bool read_ok;

    /*
     * 要求 2：只有按键启动后才允许驱动电机。
     * 等待启动/已结束/故障状态下保持停车，避免上电自动跑。
     */
    /*
     * 等待启动、结束制动由主循环控制；这里直接返回，不覆盖主循环设置的
     * 制动/停止输出。否则 50 ms 制动会被下一个 10 ms 中断强行清零。
     */
    if (!s_r2_drive_enabled) {
        return;
    }

    if (!line_follow_m1_m2_available()) {
        line_follow_stop_drive(LINE_FOLLOW_STATE_MOTOR_BLOCKED);
        return;
    }

    read_ok = line_follow_read_gray_status(&snapshot);
    s_gray_using_stale = false;
    if (read_ok) {
        line_follow_merge_gray_status(&s_last_valid_gray, &snapshot);
        s_have_valid_gray = true;
        s_gray_ok = true;
    } else if (s_have_valid_gray) {
        snapshot = s_last_valid_gray;
        s_gray_ok = true;
        s_gray_using_stale = true;
    } else {
        s_gray_ok = false;
        s_line_error = 0;
        s_active_bits = 0U;
        s_active_count = 0U;
        line_follow_apply_straight_forward(LINE_FOLLOW_STATE_BOOT);
        return;
    }

    line_follow_merge_gray_status(&s_gray_snapshot, &snapshot);
    s_active_bits = s_gray_snapshot.compare_bits;
    has_line = line_follow_compute_error(s_active_bits, &error, &active_count);
    s_active_count = active_count;

    /*
     * 转弯计数与第二弯降速：
     * 只在按键启动后、本周期识别到黑线时计数；
     * 识别到第 2 个弯后由 line_follow_get_base_speed() 接管 3 s 降速。
     */
    line_follow_update_turn_profile(error, has_line, system_time_get_tick_ms());

    /*
     * 每 100 ms 补读 ADC：G1~G8 为各路反射光 12 位原始值（0~4095），仅用于 LCD 观测。
     */
    s_gray_adc_counter++;
    if ((s_gray_adc_counter >= LINE_FOLLOW_GRAY_ADC_PERIOD) && read_ok) {
        s_gray_adc_counter = 0U;
        if (line_follow_refresh_gray_adc(&s_gray_snapshot)) {
            memcpy(s_last_valid_gray.adc,
                   s_gray_snapshot.adc,
                   sizeof(s_gray_snapshot.adc));
        }
    }

    /*
     * 要求 2 判定（10 ms 周期）：
     * 1. 起点脱离：连续 100 ms 压黑路数 <= 2，才允许识别 A 点；
     * 2. 终点：已武装、压黑路数 >= FINISH_SENSOR_MIN（当前 4 路），
     *    连续保持 FINISH_HOLD_MS（当前 50 ms）即确认，不再限制运行时间；
     * 3. 超时：>= 20 s；
     * 4. 丢线：连续 1 s 压黑路数为 0。
     * 命中后立即锁存成绩、关闭驱动，由主循环完成制动和显示。
     */
    if (s_r2_drive_enabled) {
        uint32_t now = system_time_get_tick_ms();
        uint32_t elapsed = now - s_r2_start_ms;

        if (!s_r2_finish_armed) {
            if (active_count <= LINE_FOLLOW_R2_START_CLEAR_MAX_SENSORS) {
                if (s_r2_start_clear_count == 0U) {
                    s_r2_start_clear_tick = now;
                }
                s_r2_start_clear_count++;
                if ((now - s_r2_start_clear_tick) >=
                    LINE_FOLLOW_R2_START_CLEAR_MS) {
                    s_r2_finish_armed = true;
                }
            } else {
                s_r2_start_clear_count = 0U;
            }
        }

        if (s_r2_finish_armed &&
            line_follow_is_finish_count(active_count)) {
            s_r2_finish_count++;
            if (s_r2_finish_count >= LINE_FOLLOW_R2_FINISH_CONFIRM_COUNT) {
                s_r2_final_ms = elapsed;
                s_r2_drive_enabled = false;
                s_r2_state = LINE_FOLLOW_R2_FINISHED;
                line_follow_stop_drive(LINE_FOLLOW_STATE_RUN);
                return;
            }
        } else {
            s_r2_finish_count = 0U;
        }

        if (elapsed >= LINE_FOLLOW_R2_TIMEOUT_MS) {
            s_r2_final_ms = elapsed;
            s_r2_drive_enabled = false;
            s_r2_state = LINE_FOLLOW_R2_TIMEOUT;
            line_follow_stop_drive(LINE_FOLLOW_STATE_RUN);
            return;
        }

        if (active_count == 0U) {
            if (s_r2_lost_count == 0U) {
                s_r2_lost_tick = now;
            }
            s_r2_lost_count++;
            if ((now - s_r2_lost_tick) >= LINE_FOLLOW_R2_LOST_STOP_MS) {
                s_r2_final_ms = elapsed;
                s_r2_drive_enabled = false;
                s_r2_state = LINE_FOLLOW_R2_LOST;
                line_follow_stop_drive(LINE_FOLLOW_STATE_RUN);
                return;
            }
        } else {
            s_r2_lost_count = 0U;
        }
    }

    if (has_line) {
        s_ever_had_line = true;
        s_line_error = error;
        s_last_line_error = error;
        line_follow_apply_drive_from_error(error, LINE_FOLLOW_STATE_RUN);
        return;
    }

    if (s_ever_had_line) {
        /*
         * 丢线：沿用上一次加权得到的左右轮逻辑速度，而不是切回 150 直行。
         */
        s_line_error = s_last_line_error;
        line_follow_apply_forward_speed(s_last_m1_forward,
                                        s_last_m2_forward,
                                        LINE_FOLLOW_STATE_LOST);
        return;
    }

    /*
     * 尚未出现过黑线加权：保持 150 直行，等待进入循迹。
     */
    s_line_error = 0;
    line_follow_apply_straight_forward(LINE_FOLLOW_STATE_BOOT);
}

static void line_follow_update_motor_status(void)
{
    /*
     * 主循环 50 ms 刷新电机电流/ADC，供 LCD 与串口显示。
     * 不在 10 ms 控制 ISR 内执行，避免拉长中断占用时间。
     */
    motor_refresh_adc();
    (void)motor_get_status(MOTOR_M1, &s_motor_m1);
    (void)motor_get_status(MOTOR_M2, &s_motor_m2);
}

static void line_follow_update_imu(void)
{
    s_imu_ready = IMU_isReady();
    if (s_imu_ready) {
        IMU_getYawPitchRoll(s_imu_ypr);
    } else {
        s_imu_ypr[0] = 0.0f;
        s_imu_ypr[1] = 0.0f;
        s_imu_ypr[2] = 0.0f;
    }
}

static void line_follow_lcd_invalidate(void)
{
    memset(s_lcd_cache, 0, sizeof(s_lcd_cache));
}

static void line_follow_lcd_clear(void)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
    line_follow_lcd_invalidate();
}

static void line_follow_lcd_line(uint8_t line, uint16_t color, const char *fmt, ...)
{
    char text[LINE_FOLLOW_LCD_TEXT_CHARS + 1U];
    char padded[LINE_FOLLOW_LCD_TEXT_CHARS + 1U];
    va_list args;
    uint16_t y;

    if ((line >= LINE_FOLLOW_LCD_MAX_LINES) || (fmt == NULL)) {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    memset(padded, ' ', LINE_FOLLOW_LCD_TEXT_CHARS);
    padded[LINE_FOLLOW_LCD_TEXT_CHARS] = '\0';
    (void)snprintf(padded,
                   sizeof(padded),
                   "%-*.*s",
                   (int)LINE_FOLLOW_LCD_TEXT_CHARS,
                   (int)LINE_FOLLOW_LCD_TEXT_CHARS,
                   text);

    if ((s_lcd_cache[line].valid != false) &&
        (s_lcd_cache[line].color == color) &&
        (strncmp(s_lcd_cache[line].text, padded, sizeof(padded)) == 0)) {
        return;
    }

    /*
     * 使用固定 35 字符宽度覆盖整行。
     * LCD_ShowString(mode=0) 会写入字符背景色，因此无需每行先 Fill，能减少刷新闪烁。
     */
    y = (uint16_t)(line * LINE_FOLLOW_LCD_LINE_HEIGHT);
    LCD_ShowString(0U, y, (const u8 *)padded, color, BLACK, 16U, 0U);
    memcpy(s_lcd_cache[line].text, padded, sizeof(padded));
    s_lcd_cache[line].color = color;
    s_lcd_cache[line].valid = true;
}

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE == 0U)
static int32_t line_follow_angle_to_centideg(float angle)
{
    if (angle >= 0.0f) {
        return (int32_t)(angle * 100.0f + 0.5f);
    }

    return (int32_t)(angle * 100.0f - 0.5f);
}
#endif

/*
 * 精简后的调试界面（只保留终点判定和速度相关的关键指标）。
 *
 * 显示布局：
 *   0: R2 状态 + 计时
 *   1: 压黑位图 Act、压黑路数 N、加权误差 Err
 *   2: Arm（已离开起点）/ OK（压黑路数达到阈值）/ F（终点确认计数）
 *   3: 灰度模块结果、是否使用旧数据、I2C 错误计数
 *   4: K1 状态 + K2 档位/速度
 *
 * 其他行不再绘制，初始化时 LCD 已经整屏清黑。
 */
static void line_follow_draw_lcd(void)
{
    uint16_t state_color = GREEN;
    const char *r2_text;
    uint32_t now;
    uint32_t r2_elapsed;
    bool finish_count_ok;

    if (s_state == LINE_FOLLOW_STATE_I2C_ERROR) {
        state_color = RED;
    } else if (s_gray_using_stale) {
        state_color = YELLOW;
    } else if ((s_state == LINE_FOLLOW_STATE_LOST) ||
               (s_state == LINE_FOLLOW_STATE_MOTOR_BLOCKED)) {
        state_color = YELLOW;
    }

    now = system_time_get_tick_ms();
    r2_text = line_follow_r2_state_text(s_r2_state);

    if (s_r2_state == LINE_FOLLOW_R2_RUN) {
        r2_elapsed = now - s_r2_start_ms;
    } else {
        r2_elapsed = s_r2_final_ms;
    }

    if ((s_r2_state == LINE_FOLLOW_R2_TIMEOUT) ||
        (s_r2_state == LINE_FOLLOW_R2_LOST)) {
        state_color = RED;
    } else if (s_r2_state == LINE_FOLLOW_R2_WAIT_KEY) {
        state_color = CYAN;
    }

    finish_count_ok = line_follow_is_finish_count(s_active_count);

    line_follow_lcd_line(0U, state_color,
                         "R2:%s T=%lu.%02lus",
                         r2_text,
                         (unsigned long)(r2_elapsed / 10000U),
                         (unsigned long)((r2_elapsed / 100U) % 100U));
    line_follow_lcd_line(1U, WHITE,
                         "Act:%02X N:%u Err:%+4d",
                         (unsigned)s_active_bits,
                         (unsigned)s_active_count,
                         (int)s_line_error);
    line_follow_lcd_line(2U, YELLOW,
                         "Arm:%u OK:%u F:%u",
                         s_r2_finish_armed ? 1U : 0U,
                         finish_count_ok ? 1U : 0U,
                         (unsigned)s_r2_finish_count);
    line_follow_lcd_line(3U, s_gray_ok ? GREEN : RED,
                         "Gray:%s St:%u E:%lu",
                         gray_track_result_text(s_gray_snapshot.result),
                         s_gray_using_stale ? 1U : 0U,
                         (unsigned long)gray_track_get_error_count());
    line_follow_lcd_line(4U, YELLOW,
                         "K1:%-6s K2:G%u/%u %d",
                         (s_r2_state == LINE_FOLLOW_R2_WAIT_KEY) ? "START" :
                         ((s_r2_state == LINE_FOLLOW_R2_RUN) ? "RUN" : "RETRY"),
                         (unsigned)(s_line_speed_gear + 1U),
                         (unsigned)LINE_FOLLOW_SPEED_GEAR_COUNT,
                         (int)line_follow_get_base_speed());
}

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE == 0U)
static void line_follow_print_angle(const char *label, float angle)
{
    int32_t value = line_follow_angle_to_centideg(angle);
    char sign = '+';

    if (value < 0) {
        sign = '-';
        value = -value;
    }

    printf("%s:%c%03ld.%02ld",
           label,
           sign,
           (long)(value / 100),
           (long)(value % 100));
}
#endif /* LINE_FOLLOW_VOFA_GRAY_ENABLE */

/*
 * VOFA+ FireWater 协议：把待发缓冲区的 G1~G8 数据经 UART0 发出。
 * 行内容已在 20 ms 节拍中断里格式化完毕，此处只做非阻塞优先级最高的发送。
 */
static void line_follow_vofa_try_send(void)
{
#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
    if (!s_vofa_send_pending) {
        return;
    }

    s_vofa_send_pending = false;
    (void)fputs(s_vofa_line, stdout);
#endif
}

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
/*
 * 20 ms 固定节拍回调（由 system_time 的 1 ms 中断每 20 ms 调用）。
 *
 * 职责：
 * - 关中断拷贝当前 G1~G8 ADC，避免与 10 ms 灰度 ISR 并发写 snapshot。
 * - 预格式化为 FireWater 一行文本，置位 s_vofa_send_pending 供主循环发送。
 *
 * 不在本函数内 printf：UART 阻塞会占用毫秒级时间，影响 1 ms/10 ms 中断精度。
 */
void line_following_vofa_20ms_tick(void)
{
    uint16_t adc_local[GRAY_TRACK_CHANNEL_COUNT];
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(adc_local, s_gray_snapshot.adc, sizeof(adc_local));
    if (primask == 0U) {
        __enable_irq();
    }

    (void)snprintf(s_vofa_line,
                   sizeof(s_vofa_line),
                   "gray:%u,%u,%u,%u,%u,%u,%u,%u\n",
                   (unsigned)adc_local[0],
                   (unsigned)adc_local[1],
                   (unsigned)adc_local[2],
                   (unsigned)adc_local[3],
                   (unsigned)adc_local[4],
                   (unsigned)adc_local[5],
                   (unsigned)adc_local[6],
                   (unsigned)adc_local[7]);
    s_vofa_send_pending = true;
}
#endif

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE == 0U)
static void line_follow_print_uart(void)
{
    printf("[LINE] state=%s gray=%s thr=%u active=0x%02X err=%d n=%u "
           "spd=%d gear=%u turn=%u ramp=%u arm=%u "
           "m1=%d m2=%d q1=%u q2=%u imu_ready=%u ",
           line_follow_state_text(s_state),
           gray_track_result_text(s_gray_snapshot.result),
           (unsigned)LINE_FOLLOW_GRAY_THRESHOLD,
           (unsigned)s_active_bits,
           (int)s_line_error,
           (unsigned)s_active_count,
           (int)line_follow_get_base_speed(),
           (unsigned)(s_line_speed_gear + 1U),
           (unsigned)s_r2_turn_count,
           s_r2_turn2_ramp_active ? 1U : 0U,
           s_r2_finish_armed ? 1U : 0U,
           (int)s_m1_command,
           (int)s_m2_command,
           (unsigned)motor_qei_get_count(MOTOR_M1),
           (unsigned)motor_qei_get_count(MOTOR_M2),
           s_imu_ready ? 1U : 0U);
    line_follow_print_angle("Y", s_imu_ypr[0]);
    printf(" ");
    line_follow_print_angle("P", s_imu_ypr[1]);
    printf(" ");
    line_follow_print_angle("R", s_imu_ypr[2]);
    printf(" errors=%lu\r\n", (unsigned long)gray_track_get_error_count());
}
#endif /* LINE_FOLLOW_VOFA_GRAY_ENABLE */

/* ===================== 要求 2：启动 / 制动 / 主循环状态机 ===================== */

/*
 * 按键启动一次测试：复位起点脱离、终点确认、丢线计数，记录计时起点，
 * 打开 ISR 驱动。车辆从 A 点横线上出发时先由 ISR 保持直行，等远端出现
 * 黑线后进入正常循迹。
 */
static void line_follow_r2_start(void)
{
    s_r2_start_ms = system_time_get_tick_ms();
    s_r2_final_ms = 0U;
    s_r2_finish_armed = false;
    s_r2_start_clear_count = 0U;
    s_r2_finish_count = 0U;
    s_r2_lost_count = 0U;
    s_r2_result_done = false;
    s_ever_had_line = false;
    s_line_error = 0;
    s_last_line_error = 0;
    s_last_m1_forward = line_follow_get_base_speed();
    s_last_m2_forward = line_follow_get_base_speed();
    s_r2_state = LINE_FOLLOW_R2_RUN;
    s_r2_drive_enabled = true;

    /* 每次启动都清零转弯计数和降速状态（匀速模式下只清状态，不改变速度）。 */
    line_follow_reset_turn_profile();
    line_follow_lcd_invalidate();
}

/*
 * 终点/超时/丢线后的制动停车。
 * 先按“逻辑后退”给两轮一个短时反向占空比，利用电机反电动势和轮胎摩擦
 * 快速消耗动能，再切到停止。制动时长需要结合实车质量和轮胎摩擦标定。
 */
static void line_follow_r2_brake_and_stop(void)
{
    int16_t brake_m1 = line_follow_m1_forward_to_cmd(-LINE_FOLLOW_R2_BRAKE_SPEED);
    int16_t brake_m2 = line_follow_m2_forward_to_cmd(-LINE_FOLLOW_R2_BRAKE_SPEED);

    motor_set_speed(MOTOR_M1, brake_m1);
    motor_set_speed(MOTOR_M2, brake_m2);
    delay_ms(LINE_FOLLOW_R2_BRAKE_MS);
    line_follow_stop_drive(LINE_FOLLOW_STATE_RUN);
}

/*
 * 要求 2 的主循环状态机：
 *   WAIT_KEY：停车等待 K1（KEY1，PA17），按下瞬间开始计时；
 *   RUN：ISR 负责循迹和终点/超时/丢线判定，主循环只做显示；
 *   FINISHED/TIMEOUT/LOST：锁存成绩，制动一次并在 LCD/串口显示；
 *   结束后再按一次 K1 即可重测。
 */
/*
 * K2：在停车/结束状态循环切换速度档位（100 -> 150 -> 200 -> 250）。
 *
 * 只改 s_line_speed_gear，不动电机；下一次 K1 启动时
 * line_follow_r2_start() 会用新档位初始化左右轮速度。
 */
static void line_follow_cycle_speed_gear(void)
{
    uint8_t gear = (uint8_t)((s_line_speed_gear + 1U) %
                             LINE_FOLLOW_SPEED_GEAR_COUNT);

    s_line_speed_gear = gear;

    printf("[LINE] speed gear=%u/%u base=%d\r\n",
           (unsigned)(gear + 1U),
           (unsigned)LINE_FOLLOW_SPEED_GEAR_COUNT,
           (int)line_follow_get_base_speed());
}

static void line_following_r2_task(void)
{
    switch (s_r2_state) {
    case LINE_FOLLOW_R2_WAIT_KEY:
        if (user_key_take_edge(USER_KEY_K1)) {
            line_follow_r2_start();
        } else if (user_key_take_edge(USER_KEY_K2)) {
            line_follow_cycle_speed_gear();
        }
        break;

    case LINE_FOLLOW_R2_RUN:
        /*
         * 终点/超时/丢线在 10 ms ISR 内判定并锁存，这里只负责显示。
         *
         * 运行期间不响应 K1，但必须把可能产生的按键边沿清掉：
         * 否则如果运行中按过 K1 或按键抖动产生了边沿，这个边沿会一直
         * 挂在 s_edge_pending 里；终点制动结束后 FINISHED 分支一读，
         * 就会立刻调用 line_follow_r2_start()，表现为“A 点停一下又继续跑”。
         */
        (void)user_key_take_edge(USER_KEY_K1);
        (void)user_key_take_edge(USER_KEY_K2);
        break;

    case LINE_FOLLOW_R2_FINISHED:
    case LINE_FOLLOW_R2_TIMEOUT:
    case LINE_FOLLOW_R2_LOST:
        if (!s_r2_result_done) {
            s_r2_result_done = true;
            line_follow_r2_brake_and_stop();
            printf("[R2] %s time=%lu.%02lus\r\n",
                   line_follow_r2_state_text(s_r2_state),
                   (unsigned long)(s_r2_final_ms / 10000U),
                   (unsigned long)((s_r2_final_ms / 100U) % 100U));
        }
        if (user_key_take_edge(USER_KEY_K1)) {
            line_follow_r2_start();
        } else if (user_key_take_edge(USER_KEY_K2)) {
            line_follow_cycle_speed_gear();
        }
        break;

    default:
        break;
    }
}

void line_following_app_init(void)
{
    uint32_t now;

    printf("\r\n[LINE] G3519 Car init start\r\n");

    LCD_Init();
    line_follow_lcd_clear();
    line_follow_lcd_line(0U, CYAN, "Line following init...");

    LED_init();
    /*
     * 灰度使用 8 路数字输出模块，OUT4/OUT3/OUT5 占用了 IMU 的 SPI1 引脚，
     * 因此这里不再初始化 IMU/SPI1，直接初始化数字灰度。
     */
    motor_init();
    motor_qei_init();
    gray_track_init();
    if (!gray_track_set_led_threshold(LINE_FOLLOW_GRAY_THRESHOLD)) {
        printf("[LINE] WARN: set gray threshold %u failed\r\n",
               (unsigned)LINE_FOLLOW_GRAY_THRESHOLD);
    } else {
        uint16_t readback = 0U;

        if (gray_track_get_led_threshold(&readback) &&
            (readback == LINE_FOLLOW_GRAY_THRESHOLD)) {
            printf("[LINE] gray threshold stored: %u\r\n",
                   (unsigned)readback);
        } else {
            printf("[LINE] WARN: gray threshold readback=%u expect=%u\r\n",
                   (unsigned)readback,
                   (unsigned)LINE_FOLLOW_GRAY_THRESHOLD);
        }
    }
    motor_stop_all();

    s_state = LINE_FOLLOW_STATE_BOOT;
    s_gray_adc_counter = 0U;
    s_ever_had_line = false;
    s_have_valid_gray = false;
    s_gray_using_stale = false;
    s_gray_ok = false;
    s_imu_ready = IMU_isReady();
    s_line_error = 0;
    s_last_line_error = 0;
    s_last_m1_forward = line_follow_get_base_speed();
    s_last_m2_forward = line_follow_get_base_speed();
    s_m1_command = 0;
    s_m2_command = 0;
    s_active_bits = 0U;
    s_active_count = 0U;
    memset(&s_gray_snapshot, 0, sizeof(s_gray_snapshot));
    memset(&s_last_valid_gray, 0, sizeof(s_last_valid_gray));
    memset(&s_motor_m1, 0, sizeof(s_motor_m1));
    memset(&s_motor_m2, 0, sizeof(s_motor_m2));
    memset(s_imu_ypr, 0, sizeof(s_imu_ypr));

    now = system_time_get_tick_ms();
    s_motor_status_tick = now - LINE_FOLLOW_MOTOR_STATUS_MS;
    s_lcd_tick = now - LINE_FOLLOW_LCD_MS;
    s_imu_tick = now - LINE_FOLLOW_IMU_MS;
    s_uart_tick = now - LINE_FOLLOW_UART_MS;
#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
    s_vofa_send_pending = false;
    s_vofa_line[0] = '\0';
#endif

    /*
     * 要求 2：初始化完成后不自动行驶，停车等待 K1（KEY1/PA17）。
     * 按键按下后由 line_following_r2_task() 启动计时和 ISR 驱动。
     */
    user_key_init();
    s_r2_state = LINE_FOLLOW_R2_WAIT_KEY;
    s_r2_drive_enabled = false;
    s_r2_result_done = true;
    s_r2_finish_armed = false;
    s_r2_start_ms = 0U;
    s_r2_final_ms = 0U;
    s_r2_start_clear_count = 0U;
    s_r2_finish_count = 0U;
    s_r2_lost_count = 0U;
    line_follow_reset_turn_profile();
    line_follow_stop_drive(LINE_FOLLOW_STATE_BOOT);
    line_follow_lcd_invalidate();

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
    printf("[LINE] VOFA FireWater fixed %u ms (TIMA1 tick)\r\n",
           (unsigned)LINE_FOLLOW_VOFA_GRAY_MS);
#else
    printf("[LINE] ctrl=10ms_isr thr=%u base=%d kp=%d/%d\r\n",
           (unsigned)LINE_FOLLOW_GRAY_THRESHOLD,
           (int)line_follow_get_base_speed(),
           LINE_FOLLOW_KP_NUM,
           LINE_FOLLOW_KP_DEN);
#endif
}

void line_following_app_task(void)
{
    /*
     * 要求 2 的按键启动、终点制动、结果显示在此完成；
     * 实时循迹仍由 10 ms ISR 负责，二者通过 s_r2_* 标志解耦。
     */
    line_following_r2_task();

    /*
     * 灰度采样与电机调速在 10 ms ISR 内完成；主循环负责 VOFA 发送、IMU/LCD 等。
     * VOFA 快照在 20 ms 硬件节拍里生成，主循环优先发送以降低 UART 阻塞带来的抖动。
     */
#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
    line_follow_vofa_try_send();
#endif

    if (system_time_elapsed_ms(&s_motor_status_tick, LINE_FOLLOW_MOTOR_STATUS_MS)) {
        line_follow_update_motor_status();
    }

    if (system_time_elapsed_ms(&s_imu_tick, LINE_FOLLOW_IMU_MS)) {
        line_follow_update_imu();
    }

    if (system_time_elapsed_ms(&s_lcd_tick, LINE_FOLLOW_LCD_MS)) {
        line_follow_draw_lcd();
    }

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE == 0U)
    if (system_time_elapsed_ms(&s_uart_tick, LINE_FOLLOW_UART_MS)) {
        line_follow_print_uart();
    }
#endif
}
