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
 * - 上电后先以 150 直行，直到首次检测到黑线加权；丢线后沿用上一次加权速度。
 * - 线偏 G1~G4（error>0）：M1 加速、M2 减速甚至反转；线偏 G5~G8（error<0）：M2 加速、M1 减速甚至反转。
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
 * 小车逻辑前进基准速度；差速修正后单轮可正转或反转。
 * 按用户要求把基准速度从 150 提高到 300（2 倍）；上限/下限保持 650，
 * 给差速修正留出余量。若 2 倍速度下弯道开始摆动或冲线，
 * 优先降低下面的 KP 或把基准速度回调到 220~260。
 */
#define LINE_FOLLOW_BASE_SPEED_PERMILLE     (300)
#define LINE_FOLLOW_MAX_SPEED_PERMILLE      (650)
#define LINE_FOLLOW_MIN_SPEED_PERMILLE      (-650)

/*
 * 比例增益：单点 G1 有效时 error 约 +50，单点 G8 有效时约 -50。
 */
#define LINE_FOLLOW_KP_NUM                  (12)
#define LINE_FOLLOW_KP_DEN                  (10)

#define LINE_FOLLOW_LCD_LINE_HEIGHT         (16U)
#define LINE_FOLLOW_LCD_MAX_LINES           (15U)
#define LINE_FOLLOW_LCD_TEXT_CHARS          (35U)

/* ===================== 要求 2：按键启动 + 计时 + A 点停车 ===================== */

/* 最短运行时间，防止刚起步还在 A 点横线上就误判终点。 */
#define LINE_FOLLOW_R2_MIN_RUN_MS               (8000U)

/* 题目要求整圈不超过 20 s，超过即本次测试失败并主动停车。 */
#define LINE_FOLLOW_R2_TIMEOUT_MS               (20000U)

/* 起点脱离：连续 100 ms 压黑路数不超过 2 路，才允许识别 A 点终点线。 */
#define LINE_FOLLOW_R2_START_CLEAR_MAX_SENSORS  (2U)
#define LINE_FOLLOW_R2_START_CLEAR_MS           (100U)

/* A 点 5 cm 横向启停线：同时压黑的路数明显多于普通循迹线，阈值取 4 路。 */
#define LINE_FOLLOW_R2_FINISH_SENSOR_MIN        (4U)

/* 连续 2 个 10 ms 控制周期满足终点条件才确认，滤除单周期噪声。 */
#define LINE_FOLLOW_R2_FINISH_CONFIRM_COUNT     (2U)

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

static void line_follow_apply_straight_forward(line_follow_state_t drive_state)
{
    line_follow_apply_forward_speed(LINE_FOLLOW_BASE_SPEED_PERMILLE,
                                    LINE_FOLLOW_BASE_SPEED_PERMILLE,
                                    drive_state);
}

static void line_follow_apply_drive_from_error(int16_t error,
                                               line_follow_state_t drive_state)
{
    int32_t correction;
    int16_t m1_forward;
    int16_t m2_forward;

    correction = ((int32_t)error * LINE_FOLLOW_KP_NUM) / LINE_FOLLOW_KP_DEN;
    m1_forward = line_follow_clamp_i16((int32_t)LINE_FOLLOW_BASE_SPEED_PERMILLE + correction,
                                       LINE_FOLLOW_MIN_SPEED_PERMILLE,
                                       LINE_FOLLOW_MAX_SPEED_PERMILLE);
    m2_forward = line_follow_clamp_i16((int32_t)LINE_FOLLOW_BASE_SPEED_PERMILLE - correction,
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
     * 2. 终点：已武装、已跑 >= 8 s、连续 2 周期压黑路数 >= 4；
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
            (elapsed >= LINE_FOLLOW_R2_MIN_RUN_MS) &&
            (active_count >= LINE_FOLLOW_R2_FINISH_SENSOR_MIN)) {
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

static int32_t line_follow_angle_to_centideg(float angle)
{
    if (angle >= 0.0f) {
        return (int32_t)(angle * 100.0f + 0.5f);
    }

    return (int32_t)(angle * 100.0f - 0.5f);
}

static void line_follow_lcd_angle(uint8_t line,
                                  const char *label_a,
                                  float angle_a,
                                  const char *label_b,
                                  float angle_b)
{
    int32_t a = line_follow_angle_to_centideg(angle_a);
    int32_t b = line_follow_angle_to_centideg(angle_b);
    char sign_a = '+';
    char sign_b = '+';

    if (a < 0) {
        sign_a = '-';
        a = -a;
    }
    if (b < 0) {
        sign_b = '-';
        b = -b;
    }

    line_follow_lcd_line(line,
                         WHITE,
                         "%s:%c%03ld.%02ld %s:%c%03ld.%02ld",
                         label_a,
                         sign_a,
                         (long)(a / 100),
                         (long)(a % 100),
                         label_b,
                         sign_b,
                         (long)(b / 100),
                         (long)(b % 100));
}

static void line_follow_lcd_roll_line(uint8_t line, float roll, bool imu_ready)
{
    int32_t value = line_follow_angle_to_centideg(roll);
    char sign = '+';

    if (value < 0) {
        sign = '-';
        value = -value;
    }

    line_follow_lcd_line(line,
                         WHITE,
                         "Rol:%c%03ld.%02ld IMU:%u",
                         sign,
                         (long)(value / 100),
                         (long)(value % 100),
                         imu_ready ? 1U : 0U);
}

static void line_follow_draw_lcd(void)
{
    uint16_t state_color = GREEN;

    /* G1~G8 为 0x20 ADC 原始值，ISR 每 100 ms 刷新；主循环只读快照。 */

    if (s_state == LINE_FOLLOW_STATE_I2C_ERROR) {
        state_color = RED;
    } else if (s_gray_using_stale) {
        state_color = YELLOW;
    } else if ((s_state == LINE_FOLLOW_STATE_LOST) ||
               (s_state == LINE_FOLLOW_STATE_MOTOR_BLOCKED)) {
        state_color = YELLOW;
    }

    /*
     * 第 0 行显示要求 2 的状态和计时：
     * WAIT 等待 K1；RUN 显示实时时间；FINISH/TIMEOUT/LOST 显示锁存成绩。
     */
    {
        const char *r2_text = line_follow_r2_state_text(s_r2_state);
        uint32_t r2_elapsed;

        if (s_r2_state == LINE_FOLLOW_R2_RUN) {
            r2_elapsed = system_time_get_tick_ms() - s_r2_start_ms;
        } else {
            r2_elapsed = s_r2_final_ms;
        }

        if ((s_r2_state == LINE_FOLLOW_R2_TIMEOUT) ||
            (s_r2_state == LINE_FOLLOW_R2_LOST)) {
            state_color = RED;
        } else if (s_r2_state == LINE_FOLLOW_R2_WAIT_KEY) {
            state_color = CYAN;
        }

        line_follow_lcd_line(0U, state_color,
                             "R2:%s T=%lu.%02lus",
                             r2_text,
                             (unsigned long)(r2_elapsed / 10000U),
                             (unsigned long)((r2_elapsed / 100U) % 100U));
    }
    line_follow_lcd_line(1U,
                         s_gray_ok ? GREEN : RED,
                         "Gray:%s Hb:%03u St:%u",
                         gray_track_result_text(s_gray_snapshot.result),
                         (unsigned)s_gray_snapshot.heartbeat,
                         s_gray_using_stale ? 1U : 0U);
    line_follow_lcd_line(2U,
                         WHITE,
                         "Thr:%03u Act:%02X Err:%+4d N:%u",
                         (unsigned)LINE_FOLLOW_GRAY_THRESHOLD,
                         (unsigned)s_active_bits,
                         (int)s_line_error,
                         (unsigned)s_active_count);
    line_follow_lcd_line(3U,
                         WHITE,
                         "G1:%04u G2:%04u G3:%04u",
                         (unsigned)s_gray_snapshot.adc[0],
                         (unsigned)s_gray_snapshot.adc[1],
                         (unsigned)s_gray_snapshot.adc[2]);
    line_follow_lcd_line(4U,
                         WHITE,
                         "G4:%04u G5:%04u G6:%04u",
                         (unsigned)s_gray_snapshot.adc[3],
                         (unsigned)s_gray_snapshot.adc[4],
                         (unsigned)s_gray_snapshot.adc[5]);
    line_follow_lcd_line(5U,
                         WHITE,
                         "G7:%04u G8:%04u Last:%+4d",
                         (unsigned)s_gray_snapshot.adc[6],
                         (unsigned)s_gray_snapshot.adc[7],
                         (int)s_last_line_error);
    line_follow_lcd_line(6U,
                         GREEN,
                         "M1 Cmd:%+4d Raw:%04u mA:%04u",
                         (int)s_m1_command,
                         (unsigned)s_motor_m1.adc_raw,
                         (unsigned)s_motor_m1.current_ma);
    line_follow_lcd_line(7U,
                         GREEN,
                         "M2 Cmd:%+4d Raw:%04u mA:%04u",
                         (int)s_m2_command,
                         (unsigned)s_motor_m2.adc_raw,
                         (unsigned)s_motor_m2.current_ma);
    line_follow_lcd_line(8U,
                         WHITE,
                         "Q1:%c%05u Q2:%c%05u",
                         motor_qei_get_direction_up(MOTOR_M1) ? 'U' : 'D',
                         (unsigned)motor_qei_get_count(MOTOR_M1),
                         motor_qei_get_direction_up(MOTOR_M2) ? 'U' : 'D',
                         (unsigned)motor_qei_get_count(MOTOR_M2));
    line_follow_lcd_angle(9U, "Yaw", s_imu_ypr[0], "Pit", s_imu_ypr[1]);
    line_follow_lcd_roll_line(10U, s_imu_ypr[2], s_imu_ready);
    line_follow_lcd_line(11U,
                         state_color,
                         "State:%s ErrCnt:%lu",
                         line_follow_state_text(s_state),
                         (unsigned long)gray_track_get_error_count());
    line_follow_lcd_line(12U,
                         GRAY,
                         "M3/M4 off Base:%d Kp:%d/%d",
                         LINE_FOLLOW_BASE_SPEED_PERMILLE,
                         LINE_FOLLOW_KP_NUM,
                         LINE_FOLLOW_KP_DEN);
    line_follow_lcd_line(13U,
                         YELLOW,
                         (s_r2_state == LINE_FOLLOW_R2_WAIT_KEY) ? "K1: START" :
                         ((s_r2_state == LINE_FOLLOW_R2_RUN) ? "K1: RUNNING" :
                          "K1: RETRY"));
}

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

static void line_follow_print_uart(void)
{
    printf("[LINE] state=%s gray=%s thr=%u active=0x%02X err=%d n=%u "
           "m1=%d m2=%d q1=%u q2=%u imu_ready=%u ",
           line_follow_state_text(s_state),
           gray_track_result_text(s_gray_snapshot.result),
           (unsigned)LINE_FOLLOW_GRAY_THRESHOLD,
           (unsigned)s_active_bits,
           (int)s_line_error,
           (unsigned)s_active_count,
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
    s_last_m1_forward = LINE_FOLLOW_BASE_SPEED_PERMILLE;
    s_last_m2_forward = LINE_FOLLOW_BASE_SPEED_PERMILLE;
    s_r2_state = LINE_FOLLOW_R2_RUN;
    s_r2_drive_enabled = true;
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
static void line_following_r2_task(void)
{
    switch (s_r2_state) {
    case LINE_FOLLOW_R2_WAIT_KEY:
        if (user_key_take_edge(USER_KEY_K1)) {
            line_follow_r2_start();
        }
        break;

    case LINE_FOLLOW_R2_RUN:
        /* 终点/超时/丢线在 10 ms ISR 内判定并锁存，这里只负责显示。 */
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
    s_last_m1_forward = LINE_FOLLOW_BASE_SPEED_PERMILLE;
    s_last_m2_forward = LINE_FOLLOW_BASE_SPEED_PERMILLE;
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
    line_follow_stop_drive(LINE_FOLLOW_STATE_BOOT);
    line_follow_lcd_invalidate();

#if (LINE_FOLLOW_VOFA_GRAY_ENABLE != 0U)
    printf("[LINE] VOFA FireWater fixed %u ms (TIMA1 tick)\r\n",
           (unsigned)LINE_FOLLOW_VOFA_GRAY_MS);
#else
    printf("[LINE] ctrl=10ms_isr thr=%u base=%d kp=%d/%d\r\n",
           (unsigned)LINE_FOLLOW_GRAY_THRESHOLD,
           LINE_FOLLOW_BASE_SPEED_PERMILLE,
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
