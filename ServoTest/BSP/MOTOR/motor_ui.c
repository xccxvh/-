#include "MOTOR/motor_ui.h"
#include "MOTOR/motor_qei.h"
#include "LED/led.h"
#include "SPI0_LCD/lcd.h"
#include "SPI0_LCD/lcd_init.h"
#include <string.h>

/*
 * LCD 局部刷新：固定坐标 + 缓存上一帧字符串/数值，仅变化字段重绘。
 *
 * 全屏清底仅发生在 motor_ui_init() 首次，或主动调用 motor_ui_request_full_redraw()。
 * 按 K1/K2 换测试步时只清空相关字段缓存，下一帧局部重绘，不再整屏刷新。
 *
 * 平时 20Hz 刷新只更新电机速度、ADC、QEI、使能和故障等变动字段，显著减少 SPI 写入量。
 */

#define UI_FONT_SIZE     (16U)
#define UI_FONT_WIDTH    (8U)

#define UI_Y_TITLE       (4U)
#define UI_Y_STEP        (22U)
#define UI_Y_MOTOR_HDR   (42U)
#define UI_Y_M1          (60U)
#define UI_Y_M2          (78U)
#define UI_Y_M3          (96U)
#define UI_Y_M4          (114U)
#define UI_Y_STATUS      (138U)
#define UI_Y_KEYS        (160U)

#define UI_X_LABEL       (4U)
#define UI_X_SPD         (24U)
#define UI_X_ADC         (84U)
#define UI_X_QEI         (136U)
#define UI_X_EN          (208U)
#define UI_X_FL          (232U)

#define UI_FIELD_MAX     (8U)
#define UI_SPD_CHARS     (5U)
#define UI_ADC_CHARS     (4U)
#define UI_QEI_CHARS     (6U)
#define UI_FLAG_CHARS    (1U)

static const char *const s_step_title[MOTOR_UI_STEP_COUNT] = {
    "0 STOP ALL",
    "1 M1 FWD 40%",
    "2 M1 REV 40%",
    "3 M2 FWD 40%",
    "4 M2 REV 40%",
    "5 M3 FWD 40%",
    "6 M3 REV 40%",
    "7 M4 FWD 40%",
    "8 M4 REV 40%",
    "9 STATUS ONLY",
    "10 ALL FWD 25%",
};

typedef struct {
    char spd[UI_FIELD_MAX];
    char adc[UI_FIELD_MAX];
    char qei[UI_FIELD_MAX];
    char en[UI_FIELD_MAX];
    char fl[UI_FIELD_MAX];
} motor_ui_motor_cache_t;

typedef struct {
    motor_ui_motor_cache_t motor[MOTOR_COUNT];
    char step[36];
    char status[36];
    uint8_t status_code;
} motor_ui_cache_t;

static motor_ui_step_t s_step = MOTOR_UI_STEP_STOP;
static bool s_prev_any_fault;
static bool s_force_full_redraw = true;
static motor_ui_cache_t s_cache;

static uint16_t motor_ui_row_y(motor_id_t id)
{
    switch (id) {
    case MOTOR_M1: return UI_Y_M1;
    case MOTOR_M2: return UI_Y_M2;
    case MOTOR_M3: return UI_Y_M3;
    default:       return UI_Y_M4;
    }
}

static void motor_ui_text(uint16_t x, uint16_t y, const char *text, uint16_t color)
{
    LCD_ShowString(x, y, (const u8 *)text, color, BLACK, UI_FONT_SIZE, 0);
}

static void motor_ui_field_width(uint16_t x,
                                 uint16_t y,
                                 const char *text,
                                 char *cache,
                                 size_t cache_size,
                                 uint16_t color,
                                 uint8_t width_chars)
{
    uint16_t x_end;

    if ((text == NULL) || (cache == NULL) || (cache_size == 0U)) {
        return;
    }

    if (strncmp(cache, text, cache_size) != 0) {
        /*
         * 字段局部刷新前先清理自己的固定宽度矩形。
         * 各列宽度按真实显示字符数传入，避免 ADC 列清理区域覆盖到 QEI 列；
         * 之前 QEI 前缀 U/D 偶尔显示半个，就是相邻列局部清理区域重叠造成的。
         */
        x_end = (uint16_t)(x + ((uint16_t)width_chars * UI_FONT_WIDTH));
        if (x_end > LCD_W) {
            x_end = LCD_W;
        }
        LCD_Fill(x, y, x_end, (uint16_t)(y + UI_FONT_SIZE), BLACK);
        strncpy(cache, text, cache_size - 1U);
        cache[cache_size - 1U] = '\0';
        motor_ui_text(x, y, text, color);
    }
}

static void motor_ui_field(uint16_t x, uint16_t y, const char *text,
                           char *cache, size_t cache_size, uint16_t color)
{
    motor_ui_field_width(x, y, text, cache, cache_size, color,
                         (uint8_t)(cache_size - 1U));
}

static void motor_ui_clear_panel(void)
{
    /*
     * 进入电机测试页时执行一次整屏清理。
     * 电机页随后按字段局部刷新；这里清全屏是为了去掉其它测试页留下的色块、
     * AT 应答等非电机内容，避免用户误判为电机页面信息。
     */
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
}

static void motor_ui_invalidate_cache(void)
{
    memset(&s_cache, 0, sizeof(s_cache));
}

/*
 * 测试步切换后电机命令列（SPD/EN 等）会变，仅作废这些字段的缓存；
 * 不重画表头/标题，避免按键时整屏闪烁。
 */
static void motor_ui_invalidate_after_step_change(void)
{
    uint32_t i;

    s_cache.step[0] = '\0';

    for (i = 0U; i < MOTOR_COUNT; i++) {
        s_cache.motor[i].spd[0] = '\0';
        s_cache.motor[i].en[0] = '\0';
    }
}

void motor_ui_request_full_redraw(void)
{
    s_force_full_redraw = true;
}

static void motor_ui_draw_static_frame(void)
{
    motor_ui_clear_panel();

    motor_ui_text(UI_X_LABEL, UI_Y_TITLE, "G3519 Car", WHITE);
    motor_ui_text(UI_X_LABEL, UI_Y_MOTOR_HDR, "  ", WHITE);
    motor_ui_text(UI_X_SPD, UI_Y_MOTOR_HDR, "SPD", CYAN);
    motor_ui_text(UI_X_ADC, UI_Y_MOTOR_HDR, "ADC", CYAN);
    motor_ui_text(UI_X_QEI, UI_Y_MOTOR_HDR, "QEI", CYAN);
    motor_ui_text(UI_X_EN, UI_Y_MOTOR_HDR, "E", CYAN);
    motor_ui_text(UI_X_FL, UI_Y_MOTOR_HDR, "F", CYAN);

    motor_ui_text(UI_X_LABEL, UI_Y_M1, "M1", GREEN);
    motor_ui_text(UI_X_LABEL, UI_Y_M2, "M2", GREEN);
    motor_ui_text(UI_X_LABEL, UI_Y_M3, "M3", GREEN);
    motor_ui_text(UI_X_LABEL, UI_Y_M4, "M4", GREEN);

    motor_ui_text(UI_X_LABEL, UI_Y_KEYS, "K1:page  K2:step", CYAN);

}

static void motor_ui_update_step_line(void)
{
    char step_line[36];

    snprintf(step_line, sizeof(step_line), "ST:%s", s_step_title[s_step]);
    motor_ui_field(UI_X_LABEL, UI_Y_STEP, step_line,
                   s_cache.step, sizeof(s_cache.step), YELLOW);
}

static void motor_ui_build_motor_fields(motor_id_t id, motor_status_t *st,
                                        char *spd, char *adc, char *qei,
                                        char *en, char *fl, uint16_t *fl_color)
{
    snprintf(spd, UI_FIELD_MAX, "%+4d", (int)st->speed_permille);
    snprintf(adc, UI_FIELD_MAX, "%04u", (unsigned)st->adc_raw);
    snprintf(en, UI_FIELD_MAX, "%u", st->enabled ? 1U : 0U);
    snprintf(fl, UI_FIELD_MAX, "%u", st->nfault_high ? 1U : 0U);
    *fl_color = st->nfault_high ? GREEN : RED;

    if ((id == MOTOR_M1) || (id == MOTOR_M2)) {
        char qdir = motor_qei_get_direction_up(id) ? 'U' : 'D';

        snprintf(qei, UI_FIELD_MAX, "%c%05u", qdir,
                 (unsigned)motor_qei_get_count(id));
    } else {
        snprintf(qei, UI_FIELD_MAX, "  ---");
    }
}

static void motor_ui_update_motor_row(motor_id_t id)
{
    motor_status_t st;
    motor_ui_motor_cache_t *row = &s_cache.motor[id];
    uint16_t y = motor_ui_row_y(id);
    char spd[UI_FIELD_MAX];
    char adc[UI_FIELD_MAX];
    char qei[UI_FIELD_MAX];
    char en[UI_FIELD_MAX];
    char fl[UI_FIELD_MAX];
    uint16_t fl_color;

    if (!motor_get_status(id, &st)) {
        return;
    }

    motor_ui_build_motor_fields(id, &st, spd, adc, qei, en, fl, &fl_color);

    motor_ui_field_width(UI_X_SPD, y, spd,
                         row->spd, sizeof(row->spd), GREEN, UI_SPD_CHARS);
    motor_ui_field_width(UI_X_ADC, y, adc,
                         row->adc, sizeof(row->adc), GREEN, UI_ADC_CHARS);
    motor_ui_field_width(UI_X_QEI, y, qei,
                         row->qei, sizeof(row->qei), GREEN, UI_QEI_CHARS);
    motor_ui_field_width(UI_X_EN, y, en,
                         row->en, sizeof(row->en), GREEN, UI_FLAG_CHARS);
    motor_ui_field_width(UI_X_FL, y, fl,
                         row->fl, sizeof(row->fl), fl_color, UI_FLAG_CHARS);
}

static uint8_t motor_ui_status_code(bool any_fault, bool any_sleep, bool any_run)
{
    if (any_fault) {
        return 3U;
    }
    if (any_sleep) {
        return 2U;
    }
    if (any_run) {
        return 1U;
    }
    return 0U;
}

static void motor_ui_update_status(uint8_t code)
{
    const char *text;
    uint16_t color;

    switch (code) {
    case 3U:
        text = "FAULT: nFAULT=0";
        color = RED;
        break;
    case 2U:
        text = "SLEEP: nSLEEP=0";
        color = YELLOW;
        break;
    case 1U:
        text = "RUN   LED1 on";
        color = GREEN;
        break;
    default:
        text = "IDLE";
        color = GRAY;
        break;
    }

    if ((code != s_cache.status_code) || (s_cache.status[0] == '\0')) {
        motor_ui_field(UI_X_LABEL, UI_Y_STATUS, text,
                       s_cache.status, sizeof(s_cache.status), color);
        s_cache.status_code = code;
    }
}

static void motor_ui_update_leds(bool any_fault, bool any_run)
{
    if (any_fault) {
        LED1(0);
        LED2(1);
    } else if (any_run) {
        LED1(1);
        LED2(0);
    } else {
        LED1(0);
        LED2(0);
    }
}

void motor_ui_init(void)
{
    s_step = MOTOR_UI_STEP_STOP;

    LED_init();
    LED1(0);
    LED2(0);

    motor_qei_init();
    motor_ui_request_full_redraw();
    motor_ui_apply_step(MOTOR_UI_STEP_STOP);
    motor_ui_refresh(NULL, false);
}

void motor_ui_apply_step(motor_ui_step_t step)
{
    if (step >= MOTOR_UI_STEP_COUNT) {
        step = MOTOR_UI_STEP_STOP;
    }

    s_step = step;
    motor_stop_all();

    switch (step) {
    case MOTOR_UI_STEP_M1_FWD:
        motor_set_speed(MOTOR_M1, 400);
        break;
    case MOTOR_UI_STEP_M1_REV:
        motor_set_speed(MOTOR_M1, -400);
        break;
    case MOTOR_UI_STEP_M2_FWD:
        motor_set_speed(MOTOR_M2, 400);
        break;
    case MOTOR_UI_STEP_M2_REV:
        motor_set_speed(MOTOR_M2, -400);
        break;
    case MOTOR_UI_STEP_M3_FWD:
        motor_set_speed(MOTOR_M3, 400);
        break;
    case MOTOR_UI_STEP_M3_REV:
        motor_set_speed(MOTOR_M3, -400);
        break;
    case MOTOR_UI_STEP_M4_FWD:
        motor_set_speed(MOTOR_M4, 400);
        break;
    case MOTOR_UI_STEP_M4_REV:
        motor_set_speed(MOTOR_M4, -400);
        break;
    case MOTOR_UI_STEP_ALL_FWD_LOW:
        motor_set_speed(MOTOR_M1, 250);
        motor_set_speed(MOTOR_M2, 250);
        motor_set_speed(MOTOR_M3, 250);
        motor_set_speed(MOTOR_M4, 250);
        break;
    case MOTOR_UI_STEP_STATUS_ONLY:
    case MOTOR_UI_STEP_STOP:
    default:
        break;
    }

    motor_ui_invalidate_after_step_change();
}

void motor_ui_next_step(void)
{
    motor_ui_step_t next = (motor_ui_step_t)((uint32_t)s_step + 1U);

    if (next >= MOTOR_UI_STEP_COUNT) {
        next = MOTOR_UI_STEP_STOP;
    }

    motor_ui_apply_step(next);
}

void motor_ui_emergency_stop(void)
{
    motor_ui_apply_step(MOTOR_UI_STEP_STOP);
}

motor_ui_step_t motor_ui_get_step(void)
{
    return s_step;
}

void motor_ui_refresh(const float ypr[3], bool imu_ready)
{
    bool any_fault;
    bool any_sleep;
    bool any_run;
    uint8_t status_code;
    uint32_t i;

    motor_refresh_adc();

    any_fault = motor_any_fault();
    any_sleep = motor_any_asleep();
    any_run   = motor_any_running();
    status_code = motor_ui_status_code(any_fault, any_sleep, any_run);

    if (any_fault && !s_prev_any_fault) {
        printf("Motor nFAULT active!\r\n");
    }
    s_prev_any_fault = any_fault;

    if (s_force_full_redraw) {
        motor_ui_invalidate_cache();
        motor_ui_draw_static_frame();
        s_force_full_redraw = false;
    }

    motor_ui_update_step_line();

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_ui_update_motor_row((motor_id_t)i);
    }

    /*
     * 电机页只展示电机、ADC 和 QEI 信息；IMU 有独立测试页。
     * 保留 ypr/imu_ready 参数是为了兼容既有接口，当前页面不消费它们。
     */
    (void)ypr;
    (void)imu_ready;
    motor_ui_update_status(status_code);
    motor_ui_update_leds(any_fault, any_run);
}
