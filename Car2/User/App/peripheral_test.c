#include "App/peripheral_test.h"
#include <stdarg.h>

/*
 * 全外设测试调度层。
 *
 * 文件职责：
 * - 统一初始化当前工程已经接入的外设，并记录初始化/在线状态。
 * - 通过 LCD 和 UART0 输出当前页面状态。
 * - 通过 K1/K2 把每个外设的测试动作拆成互斥页面，避免电机、舵机、蜂鸣器、
 *   RGB 等输出设备同时动作造成供电或总线干扰。
 *
 * 重要时序：
 * - LCD/SPI0、IMU/SPI1、UART、CAN、PWM/QEI/ADC 等底层配置均来自 SysConfig。
 * - 本模块不创建新的中断入口；所有 ISR 仍在 BSP 模块中实现，应用层只消费状态。
 */

#define APP_TEST_LCD_LINE_HEIGHT       (16U)
#define APP_TEST_LCD_MAX_LINES         (15U)
#define APP_TEST_LCD_TEXT_CHARS        (35U)
/* LCD 调度周期固定为 50ms，即 20Hz；脏行缓存会避免无变化内容重复写屏。 */
#define APP_TEST_LCD_REFRESH_MS        (50U)
#define APP_TEST_UART_REFRESH_MS       (1000U)
#define APP_TEST_MOTOR_REFRESH_MS      (50U)
#define APP_TEST_IMU_REFRESH_MS        (50U)
#define APP_TEST_STS_REFRESH_MS        (250U)
#define APP_TEST_CAN_TX_MS             (1000U)

#define APP_TEST_STS_ID                (1U)
#define APP_TEST_STS_SPEED             (1000U)
#define APP_TEST_STS_POS_LOW           (1536U)
#define APP_TEST_STS_POS_HIGH          (2560U)
#define APP_TEST_CAN_ID                (CAN_BUS_TEST_TX_ID)
#define APP_TEST_RGB_BLOCK_X0          (176U)
#define APP_TEST_RGB_BLOCK_Y0          (176U)
#define APP_TEST_RGB_BLOCK_X1          (272U)
#define APP_TEST_RGB_BLOCK_Y1          (232U)
#define APP_TEST_UART_AT_TIMEOUT_MS    (1200U)
#define APP_TEST_UART_RX_BUFFER_SIZE   (192U)

typedef enum {
    APP_PAGE_OVERVIEW = 0,
    APP_PAGE_RGB_BUZZER,
    APP_PAGE_MOTOR,
    APP_PAGE_IMU,
    APP_PAGE_STS3032,
    APP_PAGE_CAN,
    APP_PAGE_UART_MODULES,
    APP_PAGE_COUNT
} app_test_page_t;

typedef enum {
    APP_STATUS_NOT_TESTED = 0,
    APP_STATUS_OK,
    APP_STATUS_FAIL
} app_status_t;

typedef struct {
    app_status_t lcd;
    app_status_t timer;
    app_status_t key;
    app_status_t led;
    app_status_t rgb;
    app_status_t buzzer;
    app_status_t motor;
    app_status_t qei;
    app_status_t adc_vref;
    app_status_t imu;
    app_status_t wifi;
    app_status_t bluetooth;
    app_status_t sts3032;
    app_status_t can;
} app_init_status_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t tone_hz;
    const char *name;
} app_rgb_profile_t;

static const app_rgb_profile_t s_rgb_profiles[] = {
    {255U, 0U,   0U,   1000U, "RED"},
    {0U,   255U, 0U,   1500U, "GREEN"},
    {0U,   0U,   255U, 2000U, "BLUE"},
    {255U, 255U, 0U,   2500U, "YELLOW"},
    {0U,   255U, 255U, 3000U, "CYAN"},
    {255U, 0U,   255U, 3500U, "MAGENTA"},
    {0U,   0U,   0U,   0U,    "OFF"},
};

static app_init_status_t s_init_status;
static app_test_page_t s_page = APP_PAGE_OVERVIEW;
static uint32_t s_lcd_tick = 0U;
static uint32_t s_uart_tick = 0U;
static uint32_t s_motor_tick = 0U;
static uint32_t s_imu_tick = 0U;
static uint32_t s_sts_tick = 0U;
static uint32_t s_can_tx_tick = 0U;
static uint16_t s_can_sequence = 0U;
static uint8_t s_rgb_index = 0U;
static uint8_t s_uart_test_selector = 0U;
static uint16_t s_sts_target = APP_TEST_STS_POS_HIGH;
static uint16_t s_sts_position = 0U;
static bool s_sts_position_valid = false;
static bool s_sts_torque_on = false;
static bool s_force_lcd_redraw = true;
static bool s_lcd_frame_active = false;
static bool s_lcd_clear_untouched_on_next_frame = false;
static bool s_print_page_header = true;
static float s_imu_ypr[3] = {0.0f, 0.0f, 0.0f};
static uint8_t s_imu_whoami = 0U;
static int s_imu_probe_rc = -1;
static char s_lcd_line_cache[APP_TEST_LCD_MAX_LINES][APP_TEST_LCD_TEXT_CHARS + 1U];
static uint16_t s_lcd_color_cache[APP_TEST_LCD_MAX_LINES];
static bool s_lcd_line_valid[APP_TEST_LCD_MAX_LINES];
static bool s_lcd_line_touched[APP_TEST_LCD_MAX_LINES];
static bool s_rgb_block_valid = false;
static uint8_t s_rgb_block_r = 0U;
static uint8_t s_rgb_block_g = 0U;
static uint8_t s_rgb_block_b = 0U;
static char s_uart_last_module[8] = "NONE";
static char s_uart_last_cmd[16] = "-";
static char s_uart_last_rx[APP_TEST_LCD_TEXT_CHARS + 1U] = "Press K2 to test";
static uint32_t s_uart_last_rx_len = 0U;
static bool s_uart_last_ok = false;
static bool s_uart_last_valid = false;

static const char *app_status_text(app_status_t status)
{
    switch (status) {
    case APP_STATUS_OK:
        return "OK";
    case APP_STATUS_FAIL:
        return "FAIL";
    case APP_STATUS_NOT_TESTED:
    default:
        return "PEND";
    }
}

static uint16_t app_status_color(app_status_t status)
{
    switch (status) {
    case APP_STATUS_OK:
        return GREEN;
    case APP_STATUS_FAIL:
        return RED;
    case APP_STATUS_NOT_TESTED:
    default:
        return YELLOW;
    }
}

static const char *app_page_name(app_test_page_t page)
{
    switch (page) {
    case APP_PAGE_OVERVIEW:
        return "Overview";
    case APP_PAGE_RGB_BUZZER:
        return "RGB+Buzzer+LED";
    case APP_PAGE_MOTOR:
        return "Motor+QEI+ADC";
    case APP_PAGE_IMU:
        return "IMU+SPI1";
    case APP_PAGE_STS3032:
        return "STS3032+UART6";
    case APP_PAGE_CAN:
        return "CAN+PA12/PA13";
    case APP_PAGE_UART_MODULES:
        return "UART3/4 Modules";
    default:
        return "Unknown";
    }
}

/*
 * LCD 行级脏刷新缓存。
 *
 * 设计原因：
 * - 旧实现每 200ms 把当前页面所有行先清黑再重写，LCD 看起来会闪烁。
 * - 当前测试页面大多是固定文本加少量数字变化，因此缓存上一帧每一行的文本和颜色；
 *   文本/颜色未变化时直接跳过 SPI 写屏。
 *
 * 时序约束：
 * - LCD_Fill/LCD_ShowString 都是阻塞式 SPI 写入，只能在主循环调用，不能放入 ISR。
 * - 进入/离开由 motor_ui 独立绘制的电机页面时，应用层缓存与真实屏幕内容可能不同，
 *   需要主动作废缓存，并在下一帧把未使用的行逐行清掉。
 */
static void app_lcd_invalidate_cache(void)
{
    memset(s_lcd_line_cache, 0, sizeof(s_lcd_line_cache));
    memset(s_lcd_color_cache, 0, sizeof(s_lcd_color_cache));
    memset(s_lcd_line_valid, 0, sizeof(s_lcd_line_valid));
    memset(s_lcd_line_touched, 0, sizeof(s_lcd_line_touched));
    s_rgb_block_valid = false;
}

static void app_lcd_begin_frame(void)
{
    memset(s_lcd_line_touched, 0, sizeof(s_lcd_line_touched));
    s_lcd_frame_active = true;
}

static void app_lcd_clear_cached_line(uint8_t line)
{
    uint16_t y;

    if (line >= APP_TEST_LCD_MAX_LINES) {
        return;
    }

    y = (uint16_t)(line * APP_TEST_LCD_LINE_HEIGHT);
    LCD_Fill(0U, y, LCD_W, (uint16_t)(y + APP_TEST_LCD_LINE_HEIGHT), BLACK);
    s_lcd_line_cache[line][0] = '\0';
    s_lcd_color_cache[line] = BLACK;
    s_lcd_line_valid[line] = false;
}

static void app_lcd_end_frame(void)
{
    uint8_t line;

    for (line = 0U; line < APP_TEST_LCD_MAX_LINES; line++) {
        /*
         * 页面切换后，新页面没有使用的旧行需要擦掉；普通周期刷新时只擦除
         * 缓存中确实曾经写过、但这一帧没有再声明的行，避免无意义清屏。
         */
        if ((s_lcd_line_touched[line] == false) &&
            ((s_lcd_line_valid[line] != false) ||
             (s_lcd_clear_untouched_on_next_frame != false))) {
            app_lcd_clear_cached_line(line);
        }
    }

    s_lcd_clear_untouched_on_next_frame = false;
    s_lcd_frame_active = false;
}

static void app_lcd_touch_rect_lines(uint16_t y0, uint16_t y1)
{
    uint8_t first_line;
    uint8_t last_line;
    uint8_t line;

    if (y0 >= LCD_H) {
        return;
    }
    if (y1 > LCD_H) {
        y1 = LCD_H;
    }
    if (y1 <= y0) {
        return;
    }

    first_line = (uint8_t)(y0 / APP_TEST_LCD_LINE_HEIGHT);
    last_line = (uint8_t)((y1 - 1U) / APP_TEST_LCD_LINE_HEIGHT);
    if (last_line >= APP_TEST_LCD_MAX_LINES) {
        last_line = APP_TEST_LCD_MAX_LINES - 1U;
    }

    for (line = first_line; line <= last_line; line++) {
        s_lcd_line_touched[line] = true;
    }
}

static void app_lcd_clear_rect_rows(uint16_t y0, uint16_t y1)
{
    uint8_t first_line;
    uint8_t last_line;
    uint8_t line;
    uint16_t clear_y0;
    uint16_t clear_y1;

    if (y0 >= LCD_H) {
        return;
    }
    if (y1 > LCD_H) {
        y1 = LCD_H;
    }
    if (y1 <= y0) {
        return;
    }

    first_line = (uint8_t)(y0 / APP_TEST_LCD_LINE_HEIGHT);
    last_line = (uint8_t)((y1 - 1U) / APP_TEST_LCD_LINE_HEIGHT);
    if (last_line >= APP_TEST_LCD_MAX_LINES) {
        last_line = APP_TEST_LCD_MAX_LINES - 1U;
    }

    clear_y0 = (uint16_t)(first_line * APP_TEST_LCD_LINE_HEIGHT);
    clear_y1 = (uint16_t)((last_line + 1U) * APP_TEST_LCD_LINE_HEIGHT);
    if (clear_y1 > LCD_H) {
        clear_y1 = LCD_H;
    }

    LCD_Fill(0U, clear_y0, LCD_W, clear_y1, BLACK);
    for (line = first_line; line <= last_line; line++) {
        s_lcd_line_cache[line][0] = '\0';
        s_lcd_color_cache[line] = BLACK;
        s_lcd_line_valid[line] = false;
        s_lcd_line_touched[line] = true;
    }
}

/*
 * LCD 单行安全输出。
 * 函数会把短文本补齐到整行字符宽度，并与上一帧缓存比较；只有内容或颜色变化时
 * 才清除并重写这一行。LCD_ShowString 仅支持 ASCII 字库，因此屏幕显示使用英文缩写，
 * 详细中文说明通过代码注释和 UART 日志体现。
 */
static void app_lcd_line(uint8_t line, uint16_t color, const char *text)
{
    char buffer[APP_TEST_LCD_TEXT_CHARS + 1U];
    uint16_t y;

    if (line >= APP_TEST_LCD_MAX_LINES) {
        return;
    }

    y = (uint16_t)(line * APP_TEST_LCD_LINE_HEIGHT);
    memset(buffer, ' ', APP_TEST_LCD_TEXT_CHARS);
    buffer[APP_TEST_LCD_TEXT_CHARS] = '\0';
    if (text != NULL) {
        (void)snprintf(buffer, sizeof(buffer), "%-*.*s",
                       (int)APP_TEST_LCD_TEXT_CHARS,
                       (int)APP_TEST_LCD_TEXT_CHARS,
                       text);
    }

    if (s_lcd_frame_active != false) {
        s_lcd_line_touched[line] = true;
    }

    if ((s_lcd_line_valid[line] != false) &&
        (s_lcd_color_cache[line] == color) &&
        (strncmp(s_lcd_line_cache[line], buffer, sizeof(buffer)) == 0)) {
        return;
    }

    /*
     * LCD_ShowString(mode=0) 会为每个字符写入前景/背景像素。这里使用 35 个
     * ASCII 字符刚好覆盖 280 像素宽度，因此不再先整行 LCD_Fill 清黑；
     * 对 IMU 这类 20Hz 连续变化的数据，直接覆盖字符格能明显减少频闪。
     */
    LCD_ShowString(0U, y, (const u8 *)buffer, color, BLACK, 16U, 0U);
    memcpy(s_lcd_line_cache[line], buffer, sizeof(buffer));
    s_lcd_color_cache[line] = color;
    s_lcd_line_valid[line] = true;
}

static void app_lcd_printf(uint8_t line, uint16_t color, const char *fmt, ...)
{
    char text[APP_TEST_LCD_TEXT_CHARS + 1U];
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    app_lcd_line(line, color, text);
}

static void app_lcd_clear(void)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
    app_lcd_invalidate_cache();
}

static uint16_t app_rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)((((uint16_t)r & 0xF8U) << 8) |
                      (((uint16_t)g & 0xFCU) << 3) |
                      ((uint16_t)b >> 3));
}

static void app_lcd_rgb_compare_block(uint8_t r, uint8_t g, uint8_t b)
{
    app_lcd_touch_rect_lines(APP_TEST_RGB_BLOCK_Y0, APP_TEST_RGB_BLOCK_Y1);

    /*
     * 色块占用的是纯图形区域，不对应 app_lcd_line() 文本行。
     * 每次进入 RGB 页面时先把色块覆盖到的整行清黑，清掉上一页面在同一高度留下
     * 的文字；同一页面后续只在颜色变化时重画色块本身。
     */
    if (s_rgb_block_valid == false) {
        app_lcd_clear_rect_rows(APP_TEST_RGB_BLOCK_Y0, APP_TEST_RGB_BLOCK_Y1);
    }

    if ((s_rgb_block_valid != false) &&
        (s_rgb_block_r == r) &&
        (s_rgb_block_g == g) &&
        (s_rgb_block_b == b)) {
        return;
    }

    LCD_Fill(APP_TEST_RGB_BLOCK_X0,
             APP_TEST_RGB_BLOCK_Y0,
             APP_TEST_RGB_BLOCK_X1,
             APP_TEST_RGB_BLOCK_Y1,
             app_rgb888_to_rgb565(r, g, b));
    LCD_DrawRectangle(APP_TEST_RGB_BLOCK_X0,
                      APP_TEST_RGB_BLOCK_Y0,
                      (uint16_t)(APP_TEST_RGB_BLOCK_X1 - 1U),
                      (uint16_t)(APP_TEST_RGB_BLOCK_Y1 - 1U),
                      WHITE);

    s_rgb_block_r = r;
    s_rgb_block_g = g;
    s_rgb_block_b = b;
    s_rgb_block_valid = true;
}

static void app_lcd_header(void)
{
    app_lcd_printf(0U, CYAN, "P%u/%u %s",
                   (unsigned)((uint8_t)s_page + 1U),
                   (unsigned)APP_PAGE_COUNT,
                   app_page_name(s_page));
    app_lcd_line(1U, YELLOW, "K1:Next  K2:Action");
}

static int32_t app_angle_to_centideg(float angle)
{
    if (angle >= 0.0f) {
        return (int32_t)(angle * 100.0f + 0.5f);
    }

    return (int32_t)(angle * 100.0f - 0.5f);
}

static void app_print_angle(const char *label, float angle)
{
    int32_t value = app_angle_to_centideg(angle);
    char sign = '+';

    if (value < 0) {
        sign = '-';
        value = -value;
    }

    printf("%s:%c%03ld.%02ld", label, sign,
           (long)(value / 100), (long)(value % 100));
}

static void app_lcd_angle_line(uint8_t line, const char *label, float angle)
{
    int32_t value = app_angle_to_centideg(angle);
    char sign = '+';

    if (value < 0) {
        sign = '-';
        value = -value;
    }

    app_lcd_printf(line, WHITE, "%s:%c%03ld.%02ld",
                   label,
                   sign,
                   (long)(value / 100),
                   (long)(value % 100));
}

/*
 * 停止所有输出型外设。
 * 输入输出：无参数；执行后 RGB 熄灭、蜂鸣器停止、电机停止，STS3032 扭矩尽量关闭。
 * 说明：切页前调用，保证“当前测试页面之外的外设不工作”，但不关闭 UART/CAN/Timer 等中断。
 */
static void app_stop_active_outputs(void)
{
    sk6812_demo_set_enable(false);
    (void)sk6812_apply_color(0U, 0U, 0U);
    buzzer_stop();
    motor_stop_all();
    LED1(0);
    LED2(0);

    if (s_sts_torque_on != false) {
        (void)sts3032_set_torque(APP_TEST_STS_ID, false, STS3032_RESPONSE_TIMEOUT_MS);
        s_sts_torque_on = false;
    }
}

static void app_rgb_apply_profile(void)
{
    const app_rgb_profile_t *profile = &s_rgb_profiles[s_rgb_index];

    (void)sk6812_apply_color(profile->r, profile->g, profile->b);
    if (profile->tone_hz == 0U) {
        buzzer_stop();
    } else {
        buzzer_play(profile->tone_hz, 30U);
    }

    if ((s_rgb_index & 0x01U) == 0U) {
        LED1(1);
        LED2(0);
    } else {
        LED1(0);
        LED2(1);
    }
}

static void app_enter_overview(void)
{
    app_stop_active_outputs();
}

static void app_enter_rgb(void)
{
    app_stop_active_outputs();
    /*
     * RGB 页只展示当前待测状态，不做自动换色。
     * 进入页面时保持输出关闭，用户每按一次 K2 才推进到下一个颜色/蜂鸣器组合。
     */
    s_rgb_index = (uint8_t)((sizeof(s_rgb_profiles) / sizeof(s_rgb_profiles[0])) - 1U);
}

static void app_enter_motor(void)
{
    app_stop_active_outputs();
    motor_ui_init();
    s_motor_tick = system_time_get_tick_ms();
}

static void app_enter_imu(void)
{
    app_stop_active_outputs();
    s_imu_tick = system_time_get_tick_ms();
    s_imu_probe_rc = IMU_probe_whoami(&s_imu_whoami);
    s_init_status.imu = (s_imu_probe_rc == 0) ? APP_STATUS_OK : APP_STATUS_FAIL;
}

static void app_enter_sts3032(void)
{
    app_stop_active_outputs();
    s_sts_tick = system_time_get_tick_ms();
    s_sts_position_valid = false;

    if (sts3032_set_torque(APP_TEST_STS_ID, true, STS3032_RESPONSE_TIMEOUT_MS) != false) {
        s_sts_torque_on = true;
        (void)sts3032_set_position(APP_TEST_STS_ID,
                                   s_sts_target,
                                   0U,
                                   APP_TEST_STS_SPEED,
                                   STS3032_RESPONSE_TIMEOUT_MS);
    }
}

static void app_enter_can(void)
{
    app_stop_active_outputs();
    s_can_tx_tick = system_time_get_tick_ms() - APP_TEST_CAN_TX_MS;
}

static void app_enter_uart_modules(void)
{
    app_stop_active_outputs();
}

static void app_enter_page(app_test_page_t page)
{
    app_test_page_t old_page = s_page;

    s_page = page;
    s_force_lcd_redraw = true;
    s_lcd_clear_untouched_on_next_frame = true;
    s_rgb_block_valid = false;
    s_print_page_header = true;

    /*
     * 电机测试页由 motor_ui.c 直接绘制表格和局部字段，应用层 LCD 行缓存无法
     * 准确知道屏幕上已经被 motor_ui 改写了哪些区域。进入或离开电机页时作废
     * 行缓存，下一帧会逐行重建当前页面，避免因为“缓存认为没变”而跳过真实重绘。
     */
    if ((old_page == APP_PAGE_MOTOR) || (page == APP_PAGE_MOTOR)) {
        app_lcd_invalidate_cache();
    }

    switch (s_page) {
    case APP_PAGE_OVERVIEW:
        app_enter_overview();
        break;
    case APP_PAGE_RGB_BUZZER:
        app_enter_rgb();
        break;
    case APP_PAGE_MOTOR:
        app_enter_motor();
        break;
    case APP_PAGE_IMU:
        app_enter_imu();
        break;
    case APP_PAGE_STS3032:
        app_enter_sts3032();
        break;
    case APP_PAGE_CAN:
        app_enter_can();
        break;
    case APP_PAGE_UART_MODULES:
        app_enter_uart_modules();
        break;
    default:
        break;
    }
}

static void app_next_page(void)
{
    uint8_t next = (uint8_t)s_page + 1U;

    if (next >= (uint8_t)APP_PAGE_COUNT) {
        next = 0U;
    }

    app_enter_page((app_test_page_t)next);
}

static void app_print_init_status(void)
{
    printf("\r\n[INIT] LCD/SPI0=%s Timer=%s Key=%s LED=%s\r\n",
           app_status_text(s_init_status.lcd),
           app_status_text(s_init_status.timer),
           app_status_text(s_init_status.key),
           app_status_text(s_init_status.led));
    printf("[INIT] RGB/PWM=%s Buzzer=%s Motor/PWM=%s QEI=%s ADC/VREF=%s\r\n",
           app_status_text(s_init_status.rgb),
           app_status_text(s_init_status.buzzer),
           app_status_text(s_init_status.motor),
           app_status_text(s_init_status.qei),
           app_status_text(s_init_status.adc_vref));
    printf("[INIT] IMU/SPI1=%s WiFi/UART4=%s BT/UART3=%s STS/UART6=%s CAN=%s\r\n",
           app_status_text(s_init_status.imu),
           app_status_text(s_init_status.wifi),
           app_status_text(s_init_status.bluetooth),
           app_status_text(s_init_status.sts3032),
           app_status_text(s_init_status.can));
}

static void app_draw_status_pair(uint8_t line,
                                 const char *left_name,
                                 app_status_t left_status,
                                 const char *right_name,
                                 app_status_t right_status)
{
    app_lcd_printf(line, WHITE, "%s:%s  %s:%s",
                   left_name,
                   app_status_text(left_status),
                   right_name,
                   app_status_text(right_status));
}

static void app_draw_overview(void)
{
    app_lcd_header();
    app_draw_status_pair(2U, "LCD", s_init_status.lcd, "TMR", s_init_status.timer);
    app_draw_status_pair(3U, "KEY", s_init_status.key, "LED", s_init_status.led);
    app_draw_status_pair(4U, "RGB", s_init_status.rgb, "BUZ", s_init_status.buzzer);
    app_draw_status_pair(5U, "MOT", s_init_status.motor, "QEI", s_init_status.qei);
    app_draw_status_pair(6U, "ADC", s_init_status.adc_vref, "IMU", s_init_status.imu);
    app_draw_status_pair(7U, "WF4", s_init_status.wifi, "BT3", s_init_status.bluetooth);
    app_draw_status_pair(8U, "STS", s_init_status.sts3032, "CAN", s_init_status.can);
    app_lcd_line(10U, YELLOW, "K2: Print init status");
}

static void app_draw_rgb(void)
{
    bool buzzer_active;
    uint16_t tone_hz;
    uint8_t duty;
    uint8_t scaled_r;
    uint8_t scaled_g;
    uint8_t scaled_b;
    const app_rgb_profile_t *profile = &s_rgb_profiles[s_rgb_index];

    buzzer_get_context(&buzzer_active, &tone_hz, &duty);
    scaled_r = sk6812_brightness_scale(profile->r);
    scaled_g = sk6812_brightness_scale(profile->g);
    scaled_b = sk6812_brightness_scale(profile->b);

    app_lcd_header();
    app_lcd_printf(3U, WHITE, "Color:%s", profile->name);
    app_lcd_printf(4U, RED, "R:%3u G:%3u B:%3u",
                   (unsigned)scaled_r,
                   (unsigned)scaled_g,
                   (unsigned)scaled_b);
    app_lcd_printf(5U, buzzer_active ? GREEN : YELLOW,
                   "Buzzer:%s %uHz",
                   buzzer_active ? "ON" : "OFF",
                   (unsigned)tone_hz);
    app_lcd_printf(6U, WHITE, "SK6812 busy:%u lock:%u",
                   sk6812_is_busy() ? 1U : 0U,
                   sk6812_rgb_is_locked() ? 1U : 0U);
    app_lcd_line(8U, YELLOW, "K2: Color/Tone next");
    app_lcd_line(9U, CYAN, "LCD compare block:");
    app_lcd_rgb_compare_block(scaled_r, scaled_g, scaled_b);
}

static void app_print_rgb_status(void)
{
    bool buzzer_active;
    uint16_t tone_hz;
    uint8_t duty;
    const app_rgb_profile_t *profile = &s_rgb_profiles[s_rgb_index];

    buzzer_get_context(&buzzer_active, &tone_hz, &duty);
    printf("[RGB] color=%s raw=(%u,%u,%u) scaled=(%u,%u,%u) buzzer=%u %uHz duty=%u\r\n",
           profile->name,
           (unsigned)profile->r,
           (unsigned)profile->g,
           (unsigned)profile->b,
           (unsigned)sk6812_brightness_scale(profile->r),
           (unsigned)sk6812_brightness_scale(profile->g),
           (unsigned)sk6812_brightness_scale(profile->b),
           buzzer_active ? 1U : 0U,
           (unsigned)tone_hz,
           (unsigned)duty);
}

static void app_print_motor_status(void)
{
    motor_status_t status;
    uint32_t i;

    printf("[MOTOR] step=%u anyFault=%u anySleep=%u anyRun=%u\r\n",
           (unsigned)motor_ui_get_step(),
           motor_any_fault() ? 1U : 0U,
           motor_any_asleep() ? 1U : 0U,
           motor_any_running() ? 1U : 0U);

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (motor_get_status((motor_id_t)i, &status) != false) {
            printf("[MOTOR] M%lu speed=%d adc=%u current=%umA fault=%u sleep=%u qei=%u dir=%c\r\n",
                   (unsigned long)(i + 1U),
                   (int)status.speed_permille,
                   (unsigned)status.adc_raw,
                   (unsigned)status.current_ma,
                   status.fault ? 1U : 0U,
                   status.asleep ? 1U : 0U,
                   (unsigned)motor_qei_get_count((motor_id_t)i),
                   motor_qei_get_direction_up((motor_id_t)i) ? 'U' : 'D');
        }
    }
}

static void app_draw_imu(void)
{
    app_lcd_header();
    app_lcd_printf(3U, app_status_color(s_init_status.imu),
                   "Ready:%u RC:%d WHO:0x%02X",
                   IMU_isReady() ? 1U : 0U,
                   s_imu_probe_rc,
                   (unsigned)s_imu_whoami);
    app_lcd_angle_line(4U, "Yaw  ", s_imu_ypr[0]);
    app_lcd_angle_line(5U, "Pitch", s_imu_ypr[1]);
    app_lcd_angle_line(6U, "Roll ", s_imu_ypr[2]);
    app_lcd_line(8U, YELLOW, "K2: Probe WHO_AM_I");
}

static void app_print_imu_status(void)
{
    printf("[IMU] ready=%u probe_rc=%d whoami=0x%02X ",
           IMU_isReady() ? 1U : 0U,
           s_imu_probe_rc,
           (unsigned)s_imu_whoami);
    app_print_angle("Y", s_imu_ypr[0]);
    printf(" ");
    app_print_angle("P", s_imu_ypr[1]);
    printf(" ");
    app_print_angle("R", s_imu_ypr[2]);
    printf("\r\n");
}

static void app_draw_sts(void)
{
    app_lcd_header();
    app_lcd_printf(3U, app_status_color(s_init_status.sts3032),
                   "UART6:%lu ID:%u",
                   (unsigned long)sts3032_get_configured_baud_rate(),
                   (unsigned)APP_TEST_STS_ID);
    app_lcd_printf(4U, WHITE, "Torque:%u Target:%u",
                   s_sts_torque_on ? 1U : 0U,
                   (unsigned)s_sts_target);
    if (s_sts_position_valid != false) {
        app_lcd_printf(5U, GREEN, "Position:%u", (unsigned)s_sts_position);
    } else {
        app_lcd_line(5U, YELLOW, "Position:----");
    }
    app_lcd_printf(6U, WHITE, "Err:%s Stat:%u",
                   sts3032_get_error_string(sts3032_get_last_error()),
                   (unsigned)sts3032_get_last_device_status());
    app_lcd_line(8U, YELLOW, "K2: Toggle target");
}

static void app_print_sts_status(void)
{
    printf("[STS] id=%u torque=%u target=%u pos=",
           (unsigned)APP_TEST_STS_ID,
           s_sts_torque_on ? 1U : 0U,
           (unsigned)s_sts_target);
    if (s_sts_position_valid != false) {
        printf("%u", (unsigned)s_sts_position);
    } else {
        printf("----");
    }
    printf(" err=%s status=%u tx=%lu rx=%lu ov=%lu\r\n",
           sts3032_get_error_string(sts3032_get_last_error()),
           (unsigned)sts3032_get_last_device_status(),
           (unsigned long)sts3032_get_tx_packet_count(),
           (unsigned long)sts3032_get_rx_packet_count(),
           (unsigned long)sts3032_get_rx_overrun_count());
}

static void app_can_print_frame(const CanBusFrame *frame)
{
    uint8_t i;

    if (frame == NULL) {
        return;
    }

    printf("[CAN] RX %s ID=0x%lX DLC=%u LEN=%u DATA",
           frame->extended ? "EXT" : "STD",
           (unsigned long)frame->id,
           (unsigned)frame->dlc,
           (unsigned)frame->length);
    for (i = 0U; i < frame->length; i++) {
        printf(" %02X", (unsigned)frame->data[i]);
    }
    if (frame->truncated != false) {
        printf(" ...");
    }
    printf("\r\n");
}

static bool app_can_send_frame(void)
{
    uint32_t tick = system_time_get_tick_ms();
    uint8_t payload[CAN_BUS_MAX_DATA_LENGTH];

    payload[0] = (uint8_t)(s_can_sequence & 0xFFU);
    payload[1] = (uint8_t)((s_can_sequence >> 8U) & 0xFFU);
    payload[2] = (uint8_t)(tick & 0xFFU);
    payload[3] = (uint8_t)((tick >> 8U) & 0xFFU);
    payload[4] = (uint8_t)((tick >> 16U) & 0xFFU);
    payload[5] = (uint8_t)((tick >> 24U) & 0xFFU);
    payload[6] = 0xA5U;
    payload[7] = 0x5AU;

    if (can_bus_send_standard(APP_TEST_CAN_ID, payload, (uint8_t)sizeof(payload)) == false) {
        return false;
    }

    printf("[CAN] TX STD ID=0x%03X SEQ=%u\r\n",
           (unsigned)APP_TEST_CAN_ID,
           (unsigned)s_can_sequence);
    s_can_sequence++;
    return true;
}

static void app_draw_can(void)
{
    app_lcd_header();
    app_lcd_printf(3U, app_status_color(s_init_status.can),
                   "Ready:%u Bit:%uk",
                   can_bus_is_ready() ? 1U : 0U,
                   (unsigned)CAN_BUS_DEFAULT_BITRATE_KBPS);
    app_lcd_printf(4U, WHITE, "TX:%lu Busy:%lu",
                   (unsigned long)can_bus_get_tx_count(),
                   (unsigned long)can_bus_get_tx_busy_count());
    app_lcd_printf(5U, WHITE, "RX:%lu Drop:%lu",
                   (unsigned long)can_bus_get_rx_count(),
                   (unsigned long)can_bus_get_rx_drop_count());
    app_lcd_printf(6U, WHITE, "Err:%lu Seq:%u",
                   (unsigned long)can_bus_get_error_count(),
                   (unsigned)s_can_sequence);
    app_lcd_line(8U, YELLOW, "K2: Send once");
}

static void app_print_can_status(void)
{
    printf("[CAN] ready=%u bitrate=%uk tx=%lu busy=%lu rx=%lu drop=%lu err=%lu\r\n",
           can_bus_is_ready() ? 1U : 0U,
           (unsigned)CAN_BUS_DEFAULT_BITRATE_KBPS,
           (unsigned long)can_bus_get_tx_count(),
           (unsigned long)can_bus_get_tx_busy_count(),
           (unsigned long)can_bus_get_rx_count(),
           (unsigned long)can_bus_get_rx_drop_count(),
           (unsigned long)can_bus_get_error_count());
}

static bool app_uart_response_has_ok(const uint8_t *rx_buf, uint32_t rx_len)
{
    if ((rx_buf == NULL) || (rx_len == 0U)) {
        return false;
    }

    return (strstr((const char *)rx_buf, "OK") != NULL);
}

static void app_uart_make_preview(char *dst,
                                  size_t dst_size,
                                  const uint8_t *rx_buf,
                                  uint32_t rx_len)
{
    uint32_t i;
    size_t out = 0U;
    bool prev_space = false;

    if ((dst == NULL) || (dst_size == 0U)) {
        return;
    }

    if ((rx_buf == NULL) || (rx_len == 0U)) {
        (void)snprintf(dst, dst_size, "NO RESPONSE");
        return;
    }

    /*
     * LCD 一行只能显示 ASCII 摘要，因此把 AT 应答中的 CR/LF 合并为空格，
     * 其它不可见字符替换成 '.'。UART0 仍会打印原始应答，便于完整核对。
     */
    for (i = 0U; (i < rx_len) && (out < (dst_size - 1U)); i++) {
        char ch = (char)rx_buf[i];

        if ((ch == '\r') || (ch == '\n') || (ch == '\t')) {
            if ((out > 0U) && (prev_space == false)) {
                dst[out++] = ' ';
                prev_space = true;
            }
            continue;
        }

        if (((uint8_t)ch < 32U) || ((uint8_t)ch > 126U)) {
            ch = '.';
        }

        dst[out++] = ch;
        prev_space = (ch == ' ');
    }

    dst[out] = '\0';
}

static void app_uart_record_response(const char *module,
                                     const char *cmd,
                                     const uint8_t *rx_buf,
                                     uint32_t rx_len,
                                     bool ok)
{
    (void)snprintf(s_uart_last_module, sizeof(s_uart_last_module), "%s", module);
    (void)snprintf(s_uart_last_cmd, sizeof(s_uart_last_cmd), "%s", cmd);
    app_uart_make_preview(s_uart_last_rx, sizeof(s_uart_last_rx), rx_buf, rx_len);
    s_uart_last_rx_len = rx_len;
    s_uart_last_ok = ok;
    s_uart_last_valid = true;
}

static bool app_uart_probe_wb2(void)
{
    static uint8_t rx_buf[APP_TEST_UART_RX_BUFFER_SIZE];
    const char *cmd = "AT";
    uint32_t n;
    bool ok;

    wb2_wifi_flush_rx();
    printf("[WB2] TX: %s\r\n", cmd);
    if (wb2_wifi_send_cmd(cmd) == false) {
        app_uart_record_response("WB2", cmd, NULL, 0U, false);
        printf("[WB2] TX failed\r\n");
        return false;
    }

    n = wb2_wifi_read(rx_buf, sizeof(rx_buf), APP_TEST_UART_AT_TIMEOUT_MS);
    ok = app_uart_response_has_ok(rx_buf, n);
    app_uart_record_response("WB2", cmd, rx_buf, n, ok);

    printf("[WB2] RX (%lu B):\r\n%s\r\n",
           (unsigned long)n,
           (n > 0U) ? (const char *)rx_buf : "NO RESPONSE");
    return ok;
}

static bool app_uart_probe_hc05(void)
{
    static uint8_t rx_buf[APP_TEST_UART_RX_BUFFER_SIZE];
    const char *cmd = "AT";
    uint32_t n;
    bool ok;

    hc05_bt_flush_rx();
    printf("[HC05] TX: %s\r\n", cmd);
    if (hc05_bt_send_cmd(cmd) == false) {
        app_uart_record_response("HC05", cmd, NULL, 0U, false);
        printf("[HC05] TX failed\r\n");
        return false;
    }

    n = hc05_bt_read(rx_buf, sizeof(rx_buf), APP_TEST_UART_AT_TIMEOUT_MS);
    ok = app_uart_response_has_ok(rx_buf, n);
    app_uart_record_response("HC05", cmd, rx_buf, n, ok);

    printf("[HC05] RX (%lu B):\r\n%s\r\n",
           (unsigned long)n,
           (n > 0U) ? (const char *)rx_buf : "NO RESPONSE");
    return ok;
}

static void app_draw_uart_modules(void)
{
    app_lcd_header();
    app_lcd_printf(3U, WHITE, "WB2 UART4:%u 8N1", (unsigned)UART_WF_BAUD_RATE);
    app_lcd_printf(4U, WHITE, "HC05 UART3:%u 8N1", (unsigned)UART_BL_BAUD_RATE);
    app_lcd_line(6U, YELLOW, "K2: AT self-test");
    app_lcd_printf(7U, WHITE, "Next:%s",
                   (s_uart_test_selector == 0U) ? "WB2" : "HC05");
    app_lcd_printf(9U, s_uart_last_ok ? GREEN : YELLOW, "Last:%s %s %s",
                   s_uart_last_module,
                   s_uart_last_cmd,
                   s_uart_last_valid ? (s_uart_last_ok ? "OK" : "FAIL") : "PEND");
    app_lcd_printf(10U, WHITE, "RX(%luB):", (unsigned long)s_uart_last_rx_len);
    app_lcd_line(11U,
                 s_uart_last_valid ? (s_uart_last_ok ? GREEN : RED) : YELLOW,
                 s_uart_last_rx);
}

static void app_print_uart_status(void)
{
    printf("[UART] debug UART0 active, WB2 UART4=%u, HC05 UART3=%u, STS UART6=%lu last=%s %s %s rx=%luB \"%s\"\r\n",
           (unsigned)UART_WF_BAUD_RATE,
           (unsigned)UART_BL_BAUD_RATE,
           (unsigned long)sts3032_get_configured_baud_rate(),
           s_uart_last_module,
           s_uart_last_cmd,
           s_uart_last_valid ? (s_uart_last_ok ? "OK" : "FAIL") : "PEND",
           (unsigned long)s_uart_last_rx_len,
           s_uart_last_rx);
}

static void app_draw_current_page(void)
{
    if (s_page == APP_PAGE_MOTOR) {
        if (s_force_lcd_redraw != false) {
            motor_ui_request_full_redraw();
            s_force_lcd_redraw = false;
        }
        return;
    }

    app_lcd_begin_frame();

    switch (s_page) {
    case APP_PAGE_OVERVIEW:
        app_draw_overview();
        break;
    case APP_PAGE_RGB_BUZZER:
        app_draw_rgb();
        break;
    case APP_PAGE_MOTOR:
        break;
    case APP_PAGE_IMU:
        app_draw_imu();
        break;
    case APP_PAGE_STS3032:
        app_draw_sts();
        break;
    case APP_PAGE_CAN:
        app_draw_can();
        break;
    case APP_PAGE_UART_MODULES:
        app_draw_uart_modules();
        break;
    default:
        break;
    }

    app_lcd_end_frame();
    s_force_lcd_redraw = false;
}

static void app_print_current_page_status(void)
{
    if (s_print_page_header != false) {
        printf("\r\n[PAGE] %u/%u %s\r\n",
               (unsigned)((uint8_t)s_page + 1U),
               (unsigned)APP_PAGE_COUNT,
               app_page_name(s_page));
        s_print_page_header = false;
    }

    switch (s_page) {
    case APP_PAGE_OVERVIEW:
        app_print_init_status();
        break;
    case APP_PAGE_RGB_BUZZER:
        app_print_rgb_status();
        break;
    case APP_PAGE_MOTOR:
        app_print_motor_status();
        break;
    case APP_PAGE_IMU:
        app_print_imu_status();
        break;
    case APP_PAGE_STS3032:
        app_print_sts_status();
        break;
    case APP_PAGE_CAN:
        app_print_can_status();
        break;
    case APP_PAGE_UART_MODULES:
        app_print_uart_status();
        break;
    default:
        break;
    }
}

static void app_action_overview(void)
{
    s_print_page_header = true;
    app_print_init_status();
}

static void app_action_rgb(void)
{
    s_rgb_index++;
    if (s_rgb_index >= (uint8_t)(sizeof(s_rgb_profiles) / sizeof(s_rgb_profiles[0]))) {
        s_rgb_index = 0U;
    }
    app_rgb_apply_profile();
    s_force_lcd_redraw = true;
    app_print_rgb_status();
}

static void app_action_motor(void)
{
    motor_ui_next_step();
    printf("[MOTOR] K2 step -> %u\r\n", (unsigned)motor_ui_get_step());
}

static void app_action_imu(void)
{
    s_imu_probe_rc = IMU_probe_whoami(&s_imu_whoami);
    s_init_status.imu = (s_imu_probe_rc == 0) ? APP_STATUS_OK : APP_STATUS_FAIL;
    printf("[IMU] K2 probe rc=%d whoami=0x%02X\r\n",
           s_imu_probe_rc,
           (unsigned)s_imu_whoami);
}

static void app_action_sts(void)
{
    if (s_sts_target == APP_TEST_STS_POS_HIGH) {
        s_sts_target = APP_TEST_STS_POS_LOW;
    } else {
        s_sts_target = APP_TEST_STS_POS_HIGH;
    }

    if (sts3032_set_position(APP_TEST_STS_ID,
                             s_sts_target,
                             0U,
                             APP_TEST_STS_SPEED,
                             STS3032_RESPONSE_TIMEOUT_MS) == false) {
        s_init_status.sts3032 = APP_STATUS_FAIL;
    }

    printf("[STS] K2 target=%u\r\n", (unsigned)s_sts_target);
}

static void app_action_can(void)
{
    if (app_can_send_frame() == false) {
        printf("[CAN] K2 send failed: busy=%lu err=%lu\r\n",
               (unsigned long)can_bus_get_tx_busy_count(),
               (unsigned long)can_bus_get_error_count());
    }
}

static void app_action_uart_modules(void)
{
    if (s_uart_test_selector == 0U) {
        bool ok = app_uart_probe_wb2();

        s_init_status.wifi = ok ? APP_STATUS_OK : APP_STATUS_FAIL;
        printf("[WB2] K2 result=%s\r\n", ok ? "OK" : "FAIL");
        s_uart_test_selector = 1U;
    } else {
        bool ok = app_uart_probe_hc05();

        s_init_status.bluetooth = ok ? APP_STATUS_OK : APP_STATUS_FAIL;
        printf("[HC05] K2 result=%s\r\n", ok ? "OK" : "FAIL");
        s_uart_test_selector = 0U;
    }

    s_force_lcd_redraw = true;
}

static void app_handle_page_action(void)
{
    switch (s_page) {
    case APP_PAGE_OVERVIEW:
        app_action_overview();
        break;
    case APP_PAGE_RGB_BUZZER:
        app_action_rgb();
        break;
    case APP_PAGE_MOTOR:
        app_action_motor();
        break;
    case APP_PAGE_IMU:
        app_action_imu();
        break;
    case APP_PAGE_STS3032:
        app_action_sts();
        break;
    case APP_PAGE_CAN:
        app_action_can();
        break;
    case APP_PAGE_UART_MODULES:
        app_action_uart_modules();
        break;
    default:
        break;
    }
}

static void app_task_motor(void)
{
    if (system_time_elapsed_ms(&s_motor_tick, APP_TEST_MOTOR_REFRESH_MS) != false) {
        motor_ui_refresh(NULL, false);
    }
}

static void app_task_imu(void)
{
    if (system_time_elapsed_ms(&s_imu_tick, APP_TEST_IMU_REFRESH_MS) != false) {
        if (IMU_isReady() != false) {
            IMU_getYawPitchRoll(s_imu_ypr);
        }
    }
}

static void app_task_sts(void)
{
    if (system_time_elapsed_ms(&s_sts_tick, APP_TEST_STS_REFRESH_MS) != false) {
        s_sts_position_valid =
            sts3032_read_position(APP_TEST_STS_ID,
                                  &s_sts_position,
                                  STS3032_RESPONSE_TIMEOUT_MS);
        if (s_sts_position_valid == false) {
            s_init_status.sts3032 = APP_STATUS_FAIL;
        }
    }
}

static void app_task_can(void)
{
    CanBusFrame frame;
    uint32_t error_status;

    while (can_bus_poll_rx(&frame) != false) {
        app_can_print_frame(&frame);
    }

    if (can_bus_take_error_status(&error_status) != false) {
        printf("[CAN] IRQ error=0x%08lX\r\n", (unsigned long)error_status);
    }

    if (system_time_elapsed_ms(&s_can_tx_tick, APP_TEST_CAN_TX_MS) != false) {
        (void)app_can_send_frame();
    }
}

static void app_run_current_page_task(void)
{
    switch (s_page) {
    case APP_PAGE_RGB_BUZZER:
        break;
    case APP_PAGE_MOTOR:
        app_task_motor();
        break;
    case APP_PAGE_IMU:
        app_task_imu();
        break;
    case APP_PAGE_STS3032:
        app_task_sts();
        break;
    case APP_PAGE_CAN:
        app_task_can();
        break;
    case APP_PAGE_OVERVIEW:
    case APP_PAGE_UART_MODULES:
    default:
        break;
    }
}

static void app_handle_keys(void)
{
    if (user_key_take_edge(USER_KEY_K1) != false) {
        app_next_page();
    }

    if (user_key_take_edge(USER_KEY_K2) != false) {
        app_handle_page_action();
        if (s_page != APP_PAGE_MOTOR) {
            s_force_lcd_redraw = true;
        }
    }
}

static void app_detect_basic_status(void)
{
    uint32_t timer_start = system_time_get_tick_ms();

    s_init_status.lcd = APP_STATUS_OK;
    s_init_status.timer = system_time_wait_for_tick(CPUCLK_FREQ / 100U) ?
                          APP_STATUS_OK :
                          APP_STATUS_FAIL;
    s_init_status.key = APP_STATUS_OK;
    s_init_status.led = APP_STATUS_OK;
    s_init_status.rgb = sk6812_rgb_is_locked() ? APP_STATUS_OK : APP_STATUS_FAIL;
    s_init_status.buzzer = APP_STATUS_OK;
    s_init_status.motor = (motor_any_fault() || motor_any_asleep()) ?
                          APP_STATUS_FAIL :
                          APP_STATUS_OK;
    s_init_status.qei = APP_STATUS_OK;
    s_init_status.adc_vref = APP_STATUS_OK;
    s_init_status.imu = IMU_isReady() ? APP_STATUS_OK : APP_STATUS_FAIL;
    s_init_status.wifi = APP_STATUS_NOT_TESTED;
    s_init_status.bluetooth = APP_STATUS_NOT_TESTED;
    s_init_status.sts3032 = sts3032_ping(APP_TEST_STS_ID, STS3032_RESPONSE_TIMEOUT_MS) ?
                            APP_STATUS_OK :
                            APP_STATUS_FAIL;
    s_init_status.can = can_bus_is_ready() ? APP_STATUS_OK : APP_STATUS_FAIL;

    s_lcd_tick = timer_start;
    s_uart_tick = timer_start;
}

void peripheral_test_init(void)
{
    /*
     * 初始化顺序说明：
     * - LCD 最先启动，便于后续把初始化状态显示出来。
     * - 电机、RGB、蜂鸣器属于输出设备，初始化后立即停止，避免上电时动作。
     * - WiFi/蓝牙只做 UART 侧初始化，不主动发送 AT；对应自检放到 UART 页面 K2。
     * - STS3032 只做 PING，不主动转动；真正位置命令放到 STS 页面。
     */
    printf("\r\n[APP] G3519 Car init start\r\n");

    LCD_Init();
    app_lcd_clear();
    app_lcd_line(0U, CYAN, "Peripheral init...");

    LED_init();
    IMU_init();
    SPI1_CS_IMU(1);
    motor_init();
    motor_qei_init();
    sk6812_init();
    (void)sk6812_apply_color(0U, 0U, 0U);
    buzzer_stop();
    wb2_wifi_init();
    hc05_bt_init();
    (void)sts3032_init(STS3032_DEFAULT_BAUD_RATE);
    (void)can_bus_init();

    app_stop_active_outputs();
    app_detect_basic_status();
    app_print_init_status();
    app_enter_page(APP_PAGE_OVERVIEW);
}

void peripheral_test_task(void)
{
    app_handle_keys();
    app_run_current_page_task();

    if ((s_force_lcd_redraw != false) ||
        (system_time_elapsed_ms(&s_lcd_tick, APP_TEST_LCD_REFRESH_MS) != false)) {
        app_draw_current_page();
    }

    if (system_time_elapsed_ms(&s_uart_tick, APP_TEST_UART_REFRESH_MS) != false) {
        app_print_current_page_status();
    }
}
