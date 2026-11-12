#include "SK6812/sk6812_demo.h"
#include "SK6812/sk6812.h"
#include "SystemTime/system_time.h"

/*
 * 预设颜色表，按固定时间间隔轮询切换。
 */

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} sk6812_demo_color_t;

static const sk6812_demo_color_t s_palette[] = {
    { 255U, 0U,   0U   },
    { 0U,   255U, 0U   },
    { 0U,   0U,   255U },
    { 255U, 255U, 0U   },
    { 0U,   255U, 255U },
    { 255U, 0U,   255U },
    { 255U, 128U, 0U   },
};

static uint8_t s_palette_index;
static uint32_t s_last_change_tick;
static uint32_t s_interval_ms = 800U;
static bool s_enabled = true;

/* 与灯珠当前 GRB 输出一致的 RGB 缓存，供 LCD 色块读取。 */
static uint8_t s_current_r;
static uint8_t s_current_g;
static uint8_t s_current_b;

static void sk6812_demo_store_color(uint8_t r, uint8_t g, uint8_t b)
{
    s_current_r = r;
    s_current_g = g;
    s_current_b = b;
}

void sk6812_demo_get_color(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (r != NULL) {
        *r = s_current_r;
    }
    if (g != NULL) {
        *g = s_current_g;
    }
    if (b != NULL) {
        *b = s_current_b;
    }
}

void sk6812_demo_init(void)
{
    s_palette_index    = 0U;
    s_last_change_tick = system_time_get_tick_ms();
    s_enabled          = true;
    sk6812_demo_store_color(sk6812_brightness_scale(s_palette[0].r),
                            sk6812_brightness_scale(s_palette[0].g),
                            sk6812_brightness_scale(s_palette[0].b));

    if (sk6812_apply_color(s_palette[0].r, s_palette[0].g, s_palette[0].b)) {
        printf("SK6812 demo: R%d G%d B%d (%u%%)\r\n",
               (int)sk6812_brightness_scale(s_palette[0].r),
               (int)sk6812_brightness_scale(s_palette[0].g),
               (int)sk6812_brightness_scale(s_palette[0].b),
               (unsigned)SK6812_BRIGHTNESS_PERCENT);
    }
}

void sk6812_demo_set_enable(bool enable)
{
    s_enabled = enable;
    if (!enable) {
        sk6812_demo_store_color(0U, 0U, 0U);
        sk6812_apply_color(0U, 0U, 0U);
    } else {
        s_last_change_tick = system_time_get_tick_ms();
        sk6812_demo_store_color(
            sk6812_brightness_scale(s_palette[s_palette_index].r),
            sk6812_brightness_scale(s_palette[s_palette_index].g),
            sk6812_brightness_scale(s_palette[s_palette_index].b));
    }
}

void sk6812_demo_set_interval_ms(uint32_t interval_ms)
{
    if (interval_ms < 100U) {
        interval_ms = 100U;
    }

    s_interval_ms = interval_ms;
}

void sk6812_demo_task(void)
{
    const sk6812_demo_color_t *color;

    if (!s_enabled) {
        return;
    }

    /* 上一帧仍在发码时不重入 */
    if (sk6812_is_busy()) {
        return;
    }

    if (!system_time_elapsed_ms(&s_last_change_tick, s_interval_ms)) {
        return;
    }

    s_palette_index = (uint8_t)((s_palette_index + 1U) % (sizeof(s_palette) / sizeof(s_palette[0])));
    color           = &s_palette[s_palette_index];
    sk6812_demo_store_color(sk6812_brightness_scale(color->r),
                            sk6812_brightness_scale(color->g),
                            sk6812_brightness_scale(color->b));

    if (sk6812_apply_color(color->r, color->g, color->b)) {
        printf("RGB tick %u R%d G%d B%d (%u%%)\r\n",
               (unsigned)s_palette_index,
               (int)sk6812_brightness_scale(color->r),
               (int)sk6812_brightness_scale(color->g),
               (int)sk6812_brightness_scale(color->b),
               (unsigned)SK6812_BRIGHTNESS_PERCENT);
    }
}
