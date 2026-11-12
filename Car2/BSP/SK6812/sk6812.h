#ifndef SK6812_H
#define SK6812_H

/*
 * SK6812 RGB（PA15）+ 蜂鸣器（PA16）共用 TIMG12 / PWM_1。
 *
 * 发送：PWM 800kHz 位时序 + LOAD 中断更新比较值。
 * 推荐 sk6812_apply_color(r,g,b)；蜂鸣器仅在 PA15 锁定后可用。
 */
#include "bsp_common.h"

#ifndef SK6812_LED_COUNT
#define SK6812_LED_COUNT (1U)
#endif

/*
 * 输出亮度百分比（0~100），默认 20% 减轻刺眼；可在 Keil 预定义覆盖。
 */
#ifndef SK6812_BRIGHTNESS_PERCENT
#define SK6812_BRIGHTNESS_PERCENT (20U)
#endif

typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} sk6812_color_t;

/*
 * 初始化 SK6812 驱动的软件状态和共享 PWM 仲裁状态。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后调用，因为 TIMG12/PWM_1 和 PA15 复用来自 SysConfig。
 *
 * 输出约束：
 * - 初始化后不会自动点亮灯珠；应用层需要调用 sk6812_apply_color() 或 refresh 接口发送颜色。
 */
void sk6812_init(void);

/*
 * 按全局亮度百分比缩放单个颜色通道。
 *
 * 参数：
 * - channel：原始 0~255 颜色通道值。
 *
 * 返回值：
 * - 缩放后的 0~255 通道值，缩放比例由 SK6812_BRIGHTNESS_PERCENT 控制。
 */
uint8_t sk6812_brightness_scale(uint8_t channel);

/*
 * 清空 SK6812 软件颜色缓冲。
 *
 * 说明：
 * - 只修改内存中的颜色缓存，不会立即写到灯珠；需要随后调用 sk6812_refresh() 才会生效。
 */
void sk6812_clear(void);

/*
 * 设置单个 SK6812 像素的原始 RGB 颜色。
 *
 * 参数：
 * - index：灯珠序号，范围 0~SK6812_LED_COUNT-1。
 * - r/g/b：原始 0~255 RGB 值，实际发送时会按亮度比例缩放。
 */
void sk6812_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/*
 * 设置所有 SK6812 灯珠为同一 RGB 颜色。
 *
 * 参数：
 * - r/g/b：原始 0~255 RGB 值。
 *
 * 说明：
 * - 只更新软件缓冲，调用 sk6812_refresh() 后才会真正输出到 PA15。
 */
void sk6812_set_all(uint8_t r, uint8_t g, uint8_t b);

/*
 * 启动一次 SK6812 波形刷新。
 *
 * 返回值：
 * - true：成功开始发送或发送已经完成。
 * - false：上一次刷新仍在进行，或者 PWM 仲裁状态不允许开始新传输。
 *
 * 中断约束：
 * - 发送过程使用 TIMG12 LOAD 中断逐位更新比较值，主循环应通过 sk6812_is_busy()
 *   判断是否仍在发送。
 */
bool sk6812_refresh(void);

/*
 * 立即设置并刷新单颗/全部 SK6812 颜色。
 *
 * 参数：
 * - r/g/b：原始 0~255 RGB 值。
 *
 * 返回值：
 * - true：颜色刷新请求已成功提交。
 * - false：当前仍忙或底层 PWM 状态异常。
 */
bool sk6812_apply_color(uint8_t r, uint8_t g, uint8_t b);

/*
 * 查询 SK6812 是否正在发送 800kHz 位时序。
 *
 * 返回值：
 * - true：TIMG12 中断仍在输出数据位，此时不要切换到蜂鸣器 PWM 模式。
 * - false：当前没有 SK6812 传输进行。
 */
bool sk6812_is_busy(void);

/*
 * 查询 PA15 是否已经被锁定为 RGB 空闲状态。
 *
 * 返回值：
 * - true：RGB 发码结束后 PA15 已锁定，不会被蜂鸣器低频 PWM 干扰。
 * - false：尚未完成锁定，蜂鸣器不应接管 TIMG12 周期。
 */
bool sk6812_rgb_is_locked(void);

#endif
