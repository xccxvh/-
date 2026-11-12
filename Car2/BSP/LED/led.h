#ifndef LED_H
#define LED_H

/*
 * LED 模块头文件。
 * 这里只封装板载 LED 的控制宏，因此仅依赖公共头文件即可。
 */
#include "bsp_common.h"
	
//#define LED_L2_PORT LED_L2_PORT
//#define LED_L1_PORT LED_L1_PORT

/* 控制 LED1 输出高低电平。 */
#define LED1(en)                                      \
    if (en)                                           \
        DL_GPIO_setPins(LED_L1_PORT, LED_L1_PIN);     \
    else                                              \
        DL_GPIO_clearPins(LED_L1_PORT, LED_L1_PIN);

/* 翻转 LED1 当前电平。 */
#define LED1_toggle DL_GPIO_togglePins(LED_L1_PORT, LED_L1_PIN);

/* 控制 LED2 输出高低电平。 */
#define LED2(en)                                      \
    if (en)                                           \
        DL_GPIO_setPins(LED_L2_PORT, LED_L2_PIN);     \
    else                                              \
        DL_GPIO_clearPins(LED_L2_PORT, LED_L2_PIN);

/* 翻转 LED2 当前电平。 */
#define LED2_toggle DL_GPIO_togglePins(LED_L2_PORT, LED_L2_PIN);

/*
 * 初始化板载 LED 的安全输出状态。
 *
 * 调用时机：
 * - 应在 SYSCFG_DL_init() 之后调用，确保 LED_L1/LED_L2 的 GPIO 输出配置已经生效。
 *
 * 输出约束：
 * - 函数会把 LED1/LED2 置为熄灭状态，避免上电后测试页面切换前出现误亮。
 */
void LED_init(void);

#endif
