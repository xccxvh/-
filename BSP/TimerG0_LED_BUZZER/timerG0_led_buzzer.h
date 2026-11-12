#ifndef TIMERG0_LED_BUZZER_H
#define TIMERG0_LED_BUZZER_H

/*
 * TIMG0 蜂鸣器与指示灯 PWM 模块头文件。
 * 对外提供初始化接口和统一的占空比、频率设置接口。
 */
#include "bsp_common.h"

void timg0_led_buzzer_init(void);
void set_led_buzzer(uint16_t hz,uint16_t led_duty,uint16_t buzzer_duty);

#endif