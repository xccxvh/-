#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

/*
 * 4x4 矩阵键盘模块头文件。
 * 该模块在按键映射逻辑中会直接调用蜂鸣器 PWM 接口，
 * 因此这里显式包含蜂鸣器模块头文件，而不是继续依赖 bsp.h 的透传能力。
 */
#include "bsp_common.h"
#include "TIMERG0_LED_BUZZER/timerG0_led_buzzer.h"

#define KEY_H1_PORT                                                      (GPIOA)
#define KEY_H1_PIN                                              (DL_GPIO_PIN_17)
#define KEY_H1_IOMUX                                             (IOMUX_PINCM39)
#define KEY_H2_PORT                                                      (GPIOA)
#define KEY_H2_PIN                                              (DL_GPIO_PIN_22)
#define KEY_H2_IOMUX                                             (IOMUX_PINCM47)
#define KEY_H3_PORT                                                      (GPIOB)
#define KEY_H3_PIN                                              (DL_GPIO_PIN_10)
#define KEY_H3_IOMUX                                             (IOMUX_PINCM27)
#define KEY_H4_PORT                                                      (GPIOB)
#define KEY_H4_PIN                                              (DL_GPIO_PIN_11)
#define KEY_H4_IOMUX                                             (IOMUX_PINCM28)
#define KEY_V1_PORT                                                      (GPIOB)
#define KEY_V1_PIN                                              (DL_GPIO_PIN_20)
#define KEY_V1_IOMUX                                             (IOMUX_PINCM48)
#define KEY_V2_PORT                                                      (GPIOB)
#define KEY_V2_PIN                                              (DL_GPIO_PIN_24)
#define KEY_V2_IOMUX                                             (IOMUX_PINCM52)
#define KEY_V3_PORT                                                      (GPIOB)
#define KEY_V3_PIN                                              (DL_GPIO_PIN_25)
#define KEY_V3_IOMUX                                             (IOMUX_PINCM56)
#define KEY_V4_PORT                                                      (GPIOB)
#define KEY_V4_PIN                                              (DL_GPIO_PIN_27)
#define KEY_V4_IOMUX                                             (IOMUX_PINCM58)

void keyboard_init(void);
char get_keyboard_value(void);

#endif