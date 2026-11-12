#ifndef KEY_H
#define KEY_H

/*
 * 独立按键模块头文件。
 * 该模块除了需要按键 GPIO 定义外，还会在中断处理里直接翻转 LED2，
 * 因此这里显式包含 LED 模块头文件，避免依赖 bsp.h 的传递式包含。
 */
#include "bsp_common.h"
#include "LED/led.h"

#define KEY DL_GPIO_readPins(key_PORT, key_user_PIN)

void key_init(void);

#endif