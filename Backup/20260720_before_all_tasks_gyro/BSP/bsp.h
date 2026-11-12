#ifndef BSP_H
#define BSP_H

/*
 * BSP 聚合头文件。
 * 使用规则：
 * 1. 仅在应用层需要一次性接入全部板级模块时包含本文件。
 * 2. 各模块头文件禁止反向包含本文件，避免形成重复依赖和循环引用。
 * 3. 具体模块源码优先包含自己对应的模块头文件，而不是直接包含本文件。
 */

#include "bsp_common.h"

#include "LED/led.h"
#include "UART0/uart0.h"
#include "TimerG6_PWM_RGB/timerG6_pwm_rgb.h"
#include "KEY/key.h"
#include "KeyBoard/keyboard.h"
#include "SPI0_OLED/spi0_oled.h"
#include "TimerA1/TimerA1.h"
#include "SPI1/spi1.h"
#include "IMU.h"
#include "ADC0/adc0.h"
#include "W25Q64/w25q64.h"
#include "MYI2C1/myi2c.h"
#include "MPU6050/mpu6050.h"
#include "TIMERG0_LED_BUZZER/timerG0_led_buzzer.h"
#include "MOTOR/motor.h"
#include "Infrared/infrared.h"

#endif
