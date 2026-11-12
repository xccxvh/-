#ifndef BSP_H
#define BSP_H

/*
 * BSP 聚合头文件。
 *
 * 使用范围：
 * - 仅建议应用层文件包含本头文件，用于一次性接入多个板级驱动模块。
 * - BSP 内部模块头文件不得包含本文件，避免模块之间形成隐式依赖。
 */
#include "bsp_common.h"

#include "SystemTime/system_time.h"
#include "UART0/uart0.h"
#include "SPI1/spi1.h"
#include "IMU/IMU/IMU.h"
#include "LED/led.h"
#include "SK6812/sk6812.h"
#include "SK6812/sk6812_demo.h"
#include "Buzzer/buzzer.h"
#include "PWM1_Shared/pwm1_shared.h"
#include "SPI0_LCD/lcd.h"
#include "SPI0_LCD/lcd_init.h"
#include "KEY/user_key.h"
#include "MOTOR/drv8874.h"
#include "MOTOR/motor_qei.h"
#include "MOTOR/motor_ui.h"
#include "WiFi/wb2_wifi.h"
#include "Bluetooth/hc05_bt.h"
#include "STS3032/sts3032.h"
#include "CAN/can_bus.h"

#endif
