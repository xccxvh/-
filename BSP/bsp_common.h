#ifndef BSP_COMMON_H
#define BSP_COMMON_H

/*
 * 公共基础头文件。
 * 作用：
 * 1. 统一收口标准库、MSPM0 底层库、SysConfig 生成配置和基础类型别名。
 * 2. 作为 BSP 各模块头文件的共同依赖，避免各模块头文件再反向包含聚合头 bsp.h。
 * 3. 统一声明跨模块都会直接用到的公共接口，例如延时函数。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "ti_msp_dl_config.h"

/*
 * 保留工程历史中已经广泛使用的类型缩写，避免整理头文件时引发大面积连锁修改。
 */
#define u8  unsigned char
#define u16 uint16_t
#define u32 unsigned int

/*
 * 多个模块会在头文件宏中直接调用延时函数，例如 OLED 复位、软件 I2C 时序等。
 * 因此把声明提升到公共头文件中，具体实现仍然由 UART0 模块提供。
 */
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif