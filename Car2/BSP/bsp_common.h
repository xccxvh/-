#ifndef BSP_COMMON_H
#define BSP_COMMON_H

/*
 * BSP 公共基础头文件。
 *
 * 职责：
 * 1. 统一收口 C 标准库、MSPM0 DriverLib、CMSIS 内核接口和 SysConfig 生成的硬件宏。
 * 2. 作为 BSP 各模块头文件的公共依赖，避免模块头文件反向包含聚合头 bsp.h。
 * 3. 统一声明跨模块都会用到的基础接口，例如 UART0 中实现的阻塞延时函数。
 *
 * 约束：
 * - 本文件只能放公共基础能力，不能把某个具体 BSP 外设模块的业务接口放进来。
 * - 新增 BSP 模块头文件应默认包含本文件，而不是直接包含 bsp.h。
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
 * 毫秒级阻塞延时。
 *
 * 参数：
 * - ms：需要阻塞等待的毫秒数。
 *
 * 实现位置：
 * - 由 BSP/UART0/uart0.c 提供，公共声明放在这里便于 SPI、IMU、LCD 等底层模块使用。
 *
 * 使用约束：
 * - 该函数为忙等阻塞，不应在中断服务函数中调用。
 */
void delay_ms(uint32_t ms);

/*
 * 微秒级阻塞延时。
 *
 * 参数：
 * - us：需要阻塞等待的微秒数。
 *
 * 用途：
 * - IMU 官方驱动初始化、寄存器写入后的硬件稳定等待，以及 LCD 复位等短延时场景。
 *
 * 使用约束：
 * - 该函数为忙等阻塞，时间较长时会影响主循环实时性，不应在 ISR 中调用。
 */
void delay_us(uint32_t us);

#endif
