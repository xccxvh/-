#ifndef __LCD_INIT_H
#define __LCD_INIT_H

/*
 * ST7789 彩色 LCD 底层驱动头文件。
 *
 * 硬件连接由 User/config.syscfg 生成的 LCD_* 与 SPI_0_* 宏描述：
 * - SPI0 当前用于 LCD 串行写入，SCLK/MOSI 由 SysConfig 绑定。
 * - BLK 为背光控制，DC 为命令/数据选择，RES 为硬件复位（当前 PC0）。
 * - 当前 DRV8874 测试工程没有单独配置 LCD_CS，默认认为 LCD 片选硬件接低；
 *   如果后续硬件把 CS 接到 MCU，并在 SysConfig 增加 LCD_CS_PIN，本文件会自动
 *   切换为 GPIO 片选控制，不需要改动发送函数。
 *
 * 约束说明：
 * - BSP 模块头文件只包含 bsp_common.h，避免直接依赖聚合头 bsp.h。
 * - LCD 初始化时序依赖 delay_ms()，调用前必须已经完成 SYSCFG_DL_init()。
 */

#include "bsp_common.h"

/*
 * BLK/DC 在 GPIOA、RES 在 GPIOC 时，SysConfig 会分别生成 *_PORT 宏。
 * 若某宏未生成，再回退到 LCD_PORT（同组引脚）。
 */
#ifndef LCD_BLK_PORT
#ifdef LCD_PORT
#define LCD_BLK_PORT LCD_PORT
#endif
#endif
#ifndef LCD_DC_PORT
#ifdef LCD_PORT
#define LCD_DC_PORT LCD_PORT
#endif
#endif
#ifndef LCD_RES_PORT
#ifdef LCD_PORT
#define LCD_RES_PORT LCD_PORT
#endif
#endif
#if defined(LCD_CS_PIN) && !defined(LCD_CS_PORT)
#define LCD_CS_PORT  LCD_PORT
#endif

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

/*
 * 屏幕方向配置：
 * - 0/1：竖屏，逻辑分辨率 240x280。
 * - 2/3：横屏，逻辑分辨率 280x240。
 * 当前测试界面按横屏排版，保持与原 LCD 例程一致的 USE_HORIZONTAL=2。
 */
#define USE_HORIZONTAL 2

#if (USE_HORIZONTAL == 0) || (USE_HORIZONTAL == 1)
#define LCD_W 240
#define LCD_H 280
#else
#define LCD_W 280
#define LCD_H 240
#endif

#define LCD_RES_Clr() DL_GPIO_clearPins(LCD_RES_PORT, LCD_RES_PIN)
#define LCD_RES_Set() DL_GPIO_setPins(LCD_RES_PORT, LCD_RES_PIN)

#define LCD_DC_Clr()  DL_GPIO_clearPins(LCD_DC_PORT, LCD_DC_PIN)
#define LCD_DC_Set()  DL_GPIO_setPins(LCD_DC_PORT, LCD_DC_PIN)

/*
 * 片选处理：
 * - 未生成 LCD_CS_PIN 时，认为屏幕 CS 已经硬件接低，Clr/Set 为空操作。
 * - 生成 LCD_CS_PIN 时，默认启用 GPIO 片选，发送每个字节前拉低、发送结束后拉高。
 */
#ifndef LCD_CS_HW_ALWAYS_LOW
#if defined(LCD_CS_PIN)
#define LCD_CS_HW_ALWAYS_LOW 0
#else
#define LCD_CS_HW_ALWAYS_LOW 1
#endif
#endif

#if LCD_CS_HW_ALWAYS_LOW
#define LCD_CS_Clr() ((void)0)
#define LCD_CS_Set() ((void)0)
#else
#define LCD_CS_Clr() DL_GPIO_clearPins(LCD_CS_PORT, LCD_CS_PIN)
#define LCD_CS_Set() DL_GPIO_setPins(LCD_CS_PORT, LCD_CS_PIN)
#endif

#define LCD_BLK_Clr() DL_GPIO_clearPins(LCD_BLK_PORT, LCD_BLK_PIN)
#define LCD_BLK_Set() DL_GPIO_setPins(LCD_BLK_PORT, LCD_BLK_PIN)

/*
 * 初始化 LCD 控制 GPIO 的空闲电平并执行硬复位时序。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后调用，确保 BLK/DC/RES/可选 CS GPIO 已配置为输出。
 *
 * 硬件约束：
 * - RES 会按 ST7789 上电要求拉低/拉高并延时，调用期间为阻塞式。
 * - 背光 BLK 初始化阶段先关闭，LCD_Init() 完成寄存器配置后再打开。
 */
void LCD_GPIO_Init(void);

/*
 * 通过 SPI0 向 LCD 写入 1 个原始字节。
 *
 * 参数：
 * - dat：要发送到 ST7789 的命令或数据字节。
 *
 * 时序约束：
 * - 函数会按当前 DC 状态发送字节，并用 LCD_CS_Clr/Set 包围传输。
 * - SPI0 为阻塞发送，确保字节移出后再返回。
 */
void LCD_Writ_Bus(u8 dat);

/*
 * 向 LCD 写入 8 位数据。
 *
 * 参数：
 * - dat：数据字节。
 *
 * 说明：
 * - 调用前应确保 DC 已处于数据状态；通常由上层图形函数间接使用。
 */
void LCD_WR_DATA8(u8 dat);

/*
 * 向 LCD 写入 16 位 RGB565 数据。
 *
 * 参数：
 * - dat：16 位数据，高字节先发送，低字节后发送。
 *
 * 用途：
 * - 主要用于写入 RGB565 像素颜色。
 */
void LCD_WR_DATA(u16 dat);

/*
 * 向 ST7789 写入 8 位寄存器命令。
 *
 * 参数：
 * - dat：寄存器命令码。
 *
 * 时序约束：
 * - 函数会临时拉低 DC 发送命令，发送完成后恢复 DC 为数据状态。
 */
void LCD_WR_REG(u8 dat);

/*
 * 设置后续显存写入窗口。
 *
 * 参数：
 * - x1/y1：窗口左上角坐标。
 * - x2/y2：窗口右下角坐标，包含该像素。
 *
 * 说明：
 * - 内部会根据 USE_HORIZONTAL 自动补偿 ST7789 可视窗口偏移。
 */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2);

/*
 * 完成 ST7789 屏幕初始化。
 *
 * 调用时机：
 * - 应在 SYSCFG_DL_init() 和 uart0/system_time 可用后调用，因底层依赖 delay_ms()。
 *
 * 行为：
 * - 初始化控制脚、执行硬复位、写入 ST7789 配置寄存器、设置屏幕方向并打开背光。
 */
void LCD_Init(void);

#endif
