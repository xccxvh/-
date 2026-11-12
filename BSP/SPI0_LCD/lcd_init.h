#ifndef __LCD_INIT_H
#define __LCD_INIT_H

/*
 * 复用车厂 LCD 驱动时，需要用到 u8/u16/u32 类型和 GPIO 宏，
 * 因此改为包含本工程的公共头 bsp_common.h，而不是只包含 stdint.h。
 */
#include "bsp_common.h"

/*
 * 车底板 1.69 寸 TFT 排线（丝印 FP-169HSC01A）的实际接线：
 *   GND, 3V3, PB18(SCL), PB17(SDA), PC0(RES), PA14(DC), GND(CS), PA7(BLK)
 *
 * 说明：
 * 1. 该屏不使用核心板 SPI0（PB2/PB3/PC8/PC9/PB23）那组引脚，
 *    因此这里改成普通 GPIO + 软件 SPI（bit-bang）。
 * 2. CS 在排线上接地，恒选中，软件不再控制。
 * 3. 背光 BLK 接 PA7，高电平点亮。
 * 4. PA14 同时是核心板 LED1，TimerA1 的 LED1 心跳必须关闭，
 *    否则每 500 ms 翻转一次会破坏 DC 时序（见 TimerA1.c）。
 */
#define LCD_RES_PORT   GPIOC
#define LCD_RES_PIN    DL_GPIO_PIN_0
#define LCD_DC_PORT    GPIOA
#define LCD_DC_PIN     DL_GPIO_PIN_14

#define LCD_SCL_PORT   GPIOB
#define LCD_SCL_PIN    DL_GPIO_PIN_18
#define LCD_SDA_PORT   GPIOB
#define LCD_SDA_PIN    DL_GPIO_PIN_17
#define LCD_BLK_PORT   GPIOA
#define LCD_BLK_PIN    DL_GPIO_PIN_7

#define LCD_SCL_Clr()  DL_GPIO_clearPins(LCD_SCL_PORT, LCD_SCL_PIN)
#define LCD_SCL_Set()  DL_GPIO_setPins(LCD_SCL_PORT, LCD_SCL_PIN)
#define LCD_SDA_Clr()  DL_GPIO_clearPins(LCD_SDA_PORT, LCD_SDA_PIN)
#define LCD_SDA_Set()  DL_GPIO_setPins(LCD_SDA_PORT, LCD_SDA_PIN)

/* IOMUX 编号：PB18=44、PB17=43、PC0=74、PA14=36、PA7=14。 */
#define LCD_SCL_IOMUX  IOMUX_PINCM44
#define LCD_SDA_IOMUX  IOMUX_PINCM43
#define LCD_RES_IOMUX  IOMUX_PINCM74
#define LCD_DC_IOMUX   IOMUX_PINCM36
#define LCD_BLK_IOMUX  IOMUX_PINCM14

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif


//-----------------LCD端口定义---------------- 

//#define LCD_SCLK_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_12)//SCL=SCLK
//#define LCD_SCLK_Set() GPIO_SetBits(GPIOG,GPIO_Pin_12)

//#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_5)//SDA=MOSI
//#define LCD_MOSI_Set() GPIO_SetBits(GPIOD,GPIO_Pin_5)

#define LCD_RES_Clr()  DL_GPIO_clearPins(LCD_RES_PORT,LCD_RES_PIN)//RES
#define LCD_RES_Set()  DL_GPIO_setPins(LCD_RES_PORT,LCD_RES_PIN)

#define LCD_DC_Clr()   DL_GPIO_clearPins(LCD_DC_PORT,LCD_DC_PIN)//DC
#define LCD_DC_Set()   DL_GPIO_setPins(LCD_DC_PORT,LCD_DC_PIN)
 		     
/* CS 在排线上接地，恒选中；保留空宏兼容厂家驱动的调用点。 */
#define LCD_CS_Clr()   do { } while (0)
#define LCD_CS_Set()   do { } while (0)

/* 背光 BLK：PA7，高电平点亮。 */
#define LCD_BLK_Clr()  DL_GPIO_clearPins(LCD_BLK_PORT, LCD_BLK_PIN)
#define LCD_BLK_Set()  DL_GPIO_setPins(LCD_BLK_PORT, LCD_BLK_PIN)


void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




