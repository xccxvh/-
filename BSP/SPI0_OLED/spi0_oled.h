#ifndef OLED_SPI0_H
#define OLED_SPI0_H

/*
 * OLED 显示模块头文件。
 * 该模块负责 SSD1306 类 SPI OLED 的控制引脚宏、基础显示参数和对外接口声明。
 */
#include "bsp_common.h"

#define OLED_CMD 0
#define OLED_DATA 1
#define OLED_MODE 0

/*
 * OLED 的 RES、DC、CS 在 SysConfig 中分别生成了独立的端口宏，
 * 因此这里按信号分别使用对应的 *_PORT 和 *_PIN，避免再假设存在统一的 OLED_PORT。
 */
#define OLED_CS_Clr() DL_GPIO_clearPins(OLED_CS_PORT, OLED_CS_PIN)
#define OLED_CS_Set() DL_GPIO_setPins(OLED_CS_PORT, OLED_CS_PIN)

#define OLED_RST_Clr() DL_GPIO_clearPins(OLED_RES_PORT, OLED_RES_PIN)
#define OLED_RST_Set() DL_GPIO_setPins(OLED_RES_PORT, OLED_RES_PIN)

#define OLED_DC_Clr() DL_GPIO_clearPins(OLED_DC_PORT, OLED_DC_PIN)
#define OLED_DC_Set() DL_GPIO_setPins(OLED_DC_PORT, OLED_DC_PIN)

#define SIZE 16
#define XLevelL 0x02
#define XLevelH 0x10
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xFF
#define X_WIDTH 128
#define Y_WIDTH 64

void OLED_spi0_init(void);
void OLED_WR_Byte(u8 dat, u8 cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x, u8 y, u8 t);
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot);
void OLED_ShowChar(u8 x, u8 y, u8 chr);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2);
void OLED_ShowString(u8 x, u8 y, u8 *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x, u8 y, u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);

#endif