#include "bsp.h"

/* 主机测试不需要真实 LCD，所有绘图层接口都做成空实现。 */

void LCD_Init(void)
{
}

void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
    (void)xsta;
    (void)ysta;
    (void)xend;
    (void)yend;
    (void)color;
}

void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    (void)x;
    (void)y;
    (void)p;
    (void)fc;
    (void)bc;
    (void)sizey;
    (void)mode;
}
