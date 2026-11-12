#ifndef SPI0_LCD_LCD_H
#define SPI0_LCD_LCD_H

/*
 * LCD 绘图层接口。
 *
 * 文件职责：
 * - 在 ST7789 底层寄存器和 SPI 写字节接口之上，提供点、线、矩形、圆、字符、
 *   字符串、图片和简单曲线绘制函数。
 * - 坐标系、屏幕宽高、RGB565 颜色常量均与 lcd_init.h 中的 USE_HORIZONTAL/LCD_W/LCD_H
 *   保持一致；当前测试框架按横屏 280x240 使用。
 *
 * 调用约束：
 * - 必须先调用 SYSCFG_DL_init()，让 SPI0 与 LCD 控制 GPIO 生效。
 * - 必须先调用 LCD_Init()，完成 ST7789 复位、寄存器初始化和背光打开。
 * - 本绘图层为阻塞式 SPI 写屏接口，不适合在中断服务函数里调用；中断里只记录事件，
 *   主循环再刷新 LCD，避免长时间占用中断。
 *
 * 头文件规则：
 * - BSP 模块头文件只依赖公共基础头 bsp_common.h 和本 LCD 模块底层头 lcd_init.h，
 *   不包含聚合头 bsp.h。
 */
#include "bsp_common.h"
#include "SPI0_LCD/lcd_init.h"

/*
 * 用 RGB565 颜色填充矩形区域。
 *
 * 参数：
 * - xsta/ysta：填充区域左上角坐标。
 * - xend/yend：填充区域右下边界，函数按 [xsta,xend)、[ysta,yend) 半开区间绘制。
 * - color：填充颜色，格式 RGB565。
 *
 * 时序约束：
 * - 该函数会阻塞写入大量 SPI 数据，不应在中断服务函数中调用。
 */
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);

/*
 * 在指定坐标写入一个 RGB565 像素点。
 *
 * 参数：
 * - x/y：目标像素坐标，必须位于 LCD_W/LCD_H 范围内。
 * - color：像素颜色，格式 RGB565。
 */
void LCD_DrawPoint(u16 x, u16 y, u16 color);

/*
 * 绘制任意方向直线。
 *
 * 参数：
 * - x1/y1：直线起点坐标。
 * - x2/y2：直线终点坐标。
 * - color：线条颜色，格式 RGB565。
 *
 * 实现说明：
 * - 使用整数 Bresenham 算法逐点绘制，适合测试界面表格和简单几何图形。
 */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);

/*
 * 绘制空心矩形边框。
 *
 * 参数：
 * - x1/y1：矩形左上角坐标。
 * - x2/y2：矩形右下角坐标。
 * - color：边框颜色，格式 RGB565。
 */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);

/*
 * 绘制空心圆。
 *
 * 参数：
 * - x0/y0：圆心坐标。
 * - r：圆半径，单位像素。
 * - color：圆周颜色，格式 RGB565。
 *
 * 说明：
 * - 函数名 Draw_Circle 沿用原例程以兼容旧代码。
 */
void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color);

/*
 * 按 sizey 自动选择中文点阵字库并连续显示字符串。
 *
 * 参数：
 * - x/y：起始显示坐标。
 * - s：GBK 双字节中文字符串指针，每个汉字占 2 字节。
 * - fc/bc：前景色/背景色，格式 RGB565。
 * - sizey：字高，支持 12/16/24/32。
 * - mode：0 覆盖背景，1 叠加显示。
 */
void LCD_ShowChinese(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 显示一个 12x12 中文点阵字符。
 *
 * 参数：
 * - x/y：字符左上角坐标。
 * - s：指向 GBK 双字节字符。
 * - fc/bc：前景色/背景色，格式 RGB565。
 * - sizey：应传 12，用于和原例程接口保持一致。
 * - mode：0 覆盖背景，1 只绘制前景点。
 */
void LCD_ShowChinese12x12(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 显示一个 16x16 中文点阵字符。
 *
 * 参数含义与 LCD_ShowChinese12x12() 相同，sizey 应传 16。
 */
void LCD_ShowChinese16x16(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 显示一个 24x24 中文点阵字符。
 *
 * 参数含义与 LCD_ShowChinese12x12() 相同，sizey 应传 24。
 */
void LCD_ShowChinese24x24(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 显示一个 32x32 中文点阵字符。
 *
 * 参数含义与 LCD_ShowChinese12x12() 相同，sizey 应传 32。
 */
void LCD_ShowChinese32x32(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 显示单个 ASCII 字符。
 *
 * 参数：
 * - x/y：字符左上角坐标。
 * - num：ASCII 字符码，支持字库内的可打印字符。
 * - fc/bc：前景色/背景色，格式 RGB565。
 * - sizey：字高，支持 12/16/24/32，对应宽度为 sizey/2。
 * - mode：0 覆盖背景，1 叠加显示。
 */
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 显示以 '\0' 结尾的 ASCII 字符串。
 *
 * 参数：
 * - x/y：字符串起始坐标。
 * - p：ASCII 字符串指针。
 * - fc/bc：前景色/背景色，格式 RGB565。
 * - sizey：字高，字符宽度为 sizey/2。
 * - mode：0 覆盖背景，1 叠加显示。
 */
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode);

/*
 * 计算 m 的 n 次幂。
 *
 * 参数：
 * - m：底数。
 * - n：指数。
 *
 * 返回值：
 * - m^n 的无符号 32 位结果；用于数字显示时拆分十进制位。
 */
u32 mypow(u8 m, u8 n);

/*
 * 按指定宽度显示无符号整数。
 *
 * 参数：
 * - x/y：起始坐标。
 * - num：待显示整数。
 * - len：显示位宽，前导 0 会以空格显示。
 * - fc/bc：前景色/背景色。
 * - sizey：字高。
 */
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey);

/*
 * 按一位小数格式显示浮点数。
 *
 * 参数：
 * - x/y：起始坐标。
 * - num：待显示数值。
 * - len：整数+小数字符显示宽度，沿用原例程算法。
 * - fc/bc：前景色/背景色。
 * - sizey：字高。
 */
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey);

/*
 * 显示 RGB565 图片数据。
 *
 * 参数：
 * - x/y：图片左上角坐标。
 * - length：图片宽度，单位像素。
 * - width：图片高度，单位像素。
 * - pic：RGB565 高字节在前的连续图片数组。
 */
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[]);

/*
 * 绘制简单曲线示波区域的边框、横向网格和刻度值。
 *
 * 用途：
 * - 供临时观察 ADC/传感器曲线使用；当前外设测试主界面未默认启用。
 */
void Draw_Grid_And_Axis(void);

/*
 * 向简单曲线缓冲追加一个采样值。
 *
 * 参数：
 * - new_data：待加入的采样值，函数内部会限制到 DATA_MIN~DATA_MAX。
 */
void Add_New_Data(u16 new_data);

/*
 * 按当前曲线缓冲重绘数据曲线。
 *
 * 说明：
 * - 会先清除曲线绘制区域，再按缓冲内容重画采样点和连线。
 */
void Draw_Data_Curve(void);

/*
 * 向环形缓冲追加一个采样值。
 *
 * 说明：
 * - 保留给旧曲线优化例程使用；当前主测试框架没有默认调用。
 */
void RingBuffer_Add_Data(u16 new_data);

/*
 * 使用优化方式重绘数据曲线。
 *
 * 说明：
 * - 保留旧接口，便于后续把全量重绘改成局部滚动绘制。
 */
void Optimized_Draw_Data_Curve(void);

/* LCD 常用基础颜色，格式均为 RGB565。 */
#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0xF81F
#define GRED           0xFFE0
#define GBLUE          0x07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN          0xBC40
#define BRRED          0xFC07
#define GRAY           0x8430
#define DARKBLUE       0x01CF
#define LIGHTBLUE      0x7D7C
#define GRAYBLUE       0x5458
#define LIGHTGREEN     0x841F
#define LGRAY          0xC618
#define LGRAYBLUE      0xA651
#define LBBLUE         0x2B12

#endif
