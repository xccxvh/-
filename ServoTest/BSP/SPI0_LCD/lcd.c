#include "SPI0_LCD/lcd.h"
#include "SPI0_LCD/lcdfont.h"
#include "SPI0_LCD/lcd_init.h"

/*
 * ST7789 LCD 绘图层实现。
 *
 * 文件职责：
 * - 在 lcd_init.c 提供的寄存器/数据写入接口之上，实现点、线、矩形、圆、字符、
 *   字符串、图片和简单曲线绘制。
 * - 所有写屏函数均为阻塞式 SPI0 写入，适合主循环或初始化流程调用，不适合放在 ISR。
 *
 * 坐标约束：
 * - 坐标系由 lcd_init.h 中的 USE_HORIZONTAL、LCD_W、LCD_H 决定；当前工程为横屏 280x240。
 * - 本文件沿用原例程接口，不在每个绘图函数中做完整边界裁剪，调用者应保证坐标有效。
 */

void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
    u16 i;
    u16 j;

    /*
     * xend/yend 按半开区间处理，因此写入窗口右下角需要减 1。
     * 窗口设置完成后，连续写入 RGB565 像素即可填满该区域。
     */
    LCD_Address_Set(xsta, ysta, (u16)(xend - 1U), (u16)(yend - 1U));
    for (i = ysta; i < yend; i++) {
        for (j = xsta; j < xend; j++) {
            LCD_WR_DATA(color);
        }
    }
}

void LCD_DrawPoint(u16 x, u16 y, u16 color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    u16 t;
    int xerr = 0;
    int yerr = 0;
    int delta_x;
    int delta_y;
    int distance;
    int incx;
    int incy;
    int uRow;
    int uCol;

    /*
     * Bresenham 直线算法：
     * - 先把任意方向的起止点转换为 x/y 方向步进。
     * - 再按主轴距离逐点绘制，避免使用浮点数和除法。
     */
    delta_x = (int)x2 - (int)x1;
    delta_y = (int)y2 - (int)y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0) {
        incx = 1;
    } else if (delta_x == 0) {
        incx = 0;
    } else {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0) {
        incy = 1;
    } else if (delta_y == 0) {
        incy = 0;
    } else {
        incy = -1;
        delta_y = -delta_y;
    }

    distance = (delta_x > delta_y) ? delta_x : delta_y;
    for (t = 0; t < (u16)(distance + 1); t++) {
        LCD_DrawPoint((u16)uRow, (u16)uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color)
{
    int a = 0;
    int b = r;

    /*
     * 使用八分圆对称绘制空心圆。
     * a 从 0 递增，b 根据半径误差递减，避免浮点三角函数。
     */
    while (a <= b) {
        LCD_DrawPoint((u16)(x0 - b), (u16)(y0 - a), color);
        LCD_DrawPoint((u16)(x0 + b), (u16)(y0 - a), color);
        LCD_DrawPoint((u16)(x0 - a), (u16)(y0 + b), color);
        LCD_DrawPoint((u16)(x0 - a), (u16)(y0 - b), color);
        LCD_DrawPoint((u16)(x0 + b), (u16)(y0 + a), color);
        LCD_DrawPoint((u16)(x0 + a), (u16)(y0 - b), color);
        LCD_DrawPoint((u16)(x0 + a), (u16)(y0 + b), color);
        LCD_DrawPoint((u16)(x0 - b), (u16)(y0 + a), color);

        a++;
        if (((a * a) + (b * b)) > ((int)r * (int)r)) {
            b--;
        }
    }
}

void LCD_ShowChinese(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    while (*s != 0U) {
        if (sizey == 12U) {
            LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, mode);
        } else if (sizey == 16U) {
            LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, mode);
        } else if (sizey == 24U) {
            LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, mode);
        } else if (sizey == 32U) {
            LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, mode);
        } else {
            return;
        }

        /* 中文字库按 GBK 双字节索引，每显示一个字移动 2 字节和一个字宽。 */
        s += 2;
        x = (u16)(x + sizey);
    }
}

void LCD_ShowChinese12x12(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 i;
    u8 j;
    u8 m = 0;
    u16 k;
    u16 HZnum;
    u16 TypefaceNum;
    u16 x0 = x;

    TypefaceNum = (u16)((sizey / 8U + ((sizey % 8U) ? 1U : 0U)) * sizey);
    HZnum = (u16)(sizeof(tfont12) / sizeof(typFNT_GB12));
    for (k = 0; k < HZnum; k++) {
        if ((tfont12[k].Index[0] == *s) && (tfont12[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, (u16)(x + sizey - 1U), (u16)(y + sizey - 1U));
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8U; j++) {
                    if (mode == 0U) {
                        LCD_WR_DATA((tfont12[k].Msk[i] & (0x01U << j)) ? fc : bc);
                        m++;
                        if ((m % sizey) == 0U) {
                            m = 0U;
                            break;
                        }
                    } else {
                        if (tfont12[k].Msk[i] & (0x01U << j)) {
                            LCD_DrawPoint(x, y, fc);
                        }
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
    }
}

void LCD_ShowChinese16x16(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 i;
    u8 j;
    u8 m = 0;
    u16 k;
    u16 HZnum;
    u16 TypefaceNum;
    u16 x0 = x;

    TypefaceNum = (u16)((sizey / 8U + ((sizey % 8U) ? 1U : 0U)) * sizey);
    HZnum = (u16)(sizeof(tfont16) / sizeof(typFNT_GB16));
    for (k = 0; k < HZnum; k++) {
        if ((tfont16[k].Index[0] == *s) && (tfont16[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, (u16)(x + sizey - 1U), (u16)(y + sizey - 1U));
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8U; j++) {
                    if (mode == 0U) {
                        LCD_WR_DATA((tfont16[k].Msk[i] & (0x01U << j)) ? fc : bc);
                        m++;
                        if ((m % sizey) == 0U) {
                            m = 0U;
                            break;
                        }
                    } else {
                        if (tfont16[k].Msk[i] & (0x01U << j)) {
                            LCD_DrawPoint(x, y, fc);
                        }
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
    }
}

void LCD_ShowChinese24x24(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 i;
    u8 j;
    u8 m = 0;
    u16 k;
    u16 HZnum;
    u16 TypefaceNum;
    u16 x0 = x;

    TypefaceNum = (u16)((sizey / 8U + ((sizey % 8U) ? 1U : 0U)) * sizey);
    HZnum = (u16)(sizeof(tfont24) / sizeof(typFNT_GB24));
    for (k = 0; k < HZnum; k++) {
        if ((tfont24[k].Index[0] == *s) && (tfont24[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, (u16)(x + sizey - 1U), (u16)(y + sizey - 1U));
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8U; j++) {
                    if (mode == 0U) {
                        LCD_WR_DATA((tfont24[k].Msk[i] & (0x01U << j)) ? fc : bc);
                        m++;
                        if ((m % sizey) == 0U) {
                            m = 0U;
                            break;
                        }
                    } else {
                        if (tfont24[k].Msk[i] & (0x01U << j)) {
                            LCD_DrawPoint(x, y, fc);
                        }
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
    }
}

void LCD_ShowChinese32x32(u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 i;
    u8 j;
    u8 m = 0;
    u16 k;
    u16 HZnum;
    u16 TypefaceNum;
    u16 x0 = x;

    TypefaceNum = (u16)((sizey / 8U + ((sizey % 8U) ? 1U : 0U)) * sizey);
    HZnum = (u16)(sizeof(tfont32) / sizeof(typFNT_GB32));
    for (k = 0; k < HZnum; k++) {
        if ((tfont32[k].Index[0] == *s) && (tfont32[k].Index[1] == *(s + 1))) {
            LCD_Address_Set(x, y, (u16)(x + sizey - 1U), (u16)(y + sizey - 1U));
            for (i = 0; i < TypefaceNum; i++) {
                for (j = 0; j < 8U; j++) {
                    if (mode == 0U) {
                        LCD_WR_DATA((tfont32[k].Msk[i] & (0x01U << j)) ? fc : bc);
                        m++;
                        if ((m % sizey) == 0U) {
                            m = 0U;
                            break;
                        }
                    } else {
                        if (tfont32[k].Msk[i] & (0x01U << j)) {
                            LCD_DrawPoint(x, y, fc);
                        }
                        x++;
                        if ((x - x0) == sizey) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
    }
}

void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 temp;
    u8 sizex;
    u8 t;
    u8 m = 0;
    u16 i;
    u16 TypefaceNum;
    u16 x0 = x;

    sizex = (u8)(sizey / 2U);
    TypefaceNum = (u16)((sizex / 8U + ((sizex % 8U) ? 1U : 0U)) * sizey);
    num = (u8)(num - ' ');
    LCD_Address_Set(x, y, (u16)(x + sizex - 1U), (u16)(y + sizey - 1U));

    for (i = 0; i < TypefaceNum; i++) {
        if (sizey == 12U) {
            temp = ascii_1206[num][i];
        } else if (sizey == 16U) {
            temp = ascii_1608[num][i];
        } else if (sizey == 24U) {
            temp = ascii_2412[num][i];
        } else if (sizey == 32U) {
            temp = ascii_3216[num][i];
        } else {
            return;
        }

        for (t = 0; t < 8U; t++) {
            if (mode == 0U) {
                LCD_WR_DATA((temp & (0x01U << t)) ? fc : bc);
                m++;
                if ((m % sizex) == 0U) {
                    m = 0U;
                    break;
                }
            } else {
                if (temp & (0x01U << t)) {
                    LCD_DrawPoint(x, y, fc);
                }
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    while (*p != '\0') {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x = (u16)(x + (sizey / 2U));
        p++;
    }
}

u32 mypow(u8 m, u8 n)
{
    u32 result = 1U;

    while (n-- != 0U) {
        result *= m;
    }

    return result;
}

void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 t;
    u8 temp;
    u8 enshow = 0U;
    u8 sizex = (u8)(sizey / 2U);

    for (t = 0; t < len; t++) {
        temp = (u8)((num / mypow(10U, (u8)(len - t - 1U))) % 10U);
        if ((enshow == 0U) && (t < (len - 1U))) {
            if (temp == 0U) {
                LCD_ShowChar((u16)(x + t * sizex), y, ' ', fc, bc, sizey, 0U);
                continue;
            }
            enshow = 1U;
        }
        LCD_ShowChar((u16)(x + t * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0U);
    }
}

void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 t;
    u8 temp;
    u8 sizex = (u8)(sizey / 2U);
    u16 num1;

    /*
     * 原例程按 num*100 取整后插入小数点，函数名虽写 FloatNum1，但实际显示两位
     * 缩放后的十进制数字。这里保留算法，避免改变旧页面显示结果。
     */
    num1 = (u16)(num * 100.0f);
    for (t = 0; t < len; t++) {
        temp = (u8)((num1 / mypow(10U, (u8)(len - t - 1U))) % 10U);
        if (t == (u8)(len - 2U)) {
            LCD_ShowChar((u16)(x + (len - 2U) * sizex), y, '.', fc, bc, sizey, 0U);
            t++;
            len++;
        }
        LCD_ShowChar((u16)(x + t * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0U);
    }
}

void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[])
{
    u16 i;
    u16 j;
    u32 k = 0U;

    LCD_Address_Set(x, y, (u16)(x + length - 1U), (u16)(y + width - 1U));
    for (i = 0; i < length; i++) {
        for (j = 0; j < width; j++) {
            LCD_WR_DATA8(pic[k * 2U]);
            LCD_WR_DATA8(pic[k * 2U + 1U]);
            k++;
        }
    }
}

/*
 * 简单曲线示波区域配置。
 * 这组接口是旧 LCD 例程保留下来的临时调试工具，当前外设测试主界面没有默认启用。
 */
#define GRAPH_START_X       (10U)
#define GRAPH_END_X         (130U)
#define GRAPH_START_Y       (30U)
#define GRAPH_END_Y         (210U)
#define GRAPH_WIDTH         (GRAPH_END_X - GRAPH_START_X)
#define GRAPH_HEIGHT        (GRAPH_END_Y - GRAPH_START_Y)
#define DATA_BUFFER_SIZE    (60U)
#define DATA_MAX            (50U)
#define DATA_MIN            (0U)
#define BACKGROUND_COLOR    WHITE
#define CURVE_COLOR         BLUE
#define AXIS_COLOR          BLACK
#define GRID_COLOR          BLACK
#define TEXT_COLOR          BLACK

static u16 data_buffer[DATA_BUFFER_SIZE] = {0};
static u16 buffer_index = 0U;

static void Map_Data_To_Point(u16 data_index, u16 data_value, u16 *x, u16 *y)
{
    u16 mapped_y;

    /*
     * X 方向按采样序号均匀铺开；Y 方向把数值映射到曲线区域，LCD 坐标向下递增，
     * 因此数值越大，映射后的 y 越靠上。
     */
    *x = (u16)(GRAPH_START_X + (data_index * GRAPH_WIDTH) / (DATA_BUFFER_SIZE - 1U));
    mapped_y = (u16)(GRAPH_END_Y -
                     ((data_value - DATA_MIN) * GRAPH_HEIGHT) / (DATA_MAX - DATA_MIN));

    if (mapped_y < GRAPH_START_Y) {
        mapped_y = GRAPH_START_Y;
    }
    if (mapped_y > GRAPH_END_Y) {
        mapped_y = GRAPH_END_Y;
    }

    *y = mapped_y;
}

void Draw_Grid_And_Axis(void)
{
    u16 i;

    LCD_DrawRectangle(GRAPH_START_X, GRAPH_START_Y, GRAPH_END_X, GRAPH_END_Y, AXIS_COLOR);

    /* 绘制 5 条水平网格线，并在右侧显示刻度值。 */
    for (i = 0U; i < 5U; i++) {
        u16 y = (u16)(GRAPH_START_Y + (i * GRAPH_HEIGHT) / 5U);
        u16 value = (u16)(DATA_MAX - (i * (DATA_MAX - DATA_MIN)) / 5U);

        LCD_DrawLine(GRAPH_START_X, y, GRAPH_END_X, y, GRID_COLOR);
        LCD_ShowIntNum((u16)(GRAPH_END_X + 5U), (u16)(y - 8U),
                       value, 3U, TEXT_COLOR, BACKGROUND_COLOR, 16U);
    }
}

void Add_New_Data(u16 new_data)
{
    u16 i;

    if (new_data > DATA_MAX) {
        new_data = DATA_MAX;
    }
    if (new_data < DATA_MIN) {
        new_data = DATA_MIN;
    }

    /*
     * 旧实现使用线性缓冲：满后整体左移一个采样点，再把最新值放到末尾。
     * 数据量只有 60 点，移动成本很小，适合临时调试曲线。
     */
    if (buffer_index >= DATA_BUFFER_SIZE) {
        for (i = 1U; i < DATA_BUFFER_SIZE; i++) {
            data_buffer[i - 1U] = data_buffer[i];
        }
        buffer_index = (u16)(DATA_BUFFER_SIZE - 1U);
    }

    data_buffer[buffer_index] = new_data;
    buffer_index++;
}

void Draw_Data_Curve(void)
{
    u16 x_prev = 0U;
    u16 y_prev = 0U;
    u16 x_curr;
    u16 y_curr;
    u16 i;

    LCD_Fill((u16)(GRAPH_START_X + 1U), (u16)(GRAPH_START_Y + 1U),
             (u16)(GRAPH_END_X - 1U), (u16)(GRAPH_END_Y - 1U), BACKGROUND_COLOR);

    for (i = 0U; i < buffer_index; i++) {
        Map_Data_To_Point(i, data_buffer[i], &x_curr, &y_curr);

        if (i < (u16)(buffer_index - 1U)) {
            LCD_Fill((u16)(x_curr - 1U), (u16)(y_curr - 1U),
                     (u16)(x_curr + 1U), (u16)(y_curr + 1U), CURVE_COLOR);
        }

        if (i > 0U) {
            LCD_DrawLine(x_prev, y_prev, x_curr, y_curr, CURVE_COLOR);
        }

        x_prev = x_curr;
        y_prev = y_curr;
    }
}

void RingBuffer_Add_Data(u16 new_data)
{
    /*
     * 兼容旧头文件接口。当前实现仍复用线性缓冲，保持行为和 Add_New_Data() 一致。
     */
    Add_New_Data(new_data);
}

void Optimized_Draw_Data_Curve(void)
{
    /*
     * 兼容旧头文件接口。后续若需要优化为局部滚动刷新，可只替换本函数实现。
     */
    Draw_Data_Curve();
}
