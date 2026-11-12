#include "SPI0_OLED/spi0_oled.h"
#include "SPI0_LCD/lcd.h"
#include "SPI0_LCD/lcd_init.h"

/*
 * 显示层适配说明：
 *
 * 1. 原工程使用 SSD1306 单色 OLED，但当前核心板实际配套的是 1.69 寸
 *    ST7789 TFT 彩屏，两者控制器和初始化命令完全不同，直接沿用 OLED
 *    驱动会导致屏幕不亮。
 * 2. 厂家资料 Code/EVM_TEST_OLED/BSP/SPI0_LCD 中提供了这块 TFT 的
 *    完整驱动，本文件把厂家 LCD_* 接口包装成原来的 OLED_* 接口，
 *    上层 main_h2.c / main.c 不需要改显示调用。
 * 3. 坐标换算：原 OLED 接口的 y 是“页”（1 页 = 8 像素），而 TFT 驱动的
 *    y 是像素坐标，因此这里统一使用 y * 8 转换；字体沿用 8x16。
 * 4. 背光：核心板原理图中 TFT 背光 BLK 接在 TIMG6 的 PA30 上，
 *    SysConfig 默认只给 5% 占空比，屏幕会很暗。这里在初始化时把
 *    PA30（CC1）设为 100%，同时把 PA29（CC0，RGB/复用脚）清零。
 */

/* 原 SSD1306 驱动已备份为 spi0_oled_ssd1306.c.bak，不再参与编译。 */

/*
 * 兼容接口：原 OLED 的 SPI 初始化。
 * TFT 的复位、SPI 配置和初始化命令全部由 LCD_Init() 完成，
 * 这里保留空实现是为了不破坏上层可能存在的调用。
 */
void OLED_spi0_init(void)
{
}

/*
 * 兼容接口：原 OLED 的单字节写入。
 * TFT 驱动内部使用 LCD_Writ_Bus()，不再需要这个入口，
 * 保留空实现避免旧代码链接失败。
 */
void OLED_WR_Byte(u8 dat, u8 cmd)
{
    (void)dat;
    (void)cmd;
}

/*
 * 显示初始化。
 *
 * 执行顺序：
 * 1. 调用厂家 LCD_Init()，完成 ST7789 复位和初始化序列；
 * 2. TIMG6 在核心板定义里是 RGB/TFT 背光，但车底板可能复用这些引脚；
 *    为避免上电时影响轮子或其他功能，这里先把 CC0/CC1 清零并启动计数器。
 * 3. 车底板 1.69 寸 TFT 的背光实际接在排线的 PA7(BLK) 上，
 *    由 LCD_GPIO_Init() 拉高点亮，不再经过 TIMG6。
 */
void OLED_Init(void)
{
    LCD_Init();

    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
    DL_TimerG_startCounter(TimerG6_PWM_INST);

    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
}

/* 清屏：整屏填黑。 */
void OLED_Clear(void)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
}

/* 打开显示：TFT 初始化时已经打开，这里保留空实现。 */
void OLED_Display_On(void)
{
}

/* 关闭显示：直接清黑，等效于关闭画面。 */
void OLED_Display_Off(void)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
}

/* 画点：t 非 0 画白点，否则画黑点。 */
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    LCD_DrawPoint(x, y, (t != 0U) ? WHITE : BLACK);
}

/* 区域填充：dot 非 0 填白，否则填黑。 */
void OLED_Fill(u8 x1, u8 y1, u8 x2, u8 y2, u8 dot)
{
    LCD_Fill(x1, y1, x2, y2, (dot != 0U) ? WHITE : BLACK);
}

/*
 * 显示一个 8x16 ASCII 字符。
 * 原 OLED 的 y 是页地址，TFT 需要像素坐标，所以乘 8。
 */
void OLED_ShowChar(u8 x, u8 y, u8 chr)
{
    LCD_ShowChar(x, (u16)((u16)y * 8U), chr, WHITE, BLACK, 16U, 0U);
}

/*
 * 显示整数。
 * 原接口支持 size2 选择字体，这里统一按 16 像素高字体显示；
 * 数字范围与 TFT 驱动的 u16 形参一致，超出请改用字符串接口。
 */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size2)
{
    (void)size2;
    LCD_ShowIntNum(x, (u16)((u16)y * 8U), (u16)num, len, WHITE, BLACK, 16U);
}

/* 显示字符串，坐标换算同 OLED_ShowChar。 */
void OLED_ShowString(u8 x, u8 y, u8 *p)
{
    LCD_ShowString(x, (u16)((u16)y * 8U), p, WHITE, BLACK, 16U, 0U);
}

/* 兼容接口：TFT 驱动按坐标直接寻址，不需要单独的 Set_Pos。 */
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    (void)x;
    (void)y;
}

/*
 * 兼容接口：汉字显示。
 * 当前要求 2 的界面全部使用 ASCII，暂不提供汉字字库，
 * 保留空实现避免旧代码链接失败。
 */
void OLED_ShowCHinese(u8 x, u8 y, u8 no)
{
    (void)x;
    (void)y;
    (void)no;
}

/*
 * 兼容接口：位图显示。
 * 旧工程的位图接口基于 SSD1306 的单色页格式，和 TFT 的 RGB565 不兼容，
 * 这里保留空实现；需要显示图片时请直接使用 LCD_ShowPicture()。
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0,
                  unsigned char x1, unsigned char y1,
                  unsigned char BMP[])
{
    (void)x0;
    (void)y0;
    (void)x1;
    (void)y1;
    (void)BMP;
}
