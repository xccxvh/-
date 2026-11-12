#include "SPI0_LCD/lcd_init.h"

/*
 * ST7789 底层 SPI 写接口与寄存器初始化。
 *
 * 参考例程 C:\Users\kdong\Downloads\Car_sum 使用 SPI mode 3
 * (CPOL=1、CPHA=1)，当前工程已在 User/config.syscfg 中同步该模式。
 * LCD 只需要 MCU 向屏幕写命令/数据，因此 SPI0 配置为 PICO(MOSI-only)。
 */

static void lcd_spi0_write_byte(uint8_t data)
{
    /*
     * 阻塞发送保证字节已经从 SPI 移位寄存器完整发出后再返回。
     * 若 LCD_CS 是 GPIO 控制，这可以避免最后几个时钟还未结束就释放片选；
     * 若 CS 硬件常低，也能保证连续写命令/数据时 FIFO 不被上层写爆。
     */
    DL_SPI_transmitDataBlocking8(SPI_0_INST, data);
}

void LCD_GPIO_Init(void)
{
    /*
     * 初始化控制脚的空闲状态：
     * - CS 高电平为空闲；当前板若 CS 硬件接低，此宏为空操作。
     * - DC 高电平默认进入数据状态，发送命令时再临时拉低。
     * - 背光先关闭，避免复位和初始化命令期间屏幕闪白。
     */
    LCD_CS_Set();
    LCD_DC_Set();
    LCD_BLK_Clr();

    /*
     * ST7789 复位时序：RES 拉低至少数毫秒即可，这里保留 100ms 裕量，
     * 覆盖上电电源爬升和屏幕模块 RC 复位差异。
     */
    LCD_RES_Set();
    delay_ms(100);
    LCD_RES_Clr();
    delay_ms(100);
    LCD_RES_Set();
    delay_ms(120);
}

void LCD_Writ_Bus(u8 dat)
{
    /*
     * 与参考例程保持同样的片选包围方式。当前工程默认 CS 硬件常低，
     * 因此 LCD_CS_Clr/Set 为空操作；若后续增加 LCD_CS_PIN，会自动变为
     * 每字节拉低/释放的 GPIO 片选时序。
     */
    LCD_CS_Clr();
    lcd_spi0_write_byte(dat);
    LCD_CS_Set();
}

void LCD_WR_DATA8(u8 dat)
{
    LCD_Writ_Bus(dat);
}

void LCD_WR_DATA(u16 dat)
{
    LCD_Writ_Bus((u8)(dat >> 8));
    LCD_Writ_Bus((u8)dat);
}

void LCD_WR_REG(u8 dat)
{
    LCD_DC_Clr();
    LCD_Writ_Bus(dat);
    LCD_DC_Set();
}

void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
    /*
     * 该 1.69 寸 ST7789 屏实际显存比可视窗口更大，横屏/竖屏需要加 20
     * 像素偏移才能让 (0,0) 对齐可视区域左上角。偏移值沿用参考 LCD 例程。
     */
    if (USE_HORIZONTAL == 0) {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1 + 20U);
        LCD_WR_DATA(y2 + 20U);
        LCD_WR_REG(0x2C);
    } else if (USE_HORIZONTAL == 1) {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1 + 20U);
        LCD_WR_DATA(y2 + 20U);
        LCD_WR_REG(0x2C);
    } else if (USE_HORIZONTAL == 2) {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1 + 20U);
        LCD_WR_DATA(x2 + 20U);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2C);
    } else {
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(x1 + 20U);
        LCD_WR_DATA(x2 + 20U);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2C);
    }
}

void LCD_Init(void)
{
    LCD_GPIO_Init();

    /*
     * 退出睡眠后必须等待面板内部电源稳定，再继续写像素格式、扫描方向、
     * porch、电压与伽马参数。该初始化序列来自参考例程，适配当前 ST7789
     * 240x280 可视窗口。
     */
    LCD_WR_REG(0x11);
    delay_ms(120);

    LCD_WR_REG(0x36);
    if (USE_HORIZONTAL == 0) {
        LCD_WR_DATA8(0x00);
    } else if (USE_HORIZONTAL == 1) {
        LCD_WR_DATA8(0xC0);
    } else if (USE_HORIZONTAL == 2) {
        LCD_WR_DATA8(0x70);
    } else {
        LCD_WR_DATA8(0xA0);
    }

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x35);

    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x32);

    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x15);

    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);

    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x0F);

    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);

    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x34);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x34);

    LCD_WR_REG(0x21);
    LCD_WR_REG(0x29);
    delay_ms(20);

    /* 初始化完成后再打开背光，用户看到的第一帧应是后续清屏后的启动页。 */
    LCD_BLK_Set();
    delay_ms(20);
}
