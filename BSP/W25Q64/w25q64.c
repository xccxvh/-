#include "W25Q64/w25q64.h"

/*
 * W25Q64 Flash 驱动说明：
 * 1. 本文件基于 SPI1 封装了常用的读 ID、写使能、等待忙、扇区擦除、整段读写接口。
 * 2. 当前写接口采用“先擦后写”的简单策略，适合测试工程验证，不适合高频日志场景。
 */

/*
 * 读取 W25Q64 芯片 ID。
 * 常见返回值 0xEF16 对应 W25Q64，用于上电后快速确认外设是否在线。
 */
uint16_t W25Q64_readID(void)
{
    uint16_t ID = 0;

    SPI1_CS_W25Q64(0);
    spi1_read_write_byte(0x90);
    spi1_read_write_byte(0x00);
    spi1_read_write_byte(0x00);
    spi1_read_write_byte(0x00);
    ID |= (uint16_t)(spi1_read_write_byte(0xFF) << 8);
    ID |= spi1_read_write_byte(0xFF);
    SPI1_CS_W25Q64(1);

    return ID;
}

/*
 * 发送写使能命令。
 * W25Q64 的擦除和写入操作前都必须先执行一次写使能。
 */
void W25Q64_write_enable(void)
{
    SPI1_CS_W25Q64(0);
    spi1_read_write_byte(0x06);
    SPI1_CS_W25Q64(1);
}

/*
 * 轮询忙标志位。
 * 该函数会一直等待到 Flash 内部擦写流程结束，因此调用处应避免放在严格实时路径中。
 */
void W25Q64_wait_busy(void)
{
    uint8_t byte;

    do
    {
        SPI1_CS_W25Q64(0);
        spi1_read_write_byte(0x05);
        byte = spi1_read_write_byte(0xFF);
        SPI1_CS_W25Q64(1);
    } while ((byte & 0x01U) == 1U);
}

/*
 * 擦除指定扇区。
 * 形参 addr 传入的是扇区号，函数内部会自动换算为字节地址。
 */
void W25Q64_erase_sector(uint32_t addr)
{
    addr *= 4096U;

    W25Q64_write_enable();
    W25Q64_wait_busy();

    SPI1_CS_W25Q64(0);
    spi1_read_write_byte(0x20);
    spi1_read_write_byte((uint8_t)(addr >> 16));
    spi1_read_write_byte((uint8_t)(addr >> 8));
    spi1_read_write_byte((uint8_t)addr);
    SPI1_CS_W25Q64(1);

    W25Q64_wait_busy();
}

/*
 * 从指定地址开始写入一段数据。
 * 当前实现会先擦除目标所在扇区，因此更适合配置区或测试数据写入。
 */
void W25Q64_write(uint8_t *buffer, uint32_t addr, uint16_t numbyte)
{
    uint16_t i;

    W25Q64_erase_sector(addr / 4096U);
    W25Q64_write_enable();
    W25Q64_wait_busy();

    SPI1_CS_W25Q64(0);
    spi1_read_write_byte(0x02);
    spi1_read_write_byte((uint8_t)(addr >> 16));
    spi1_read_write_byte((uint8_t)(addr >> 8));
    spi1_read_write_byte((uint8_t)addr);

    for (i = 0; i < numbyte; i++)
    {
        spi1_read_write_byte(buffer[i]);
    }

    SPI1_CS_W25Q64(1);
    W25Q64_wait_busy();
}

/*
 * 从指定地址读取一段连续数据。
 * 上层需要保证 buffer 的容量不小于 read_length。
 */
void W25Q64_read(uint8_t *buffer, uint32_t read_addr, uint16_t read_length)
{
    uint16_t i;

    SPI1_CS_W25Q64(0);
    spi1_read_write_byte(0x03);
    spi1_read_write_byte((uint8_t)(read_addr >> 16));
    spi1_read_write_byte((uint8_t)(read_addr >> 8));
    spi1_read_write_byte((uint8_t)read_addr);

    for (i = 0; i < read_length; i++)
    {
        buffer[i] = spi1_read_write_byte(0xFF);
    }

    SPI1_CS_W25Q64(1);
}
