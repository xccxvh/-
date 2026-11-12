#include "SPI1/spi1.h"

/*
 * SPI1 读写接口说明：
 * 1. 当前 SPI1 主要服务于 IMU 与 W25Q64 等挂在第二路 SPI 总线上的器件。
 * 2. 这里封装的是最基础的单字节全双工收发接口，方便上层驱动按协议顺序自行组织命令。
 */

/*
 * 通过 SPI1 发送一个字节，并读取同一时刻返回的一个字节。
 * 该接口适合用于寄存器读写、Flash 指令发送等标准 SPI 时序。
 */
uint8_t spi1_read_write_byte(uint8_t dat)
{
    uint8_t data;

    DL_SPI_transmitData8(SPI_1_INST, dat);
    while (DL_SPI_isBusy(SPI_1_INST))
    {
    }

    data = DL_SPI_receiveData8(SPI_1_INST);
    while (DL_SPI_isBusy(SPI_1_INST))
    {
    }

    return data;
}
