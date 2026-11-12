#ifndef W25Q64_H
#define W25Q64_H

/*
 * W25Q64 Flash 模块头文件。
 * 该模块通过 SPI1 与片外 Flash 通信，并直接调用 SPI1 的收发函数，
 * 因此需要显式包含 SPI1 模块头文件。
 */
#include "bsp_common.h"
#include "SPI1/spi1.h"

#define SPI1_CS_W25Q64(x) ((x) ? DL_GPIO_setPins(W25Q64_PORT, W25Q64_CS_W25Q64_PIN) : DL_GPIO_clearPins(W25Q64_PORT, W25Q64_CS_W25Q64_PIN))

uint16_t W25Q64_readID(void);
void W25Q64_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length);
void W25Q64_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte);

#endif