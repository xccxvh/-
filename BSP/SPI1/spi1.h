#ifndef SPI1_H
#define SPI1_H

/*
 * SPI1 模块头文件。
 * 该模块主要服务于 IMU、W25Q64 等 SPI 外设，因此只依赖公共底层定义。
 */
#include "bsp_common.h"

uint8_t spi1_read_write_byte(uint8_t dat);

#define SPI1_CS_IMU(x) ((x) ? DL_GPIO_setPins(IMU_PORT, IMU_CS_IMU_PIN) : DL_GPIO_clearPins(IMU_PORT, IMU_CS_IMU_PIN))

#endif