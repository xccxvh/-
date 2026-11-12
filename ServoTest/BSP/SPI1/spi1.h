#ifndef SPI1_H
#define SPI1_H

/*
 * SPI1 模块头文件。
 *
 * 职责：
 * - 提供 SPI1 单字节全双工读写接口，供 IMU 寄存器驱动发送地址、数据和 dummy 字节。
 * - 提供带成功/失败返回值的传输接口，便于初始化阶段把 SPI 超时上报给芯片驱动。
 * - 提供 IMU 片选控制宏，片选引脚由 SysConfig 生成的 IMU_PORT/IMU_CS_PIN 描述。
 *
 * 硬件约束：
 * - SPI1 的 SCLK/MOSI/MISO 当前复用到 PB16/PB15/PB14。
 * - IMU CS 是普通 GPIO，空闲必须保持高电平，完整寄存器事务期间必须保持低电平。
 * - 本头文件属于 BSP 模块头文件，只依赖 bsp_common.h，不包含聚合头 bsp.h。
 */
#include "bsp_common.h"

/*
 * 通过 SPI1 发送 1 字节并同时接收 1 字节。
 *
 * 参数：
 * - tx_data：需要写入 SPI 总线的数据字节；读寄存器时可传 dummy 字节。
 * - rx_data：接收数据输出指针；不可为 NULL。
 *
 * 返回值：
 * - true：发送和接收均完成。
 * - false：等待 TX/RX 状态超时或参数无效。
 *
 * 硬件约束：
 * - SPI1 的 PB16/PB15/PB14 时钟与引脚由 SysConfig 配置。
 * - 调用者必须在完整寄存器事务期间自行控制片选，例如使用 SPI1_CS_IMU()。
 */
bool spi1_transfer_byte(uint8_t tx_data, uint8_t *rx_data);

/*
 * SPI1 单字节全双工读写的兼容接口。
 *
 * 参数：
 * - dat：写入 SPI1 的字节。
 *
 * 返回值：
 * - 同一 SPI 时钟窗口内读回的字节；若底层传输失败，返回 0。
 *
 * 用途：
 * - 供旧 IMU 寄存器驱动沿用“写一个字节、返回一个字节”的调用风格。
 */
uint8_t spi1_read_write_byte(uint8_t dat);

/*
 * IMU 片选控制。
 *
 * 输入：
 * - level 非 0：释放 IMU 片选，CS 输出高电平。
 * - level 为 0：选中 IMU，CS 输出低电平。
 *
 * 说明：
 * - LSM6DSV SPI 事务以 CS 低电平窗口为边界，读写地址字节和连续数据字节必须在同一窗口内完成。
 */
#define SPI1_CS_IMU(level)                                                    \
    do {                                                                      \
        if ((level) != 0U) {                                                   \
            DL_GPIO_setPins(IMU_PORT, IMU_CS_PIN);                             \
        } else {                                                              \
            DL_GPIO_clearPins(IMU_PORT, IMU_CS_PIN);                           \
        }                                                                     \
    } while (0)

#endif
