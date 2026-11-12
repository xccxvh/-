#ifndef MYI2C_H
#define MYI2C_H

/*
 * PA0/PA1 软件 I2C 基础模块。
 *
 * 职责：
 * - 在 SysConfig 已将 PA0/PA1 配置为 I2C0 复用后，由 gray_track 初始化阶段调用
 *   MyI2C_Init() 将两线切回 GPIO 位操作，与已验证可用的灰度通信例程保持一致。
 *
 * 硬件约束：
 * - SDA = PA0，SCL = PA1，与 ti_msp_dl_config.h 中 I2C_GRAY 管脚定义一致。
 * - 读 SDA 时启用内部上拉，写 SDA/SCL 时配置为推挽输出并主动释放高电平。
 *
 * 约束：
 * - 本模块头文件只包含 bsp_common.h，不包含聚合头 bsp.h。
 */
#include "bsp_common.h"

/* 软件 I2C 管脚映射：复用 SysConfig 生成的 I2C_GRAY 宏，避免与硬件文档脱节。 */
#define GPIO_I2C_SW_SDA_PORT        GPIO_I2C_GRAY_SDA_PORT
#define GPIO_I2C_SW_SDA_PIN         GPIO_I2C_GRAY_SDA_PIN
#define GPIO_I2C_SW_IOMUX_SDA       GPIO_I2C_GRAY_IOMUX_SDA
#define GPIO_I2C_SW_SCL_PORT        GPIO_I2C_GRAY_SCL_PORT
#define GPIO_I2C_SW_SCL_PIN         GPIO_I2C_GRAY_SCL_PIN
#define GPIO_I2C_SW_IOMUX_SCL       GPIO_I2C_GRAY_IOMUX_SCL

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t byte_value);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t ack_bit);
uint8_t MyI2C_ReceiveAck(void);

#endif
