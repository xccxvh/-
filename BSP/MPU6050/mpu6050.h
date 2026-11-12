#ifndef MPU6050_H
#define MPU6050_H

/*
 * GY-6050（MPU6050）板级驱动接口。
 * 模块通过 SysConfig 生成的 I2C_0 外设通信，当前硬件连接固定为 PA0=SDA、
 * PA1=SCL，AD0 悬空时使用 7 位地址 0x68。本头文件只依赖 BSP 公共基础能力，
 * 避免板级模块之间通过聚合头形成循环依赖。
 */
#include "bsp_common.h"

/*
 * 唤醒并配置 MPU6050，陀螺仪量程设为 ±500 dps、输出率设为 200 Hz。
 * 返回 true 表示 WHO_AM_I 校验和寄存器配置均成功，false 表示总线或器件异常。
 */
bool mpu6050_init(void);

/*
 * 在小车完全静止时采集 Z 轴零偏。sample_period_ms 是相邻样本间隔，函数通过
 * bias_raw 返回原始零偏；第四问只进行平面航向控制，因此不需要 X/Y 轴数据。
 */
bool mpu6050_calibrate_gyro_z(uint16_t sample_count,
                              uint16_t sample_period_ms,
                              int32_t *bias_raw);

/* 读取 Z 轴陀螺仪原始值；±500 dps 量程下灵敏度为 65.5 LSB/(°/s)。 */
bool mpu6050_read_gyro_z_raw(int16_t *gyro_z_raw);

#endif
