#ifndef MOTOR_QEI_H
#define MOTOR_QEI_H

/*
 * M1/M2 正交编码器（QEI）读数接口。
 *
 * 硬件（SysConfig）：
 * - M1：TIMG9，PHA=PB29，PHB=PB30（QEI_M1）
 * - M2：TIMG8，PHA=PC6，PHB=PA22（QEI_1）
 *
 * 计数器为 16 位（0~65535），由 TIMG 在 QEI 模式下自动增减。
 */
#include "bsp_common.h"
#include "MOTOR/drv8874.h"

/*
 * 初始化 M1/M2 QEI 计数读取状态。
 *
 * 调用时机：
 * - 应在 SYSCFG_DL_init() 后调用，QEI 定时器和引脚复用由 SysConfig 完成。
 *
 * 行为：
 * - 启动 TIMG9/TIMG8 的 QEI 计数器，使后续可读取编码器计数与方向。
 */
void motor_qei_init(void);

/*
 * 读取指定电机的 QEI 计数值。
 *
 * 参数：
 * - id：MOTOR_M1 或 MOTOR_M2 有效；其它电机当前无 QEI。
 *
 * 返回值：
 * - 16 位硬件计数器当前值；无效电机返回 0。
 */
uint16_t motor_qei_get_count(motor_id_t id);

/*
 * 读取指定电机 QEI 当前计数方向。
 *
 * 参数：
 * - id：MOTOR_M1 或 MOTOR_M2 有效。
 *
 * 返回值：
 * - true：计数方向为递增。
 * - false：计数方向为递减或电机无 QEI。
 */
bool motor_qei_get_direction_up(motor_id_t id);

#endif
