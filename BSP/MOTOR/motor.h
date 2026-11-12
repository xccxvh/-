#ifndef MOTOR_H
#define MOTOR_H

/*
 * 电机模块头文件。
 * 对外暴露双电机速度控制与双编码器计数读取接口。
 */
#include "bsp_common.h"

void get_motor_qei_cnt(uint32_t *cnt);
void set_motor_speed(int m1, int m2);

#endif