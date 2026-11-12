#ifndef __IMU_H
#define __IMU_H

/*
 * IMU 模块头文件。
 * 这里只保留姿态解算模块对公共底层能力的依赖，避免通过 bsp.h 引入其他无关模块。
 */
#include "bsp_common.h"

#define M_PI (float)3.1415926535

/*
 * 三轴浮点向量结构体。
 * 主要用于表达加速度、方向向量等三维物理量。
 */
typedef struct
{
    float x, y, z;
} xyz_f_t;

void IMU_init(void);
void IMU_getYawPitchRoll(float *ypr);
void IMU_TT_getgyro(float *zsjganda);
void MPU6050_InitAng_Offset(void);

#endif