#ifndef __IMU_H
#define __IMU_H

/*
 * IMU 模块对外头文件。
 *
 * 职责：
 * - 对应用层提供 IMU 初始化、在线状态查询、姿态角读取和原始六轴数据缓存读取接口。
 * - 隐藏具体芯片型号和寄存器驱动细节；当前底层实现为 LSM6DSV/LSM6DSV16X 系列 SPI 驱动。
 *
 * 依赖约束：
 * - 本头文件属于 BSP 模块头文件，只包含 BSP/bsp_common.h。
 * - 不直接包含聚合头 BSP/bsp.h，避免 BSP 模块之间形成隐式循环依赖。
 */
#include "bsp_common.h"

#ifndef M_PI
#define M_PI (3.1415926535f)
#endif

/*
 * 三轴浮点向量。
 *
 * 用途：
 * - 保留给姿态解算或上层控制算法表示 X/Y/Z 三轴物理量。
 * - 单位由具体调用点决定，例如加速度可为 mg 或 g，角速度可为 dps 或 rad/s。
 */
typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;

/*
 * 初始化 IMU 传感器与姿态解算状态。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 和 SPI1 相关 GPIO/SPI 配置生效之后调用。
 *
 * 硬件/时序约束：
 * - 函数会通过 SPI1 访问 WHO_AM_I 和配置寄存器，片选使用 SPI1_CS_IMU()。
 * - 初始化结果会写入模块内部 ready 状态，应用层可通过 IMU_isReady() 判断是否可用。
 */
void IMU_init(void);

/*
 * 读取 IMU WHO_AM_I 芯片 ID。
 *
 * 参数：
 * - whoami：输出芯片 ID 的指针；不可为 NULL。
 *
 * 返回值：
 * - 0：读取成功且底层 SPI 事务正常。
 * - 非 0：SPI 访问失败、参数无效或 ID 不符合预期。
 *
 * 用途：
 * - 用于上电自检和 IMU 测试页 K2 探测，不会修改姿态角输出缓存。
 */
int IMU_probe_whoami(uint8_t *whoami);

/*
 * 查询 IMU 初始化/在线状态。
 *
 * 返回值：
 * - true：IMU 初始化成功，姿态数据接口可被应用层读取。
 * - false：初始化失败或芯片未响应，读取姿态时应显示无效状态。
 */
bool IMU_isReady(void);

/*
 * 读取当前 yaw/pitch/roll 姿态角。
 *
 * 参数：
 * - ypr：长度至少为 3 的 float 数组，输出顺序为 yaw、pitch、roll，单位为度。
 *
 * 时序约束：
 * - 姿态积分依赖 system_time 的 1ms 时基；应用层当前按 20Hz 调用。
 * - 当 IMU_isReady() 为 false 时，上层不应使用该输出作为有效控制量。
 */
void IMU_getYawPitchRoll(float *ypr);

/*
 * 读取并输出 IMU 原始/中间角速度数据。
 *
 * 参数：
 * - zsjganda：调用者提供的浮点数组，具体元素含义沿用原 IMU 算法实现。
 *
 * 说明：
 * - 该接口保留给旧调试代码和姿态算法验证使用，新测试页面主要使用
 *   IMU_getYawPitchRoll()。
 */
void IMU_TT_getgyro(float *zsjganda);

/*
 * 重新标定姿态角零偏。
 *
 * 用途：
 * - 在设备静止时重新计算角速度/姿态偏置，减小后续姿态积分漂移。
 *
 * 时序约束：
 * - 标定期间应保持板卡静止；该函数可能包含多次采样和阻塞延时，不应在中断中调用。
 */
void MPU6050_InitAng_Offset(void);

#endif
