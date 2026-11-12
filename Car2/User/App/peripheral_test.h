#ifndef PERIPHERAL_TEST_H
#define PERIPHERAL_TEST_H

/*
 * 全外设分页面测试应用层接口。
 *
 * 设计目标：
 * - main.c 只负责系统级启动和主循环调用，不再堆积每个外设的测试细节。
 * - K1 负责切换测试页面，K2 负责当前页面的动作/参数调整。
 * - 当前页面之外的输出型外设保持停止状态，但底层中断仍然保留，例如 UART6、CAN、
 *   TIMA1、按键 GPIO 和 SK6812 所需的 PWM 中断入口仍由各自 BSP 模块维护。
 *
 * 调用顺序：
 * 1. SYSCFG_DL_init()
 * 2. uart0_init()
 * 3. system_time_init()
 * 4. user_key_init()
 * 5. peripheral_test_init()
 * 6. while(1) peripheral_test_task()
 */
#include "bsp.h"

/*
 * 初始化全外设分页面测试应用。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init()、uart0_init()、system_time_init()、user_key_init() 之后调用。
 *
 * 行为：
 * - 初始化 LCD、LED、IMU、电机、RGB、蜂鸣器、WiFi/蓝牙串口、STS3032、CAN 等 BSP。
 * - 记录初始化状态并进入 Overview 总览页，LCD 与 UART0 均输出初始化结果。
 * - 输出型外设初始化后会保持停止，避免上电误动作。
 */
void peripheral_test_init(void);

/*
 * 全外设测试应用主循环任务。
 *
 * 调用方式：
 * - 在 while(1) 中尽可能高频调用。
 *
 * 行为：
 * - 处理 K1/K2 按键事件，K1 切换页面，K2 执行当前页面动作。
 * - 按 20Hz 调度 LCD 脏行刷新，按页面需求调度 IMU、电机、舵机、CAN 等测试任务。
 * - 周期性通过 UART0 打印当前页面状态。
 */
void peripheral_test_task(void);

#endif
