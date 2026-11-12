#ifndef MOTOR_UI_H
#define MOTOR_UI_H

/*
 * 四路 DRV8874 LCD 测试界面。
 *
 * 显示内容：
 * - 四路电机命令速度、ADC 原始值、软件使能状态和 nFAULT 状态。
 * - M1/M2 的 QEI 方向与计数，M3/M4 当前无 QEI 显示为 ---。
 *
 * 刷新约束：
 * - 固定列宽表格布局；应用层按 20Hz（50ms）调用 motor_ui_refresh()。
 * - 本界面只负责电机相关内容，IMU 姿态已经移到独立 IMU 测试页，避免页面残留和职责混杂。
 */
#include "bsp_common.h"
#include "MOTOR/drv8874.h"

typedef enum {
    MOTOR_UI_STEP_STOP = 0,
    MOTOR_UI_STEP_M1_FWD,
    MOTOR_UI_STEP_M1_REV,
    MOTOR_UI_STEP_M2_FWD,
    MOTOR_UI_STEP_M2_REV,
    MOTOR_UI_STEP_M3_FWD,
    MOTOR_UI_STEP_M3_REV,
    MOTOR_UI_STEP_M4_FWD,
    MOTOR_UI_STEP_M4_REV,
    MOTOR_UI_STEP_STATUS_ONLY,
    MOTOR_UI_STEP_ALL_FWD_LOW,
    MOTOR_UI_STEP_COUNT
} motor_ui_step_t;

/*
 * 初始化电机 LCD 测试界面。
 *
 * 行为：
 * - 停止所有电机、初始化 LED/QEI 状态、请求下一次刷新重画静态表头。
 * - 会立即刷新一次页面，进入默认 MOTOR_UI_STEP_STOP 测试步。
 */
void motor_ui_init(void);

/*
 * 请求下一次 motor_ui_refresh() 清屏并重画静态表头。
 *
 * 用途：
 * - 仅在进入电机页面或屏幕内容被其它页面覆盖后调用。
 * - 普通 K2 切换测试步不需要调用，避免整屏闪烁。
 */
void motor_ui_request_full_redraw(void);

/*
 * 应用指定电机测试步骤。
 *
 * 参数：
 * - step：目标测试步骤，超出范围时自动回到 MOTOR_UI_STEP_STOP。
 *
 * 行为：
 * - 切换前会先停止所有电机，然后按步骤启动对应电机或全部低速正转。
 */
void motor_ui_apply_step(motor_ui_step_t step);

/*
 * 切换到下一个电机测试步骤。
 *
 * 行为：
 * - 按 MOTOR_UI_STEP_* 枚举顺序循环推进，供 K2 按键调用。
 */
void motor_ui_next_step(void);

/*
 * 电机测试界面紧急停止。
 *
 * 行为：
 * - 立即应用 MOTOR_UI_STEP_STOP，停止所有电机输出。
 */
void motor_ui_emergency_stop(void);

/*
 * 获取当前电机测试步骤。
 *
 * 返回值：
 * - 当前 MOTOR_UI_STEP_* 枚举值，供 LCD/UART 状态显示使用。
 */
motor_ui_step_t motor_ui_get_step(void);

/*
 * 刷新电机页状态。
 * ypr/imu_ready 参数保留用于兼容旧调用路径，当前实现不显示 IMU 数据。
 */
void motor_ui_refresh(const float ypr[3], bool imu_ready);

#endif
