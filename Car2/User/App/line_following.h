#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

/*
 * 小车循迹应用启动接口。
 *
 * 职责：
 * - 初始化 LCD、IMU、电机、QEI、灰度模块，并将阈值写入灰度模块。
 * - TIMA1 每 10 ms 循迹；每 20 ms 固定节拍快照 G1~G8 供 VOFA FireWater 输出。
 * - 上电后先以 150 直行，直到首次检测到黑线加权；丢线后沿用上一次加权速度。
 *
 * 调用顺序：
 * 1. SYSCFG_DL_init()
 * 2. uart0_init()
 * 3. system_time_init()
 * 4. line_following_app_init()
 * 5. while(1) line_following_app_task()
 */
#include "bsp_common.h"

void line_following_app_init(void);
void line_following_app_task(void);

/*
 * 10 ms 循迹控制 ISR，由 system_time 的 TIMA1 1 ms 中断分频调用。
 * 不可在主循环中重复调用。
 */
void line_following_control_isr(void);

/*
 * 20 ms VOFA 待发帧调度，由 system_time 1 ms 中断每 20 ms 调用。
 */
void line_following_vofa_20ms_tick(void);

#endif
