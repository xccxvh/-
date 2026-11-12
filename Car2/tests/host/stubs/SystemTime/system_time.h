#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

/*
 * 主机端系统时基替身。
 *
 * 真实工程由 TIMA1 1 ms 中断递增 nowtime；主机测试用 host_time_advance_ms()
 * 手动推进时间，便于精确构造 30 Hz 帧间隔、超时和斜率限制场景。
 */
#include "bsp_common.h"

uint32_t system_time_get_tick_ms(void);

/* 与真实工程同名的周期判断，供 ball_test.c 语法检查使用。 */
bool system_time_elapsed_ms(uint32_t *last_tick_ms, uint32_t interval_ms);

/* 测试专用：把虚拟毫秒时钟前进 ms 毫秒。 */
void host_time_advance_ms(uint32_t ms);

/* 测试专用：把虚拟毫秒时钟清零并清除舵机记录。 */
void host_time_reset(void);

#endif
