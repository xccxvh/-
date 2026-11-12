#ifndef TIMERA1_H
#define TIMERA1_H

/*
 * TimerA1 模块头文件。
 * 当前该定时器承担系统节拍和部分状态灯闪烁功能。
 */
#include "bsp_common.h"

void TimerA1_init(void);

/*
 * 返回自 TimerA1 启动以来的 100 us 单调节拍。第四问用相邻两次读取的差值计算
 * 陀螺仪积分时间；无符号减法可自然处理 32 位计数回绕。
 */
uint32_t TimerA1_get_tick_100us(void);

#endif
