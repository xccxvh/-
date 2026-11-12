#include "TimerA1/TimerA1.h"
#include "LED/led.h"

/*
 * TimerA1 由 SysConfig 配置为 100 us 周期，计数在中断和主循环之间共享，因此
 * 必须声明为 volatile，防止编译器缓存旧值而破坏陀螺仪积分时间计算。
 */
/* 保留全局符号以兼容工程中既有的 ICM45686 姿态解算模块。 */
volatile uint32_t nowtime = 0U;

void TimerA1_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TimerA1_INST))
    {
    case DL_TIMERA_IIDX_LOAD:
        nowtime++;
        /* 每 500 ms 翻转一次状态灯，保留工程原有的运行心跳指示。 */
        if ((nowtime % 5000U) == 0U)
        {
            LED1_toggle;
        }
        break;
    default:
        break;
    }
}

void TimerA1_init(void)
{
    /* 清除上电阶段可能残留的挂起标志，再允许 100 us 周期中断进入 CPU。 */
    NVIC_ClearPendingIRQ(TimerA1_INST_INT_IRQN);
    NVIC_EnableIRQ(TimerA1_INST_INT_IRQN);
}

uint32_t TimerA1_get_tick_100us(void)
{
    /* Cortex-M0+ 对齐的 32 位读取为单次访问，可直接取得中断维护的时间戳。 */
    return nowtime;
}
