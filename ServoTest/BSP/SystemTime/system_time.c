#include "SystemTime/system_time.h"

/*
 * nowtime：TIMA1 每 1ms +1。
 *
 * 运行约束：
 * - User/config.syscfg 中 TIMER_A1_1MS.timerPeriod 为 1 ms，生成代码里 startTimer=DL_TIMER_STOP。
 * - 因此本模块不能只打开 NVIC，还必须显式启动 TIMA1 计数器，否则主循环的心跳、
 *   LCD 刷新、按键消抖和 IMU 更新调度都会一直等不到时间推进。
 */
volatile uint32_t nowtime = 0U;

void system_time_init(void)
{
    nowtime = 0U;
    DL_TimerA_clearInterruptStatus(TIMER_A1_1MS_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    DL_TimerA_enableInterrupt(TIMER_A1_1MS_INST, DL_TIMERA_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(TIMER_A1_1MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_A1_1MS_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_A1_1MS_INST);
}

uint32_t system_time_get_tick_ms(void)
{
    return nowtime;
}

bool system_time_wait_for_tick(uint32_t timeout_loop_count)
{
    uint32_t start_tick = nowtime;

    while (timeout_loop_count > 0U) {
        if (nowtime != start_tick) {
            return true;
        }
        timeout_loop_count--;
    }

    return false;
}

bool system_time_elapsed_ms(uint32_t *last_tick_ms, uint32_t interval_ms)
{
    uint32_t current_ms;

    if (last_tick_ms == NULL) {
        return false;
    }

    current_ms = system_time_get_tick_ms();
    if (interval_ms == 0U) {
        *last_tick_ms = current_ms;
        return true;
    }

    if ((uint32_t) (current_ms - *last_tick_ms) >= interval_ms) {
        *last_tick_ms = current_ms;
        return true;
    }

    return false;
}

void TIMER_A1_1MS_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_A1_1MS_INST)) {
    case DL_TIMERA_IIDX_LOAD:
        nowtime++;
        break;
    default:
        break;
    }
}
