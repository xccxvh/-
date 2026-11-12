#include "SystemTime/system_time.h"

/*
 * 主机测试用的虚拟系统时基。
 *
 * 真实工程里 nowtime 由 TIMA1 1 ms 中断递增；这里由测试用例显式推进，
 * 这样帧间隔、超时和斜率限制都可以精确复现。
 */

uint32_t host_servo_cc_ticks = 0U;
uint32_t host_servo_load_ticks = 0U;
bool     host_servo_started = false;

/* 模拟 UART FIFO，供 vision_link 协议解析测试使用。 */
uint8_t  host_uart_rx[HOST_UART_RX_SIZE];
volatile uint16_t host_uart_rx_head = 0U;
volatile uint16_t host_uart_rx_tail = 0U;
uint8_t  host_uart_tx[HOST_UART_TX_SIZE];
volatile uint16_t host_uart_tx_len = 0U;

static uint32_t s_host_time_ms = 0U;

uint32_t system_time_get_tick_ms(void)
{
    return s_host_time_ms;
}

bool system_time_elapsed_ms(uint32_t *last_tick_ms, uint32_t interval_ms)
{
    if (last_tick_ms == NULL) {
        return false;
    }

    if ((uint32_t)(s_host_time_ms - *last_tick_ms) >= interval_ms) {
        *last_tick_ms = s_host_time_ms;
        return true;
    }

    return false;
}

void host_time_advance_ms(uint32_t ms)
{
    s_host_time_ms += ms;
}

void host_time_reset(void)
{
    s_host_time_ms = 0U;
    host_servo_cc_ticks = 0U;
    host_servo_load_ticks = 0U;
    host_servo_started = false;
}
