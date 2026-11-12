#ifndef BSP_COMMON_H
#define BSP_COMMON_H

/*
 * 主机端（PC）单元测试用的 BSP 公共头替身。
 *
 * 目的：
 * - 让 BSP/SERVO/servo.c 和 User/App/ball_control.c 不依赖 MSPM0 硬件即可
 *   在 gcc 下编译运行，从而验证角度映射、限幅、斜率限制和 PD 逻辑；
 * - 只提供被这两个模块引用到的 TI DriverLib 符号，其余一律不实现。
 *
 * 使用方式：
 * - 编译时把本目录放在 -I 的最前面，覆盖 Car2/BSP/bsp_common.h；
 * - 本文件不参与 Keil 工程编译。
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* TIMA0 寄存器块替身：测试里只需要记录写入的比较值和周期。 */
typedef struct {
    volatile uint32_t counter;
} GPTIMER_Regs;

#define PWM_0_INST                  ((GPTIMER_Regs *)0)

typedef struct {
    uint32_t clockSel;
    uint32_t divideRatio;
    uint32_t prescale;
} DL_TimerA_ClockConfig;

#define DL_TIMER_CLOCK_BUSCLK       (0U)
#define DL_TIMER_CLOCK_DIVIDE_1     (0U)
#define DL_TIMER_CC_1_INDEX         (1U)
#define DL_TIMER_CC1_OUTPUT         (1U)

extern uint32_t host_servo_cc_ticks;
extern uint32_t host_servo_load_ticks;
extern bool     host_servo_started;

static inline void DL_TimerA_setClockConfig(GPTIMER_Regs *timer,
                                            DL_TimerA_ClockConfig *config)
{
    (void)timer;
    (void)config;
}

static inline void DL_TimerA_setLoadValue(GPTIMER_Regs *timer, uint32_t value)
{
    (void)timer;
    host_servo_load_ticks = value;
}

static inline void DL_TimerA_setCCPDirection(GPTIMER_Regs *timer, uint32_t dir)
{
    (void)timer;
    (void)dir;
}

static inline void DL_TimerA_setCaptureCompareValue(GPTIMER_Regs *timer,
                                                    uint32_t value,
                                                    uint32_t index)
{
    (void)timer;
    (void)index;
    host_servo_cc_ticks = value;
}

static inline void DL_TimerA_startCounter(GPTIMER_Regs *timer)
{
    (void)timer;
    host_servo_started = true;
}

/*
 * 以下 UART 替身只用于让 vision_link.c 通过语法/类型检查，
 * 主机的单元测试不链接 vision_link.c（真实收发依赖 MSPM0 中断）。
 */
typedef struct {
    volatile uint32_t dummy;
} UART_Regs;

#define UART_WF_INST                    ((UART_Regs *)0)
#define UART_WF_INST_FREQUENCY          (80000000U)
#define UART_WF_INST_INT_IRQN           (0)
#define UART_WF_INST_IRQHandler         UART4_IRQHandler

#define DL_UART_RX_FIFO_LEVEL_ONE_ENTRY (0U)
#define DL_UART_TX_FIFO_LEVEL_1_2_EMPTY (0U)
#define DL_UART_MAIN_INTERRUPT_RX       (1U)
#define DL_UART_MAIN_INTERRUPT_TX       (2U)
#define DL_UART_MAIN_IIDX_RX            (0U)
#define DL_UART_MAIN_IIDX_TX            (1U)

/* 模拟 UART 收发 FIFO：测试用例先把字节压入 host_uart_rx，再调用 IRQ。 */
#define HOST_UART_RX_SIZE               (512U)
#define HOST_UART_TX_SIZE               (2048U)

extern uint8_t  host_uart_rx[HOST_UART_RX_SIZE];
extern volatile uint16_t host_uart_rx_head;
extern volatile uint16_t host_uart_rx_tail;
extern uint8_t  host_uart_tx[HOST_UART_TX_SIZE];
extern volatile uint16_t host_uart_tx_len;

static inline void DL_UART_Main_changeConfig(UART_Regs *uart) { (void)uart; }
static inline void DL_UART_Main_configBaudRate(UART_Regs *uart, uint32_t clock, uint32_t baud)
{
    (void)uart; (void)clock; (void)baud;
}
static inline void DL_UART_Main_enableFIFOs(UART_Regs *uart) { (void)uart; }
static inline void DL_UART_Main_setRXFIFOThreshold(UART_Regs *uart, uint32_t level)
{
    (void)uart; (void)level;
}
static inline void DL_UART_Main_setTXFIFOThreshold(UART_Regs *uart, uint32_t level)
{
    (void)uart; (void)level;
}
static inline void DL_UART_Main_enableInterrupt(UART_Regs *uart, uint32_t mask)
{
    (void)uart; (void)mask;
}
static inline void DL_UART_Main_disableInterrupt(UART_Regs *uart, uint32_t mask)
{
    (void)uart; (void)mask;
}
static inline void DL_UART_Main_enable(UART_Regs *uart) { (void)uart; }

static inline bool DL_UART_Main_isRXFIFOEmpty(const UART_Regs *uart)
{
    (void)uart;
    return host_uart_rx_head == host_uart_rx_tail;
}

static inline bool DL_UART_Main_isTXFIFOFull(const UART_Regs *uart)
{
    (void)uart;
    return false;
}

static inline uint8_t DL_UART_Main_receiveData(const UART_Regs *uart)
{
    uint8_t data;

    (void)uart;
    data = host_uart_rx[host_uart_rx_tail];
    host_uart_rx_tail = (uint16_t)((host_uart_rx_tail + 1U) % HOST_UART_RX_SIZE);
    return data;
}

static inline void DL_UART_Main_transmitData(UART_Regs *uart, uint8_t data)
{
    (void)uart;
    if (host_uart_tx_len < HOST_UART_TX_SIZE) {
        host_uart_tx[host_uart_tx_len++] = data;
    }
}

/*
 * 简化模型：有 RX 字节时优先返回 RX，否则返回 TX。
 * 真实硬件由 IIDX 寄存器决定，测试只关心两条路径都能被调用到。
 */
static inline uint32_t DL_UART_Main_getPendingInterrupt(UART_Regs *uart)
{
    (void)uart;
    if (host_uart_rx_head != host_uart_rx_tail) {
        return DL_UART_MAIN_IIDX_RX;
    }
    return DL_UART_MAIN_IIDX_TX;
}

static inline void NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static inline void NVIC_EnableIRQ(int irq) { (void)irq; }

#endif
