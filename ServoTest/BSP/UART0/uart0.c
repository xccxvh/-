#include "UART0/uart0.h"

/*
 * 串口调试模块说明：
 * 1. 本文件负责将 printf 输出重定向到 UART0，便于调试日志统一走主串口。
 * 2. 当前该模块只管理 UART0，自定义协议用到的 UART4 已拆到 gray_sensor_uart 模块单独维护。
 * 3. 文件内还提供了基于 80MHz 主频的简单阻塞式延时函数，供其他 BSP 模块复用。
 */

#pragma(__use_no_semihosting)

struct FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}

/*
 * 将单个字符输出到 UART0。
 * Keil 的 printf 最终会落到这里，因此这里必须保持发送路径稳定可用。
 */
int fputc(int ch, FILE *f)
{
    (void)f;

    DL_UART_Main_transmitData(UART0, (uint8_t)ch);
    while (DL_UART_Main_isBusy(UART0))
    {
    }

    return ch;
}

/*
 * 将字符串逐字节输出到 UART0。
 * 某些工具链环境下，除了 fputc 之外还会使用 fputs，因此这里也一并重定向。
 */
int fputs(const char *_ptr, register FILE *_fp)
{
    uint16_t i;
    uint16_t len = strlen(_ptr);

    (void)_fp;

    for (i = 0; i < len; i++)
    {
        DL_UART_Main_transmitData(UART0, (uint8_t)_ptr[i]);
        while (DL_UART_Main_isBusy(UART0))
        {
        }
    }

    return len;
}

/*
 * 基于 80MHz 主频的毫秒级阻塞延时。
 * 该函数适用于初始化阶段和简单测试逻辑，不适合高实时性场景。
 */
void delay_ms(uint32_t ms)
{
    delay_cycles(80000 * ms);
}

/*
 * 基于 80MHz 主频的微秒级阻塞延时。
 * 主要用于需要短暂等待的外设时序，例如复位或片选切换后的最短保持时间。
 */
void delay_us(uint32_t us)
{
    delay_cycles(80 * us);
}

/*
 * 初始化调试串口。
 * 当前工程的波特率、引脚复用、收发方向由 SysConfig 生成，这里只补充运行期必须稳定保持的接收策略。
 * 关键点：
 * 1. M0G3519 的 UNICOMM UART 接收路径硬件上始终经过 RX 缓冲区，RX 中断由 RXIFLSEL 水位决定。
 * 2. 串口助手调试通常一次只发送 1 个或少量字节，因此必须把触发条件设为“接收缓冲非空”。
 * 3. 形参 baud 暂由 SysConfig 接管，保留该入口是为了兼容上层已有的 uart0_init(115200) 调用方式。
 */
void uart0_init(uint32_t baud)
{
    (void)baud;

    DL_UART_Main_setRXFIFOThreshold(UART_0_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_0_INST);

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/*
 * UART0 接收中断服务函数。
 * 职责：把 UART0 收到的每一个字节原样回发，实现“发送什么数据，回复什么数据”的串口助手回显测试。
 * 实现原因：
 * 1. 中断入口只处理 RX 事件，避免其它 UART 状态事件误触发回显逻辑。
 * 2. 进入中断后循环读空接收缓冲，防止 PC 端连续发送多个字节时只回显第一个字节。
 * 3. 回发使用阻塞发送接口，保证上一字节已经进入 TX 缓冲后再发送下一字节，避免回显数据在发送侧丢失。
 * 时序约束：该逻辑适合调试串口和低速命令回显；若后续承载大吞吐协议，应改成环形缓冲区并在主循环发送。
 */
void UART0_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_0_INST) == DL_UART_MAIN_IIDX_RX)
    {
        while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false)
        {
            uint8_t rx_data = DL_UART_Main_receiveData(UART_0_INST);
            DL_UART_Main_transmitDataBlocking(UART_0_INST, rx_data);
        }
    }
}

/*
 * 将浮点数转为字符串。
 * 这里采用“整数部分 + 手工展开小数部分”的方式，避免直接依赖较重的浮点格式化输出。
 */
void doubleToStr(double value, char *str, int precision)
{
    int i;
    long wholePart = (long)value;
    double fractionalPart = fmod(value, 1.0);

    sprintf(str, "%ld.", wholePart);
    str += strlen(str);

    for (i = 0; i < precision; ++i)
    {
        long digit;

        fractionalPart *= 10;
        digit = (long)fractionalPart;
        sprintf(str, "%ld", digit);
        str += strlen(str);
        fractionalPart -= digit;
    }

    *str = '\0';
}
