#include "UART0/uart0.h"

/*
 * 串口调试模块说明：
 * 1. 本文件负责将 printf 输出重定向到 UART0，便于调试时统一走主串口。
 * 2. 同时打开 UART0 和 UART4 中断，其中 UART4 当前用于打印收到的原始十六进制数据。
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
 * 当前工程的底层波特率参数由 SysConfig 生成，因此这里主要负责使能外设和中断。
 * 形参 baud 目前保留给上层调用，便于后续切换到手工配置串口时继续复用接口。
 */
void uart0_init(uint32_t baud)
{
    (void)baud;

    DL_UART_Main_enable(UART_0_INST);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    DL_UART_Main_enable(UART_4_INST);
    NVIC_ClearPendingIRQ(UART_4_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_4_INST_INT_IRQN);
}

/*
 * UART0 接收中断服务函数。
 * 当前仅保留最小接收动作，后续如需协议解析，可在这里接入环形缓冲区或状态机。
 */
uint8_t g_usart0_rx_buf[20];
uint8_t gEchoData = 0;

void UART0_IRQHandler(void)
{
    uint8_t res;

    if (DL_UART_Main_getPendingInterrupt(UART0) == DL_UART_MAIN_IIDX_RX)
    {
        res = DL_UART_receiveData(UART0);
        gEchoData = res;
    }
}

/*
 * UART4 接收中断服务函数。
 * 当前处理方式是将收到的字节按十六进制打印出来，适合联调外部模块时快速观察原始数据流。
 */
void UART4_IRQHandler(void)
{
    uint8_t res;

    if (DL_UART_Main_getPendingInterrupt(UART4) == DL_UART_MAIN_IIDX_RX)
    {
        res = DL_UART_receiveData(UART4);
        printf("%x ", res);
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
