#include "UART0/uart0.h"

/*
 * UART0 模块说明：
 * 1. 本文件负责将 printf 输出重定向到 UART0，并提供毫秒/微秒阻塞延时。
 * 2. 循迹模式下 UART0 作为 VOFA/调试口；滚球模式下改作 Jetson 视觉协议口，
 *    此时由 main.c 调用 uart0_set_output_enable(false) 关闭所有 printf 文本，
 *    RX 中断和二进制收发统一交给 BSP/VISION/vision_link.c。
 * 3. 本文件不再实现 UART0 接收回显，避免与视觉协议抢 RX 数据。
 */

#pragma(__use_no_semihosting)

struct FILE
{
    int handle;
};

FILE __stdout;

/* printf 文本输出开关：默认开启，滚球模式在 main.c 中关闭。 */
static volatile bool s_output_enable = true;

void uart0_set_output_enable(bool enable)
{
    s_output_enable = enable;
}

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

    if (!s_output_enable) {
        return ch;
    }

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

    if (!s_output_enable) {
        return len;
    }

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
 * 初始化调试串口 UART0（PA10/TX、PA11/RX）。
 *
 * SysConfig 的 SYSCFG_DL_UART_0_init() 会先按 config.syscfg 配置引脚与默认波特率；
 * 本函数按传入 baud 重新计算分频，确保与 VOFA/串口助手设置的 115200 一致。
 *
 * 硬件约束：
 * - UART_0_INST_FREQUENCY 为 UART0 时钟源频率（当前 40 MHz），须与 DL_UART_configBaudRate 一致。
 * - 已 enable 的 UART 改波特率前须 DL_UART_Main_changeConfig()，再恢复 FIFO。
 */
void uart0_init(uint32_t baud)
{
    if (baud == 0U) {
        baud = 115200U;
    }

    DL_UART_Main_changeConfig(UART_0_INST);
    DL_UART_Main_configBaudRate(UART_0_INST, UART_0_INST_FREQUENCY, baud);
    DL_UART_Main_enableFIFOs(UART_0_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_0_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_0_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);
    /*
     * 显式关闭 RX 中断：
     * - 循迹模式不需要 UART0 接收；
     * - 滚球模式由 Vision_Init() 统一重新打开 RX/TX 中断并接管 UART0_IRQHandler。
     * 这样可以避免 SysConfig 默认打开的 RX 中断在 NVIC 尚未使能时空跑。
     */
    DL_UART_Main_disableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_0_INST);
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
