#ifndef UART0_H
#define UART0_H

/*
 * UART0 模块头文件。
 * 该模块负责 UART0 初始化、printf 重定向辅助能力以及浮点转字符串工具函数。
 * 延时函数声明已经提升到公共头文件，这里不再重复声明。
 *
 * 用途说明：
 * - 循迹模式：UART0 继续作为 printf/VOFA 调试口；
 * - 滚球模式：UART0 改作 Jetson 视觉二进制协议口（PA10/PA11，Type-C/DAPLink），
 *   此时必须调用 uart0_set_output_enable(false) 关闭 printf 文本输出，
 *   避免调试文字混入二进制协议。
 */
#include "bsp_common.h"

/*
 * 初始化 UART0（PA10/PA11）。
 *
 * 参数：
 * - baud：期望波特率，当前工程使用 115200。
 *
 * 硬件/中断约束：
 * - 引脚与时钟由 SysConfig 初始化，本函数只重配波特率、FIFO 和使能 UART。
 * - 不再开启 RX 中断或回显；滚球模式下 RX 中断由 vision_link 模块统一接管。
 */
void uart0_init(uint32_t baud);

/*
 * 打开/关闭 printf 文本输出。
 *
 * 参数：
 * - enable：true 允许 fputc/fputs 写 UART0；false 时直接丢弃文本。
 *
 * 使用场景：
 * - 滚球模式使用 UART0 跑二进制协议，必须在任何 printf 之前关闭；
 * - 循迹模式保持开启，原有 VOFA/调试日志不受影响。
 *
 * 注意：
 * - 只影响 printf 重定向，不影响 vision_link 直接通过 UART0 寄存器发送的遥测帧。
 */
void uart0_set_output_enable(bool enable);

/*
 * 将 double 数值转换为定点小数字符串。
 *
 * 参数：
 * - value：待转换的浮点值。
 * - str：输出缓冲区，调用者负责保证空间足够。
 * - precision：小数位数。
 *
 * 用途：
 * - 兼容部分轻量级 printf 环境无法直接输出浮点数的场景，主要用于调试日志或 LCD 文本。
 */
void doubleToStr(double value, char *str, int precision);

#endif
