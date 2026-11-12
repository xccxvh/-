#ifndef UART0_H
#define UART0_H

/*
 * UART0 模块头文件。
 * 该模块负责调试串口初始化、printf 重定向辅助能力以及浮点转字符串工具函数。
 * 延时函数声明已经提升到公共头文件，这里不再重复声明。
 */
#include "bsp_common.h"

/*
 * 初始化调试串口 UART0。
 *
 * 参数：
 * - baud：期望波特率，当前工程通常使用 115200。
 *
 * 硬件/中断约束：
 * - UART0 的 PA10/PA11 引脚复用、基础时钟和默认波特率由 SysConfig 初始化。
 * - 本函数负责按传入 baud 更新分频参数、打开 RX 中断，并准备 printf 重定向输出。
 * - 应在 SYSCFG_DL_init() 之后调用，且在需要 printf 日志的 BSP 初始化之前调用。
 */
void uart0_init(uint32_t baud);

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
