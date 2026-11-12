#ifndef UART0_H
#define UART0_H

/*
 * UART0 模块头文件。
 * 该模块负责调试串口初始化、printf 重定向辅助能力以及浮点转字符串工具函数。
 * 延时函数声明已经提升到公共头文件，这里不再重复声明。
 */
#include "bsp_common.h"

void uart0_init(uint32_t baud);
void doubleToStr(double value, char *str, int precision);

#endif