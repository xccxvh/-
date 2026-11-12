#ifndef ADC0_H
#define ADC0_H

/*
 * ADC0 模块头文件。
 * 该模块只依赖基础类型和底层外设定义，因此直接包含公共头文件即可。
 */
#include "bsp_common.h"

void adc0_init(void);
void get_adc0_num_val(uint32_t *adc0);

#endif