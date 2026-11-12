#ifndef __INFRARED_H__
#define __INFRARED_H__

/*
 * 八路数字灰度（红外光电）循迹模块头文件。
 *
 * 硬件与约定：
 * 1. 八路探头 I1~I8 由 SysConfig 分别配置为数字输入，端口和引脚宏由
 *    ti_msp_dl_config.h 生成，本文件只负责把原始电平归一化成“黑线位图”。
 * 2. 位图 bit0~bit7 依次对应 I1~I8，位为 1 表示该路压在黑线上。
 *    上层循迹算法只依赖位图，不再关心传感器模块原始电平高低。
 * 3. 若更换灰度模块后发现“压黑线”被反向识别，只需要修改
 *    INF_BLACK_LINE_LEVEL 一个宏，不要交换八路接线。
 */
#include "bsp_common.h"

#define INF_1 DL_GPIO_readPins(Infrared_I1_PORT, Infrared_I1_PIN)
#define INF_2 DL_GPIO_readPins(Infrared_I2_PORT, Infrared_I2_PIN)
#define INF_3 DL_GPIO_readPins(Infrared_I3_PORT, Infrared_I3_PIN)
#define INF_4 DL_GPIO_readPins(Infrared_I4_PORT, Infrared_I4_PIN)
#define INF_5 DL_GPIO_readPins(Infrared_I5_PORT, Infrared_I5_PIN)
#define INF_6 DL_GPIO_readPins(Infrared_I6_PORT, Infrared_I6_PIN)
#define INF_7 DL_GPIO_readPins(Infrared_I7_PORT, Infrared_I7_PIN)
#define INF_8 DL_GPIO_readPins(Infrared_I8_PORT, Infrared_I8_PIN)

/*
 * 黑线有效电平：
 * 当前实车模块在压到黑线时输出高电平，因此取 1U。
 * 如果实测发现白底输出高电平、黑线输出低电平，把它改为 0U 即可。
 */
#define INF_BLACK_LINE_LEVEL 1U

/*
 * 丢线返回值。
 * 加权偏差本身的范围是 -7~+7，用 100 表示“八路都没有看到黑线”，
 * 避免与任何合法偏差混淆。该值沿用工程历史约定。
 */
#define INF_LINE_LOST_VALUE 100

/*
 * 单次采样并归一化。
 * 返回：bit0~bit7 对应 I1~I8，1 表示压黑线。
 * 说明：只读一次 GPIO，没有消抖，供消抖函数内部调用。
 */
uint8_t inf_read_mask_raw(void);

/*
 * 多次采样投票消抖。
 * 参数：
 *   samples   —— 连续采样次数，0 按 1 处理；
 *   threshold —— 某一路在 samples 次中至少命中的次数，才算稳定压线。
 *                函数内部会把非法值收敛到 [1, samples]。
 * 返回：稳定后的黑线位图。
 * 时序：采样间隔 1 ms，总耗时约 (samples - 1) ms，期间阻塞 CPU。
 */
uint8_t inf_read_mask_stable(uint8_t samples, uint8_t threshold);

/*
 * 位图按探头位置加权求横向偏差。
 * 权重从左到右（I1→I8）为 -7、-5、-3、-1、+1、+3、+5、+7。
 * 返回：-(加权和 / 命中路数)，范围 -7~+7，正值表示黑线偏向右侧，
 *       正值时应提高左轮、降低右轮，使车体向右修正；
 *       若位图为 0（完全丢线），返回 INF_LINE_LOST_VALUE。
 */
int inf_error_from_mask(uint8_t mask);

/* 统计位图中压黑线的路数，范围 0~8。 */
uint8_t inf_count_bits(uint8_t mask);

/*
 * 以下两个接口为工程历史兼容接口，行为与旧版本保持一致：
 * read_inf_mask() —— 7 次采样、命中 4 次判有效，返回稳定位图；
 * read_inf()      —— 返回加权偏差或 INF_LINE_LOST_VALUE。
 */
uint8_t read_inf_mask(void);
int read_inf(void);

#endif
