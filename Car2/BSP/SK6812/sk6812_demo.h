#ifndef SK6812_DEMO_H
#define SK6812_DEMO_H

/*
 * SK6812 定时换色演示（基于 system_time / TIMA1 1ms 节拍）。
 *
 * 在主循环中周期调用 sk6812_demo_task()，到点自动切换 RGB，
 * 用于确认系统未卡死（发码期间会短暂屏蔽 UART/TIMA1，属正常）。
 */
#include "bsp_common.h"

/*
 * 初始化 SK6812 演示状态机。
 *
 * 行为：
 * - 清零内部颜色索引和时间戳，默认不强制持续换色；是否运行由
 *   sk6812_demo_set_enable() 控制。
 */
void sk6812_demo_init(void);

/*
 * 执行一次 SK6812 演示任务。
 *
 * 调用约束：
 * - 只能在主循环中周期调用，不应放入中断。
 * - 内部根据 system_time 的毫秒计数判断是否到达换色周期，到点后调用 SK6812 驱动刷新。
 */
void sk6812_demo_task(void);

/*
 * 使能或暂停 SK6812 自动换色演示。
 *
 * 参数：
 * - enable：true 开启自动换色；false 停止演示并保持当前颜色缓存。
 *
 * 说明：
 * - 当前全外设测试页面默认不自动换色，RGB 状态由 K2 明确触发。
 */
void sk6812_demo_set_enable(bool enable);

/*
 * 设置 SK6812 演示换色周期。
 *
 * 参数：
 * - interval_ms：相邻两次颜色切换的时间间隔，单位 ms。
 */
void sk6812_demo_set_interval_ms(uint32_t interval_ms);

/*
 * 读取当前 SK6812 演示输出的 RGB（0~255）。
 *
 * 参数：
 * - r/g/b：输出当前演示颜色的指针，允许传 NULL 跳过某个通道。
 *
 * 演示关闭或未初始化时返回 (0,0,0)，供 LCD 左下角色块同步显示。
 */
void sk6812_demo_get_color(uint8_t *r, uint8_t *g, uint8_t *b);

#endif
