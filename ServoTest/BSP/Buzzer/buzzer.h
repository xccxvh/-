#ifndef BUZZER_H
#define BUZZER_H

/*
 * PA16 蜂鸣器（TIMG12 CCP1，与 SK6812 共用 PWM_1）。
 * 必须在 sk6812 成功刷新并锁定 PA15 后调用 buzzer_play()。
 */
#include "bsp_common.h"

/*
 * 初始化蜂鸣器共享 PWM 状态。
 *
 * 调用时机：
 * - 应在 SYSCFG_DL_init() 之后调用，TIMG12/PWM_1 的底层配置由 SysConfig 完成。
 *
 * 输出约束：
 * - 初始化不会发声，默认保持 PA16 输出关闭。
 */
void buzzer_init(void);

/*
 * 按指定频率和占空比驱动蜂鸣器。
 *
 * 参数：
 * - hz：蜂鸣器 PWM 频率，单位 Hz；为 0 时应改用 buzzer_stop()。
 * - duty_percent：占空比百分比，范围建议 0~100。
 *
 * 硬件约束：
 * - 蜂鸣器与 SK6812 共用 TIMG12 周期；只有在 PA15 已由 RGB 驱动锁定后才允许接管低频 PWM。
 */
void buzzer_play(uint16_t hz, uint8_t duty_percent);

/*
 * 停止蜂鸣器输出。
 *
 * 行为：
 * - 关闭 PA16 的 PWM 输出并记录当前蜂鸣器为非活动状态。
 * - 不会修改 SK6812 颜色缓存。
 */
void buzzer_stop(void);

/*
 * 查询蜂鸣器当前是否处于发声状态。
 *
 * 返回值：
 * - true：最近一次命令为有效 buzzer_play()。
 * - false：蜂鸣器已停止或尚未初始化。
 */
bool buzzer_is_active(void);

/*
 * 在 RGB 发码结束后恢复蜂鸣器 PWM。
 *
 * 用途：
 * - SK6812 发码会临时把 TIMG12 切到 800kHz 位时序；发码结束后若蜂鸣器之前处于
 *   活动状态，可调用本函数恢复低频蜂鸣器配置。
 */
void buzzer_restore_after_rgb(void);

/*
 * 读取蜂鸣器当前上下文。
 *
 * 参数：
 * - active：输出是否正在发声，可为 NULL。
 * - hz：输出当前频率，单位 Hz，可为 NULL。
 * - duty：输出当前占空比百分比，可为 NULL。
 *
 * 用途：
 * - 供 LCD 测试页面显示当前蜂鸣器状态，不直接访问驱动内部变量。
 */
void buzzer_get_context(bool *active, uint16_t *hz, uint8_t *duty);

#endif
