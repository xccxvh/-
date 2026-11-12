#ifndef PWM1_SHARED_H
#define PWM1_SHARED_H

/*
 * TIMG12（PWM_1）仲裁：PA15=SK6812，PA16=蜂鸣器，共用同一定时器周期寄存器。
 *
 * 优先级（用户约定）：
 * 1. RGB 发码：周期切到 800kHz 位时序，仅短暂占用，可打断蜂鸣器；
 * 2. 发码结束：PA15 锁定为 GPIO 低电平，不再随蜂鸣器 PWM 变化；
 * 3. 蜂鸣器：仅在 PA15 已锁定时，调整 TIMG12 低频周期 + CC1 占空比。
 */
#include "bsp_common.h"

/*
 * 初始化 TIMG12 共享仲裁状态。
 *
 * 行为：
 * - 清空 RGB/蜂鸣器的软件占用标志，确保后续模式切换从已知状态开始。
 * - 不重新配置 SysConfig 已经生成的 TIMG12 外设基础时钟和引脚复用。
 */
void pwm1_shared_init(void);

/*
 * 查询 RGB 输出脚 PA15 是否已经锁定为空闲低电平。
 *
 * 返回值：
 * - true：SK6812 发码已结束，PA15 不会被蜂鸣器 PWM 周期带动。
 * - false：RGB 发码尚未完成或尚未锁定，不建议启动蜂鸣器 PWM。
 */
bool pwm1_rgb_is_locked(void);

/*
 * 切换 TIMG12 到 SK6812 800kHz 发码模式。
 *
 * 硬件约束：
 * - 会改变 TIMG12 周期寄存器和 PA15 比较输出，调用期间蜂鸣器 PWM 会被临时打断。
 * - 仅应由 SK6812 驱动在准备发送位流时调用。
 */
void pwm1_enter_rgb_tx_mode(void);

/*
 * 将 RGB 输出脚 PA15 锁定为发码结束后的安全空闲状态。
 *
 * 用途：
 * - SK6812 数据发送完成后调用，避免后续蜂鸣器低频 PWM 继续影响 PA15。
 */
void pwm1_rgb_lock_pa15(void);

/*
 * 切换 TIMG12 到蜂鸣器低频 PWM 模式。
 *
 * 参数：
 * - hz：蜂鸣器频率，单位 Hz。
 * - duty_percent：蜂鸣器占空比百分比，建议范围 0~100。
 *
 * 约束：
 * - 应在 pwm1_rgb_is_locked() 为 true 后调用，否则可能扰动 SK6812 数据线。
 */
void pwm1_enter_buzzer_pwm_mode(uint16_t hz, uint8_t duty_percent);

/*
 * 停止蜂鸣器 PWM 输出。
 *
 * 行为：
 * - 关闭/拉低 PA16 对应比较输出，但不改变 RGB 锁定状态。
 */
void pwm1_buzzer_pwm_stop(void);

#endif
