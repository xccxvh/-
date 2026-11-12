#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

/*
 * SystemTime：TIMA1 1ms 节拍。
 *
 * - SysConfig 当前将 TIMER_A1_1MS.timerPeriod 配为 1 ms，生成的 LOAD_VALUE 为 9999。
 * - nowtime 每次 LOAD 中断 +1，因此 nowtime 的单位就是 ms。
 * - system_time_init() 负责清零计数、清中断、开启 NVIC 并启动 TIMA1 计数器。
 * - IMU 姿态积分用同一个 nowtime 计算 dt，必须和这里的真实节拍保持一致。
 */
#include "bsp_common.h"

#define SYSTEM_TIME_TICK_HZ        (1000U)
#define SYSTEM_TIME_TICKS_PER_MS   (1U)

extern volatile uint32_t nowtime;

/*
 * 初始化 1ms 系统时基。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后调用，因为定时器实例、时钟和中断号来自 SysConfig。
 *
 * 硬件/中断约束：
 * - 该函数会清零 nowtime、清除 TIMA1 LOAD 中断标志、打开 NVIC，并启动 TIMA1。
 * - TIMA1_IRQHandler 中每 1ms 累加 nowtime，主循环和驱动层均以此作为毫秒时间基准。
 */
void system_time_init(void);

/*
 * 读取当前系统毫秒计数。
 *
 * 返回值：
 * - 从 system_time_init() 启动 TIMA1 后累计的毫秒数，单位为 ms。
 *
 * 使用说明：
 * - 该值由中断更新，32 位计数会自然回绕；上层判断时间差时建议使用
 *   system_time_elapsed_ms()，避免手写回绕逻辑。
 */
uint32_t system_time_get_tick_ms(void);

/*
 * 等待系统时基至少前进一个 tick。
 *
 * 参数：
 * - timeout_loop_count：轮询等待循环上限，用于防止定时器或中断未工作时永久阻塞。
 *
 * 返回值：
 * - true：观察到 nowtime 变化，说明 1ms 中断正在运行。
 * - false：超出等待循环仍未变化，通常表示 TIMA1 或 NVIC 配置异常。
 */
bool system_time_wait_for_tick(uint32_t timeout_loop_count);

/*
 * 判断指定周期是否已经到达。
 *
 * 参数：
 * - last_tick_ms：输入为上次触发时间；触发成功时会被更新为当前时间。
 * - interval_ms：期望周期，单位 ms。
 *
 * 返回值：
 * - true：距离上次触发已达到 interval_ms。
 * - false：时间尚未到。
 *
 * 说明：
 * - 内部按无符号减法处理 32 位回绕，适合主循环中实现 20Hz、1Hz 等周期任务。
 */
bool system_time_elapsed_ms(uint32_t *last_tick_ms, uint32_t interval_ms);

#endif
