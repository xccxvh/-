#ifndef USER_KEY_H
#define USER_KEY_H

/*
 * 用户按键模块。
 *
 * 硬件连接：
 * - K1：PA17，按下时被外部电路拉低。
 * - K2：PC5，按下时被外部电路拉低。
 *
 * 配置职责：
 * - User/config.syscfg 负责把两个引脚配置为输入、内部上拉、下降沿中断。
 * - user_key.c 负责打开 NVIC、做 20ms 软件消抖，并把中断事件转换成主循环
 *   可以轮询消费的边沿标志。
 *
 * 使用约束：
 * - 先调用 SYSCFG_DL_init()，让 GPIO 输入/中断极性配置生效。
 * - 再调用 system_time_init()，因为消抖依赖 1ms 系统时基。
 * - 最后调用 user_key_init() 打开 GPIO 中断。
 * - BSP 模块头文件只包含 bsp_common.h，不直接包含聚合头 bsp.h。
 */
#include "bsp_common.h"

typedef enum {
    USER_KEY_K1 = 0,
    USER_KEY_K2 = 1,
    USER_KEY_COUNT
} user_key_id_t;

/*
 * 可选回调类型。当前主程序使用 user_key_take_edge() 在主循环处理按键，
 * 这样 LCD、printf、电机控制等耗时操作不会发生在 ISR 中。
 */
typedef void (*user_key_callback_t)(user_key_id_t key);

/*
 * 初始化 K1/K2 按键中断与软件消抖状态。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 和 system_time_init() 之后调用。
 *
 * 行为：
 * - 清除按键边沿标志，打开 GPIO 下降沿中断对应的 NVIC。
 * - ISR 中只记录事件和时间戳，耗时动作应放在主循环中处理。
 */
void user_key_init(void);

/*
 * 读取某个按键当前物理按下状态。
 *
 * 参数：
 * - key：USER_KEY_K1 或 USER_KEY_K2。
 *
 * 返回值：
 * - true：按键当前为按下状态。
 * - false：按键当前未按下，或 key 参数无效。
 */
bool user_key_is_pressed(user_key_id_t key);

/*
 * 取走某个按键的“已消抖按下边沿”事件。
 *
 * 参数：
 * - key：USER_KEY_K1 或 USER_KEY_K2。
 *
 * 返回值：
 * - true：自上次读取后检测到一次有效按下边沿，函数会清除该事件。
 * - false：没有新的有效按下事件。
 */
bool user_key_take_edge(user_key_id_t key);

/*
 * 设置按键事件回调函数。
 *
 * 参数：
 * - key：目标按键。
 * - callback：按键有效边沿发生后在 ISR 路径中调用的回调，可为 NULL。
 *
 * 使用约束：
 * - 回调中不要执行 LCD、printf、电机控制等耗时操作；推荐主循环轮询 user_key_take_edge()。
 */
void user_key_set_callback(user_key_id_t key, user_key_callback_t callback);

/*
 * 清除指定按键的回调函数。
 *
 * 参数：
 * - key：目标按键。
 *
 * 行为：
 * - 清空该按键已注册的 ISR 回调，不影响主循环边沿标志。
 */
void user_key_clear_callback(user_key_id_t key);

#endif
