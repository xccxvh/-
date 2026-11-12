#include "KEY/user_key.h"
#include "SystemTime/system_time.h"

/*
 * 用户按键 BSP 实现。
 *
 * 中断链路说明：
 * - SysConfig 已为 PA17/PC5 使能输入上拉和下降沿 GPIO 中断。
 * - MSPM0G351x 的 GPIOA/GPIOC 都汇入 GROUP1 向量，入口函数为 GROUP1_IRQHandler。
 * - ISR 只做状态读取、清中断、消抖和置位标志；实际电机切步、急停、LCD 刷新
 *   由 main() 在主循环中调用 user_key_take_edge() 完成，避免在中断中执行耗时逻辑。
 */

#define USER_KEY_DEBOUNCE_MS (20U)

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} user_key_hw_t;

static const user_key_hw_t s_key_hw[USER_KEY_COUNT] = {
    [USER_KEY_K1] = { .port = KEY_K1_PORT, .pin = KEY_K1_PIN },
    [USER_KEY_K2] = { .port = KEY_K2_PORT, .pin = KEY_K2_PIN },
};

static user_key_callback_t s_callbacks[USER_KEY_COUNT];
static volatile uint8_t s_edge_pending;
static volatile uint32_t s_last_irq_tick[USER_KEY_COUNT];

bool user_key_is_pressed(user_key_id_t key)
{
    const user_key_hw_t *hw;

    if (key >= USER_KEY_COUNT) {
        return false;
    }

    /*
     * 按键为低有效：释放时由内部上拉保持高电平，按下时外部电路拉低。
     * 返回 true 表示当前确实处于按下状态，可用于中断后的二次确认。
     */
    hw = &s_key_hw[key];
    return ((DL_GPIO_readPins(hw->port, hw->pin) & hw->pin) == 0U);
}

static void user_key_dispatch_irq(GPIO_Regs *port, uint32_t pin_mask, user_key_id_t key)
{
    uint32_t status;
    uint32_t now;
    uint32_t elapsed;

    /*
     * 只处理已使能且真正挂起的按键中断。GROUP1 可能同时承载多个 GPIO 端口，
     * 因此每个按键都要按自己的端口和 pin mask 单独查询并清除状态。
     */
    status = DL_GPIO_getEnabledInterruptStatus(port, pin_mask);
    if ((status & pin_mask) == 0U) {
        return;
    }

    DL_GPIO_clearInterruptStatus(port, pin_mask);

    /*
     * 简单 20ms 软件消抖。nowtime 由 system_time_init() 启动的 1ms 定时器递增，
     * 因此 user_key_init() 必须在 system_time_init() 之后调用。
     */
    now = nowtime;
    elapsed = (uint32_t)(now - s_last_irq_tick[key]);
    if (elapsed < (USER_KEY_DEBOUNCE_MS * SYSTEM_TIME_TICKS_PER_MS)) {
        return;
    }
    s_last_irq_tick[key] = now;

    /*
     * 下降沿后再读一次物理电平，过滤毛刺或释放抖动导致的误触发。
     */
    if (!user_key_is_pressed(key)) {
        return;
    }

    s_edge_pending |= (uint8_t)(1U << (uint8_t)key);
}

void user_key_init(void)
{
    uint32_t i;

    s_edge_pending = 0U;
    for (i = 0U; i < USER_KEY_COUNT; i++) {
        s_callbacks[i]     = NULL;
        s_last_irq_tick[i] = 0U;
    }

    /*
     * 使用 SysConfig 生成的 IRQn 宏打开 NVIC。对 MSPM0G351x 来说 GPIOA_INT_IRQn
     * 和 GPIOC_INT_IRQn 实际都指向 GROUP1，同值重复调用没有副作用；这样写能清楚
     * 表达 K1/K2 分属两个 GPIO 端口，也方便以后迁移到 IRQ 映射不同的器件。
     */
    NVIC_ClearPendingIRQ(KEY_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(KEY_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(KEY_GPIOC_INT_IRQN);
    NVIC_EnableIRQ(KEY_GPIOC_INT_IRQN);
}

bool user_key_take_edge(user_key_id_t key)
{
    uint8_t mask;

    if (key >= USER_KEY_COUNT) {
        return false;
    }

    mask = (uint8_t)(1U << (uint8_t)key);
    if ((s_edge_pending & mask) == 0U) {
        return false;
    }

    s_edge_pending &= (uint8_t)~mask;
    return true;
}

void user_key_set_callback(user_key_id_t key, user_key_callback_t callback)
{
    if (key >= USER_KEY_COUNT) {
        return;
    }

    s_callbacks[key] = callback;
}

void user_key_clear_callback(user_key_id_t key)
{
    user_key_set_callback(key, NULL);
}

/*
 * GROUP1 中断入口。当前只分发 K1(GPIOA.17) 和 K2(GPIOC.5)，后续如果有新的
 * GROUP1 GPIO 源，需要在这里追加对应的 dispatch 调用，避免误清或漏清中断。
 */
void GROUP1_IRQHandler(void)
{
    user_key_dispatch_irq(KEY_K1_PORT, KEY_K1_PIN, USER_KEY_K1);
    user_key_dispatch_irq(KEY_K2_PORT, KEY_K2_PIN, USER_KEY_K2);
}
