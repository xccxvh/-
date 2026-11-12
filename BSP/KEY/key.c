#include "KEY/key.h"

/*
 * 用户按键模块说明：
 * 1. 本文件负责使能用户按键对应的 NVIC 中断。
 * 2. 当前按键中断触发后，会翻转软件标志位并切换 LED2 状态，便于快速验证按键链路。
 */

uint8_t key_flag = 0;

/*
 * 初始化用户按键中断。
 * GPIO 的复用和边沿配置已经由 SysConfig 生成，因此这里主要负责打开 NVIC 中断。
 */
void key_init(void)
{
    NVIC_EnableIRQ(key_INT_IRQN);
}

/*
 * 用户按键所在 GPIO 组的中断服务函数。
 * 当检测到用户按键触发时，翻转 key_flag，并同步翻转 LED2 作为可视化反馈。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t IRQn_key = DL_GPIO_getEnabledInterruptStatus(key_PORT, key_user_PIN);

    if ((IRQn_key & key_user_PIN) == key_user_PIN)
    {
        key_flag = !key_flag;
        LED2_toggle;
    }
}
