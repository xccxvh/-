#include "LED/led.h"

void LED_init(void)
{
    /* 板载 LED 低电平点亮时，初始化先熄灭 */
    DL_GPIO_clearPins(LED_L1_PORT, LED_L1_PIN);
    DL_GPIO_clearPins(LED_L2_PORT, LED_L2_PIN);
}
