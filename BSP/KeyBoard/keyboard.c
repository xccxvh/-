#include "KeyBoard/keyboard.h"

/*
 * 4x4 矩阵键盘模块说明：
 * 1. 本文件采用逐行拉低、逐列读取的方式扫描矩阵键盘。
 * 2. 返回值为对应按键字符，同时根据按键位置触发蜂鸣器发声，便于测试按键和声音反馈链路。
 * 3. GPIO 的复用配置已经由 SysConfig 生成，因此初始化函数当前只保留为空壳接口。
 */

/*
 * 矩阵键盘初始化接口。
 * 当前所有引脚方向和上下拉配置都由 SysConfig 负责，这里保留接口以兼容上层调用。
 */
void keyboard_init(void)
{
}

/*
 * 扫描矩阵键盘并返回按键字符。
 * 若检测到有效按键，会返回 '0'~'9'、'*'、'#'、'A'~'D' 中的一个字符。
 * 若未检测到按键，则返回 0，避免出现数组越界。
 */
char get_keyboard_value(void)
{
    int h_arr[4] = {keyboard_H1_PIN, keyboard_H2_PIN, keyboard_H3_PIN, keyboard_H4_PIN};
    int v_arr[4] = {keyboard_V1_PIN, keyboard_V2_PIN, keyboard_V3_PIN, keyboard_V4_PIN};
    char board[16] = {'1', '4', '7', '*', '2', '5', '8', '0', '3', '6', '9', '#', 'A', 'B', 'C', 'D'};
    int i;
    int j;
    int key_value = 0;

    for (i = 0; i < 4; i++)
    {
        DL_GPIO_clearPins(GPIOB, h_arr[i]);
        DL_GPIO_setPins(GPIOB, h_arr[(i + 1) % 4]);
        DL_GPIO_setPins(GPIOB, h_arr[(i + 2) % 4]);
        DL_GPIO_setPins(GPIOB, h_arr[(i + 3) % 4]);

        delay_cycles(5);

        for (j = 0; j < 4; j++)
        {
            if (DL_GPIO_readPins(keyboard_PORT, v_arr[j]) == 0)
            {
                key_value = j * 4 + i + 1;
            }
        }
    }

    switch (key_value)
    {
    case 1:
        set_led_buzzer(466, 50, 50);
        break;
    case 2:
        set_led_buzzer(630, 50, 50);
        break;
    case 3:
        set_led_buzzer(880, 50, 50);
        break;
    case 5:
        set_led_buzzer(523, 50, 50);
        break;
    case 6:
        set_led_buzzer(698, 50, 50);
        break;
    case 9:
        set_led_buzzer(587, 50, 50);
        break;
    case 10:
        set_led_buzzer(784, 50, 50);
        break;
    default:
        set_led_buzzer(400, 0, 100);
        break;
    }

    if (key_value == 0)
    {
        return 0;
    }

    return board[key_value - 1];
}

