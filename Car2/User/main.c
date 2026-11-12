/*
 * G3519 小车循迹主程序。
 *
 * 启动职责：
 * - 调用 SysConfig 生成的初始化函数，使 PA0/PA1 I2C0、LCD SPI、电机 PWM/QEI、
 *   IMU SPI、UART0 调试串口和 1ms 系统时基具备基础硬件配置。
 * - 进入 line_following_app，由应用层完成灰度模块读取、M1/M2 差速循迹和 LCD 状态显示。
 *
 * 运行约束：
 * - M1/M2 用于差速循迹，M3/M4 当前保持停止。
 * - 灰度模块通信失败、灰度线丢失或 M1/M2 驱动故障时，应用层会立即停止电机。
 */
#include "ti_msp_dl_config.h"
#include "bsp.h"                                                              
#include "App/line_following.h"

int main(void)
{
    /*
     * 启动顺序说明：
     * 1. SYSCFG_DL_init() 完成时钟、GPIO、I2C0(PA0/PA1)、SPI、PWM、QEI、ADC、
     *    UART 和 VREF 等基础配置。
     * 2. uart0_init() 打开调试串口，方便观察循迹状态和灰度模块错误计数。
     * 3. system_time_init() 启动 TIMA1 1ms 节拍，循迹控制周期、LCD 刷新和 IMU
     *    姿态积分都依赖该时基。
     * 4. line_following_app_init() 初始化 LCD、IMU、电机/QEI 和灰度模块驱动。
     */
    SYSCFG_DL_init();
    uart0_init(115200U);
    printf("\r\nG3519 Car line follow (g3519_car)\r\n");

    system_time_init();
    line_following_app_init();

    while (1) {
        line_following_app_task();
    }
}
