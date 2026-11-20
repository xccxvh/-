/*
 * G3519 主程序入口。
 *
 * 本工程包含两套互斥的应用：
 * 1. CAR_APP_LINE_FOLLOW：原有灰度循迹（要求 2：K1 启动/计时/A 点停车）。
 * 2. CAR_APP_BALL_TEST：滚球自动控制测试（静态滚球 PD + 视觉链路）。
 *
 * 选择方式：
 * - 修改下面的 CAR_APP_SELECT 宏后重新编译烧录；
 * - 默认保持 CAR_APP_LINE_FOLLOW，保证原有循迹功能不受影响；
 * - 两套应用共用 SYSCFG_DL_init、UART0 调试口和 1 ms 系统时基，
 *   但不会同时初始化各自的执行机构（电机/灰度 vs 舵机/视觉）。
 *   UART0 在循迹模式下是调试口，在滚球模式下改为 Jetson 二进制协议口。
 *
 * 启动职责：
 * - 调用 SysConfig 生成的初始化函数，使 PA0/PA1 I2C0、LCD SPI、电机 PWM/QEI、
 *   IMU SPI、UART0 调试串口和 1ms 系统时基具备基础硬件配置。
 * - 循迹模式进入 line_following_app；滚球模式进入 ball_test 应用。
 *
 * 运行约束：
 * - 循迹：M1/M2 差速循迹，M3/M4 保持停止；灰度/电机故障时立即停车。
 * - 滚球：舵机接 PA9（TIMA0 CCP1），视觉接 UART0（PA10/PA11，Type-C/DAPLink），
 *   上电默认 IDLE，必须按键使能后才进入 AUTO；printf 文本输出会被关闭。
 */
#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "App/line_following.h"
#include "App/ball_test.h"

/* 应用选择常量：用宏而不是运行时变量，未选中的应用不会被链接进主循环。 */
#define CAR_APP_LINE_FOLLOW             (0)
#define CAR_APP_BALL_TEST               (1)

/*
 * 默认应用。
 *
 * 如需运行滚球控制，把这里改成 CAR_APP_BALL_TEST 再编译；
 * 也可以在 Keil 的 C/C++ Define 中添加 CAR_APP_SELECT=1，无需改源码。
 */
#ifndef CAR_APP_SELECT
#define CAR_APP_SELECT                  (CAR_APP_LINE_FOLLOW)
#endif

int main(void)
{
    /*
     * 公共启动顺序说明：
     * 1. SYSCFG_DL_init() 完成时钟、GPIO、I2C0(PA0/PA1)、SPI、PWM、QEI、ADC、
     *    UART(0/3/4/6) 和 VREF 等基础配置。
     * 2. uart0_init() 打开 UART0：循迹模式继续用于 VOFA/调试；
     *    滚球模式随后关闭 printf，UART0 交给视觉二进制协议独占。
     * 3. system_time_init() 启动 TIMA1 1ms 节拍：
     *    - 循迹的 10 ms 控制 ISR、LCD 刷新和按键消抖依赖它；
     *    - 滚球的舵机斜率限制、视觉超时和速度 dt 也依赖它。
     */
    SYSCFG_DL_init();
    uart0_init(115200U);

    system_time_init();

#if (CAR_APP_SELECT == CAR_APP_BALL_TEST)
    /*
     * 滚球控制模式：
     * - 先关闭 UART0 的 printf 文本，避免调试字符串混入二进制协议；
     * - BallTest_AppInit() 初始化 LCD、按键、舵机 PWM、UART0 视觉链路和 PD；
     * - 主循环非阻塞，30 Hz 视觉帧由 UART0 中断接收，PD 在收到新帧后计算；
     * - 调试信息只显示在 LCD 上。
     */
    uart0_set_output_enable(false);
    printf("\r\nG3519 ball balance control (ball_test)\r\n");
    BallTest_AppInit();

    while (1) {
        BallTest_AppTask();
    }
#else
    /*
     * 循迹模式（默认）：
     * - 保持原有行为完全不变，滚球模块虽然被编译，但不会被调用；
     * - UART0 继续作为 VOFA/调试口；视觉 UART 与舵机 PWM 不会被初始化，
     *   不影响灰度、电机和 WiFi 模块。
     */
    printf("\r\nG3519 Car line follow (g3519_car)\r\n");
    line_following_app_init();

    while (1) {
        line_following_app_task();
    }
#endif
}
