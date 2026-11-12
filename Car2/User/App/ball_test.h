#ifndef APP_BALL_TEST_H
#define APP_BALL_TEST_H

/*
 * 滚球控制测试应用（静态滚球闭环的整机入口）。
 *
 * 本应用与 line_following 并列，由 User/main.c 的编译期宏选择，二者不会同时
 * 运行，也不会修改循迹代码。
 *
 * 支持三种输入源（可在运行时切换）：
 * - MANUAL：不启用 PD，手动给出水平/正倾/反倾小角度，对应测试 A；
 * - SIM：用模拟钢珠位置验证控制逻辑，对应测试 B，无需 Jetson；
 * - UART：接收 Jetson 通过 UART4 发来的真实厘米位置，对应测试 C/D。
 *
 * 按键操作：
 * - K1：MANUAL 下切换手动倾角；SIM/UART 下使能/关闭 AUTO；
 * - K2：SIM/UART 下切换模拟场景/目标位置；MANUAL 下切换手动倾角（与 K1 同效）；
 * - K1+K2 同时按下（先按住 K1 再按 K2）：在 MANUAL -> SIM -> UART 之间切换。
 *
 * 显示与日志：
 * - LCD 采用逐行增量刷新：只有内容变化的行才重写，且两行之间至少间隔
 *   50 ms，避免 8 MHz SPI 整屏刷新阻塞约 60 ms、打乱 30 Hz 控制节拍；
 * - UART4 以 100 ms 周期发送二进制遥测帧，由 Jetson/PC 记录；
 * - 调试输出全部非阻塞，不会拖慢 30 Hz 控制节拍。
 *
 * 头文件规则：应用层头文件只包含公共基础头，不包含聚合头 bsp.h。
 */
#include "bsp_common.h"

/* 测试输入源。 */
typedef enum {
    BALL_TEST_MODE_MANUAL = 0,
    BALL_TEST_MODE_SIM = 1,
    BALL_TEST_MODE_UART = 2
} ball_test_mode_t;

/* 上电默认模式；改为 BALL_TEST_MODE_SIM 可在没有 Jetson 时直接做测试 B。 */
#ifndef BALL_TEST_DEFAULT_MODE
#define BALL_TEST_DEFAULT_MODE          (BALL_TEST_MODE_UART)
#endif

/*
 * 初始化测试应用。
 *
 * 调用顺序（由 main.c 保证）：
 *   SYSCFG_DL_init -> uart0_init -> system_time_init -> BallTest_AppInit
 *
 * 内部会依次初始化 LCD、按键、舵机、视觉链路和 PD 控制器，
 * 然后按 BALL_TEST_DEFAULT_MODE 进入对应模式，状态为 IDLE（未使能 AUTO）。
 */
void BallTest_AppInit(void);

/*
 * 主循环任务：解析视觉、处理按键、注入模拟数据、推进控制器、
 * 发送遥测和刷新 LCD。
 *
 * 约束：
 * - 非阻塞；LCD 刷新周期 200 ms，遥测 100 ms；
 * - 不得在中断中调用。
 */
void BallTest_AppTask(void);

/*
 * 运行时切换测试模式。
 *
 * 参数：
 * - mode：目标模式。
 *
 * 行为：
 * - 切换前先关闭 AUTO 并限速回中；
 * - 进入 MANUAL 时摆杆回水平，手动步进清零；
 * - 进入 SIM 时重置模拟钢珠状态；
 * - 进入 UART 时重置帧转发序号和链路统计。
 */
void BallTest_SetMode(ball_test_mode_t mode);

#endif
