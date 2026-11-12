/*
 * MG995 舵机调试副工程。
 *
 * 硬件：
 * - 信号：PA9（TIMA0 CCP1，PWM_0 CH1）
 * - 电源：板上 PA9/V+/GND 排针的 V+（实测 5V）
 * - 地：与车板共地
 *
 * PWM 参数：
 * - TIMA0 由 80MHz 经预分频 100 得到 800kHz 计数时钟；
 * - 周期 16000 → 50Hz（20ms）；
 * - 比较值 400~2000 → 0.5~2.5ms，对应 0~180°。
 *
 * 本工程是从 Car（整板硬件测试）复制的独立副工程，不影响 Car2 循迹工程。
 * 上电后舵机自动 0°→180°→0° 往返扫动，LCD 显示当前角度。
 */

#include "ti_msp_dl_config.h"
#include "bsp.h"
#include "SPI0_LCD/lcd.h"
#include "SPI0_LCD/lcd_init.h"

#define SERVO_PULSE_MIN     (400U)     /* 0.5ms -> 0°   */
#define SERVO_PULSE_MAX     (2000U)    /* 2.5ms -> 180° */
#define SERVO_PERIOD        (16000U)   /* 20ms  -> 50Hz */
#define SERVO_PULSE_MID     (1200U)    /* 1.5ms -> 90°  */

/*
 * 把 TIMA0 PWM_0 重新配置为 50Hz 舵机时序。
 *
 * SysConfig 里 PWM_0 默认是 80kHz、周期 1000、定时器停止；
 * 这里改写时钟预分频、周期和 CC1 比较值，并启动计数器。
 * PA9 对应 TIMA0 CCP1，因此使用 DL_TIMER_CC_1_INDEX。
 */
static void servo_pwm_init(void)
{
    static const DL_TimerA_ClockConfig servo_clock = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 99U
    };

    DL_TimerA_setClockConfig(PWM_0_INST, (DL_TimerA_ClockConfig *)&servo_clock);
    DL_TimerA_setLoadValue(PWM_0_INST, SERVO_PERIOD);
    DL_TimerA_setCCPDirection(PWM_0_INST,
                              DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    /*
     * 实测本 TIMA0 配置下，CC1 输出高电平的时间是“周期 - 比较值”。
     * 因此要得到 0.5~2.5ms 的高电平脉宽，比较值应写 16000 - pulse。
     * 初始化先给 90° 对应脉宽 1500，即比较值 14500。
     */
    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
                                     SERVO_PERIOD - SERVO_PULSE_MID,
                                     DL_TIMER_CC_1_INDEX);
    DL_TimerA_startCounter(PWM_0_INST);
}

/* 角度 0~180° 线性映射到 400~2000 比较值。 */
static void servo_set_angle(u32 angle)
{
    u32 pulse;

    if (angle > 180U)
    {
        angle = 180U;
    }

    pulse = SERVO_PULSE_MIN +
            ((SERVO_PULSE_MAX - SERVO_PULSE_MIN) * angle) / 180U;

    /*
     * 高电平时间 = 周期 - 比较值，因此这里写 16000 - pulse，
     * 让 PA9 输出的高脉宽就是 0.5~2.5ms；低电平为 17.5~19.5ms。
     */
    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
                                     SERVO_PERIOD - pulse,
                                     DL_TIMER_CC_1_INDEX);
}

/*
 * 零位标定：
 * - 把钢珠放在摆杆中心 O 点；
 * - 按 K1（KEY1/PA17）每次 +1°，按 K2（KEY2/PC5）每次 -1°；
 * - 调到摆杆水平、钢珠不再向两端滚动为止；
 * - LCD 显示当前角度和对应脉宽，记录此时的角度就是后续 PID 的 SERVO_ZERO_ANGLE。
 */
#define SERVO_ANGLE_MIN     (10U)     /* 避开 MG996R 机械端点 */
#define SERVO_ANGLE_MAX     (170U)
/*
 * 实测标定值：舵机 104° 时摆杆水平，钢珠在中心 O 点附近稳定。
 * 这个值就是后续钢球平衡 PID 的 SERVO_ZERO_ANGLE 零位基准。
 * 注意：拆装舵盘、摆杆或改变配重后零位会漂移，需要重新标定。
 */
#define SERVO_ANGLE_INIT    (104U)

int main(void)
{
    u32 angle = SERVO_ANGLE_INIT;
    u32 lcd_tick;
    u32 pulse;
    u8 changed;

    SYSCFG_DL_init();
    uart0_init(115200U);

    /* user_key 的 20 ms 消抖依赖 1 ms 系统时基，必须先启动 system_time。 */
    system_time_init();
    user_key_init();

    LCD_Init();
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
    LCD_ShowString(0U, 0U, (const u8 *)"SERVO ZERO CAL", WHITE, BLACK, 16U, 0U);
    LCD_ShowString(0U, 20U, (const u8 *)"K1:+1  K2:-1", WHITE, BLACK, 16U, 0U);

    servo_pwm_init();
    servo_set_angle(angle);
    lcd_tick = system_time_get_tick_ms() - 100U;

    while (1)
    {
        changed = 0U;

        if (user_key_take_edge(USER_KEY_K1))
        {
            if (angle < SERVO_ANGLE_MAX)
            {
                angle++;
                changed = 1U;
            }
        }

        if (user_key_take_edge(USER_KEY_K2))
        {
            if (angle > SERVO_ANGLE_MIN)
            {
                angle--;
                changed = 1U;
            }
        }

        if (changed != 0U)
        {
            servo_set_angle(angle);
            lcd_tick = system_time_get_tick_ms() - 100U;
        }

        if (system_time_elapsed_ms(&lcd_tick, 100U))
        {
            pulse = SERVO_PULSE_MIN +
                    ((SERVO_PULSE_MAX - SERVO_PULSE_MIN) * angle) / 180U;

            LCD_ShowString(0U, 50U, (const u8 *)"ANGLE:", WHITE, BLACK, 16U, 0U);
            LCD_ShowIntNum(64U, 50U, (u16)angle, 3U, WHITE, BLACK, 16U);

            LCD_ShowString(0U, 70U, (const u8 *)"PULSE:", WHITE, BLACK, 16U, 0U);
            LCD_ShowIntNum(64U, 70U, (u16)(pulse * 5U / 4U), 4U, WHITE, BLACK, 16U);
            LCD_ShowString(104U, 70U, (const u8 *)"us", WHITE, BLACK, 16U, 0U);

            LCD_ShowString(0U, 90U, (const u8 *)"ZERO = this angle", WHITE, BLACK, 16U, 0U);
        }
    }
}
