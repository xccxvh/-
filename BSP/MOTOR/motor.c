#include "MOTOR/motor.h"

/*
 * 电机驱动模块（按车底板逐引脚扫描结果重写）。
 *
 * 实测映射：
 *   左轮 M2：TIMG6 CC1（PA30），PWM 周期 100；
 *   右轮 M3：TIMA0 CC3（PA28），PWM 周期 1000。
 *
 * 扫描时单独把 PA30 拉高，左轮正转；单独把 PA28 拉高，右轮正转。
 * 这说明每颗 DRV8874 的另一个逻辑输入在底板上固定为低电平，
 * 因此软件只需要输出一路 PWM 控制速度，方向固定向前。
 * 负值按 0 处理（滑行），不再做反转/刹车；PH1/PH2 保持低电平。
 *
 * 说明：左轮和右轮使用不同定时器、不同周期，因此输入 -1000~1000
 * 分别按周期换算：
 *   左轮：TIMG6 周期 100，比较值 = |pwm| / 10；
 *   右轮：TIMA0 周期 1000，比较值 = |pwm|。
 */

/* 上层约定的 PWM 范围：-1000~1000。 */
static int clamp_motor_pwm(int pwm)
{
    if (pwm > 1000)
    {
        return 1000;
    }

    if (pwm < -1000)
    {
        return -1000;
    }

    return pwm;
}

/*
 * 电机相关外设的一次性初始化：
 * 1. 把 PA28 复用为 TIMA0 CCP3，并打开 CC3 输出方向；
 * 2. 配置 CC3 的输出控制、更新方式和初始比较值；
 * 3. 确保 TIMG6 计数运行，并把左轮 CC1 清零；
 * 4. PH1/PH2 拉低，保持与扫描时一致的“另一个输入为低”状态。
 *
 * 该函数由 set_motor_speed() 首次调用时执行，避免在 main 里再增加
 * 一个初始化调用点。
 */
static void motor_hw_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM3,
                                        IOMUX_PINCM3_PF_TIMA0_CCP3);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_28);

    DL_TimerA_setCaptureCompareOutCtl(TimerA0_PWM_INST,
                                      DL_TIMER_CC_OCTL_INIT_VAL_LOW,
                                      DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
                                      DL_TIMER_CC_OCTL_SRC_FUNCVAL,
                                      DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptCompUpdateMethod(TimerA0_PWM_INST,
                                      DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
                                      DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 0U, DL_TIMER_CC_3_INDEX);

    /* CC0/CC1 由 SysConfig 配置，这里把 CC3 一起加入输出方向掩码。 */
    DL_TimerA_setCCPDirection(TimerA0_PWM_INST,
                              DL_TIMER_CC0_OUTPUT |
                              DL_TIMER_CC1_OUTPUT |
                              DL_TIMER_CC3_OUTPUT);

    DL_TimerG_startCounter(TimerG6_PWM_INST);
    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);

    DL_GPIO_clearPins(MOTOR_PORT, MOTOR_PH1_PIN | MOTOR_PH2_PIN);
}

/*
 * 设置左右轮速度。
 * m1 = 左轮（M2，PA30/TIMG6 CC1）
 * m2 = 右轮（M3，PA28/TIMA0 CC3）
 * 正值输出 PWM，0 或负值输出 0（滑行）。
 */
void set_motor_speed(int m1, int m2)
{
    static uint8_t motor_inited = 0U;

    if (motor_inited == 0U)
    {
        motor_hw_init();
        motor_inited = 1U;
    }

    m1 = clamp_motor_pwm(m1);
    m2 = clamp_motor_pwm(m2);

    if (m1 > 0)
    {
        DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST,
                                         (uint32_t)(m1 / 10),
                                         DL_TIMER_CC_1_INDEX);
    }
    else
    {
        DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
    }

    if (m2 > 0)
    {
        DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST,
                                         (uint32_t)m2,
                                         DL_TIMER_CC_3_INDEX);
    }
    else
    {
        DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 0U, DL_TIMER_CC_3_INDEX);
    }
}

/*
 * 读取两路编码器计数。
 * 要求 2 不使用编码器，保留接口供后续速度闭环使用。
 */
void get_motor_qei_cnt(uint32_t *cnt)
{
    cnt[0] = DL_Timer_getTimerCount(QEI_M1_INST);
    cnt[1] = DL_Timer_getTimerCount(QEI_M2_INST);
}
