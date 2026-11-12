#include "PWM1_Shared/pwm1_shared.h"

#define PWM1_TIMER                TIMG12
#define PWM1_CLK_HZ               PWM_1_INST_CLK_FREQ
#define PWM1_SK6812_PERIOD        (100U)

#define PWM1_LOAD_ICLR            (DL_TIMERG_INTERRUPT_LOAD_EVENT)

static bool s_pa15_locked = false;
static bool s_rgb_tx_active = false;

static void pwm1_timer_halt(void)
{
    DL_TimerG_stopCounter(PWM_1_INST);
    DL_TimerG_disableInterrupt(PWM_1_INST, PWM1_LOAD_ICLR);
    DL_TimerG_clearInterruptStatus(PWM_1_INST, PWM1_LOAD_ICLR);
    NVIC_DisableIRQ(PWM_1_INST_INT_IRQN);
}

void pwm1_shared_init(void)
{
    s_pa15_locked = false;
    s_rgb_tx_active = false;

    DL_TimerG_setCaptureCompareOutCtl(
        PWM_1_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(
        PWM_1_INST,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    pwm1_timer_halt();
    pwm1_rgb_lock_pa15();
}

bool pwm1_rgb_is_locked(void)
{
    return s_pa15_locked;
}

void pwm1_rgb_lock_pa15(void)
{
    pwm1_timer_halt();

    /*
     * 锁定 RGB 数据线：切为 GPIO 推挽低并保持。
     * 蜂鸣器随后改 TIMG12 周期时，PA15 不再接到 PWM 输出，颜色不会被拖偏。
     */
    DL_GPIO_initDigitalOutput(GPIO_PWM_1_C0_IOMUX);
    DL_GPIO_enableOutput(GPIO_PWM_1_C0_PORT, GPIO_PWM_1_C0_PIN);
    DL_GPIO_clearPins(GPIO_PWM_1_C0_PORT, GPIO_PWM_1_C0_PIN);

    PWM1_TIMER->COUNTERREGS.CC_01[0] = 0U;
    s_pa15_locked = true;
    s_rgb_tx_active = false;
}

void pwm1_enter_rgb_tx_mode(void)
{
    pwm1_timer_halt();
    s_pa15_locked = false;
    s_rgb_tx_active = true;

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_1_C0_IOMUX, GPIO_PWM_1_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C0_PORT, GPIO_PWM_1_C0_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_1_C1_IOMUX, GPIO_PWM_1_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C1_PORT, GPIO_PWM_1_C1_PIN);

    DL_TimerG_setLoadValue(PWM_1_INST, PWM1_SK6812_PERIOD);
    DL_TimerG_setCaptCompUpdateMethod(
        PWM_1_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_OR_LOAD_EVT,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(
        PWM_1_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCCPDirection(PWM_1_INST, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);

    PWM1_TIMER->COUNTERREGS.CC_01[0] = 0U;
    PWM1_TIMER->COUNTERREGS.CC_01[1] = 0U;
    DL_Timer_setTimerCount(PWM_1_INST, 0U);
    DL_TimerG_startCounter(PWM_1_INST);
}

void pwm1_enter_buzzer_pwm_mode(uint16_t hz, uint8_t duty_percent)
{
    uint32_t period;
    uint32_t cc1;

    if (!s_pa15_locked) {
        return;
    }

    if (hz < 100U) {
        hz = 100U;
    }
    if (hz > 10000U) {
        hz = 10000U;
    }
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }

    pwm1_timer_halt();

    period = PWM1_CLK_HZ / (uint32_t) hz;
    if (period < 2U) {
        period = 2U;
    }

    if (duty_percent == 0U) {
        cc1 = 0U;
    } else {
        cc1 = (period * (uint32_t) duty_percent) / 100U;
        if (cc1 == 0U) {
            cc1 = 1U;
        }
        if (cc1 >= period) {
            cc1 = period - 1U;
        }
    }

    /* PA15 保持 GPIO 低，仅恢复 PA16 为 CCP1 输出。 */
    DL_GPIO_initDigitalOutput(GPIO_PWM_1_C0_IOMUX);
    DL_GPIO_enableOutput(GPIO_PWM_1_C0_PORT, GPIO_PWM_1_C0_PIN);
    DL_GPIO_clearPins(GPIO_PWM_1_C0_PORT, GPIO_PWM_1_C0_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_1_C1_IOMUX, GPIO_PWM_1_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_1_C1_PORT, GPIO_PWM_1_C1_PIN);

    DL_TimerG_setLoadValue(PWM_1_INST, period);
    DL_TimerG_setCaptCompUpdateMethod(
        PWM_1_INST,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCCPDirection(PWM_1_INST, DL_TIMER_CC1_OUTPUT);

    PWM1_TIMER->COUNTERREGS.CC_01[1] = cc1;
    DL_Timer_setTimerCount(PWM_1_INST, 0U);
    DL_TimerG_startCounter(PWM_1_INST);
}

void pwm1_buzzer_pwm_stop(void)
{
    if (!s_pa15_locked) {
        return;
    }

    pwm1_timer_halt();
    DL_GPIO_clearPins(GPIO_PWM_1_C1_PORT, GPIO_PWM_1_C1_PIN);
    PWM1_TIMER->COUNTERREGS.CC_01[1] = 0U;
}
