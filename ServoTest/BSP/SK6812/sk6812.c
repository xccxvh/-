#include "SK6812/sk6812.h"
#include "PWM1_Shared/pwm1_shared.h"
#include "Buzzer/buzzer.h"

/*
 * SK6812（PA15 / TIMG12 CCP0）+ 与蜂鸣器（PA16 / CCP1）共用 PWM_1。
 *
 * 发送方式：PWM 800kHz 位时序 + LOAD 中断逐位更新 CC0 比较值。
 * - sk6812_apply_color()：切 RGB 模式 → 先刷无色 → 再刷目标色 → 锁定 PA15 → 恢复蜂鸣器；
 * - 发码期间屏蔽 UART，TIMG12 中断最高优先级。
 */

#define SK6812_PWM_INST           PWM_1_INST
#define SK6812_PWM_IRQN           PWM_1_INST_INT_IRQN

#define SK6812_BIT_PERIOD         (100U)
#define SK6812_DUTY_0             (75U)
#define SK6812_DUTY_1             (50U)
#define SK6812_BITS_PER_LED       (24U)
#define SK6812_STREAM_MAX         (SK6812_LED_COUNT * SK6812_BITS_PER_LED)

#define SK6812_TIMER              TIMG12
#define SK6812_CC_REG             (SK6812_TIMER->COUNTERREGS.CC_01[0])
#define SK6812_LOAD_ICLR          (DL_TIMERG_INTERRUPT_LOAD_EVENT)

static sk6812_color_t s_led_buffer[SK6812_LED_COUNT];
static uint8_t s_pwm_stream[SK6812_STREAM_MAX];

static volatile bool s_tx_busy = false;
static volatile uint16_t s_tx_index = 0U;
static volatile uint16_t s_tx_length = 0U;

uint8_t sk6812_brightness_scale(uint8_t channel)
{
    return (uint8_t)(((uint16_t)channel * (uint16_t)SK6812_BRIGHTNESS_PERCENT) / 100U);
}

static void sk6812_encode_byte(uint8_t value, uint8_t *out)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & (uint8_t)(0x80U >> bit)) != 0U) {
            out[bit] = SK6812_DUTY_1;
        } else {
            out[bit] = SK6812_DUTY_0;
        }
    }
}

static void sk6812_build_stream(void)
{
    uint16_t led;
    uint16_t offset = 0U;

    for (led = 0U; led < SK6812_LED_COUNT; led++) {
        sk6812_encode_byte(s_led_buffer[led].g, &s_pwm_stream[offset]);
        offset = (uint16_t)(offset + 8U);
        sk6812_encode_byte(s_led_buffer[led].r, &s_pwm_stream[offset]);
        offset = (uint16_t)(offset + 8U);
        sk6812_encode_byte(s_led_buffer[led].b, &s_pwm_stream[offset]);
        offset = (uint16_t)(offset + 8U);
    }
}

static void sk6812_pwm_load_irq_enable(bool enable)
{
    if (enable) {
        DL_TimerG_clearInterruptStatus(SK6812_PWM_INST, SK6812_LOAD_ICLR);
        DL_TimerG_enableInterrupt(SK6812_PWM_INST, SK6812_LOAD_ICLR);
    } else {
        DL_TimerG_disableInterrupt(SK6812_PWM_INST, SK6812_LOAD_ICLR);
        DL_TimerG_clearInterruptStatus(SK6812_PWM_INST, SK6812_LOAD_ICLR);
    }
}

static void sk6812_nvic_configure(void)
{
    NVIC_SetPriority(SK6812_PWM_IRQN, 0U);
    NVIC_SetPriority(TIMER_A1_1MS_INST_INT_IRQN, 3U);
    NVIC_SetPriority(UART_0_INST_INT_IRQN, 3U);
}

static void sk6812_mask_irqs_during_tx(void)
{
    NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
}

static void sk6812_unmask_irqs_after_tx(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

static void sk6812_cc0_bind_load_update(void)
{
    DL_TimerG_setCaptCompUpdateMethod(
        SK6812_PWM_INST,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_OR_LOAD_EVT,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
}

static void sk6812_reset_pulse(void)
{
    SK6812_CC_REG = 0U;
    delay_us(80U);
}

static bool sk6812_refresh_raw(void)
{
    uint32_t primask;

    if (s_tx_busy) {
        return false;
    }

    sk6812_build_stream();
    sk6812_cc0_bind_load_update();

    s_tx_length = SK6812_STREAM_MAX;
    s_tx_index = 1U;
    s_tx_busy = true;

    primask = __get_PRIMASK();
    __disable_irq();

    NVIC_ClearPendingIRQ(SK6812_PWM_IRQN);
    SK6812_CC_REG = s_pwm_stream[0];
    sk6812_pwm_load_irq_enable(true);
    NVIC_EnableIRQ(SK6812_PWM_IRQN);

    if (primask == 0U) {
        __enable_irq();
    }

    sk6812_mask_irqs_during_tx();

    while (s_tx_busy) {
        __NOP();
    }

    sk6812_unmask_irqs_after_tx();
    sk6812_reset_pulse();

    return true;
}

static void sk6812_finish_rgb_session(void)
{
    pwm1_rgb_lock_pa15();
    buzzer_restore_after_rgb();
}

void sk6812_init(void)
{
    s_tx_busy = false;
    s_tx_index = 0U;
    s_tx_length = 0U;
    sk6812_clear();
    sk6812_nvic_configure();
    buzzer_init();
}

void sk6812_clear(void)
{
    uint16_t i;

    for (i = 0U; i < SK6812_LED_COUNT; i++) {
        s_led_buffer[i].g = 0U;
        s_led_buffer[i].r = 0U;
        s_led_buffer[i].b = 0U;
    }
}

void sk6812_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= SK6812_LED_COUNT) {
        return;
    }

    s_led_buffer[index].g = sk6812_brightness_scale(g);
    s_led_buffer[index].r = sk6812_brightness_scale(r);
    s_led_buffer[index].b = sk6812_brightness_scale(b);
}

void sk6812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t i;

    for (i = 0U; i < SK6812_LED_COUNT; i++) {
        sk6812_set_pixel(i, r, g, b);
    }
}

bool sk6812_is_busy(void)
{
    return s_tx_busy;
}

bool sk6812_rgb_is_locked(void)
{
    return pwm1_rgb_is_locked();
}

bool sk6812_refresh(void)
{
    if (s_tx_busy) {
        return false;
    }

    pwm1_enter_rgb_tx_mode();
    if (!sk6812_refresh_raw()) {
        pwm1_rgb_lock_pa15();
        return false;
    }

    sk6812_finish_rgb_session();
    return true;
}

bool sk6812_apply_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_tx_busy) {
        return false;
    }

    pwm1_enter_rgb_tx_mode();

    sk6812_set_all(0U, 0U, 0U);
    if (!sk6812_refresh_raw()) {
        pwm1_rgb_lock_pa15();
        return false;
    }

    sk6812_set_all(r, g, b);
    if (!sk6812_refresh_raw()) {
        pwm1_rgb_lock_pa15();
        buzzer_restore_after_rgb();
        return false;
    }

    sk6812_finish_rgb_session();
    return true;
}

void PWM_1_INST_IRQHandler(void)
{
    uint16_t idx;

    SK6812_TIMER->CPU_INT.ICLR = SK6812_LOAD_ICLR;

    if (!s_tx_busy) {
        return;
    }

    idx = s_tx_index;
    if (idx < s_tx_length) {
        SK6812_CC_REG = s_pwm_stream[idx];
        s_tx_index = (uint16_t)(idx + 1U);
        return;
    }

    s_tx_busy = false;
    SK6812_CC_REG = 0U;
    SK6812_TIMER->CPU_INT.IMASK &= ~SK6812_LOAD_ICLR;
    NVIC_DisableIRQ(SK6812_PWM_IRQN);
}
