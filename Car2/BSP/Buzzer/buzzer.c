#include "Buzzer/buzzer.h"
#include "PWM1_Shared/pwm1_shared.h"

/*
 * 蜂鸣器（PA16 / TIMG12 CCP1），与 SK6812 共用 PWM_1。
 * 仅在 RGB 数据线已锁定（PA15 GPIO 低）后才会启动，避免改动定时器周期时影响颜色。
 */

static bool s_buzzer_active = false;
static uint16_t s_buzzer_hz = 0U;
static uint8_t s_buzzer_duty = 0U;

void buzzer_init(void)
{
    s_buzzer_active = false;
    s_buzzer_hz = 0U;
    s_buzzer_duty = 0U;
    pwm1_shared_init();
}

void buzzer_play(uint16_t hz, uint8_t duty_percent)
{
    if (duty_percent == 0U) {
        buzzer_stop();
        return;
    }

    if (!pwm1_rgb_is_locked()) {
        return;
    }

    s_buzzer_active = true;
    s_buzzer_hz = hz;
    s_buzzer_duty = duty_percent;
    pwm1_enter_buzzer_pwm_mode(hz, duty_percent);
}

void buzzer_stop(void)
{
    s_buzzer_active = false;
    s_buzzer_hz = 0U;
    s_buzzer_duty = 0U;
    pwm1_buzzer_pwm_stop();
}

bool buzzer_is_active(void)
{
    return s_buzzer_active;
}

void buzzer_restore_after_rgb(void)
{
    if (s_buzzer_active && pwm1_rgb_is_locked()) {
        pwm1_enter_buzzer_pwm_mode(s_buzzer_hz, s_buzzer_duty);
    }
}

void buzzer_get_context(bool *active, uint16_t *hz, uint8_t *duty)
{
    if (active != NULL) {
        *active = s_buzzer_active;
    }
    if (hz != NULL) {
        *hz = s_buzzer_hz;
    }
    if (duty != NULL) {
        *duty = s_buzzer_duty;
    }
}
