#include "MOTOR/drv8874.h"
#include "MOTOR/drv8874_port.h"

/*
 * DRV8874 四路电机底层驱动。
 *
 * 硬件接线（PH/EN 已与 SysConfig 命名对齐）：
 * - MOTOR_PWM C0~C3（PA29/PA30/PA28/PB1）接 DRV8874 EN/IN1，由 TIMG14 硬件 PWM 调速。
 * - MOTOR_PH1~4（PA31/PB0/PB2/PB3）接 DRV8874 PH/IN2，由 GPIO 控制方向。
 *
 * SysConfig 已在 SYSCFG_DL_init() 中完成引脚复用和 MOTOR_PWM 定时器配置；
 * motor_init() 只负责拉低 PH、确认 EN 为 0% 占空比并启动 TIMG14 计数器。
 *
 * 安全约束：
 * - 检测到 nSLEEP=0 或 nFAULT=0 时，立即关断对应路 EN PWM 并将 PH 拉低。
 */

static const motor_hw_t s_motor_hw[MOTOR_COUNT] = {
    [MOTOR_M1] = {
        .ph_port       = MOTOR_PH1_PORT,
        .ph_pin        = MOTOR_PH1_PIN,
        .pwm_cc_index  = GPIO_MOTOR_PWM_C0_IDX,
        .nsleep_port   = MOTOR_nSLEEP1_PORT,
        .nsleep_pin    = MOTOR_nSLEEP1_PIN,
        .nfault_port   = MOTOR_nFAULT1_PORT,
        .nfault_pin    = MOTOR_nFAULT1_PIN,
        .adc_mem_index = ADC12_0_ADCMEM_0,
    },
    [MOTOR_M2] = {
        .ph_port       = MOTOR_PH2_PORT,
        .ph_pin        = MOTOR_PH2_PIN,
        .pwm_cc_index  = GPIO_MOTOR_PWM_C1_IDX,
        .nsleep_port   = MOTOR_nSLEEP2_PORT,
        .nsleep_pin    = MOTOR_nSLEEP2_PIN,
        .nfault_port   = MOTOR_nFAULT2_PORT,
        .nfault_pin    = MOTOR_nFAULT2_PIN,
        .adc_mem_index = ADC12_0_ADCMEM_1,
    },
    [MOTOR_M3] = {
        .ph_port       = MOTOR_PH3_PORT,
        .ph_pin        = MOTOR_PH3_PIN,
        .pwm_cc_index  = GPIO_MOTOR_PWM_C2_IDX,
        .nsleep_port   = MOTOR_nSLEEP3_PORT,
        .nsleep_pin    = MOTOR_nSLEEP3_PIN,
        .nfault_port   = MOTOR_nFAULT3_PORT,
        .nfault_pin    = MOTOR_nFAULT3_PIN,
        .adc_mem_index = ADC12_0_ADCMEM_2,
    },
    [MOTOR_M4] = {
        .ph_port       = MOTOR_PH4_PORT,
        .ph_pin        = MOTOR_PH4_PIN,
        .pwm_cc_index  = GPIO_MOTOR_PWM_C3_IDX,
        .nsleep_port   = MOTOR_nSLEEP4_PORT,
        .nsleep_pin    = MOTOR_nSLEEP4_PIN,
        .nfault_port   = MOTOR_nFAULT4_PORT,
        .nfault_pin    = MOTOR_nFAULT4_PIN,
        .adc_mem_index = ADC12_0_ADCMEM_3,
    },
};

static struct {
    bool enabled;
    int16_t speed_permille;
} s_motor_state[MOTOR_COUNT];

/* 单次 ADC 序列采样后的缓存，避免每次读取某一路电流都重新触发 ADC。 */
static uint16_t s_adc_raw_cache[MOTOR_COUNT];

/*
 * ADC 序列转换完成标志。
 *
 * ADC12_0_INST_IRQHandler 在 MEM3 装载后置位，motor_refresh_adc() 等到该标志
 * 后一次性读取 MEM0~MEM3，对应 M1~M4 的 IPROPI 原始采样值。
 */
static volatile bool s_adc_seq_done;

static void motor_gpio_write(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static bool motor_gpio_read_high(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U);
}

/*
 * 将 0~1000 占空比映射到 TimerG 比较值。
 *
 * SysConfig 生成的 MOTOR_PWM 使用 EDGE_ALIGN 递减计数，CCR 越大高电平越短；
 * 因此需要写入 period-1-duty。duty=0 时写 period-1 保持 EN 低电平；
 * duty=1000 时写 0，接近 100% 高电平。
 */
static uint16_t motor_duty_to_ccr(uint16_t duty)
{
    if (duty >= MOTOR_PWM_PERIOD_COUNTS) {
        return 0U;
    }

    if (duty == 0U) {
        return (uint16_t)(MOTOR_PWM_PERIOD_COUNTS - 1U);
    }

    return (uint16_t)((MOTOR_PWM_PERIOD_COUNTS - 1U) - duty);
}

static bool motor_hw_is_awake(const motor_hw_t *hw)
{
    /* nSLEEP 为低有效睡眠信号：高电平表示外部硬件允许驱动器工作。 */
    return motor_gpio_read_high(hw->nsleep_port, hw->nsleep_pin);
}

static bool motor_hw_is_fault(const motor_hw_t *hw)
{
    /* nFAULT 为低有效故障信号：低电平表示 DRV8874 正在报告过流、过温等异常。 */
    return !motor_gpio_read_high(hw->nfault_port, hw->nfault_pin);
}

static bool motor_hw_can_run(const motor_hw_t *hw)
{
    return motor_hw_is_awake(hw) && !motor_hw_is_fault(hw);
}

static uint16_t motor_adc_raw_to_current_ma(uint16_t raw)
{
    uint32_t current_ma;

    /*
     * I(A) = Vadc / (A_IPROPI * R)
     * I_mA = raw * VDDA_mV * 1000 / (4095 * 682mV/A)
     */
    current_ma = ((uint32_t)raw * (uint32_t)MOTOR_ADC_VDDA_MV * 1000U)
                 / ((uint32_t)MOTOR_ADC_FULL_SCALE *
                    (uint32_t)MOTOR_IPROPI_MV_PER_A);

    if (current_ma > 65535U) {
        return 65535U;
    }

    return (uint16_t)current_ma;
}

/* 写接 DRV8874 EN/IN1 的 PWM 占空比。 */
static void motor_apply_speed_pwm(motor_id_t id, uint16_t duty)
{
    const motor_hw_t *hw = &s_motor_hw[id];

    if (duty > MOTOR_PWM_PERIOD_COUNTS) {
        duty = MOTOR_PWM_PERIOD_COUNTS;
    }

    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_TIMER_INST,
                                     motor_duty_to_ccr(duty),
                                     hw->pwm_cc_index);
}

/* 写接 DRV8874 PH/IN2 的方向电平。forward=true 表示正转方向。 */
static void motor_apply_direction(motor_id_t id, bool forward)
{
    const motor_hw_t *hw = &s_motor_hw[id];

    motor_gpio_write(hw->ph_port, hw->ph_pin, forward);
}

static void motor_force_output_off(motor_id_t id)
{
    motor_apply_speed_pwm(id, 0U);
    motor_apply_direction(id, false);
    s_motor_state[id].enabled = false;
    s_motor_state[id].speed_permille = 0;
}

static void motor_guard_hardware_state(motor_id_t id)
{
    if (!motor_hw_can_run(&s_motor_hw[id])) {
        motor_force_output_off(id);
    }
}

/*
 * 将四路 PH 方向脚拉低，并确认 EN PWM 比较值为 0% 占空比。
 *
 * SysConfig 已把 MOTOR_PHx 配为推挽输出、MOTOR_PWM 引脚配为 TIMG14 复用；
 * 此处不再改写 pinmux，只设置安全的初始电平/占空比。
 */
static void motor_outputs_safe_idle(void)
{
    uint32_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_apply_direction((motor_id_t)i, false);
        motor_apply_speed_pwm((motor_id_t)i, 0U);
    }
}

/*
 * 启动 MOTOR_PWM 计数器。
 *
 * SysConfig 生成代码中 startTimer=STOP，需在此处统一启动，使 EN PWM 输出生效。
 */
static void motor_pwm_timer_start(void)
{
    DL_TimerG_setTimerCount(MOTOR_PWM_TIMER_INST, 0U);
    DL_TimerG_startCounter(MOTOR_PWM_TIMER_INST);
}

/*
 * 将 PA27~PA24 切到模拟输入。
 *
 * IPROPI 信号走外部引脚进入 ADC12_0 的通道 0~3，必须显式配置为模拟功能。
 */
static void motor_adc_gpio_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_ADC12_0_IOMUX_C0);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_ADC12_0_IOMUX_C1);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_ADC12_0_IOMUX_C2);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_ADC12_0_IOMUX_C3);
}

static void motor_adc_irq_init(void)
{
    s_adc_seq_done = false;
    DL_ADC12_clearInterruptStatus(ADC12_0_INST,
                                  DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED);
    NVIC_ClearPendingIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
}

void motor_refresh_adc(void)
{
    uint32_t i;
    uint32_t timeout;

    s_adc_seq_done = false;
    DL_ADC12_clearInterruptStatus(ADC12_0_INST,
                                  DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED);
    DL_ADC12_startConversion(ADC12_0_INST);

    timeout = 500000U;
    while (!s_adc_seq_done && (timeout > 0U)) {
        timeout--;
    }

    for (i = 0U; i < MOTOR_COUNT; i++) {
        s_adc_raw_cache[i] =
            DL_ADC12_getMemResult(ADC12_0_INST, s_motor_hw[i].adc_mem_index);
        motor_guard_hardware_state((motor_id_t)i);
    }

    DL_ADC12_enableConversions(ADC12_0_INST);
}

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
    case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
        s_adc_seq_done = true;
        break;
    default:
        break;
    }
}

void motor_init(void)
{
    uint32_t i;

    motor_outputs_safe_idle();

    for (i = 0U; i < MOTOR_COUNT; i++) {
        s_motor_state[i].enabled = false;
        s_motor_state[i].speed_permille = 0;
    }

    motor_pwm_timer_start();

    motor_adc_gpio_init();
    motor_adc_irq_init();
    motor_refresh_adc();
}

void motor_enable(motor_id_t id, bool enable)
{
    if (id >= MOTOR_COUNT) {
        return;
    }

    if (enable && !motor_hw_can_run(&s_motor_hw[id])) {
        enable = false;
    }

    if (!enable) {
        motor_force_output_off(id);
        return;
    }

    /*
     * enable(true) 只预置正转方向并记录软件使能状态，不主动给 EN PWM 占空比。
     * 实际速度仍由 motor_set_speed() 写入。
     */
    motor_apply_direction(id, true);
    s_motor_state[id].enabled = true;
}

void motor_sleep(motor_id_t id, bool sleep)
{
    if (id >= MOTOR_COUNT) {
        return;
    }

    /*
     * nSLEEP 由外部电路控制，MCU 只读不写。sleep=true 时软件关断 EN PWM。
     */
    if (sleep) {
        motor_force_output_off(id);
    }
}

void motor_set_speed(motor_id_t id, int16_t speed_permille)
{
    uint16_t duty;
    bool forward;

    if (id >= MOTOR_COUNT) {
        return;
    }

    if (speed_permille > 1000) {
        speed_permille = 1000;
    } else if (speed_permille < -1000) {
        speed_permille = -1000;
    }

    if (speed_permille == 0) {
        motor_force_output_off(id);
        return;
    }

    if (!motor_hw_can_run(&s_motor_hw[id])) {
        motor_force_output_off(id);
        return;
    }

    forward = (speed_permille >= 0);
    duty = (uint16_t)(forward ? speed_permille : -speed_permille);

    motor_apply_direction(id, forward);
    motor_apply_speed_pwm(id, duty);
    s_motor_state[id].enabled = true;
    s_motor_state[id].speed_permille = speed_permille;
}

bool motor_is_fault(motor_id_t id)
{
    if (id >= MOTOR_COUNT) {
        return false;
    }

    return motor_hw_is_fault(&s_motor_hw[id]);
}

bool motor_is_asleep(motor_id_t id)
{
    if (id >= MOTOR_COUNT) {
        return false;
    }

    return !motor_hw_is_awake(&s_motor_hw[id]);
}

uint16_t motor_read_current_ma(motor_id_t id)
{
    if (id >= MOTOR_COUNT) {
        return 0U;
    }

    return motor_adc_raw_to_current_ma(s_adc_raw_cache[id]);
}

uint16_t motor_read_adc_raw(motor_id_t id)
{
    if (id >= MOTOR_COUNT) {
        return 0U;
    }

    return s_adc_raw_cache[id];
}

bool motor_get_status(motor_id_t id, motor_status_t *status)
{
    const motor_hw_t *hw;
    bool nsleep_high;
    bool nfault_high;

    if ((id >= MOTOR_COUNT) || (status == NULL)) {
        return false;
    }

    motor_guard_hardware_state(id);

    hw = &s_motor_hw[id];
    nsleep_high = motor_hw_is_awake(hw);
    nfault_high = !motor_hw_is_fault(hw);

    status->enabled         = s_motor_state[id].enabled;
    status->asleep          = !nsleep_high;
    status->fault           = !nfault_high;
    status->nsleep_high     = nsleep_high;
    status->nfault_high     = nfault_high;
    status->speed_permille  = s_motor_state[id].speed_permille;
    status->adc_raw         = motor_read_adc_raw(id);
    status->current_ma      = motor_read_current_ma(id);

    return true;
}

bool motor_any_fault(void)
{
    uint32_t i;
    bool any = false;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (motor_hw_is_fault(&s_motor_hw[i])) {
            motor_force_output_off((motor_id_t)i);
            any = true;
        }
    }

    return any;
}

bool motor_any_asleep(void)
{
    uint32_t i;
    bool any = false;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (!motor_hw_is_awake(&s_motor_hw[i])) {
            motor_force_output_off((motor_id_t)i);
            any = true;
        }
    }

    return any;
}

bool motor_any_running(void)
{
    uint32_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_guard_hardware_state((motor_id_t)i);
        if (s_motor_state[i].enabled &&
            (s_motor_state[i].speed_permille != 0) &&
            motor_hw_can_run(&s_motor_hw[i])) {
            return true;
        }
    }

    return false;
}

void motor_clear_fault(motor_id_t id)
{
    if (id >= MOTOR_COUNT) {
        return;
    }

    motor_force_output_off(id);
    delay_ms(5);
}

void motor_stop_all(void)
{
    uint32_t i;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        motor_force_output_off((motor_id_t)i);
    }
}
