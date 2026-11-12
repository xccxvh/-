#ifndef DRV8874_PORT_H
#define DRV8874_PORT_H

/*
 * DRV8874 四路硬件映射（PH/EN 引脚已按板级实际接线命名）。
 *
 * 速度 EN/IN1（PWM，SysConfig 模块 MOTOR_PWM / TIMG14）：
 * - M1 EN：PA29 → C0 / CCP0
 * - M2 EN：PA30 → C1 / CCP1
 * - M3 EN：PA28 → C2 / CCP2
 * - M4 EN：PB1  → C3 / CCP3
 *
 * 方向 PH/IN2（GPIO，SysConfig 模块 MOTOR / PHx）：
 * - M1 PH：PA31
 * - M2 PH：PB0
 * - M3 PH：PB2
 * - M4 PH：PB3
 *
 * PWM 参数：周期 1000 计数、TimerG 时钟 40 MHz，频率约 40 kHz。
 * speed_permille 的 0~1000 可直接映射到比较值，与 SysConfig 生成配置一致。
 */
#include "bsp_common.h"

/* SysConfig MOTOR_PWM 周期计数；与 ti_msp_dl_config 中 period=1000 保持一致。 */
#define MOTOR_PWM_PERIOD_COUNTS   (1000U)

/* 四路 EN PWM 共用 TIMG14，实例宏由 SysConfig 生成。 */
#define MOTOR_PWM_TIMER_INST      (MOTOR_PWM_INST)

/*
 * IPROPI 电流换算参数。
 *
 * Vadc = Iout(A) * A_IPROPI(A/A) * R_IPROPI(ohm)。
 * 按 DRV8874 典型 A_IPROPI=455uA/A、采样电阻 1.5k 估算，约 682 mV/A。
 */
#define MOTOR_IPROPI_UA_PER_A     (455U)
#define MOTOR_IPROPI_R_OHM        (1500U)
#define MOTOR_IPROPI_MV_PER_A     (682U)

#define MOTOR_ADC_FULL_SCALE      (4095U)
#define MOTOR_ADC_VDDA_MV         (3300U)

typedef struct {
    GPIO_Regs *ph_port;              /* PH/IN2 方向 GPIO 端口。 */
    uint32_t ph_pin;                 /* PH/IN2 方向 GPIO 引脚掩码。 */
    DL_TIMER_CC_INDEX pwm_cc_index;  /* EN/IN1 所在 MOTOR_PWM 比较通道索引。 */
    GPIO_Regs *nsleep_port;
    uint32_t nsleep_pin;
    GPIO_Regs *nfault_port;
    uint32_t nfault_pin;
    uint32_t adc_mem_index;          /* IPROPI 对应 ADC12_0 MEM 索引。 */
} motor_hw_t;

#endif
