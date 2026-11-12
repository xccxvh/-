/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000



/* Defines for TimerG6_PWM */
#define TimerG6_PWM_INST                                                   TIMG6
#define TimerG6_PWM_INST_IRQHandler                             TIMG6_IRQHandler
#define TimerG6_PWM_INST_INT_IRQN                               (TIMG6_INT_IRQn)
#define TimerG6_PWM_INST_CLK_FREQ                                       80000000
/* GPIO defines for channel 0 */
#define GPIO_TimerG6_PWM_C0_PORT                                           GPIOA
#define GPIO_TimerG6_PWM_C0_PIN                                   DL_GPIO_PIN_29
#define GPIO_TimerG6_PWM_C0_IOMUX                                 (IOMUX_PINCM4)
#define GPIO_TimerG6_PWM_C0_IOMUX_FUNC                IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_TimerG6_PWM_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TimerG6_PWM_C1_PORT                                           GPIOA
#define GPIO_TimerG6_PWM_C1_PIN                                   DL_GPIO_PIN_30
#define GPIO_TimerG6_PWM_C1_IOMUX                                 (IOMUX_PINCM5)
#define GPIO_TimerG6_PWM_C1_IOMUX_FUNC                IOMUX_PINCM5_PF_TIMG6_CCP1
#define GPIO_TimerG6_PWM_C1_IDX                              DL_TIMER_CC_1_INDEX

/* Defines for TimerG0_PWM */
#define TimerG0_PWM_INST                                                   TIMG0
#define TimerG0_PWM_INST_IRQHandler                             TIMG0_IRQHandler
#define TimerG0_PWM_INST_INT_IRQN                               (TIMG0_INT_IRQn)
#define TimerG0_PWM_INST_CLK_FREQ                                          50000
/* GPIO defines for channel 0 */
#define GPIO_TimerG0_PWM_C0_PORT                                           GPIOB
#define GPIO_TimerG0_PWM_C0_PIN                                    DL_GPIO_PIN_0
#define GPIO_TimerG0_PWM_C0_IOMUX                                (IOMUX_PINCM12)
#define GPIO_TimerG0_PWM_C0_IOMUX_FUNC               IOMUX_PINCM12_PF_TIMG0_CCP0
#define GPIO_TimerG0_PWM_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TimerG0_PWM_C1_PORT                                           GPIOB
#define GPIO_TimerG0_PWM_C1_PIN                                    DL_GPIO_PIN_1
#define GPIO_TimerG0_PWM_C1_IOMUX                                (IOMUX_PINCM13)
#define GPIO_TimerG0_PWM_C1_IOMUX_FUNC               IOMUX_PINCM13_PF_TIMG0_CCP1
#define GPIO_TimerG0_PWM_C1_IDX                              DL_TIMER_CC_1_INDEX

/* Defines for TimerA0_PWM */
#define TimerA0_PWM_INST                                                   TIMA0
#define TimerA0_PWM_INST_IRQHandler                             TIMA0_IRQHandler
#define TimerA0_PWM_INST_INT_IRQN                               (TIMA0_INT_IRQn)
#define TimerA0_PWM_INST_CLK_FREQ                                        1000000
/* GPIO defines for channel 0 */
#define GPIO_TimerA0_PWM_C0_PORT                                           GPIOC
#define GPIO_TimerA0_PWM_C0_PIN                                    DL_GPIO_PIN_2
#define GPIO_TimerA0_PWM_C0_IOMUX                                (IOMUX_PINCM76)
#define GPIO_TimerA0_PWM_C0_IOMUX_FUNC               IOMUX_PINCM76_PF_TIMA0_CCP0
#define GPIO_TimerA0_PWM_C0_IDX                              DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TimerA0_PWM_C1_PORT                                           GPIOC
#define GPIO_TimerA0_PWM_C1_PIN                                    DL_GPIO_PIN_4
#define GPIO_TimerA0_PWM_C1_IOMUX                                (IOMUX_PINCM78)
#define GPIO_TimerA0_PWM_C1_IOMUX_FUNC               IOMUX_PINCM78_PF_TIMA0_CCP1
#define GPIO_TimerA0_PWM_C1_IDX                              DL_TIMER_CC_1_INDEX




/* Defines for QEI_M1 */
#define QEI_M1_INST                                                        TIMG8
#define QEI_M1_INST_IRQHandler                                  TIMG8_IRQHandler
#define QEI_M1_INST_INT_IRQN                                    (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_M1 PHA Pin */
#define GPIO_QEI_M1_PHA_PORT                                               GPIOC
#define GPIO_QEI_M1_PHA_PIN                                        DL_GPIO_PIN_6
#define GPIO_QEI_M1_PHA_IOMUX                                    (IOMUX_PINCM84)
#define GPIO_QEI_M1_PHA_IOMUX_FUNC                   IOMUX_PINCM84_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_M1 PHB Pin */
#define GPIO_QEI_M1_PHB_PORT                                               GPIOC
#define GPIO_QEI_M1_PHB_PIN                                        DL_GPIO_PIN_7
#define GPIO_QEI_M1_PHB_IOMUX                                    (IOMUX_PINCM85)
#define GPIO_QEI_M1_PHB_IOMUX_FUNC                   IOMUX_PINCM85_PF_TIMG8_CCP1

/* Defines for QEI_M2 */
#define QEI_M2_INST                                                        TIMG9
#define QEI_M2_INST_IRQHandler                                  TIMG9_IRQHandler
#define QEI_M2_INST_INT_IRQN                                    (TIMG9_INT_IRQn)
/* Pin configuration defines for QEI_M2 PHA Pin */
#define GPIO_QEI_M2_PHA_PORT                                               GPIOB
#define GPIO_QEI_M2_PHA_PIN                                       DL_GPIO_PIN_29
#define GPIO_QEI_M2_PHA_IOMUX                                    (IOMUX_PINCM66)
#define GPIO_QEI_M2_PHA_IOMUX_FUNC                   IOMUX_PINCM66_PF_TIMG9_CCP0
/* Pin configuration defines for QEI_M2 PHB Pin */
#define GPIO_QEI_M2_PHB_PORT                                               GPIOB
#define GPIO_QEI_M2_PHB_PIN                                       DL_GPIO_PIN_30
#define GPIO_QEI_M2_PHB_IOMUX                                    (IOMUX_PINCM67)
#define GPIO_QEI_M2_PHB_IOMUX_FUNC                   IOMUX_PINCM67_PF_TIMG9_CCP1


/* Defines for TimerA1 */
#define TimerA1_INST                                                     (TIMA1)
#define TimerA1_INST_IRQHandler                                 TIMA1_IRQHandler
#define TimerA1_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TimerA1_INST_LOAD_VALUE                                             (9U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           20000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_20_MHZ_115200_BAUD                                      (10)
#define UART_0_FBRD_20_MHZ_115200_BAUD                                      (54)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_5
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_4
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM18)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM17)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM18_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM17_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_1_FBRD_40_MHZ_115200_BAUD                                      (45)
/* Defines for UART_4 */
#define UART_4_INST                                                        UART4
#define UART_4_INST_FREQUENCY                                           80000000
#define UART_4_INST_IRQHandler                                  UART4_IRQHandler
#define UART_4_INST_INT_IRQN                                      UART4_INT_IRQn
#define GPIO_UART_4_RX_PORT                                                GPIOB
#define GPIO_UART_4_TX_PORT                                                GPIOB
#define GPIO_UART_4_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_4_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_4_IOMUX_RX                                     (IOMUX_PINCM28)
#define GPIO_UART_4_IOMUX_TX                                     (IOMUX_PINCM27)
#define GPIO_UART_4_IOMUX_RX_FUNC                      IOMUX_PINCM28_PF_UART4_RX
#define GPIO_UART_4_IOMUX_TX_FUNC                      IOMUX_PINCM27_PF_UART4_TX
#define UART_4_BAUD_RATE                                                  (9600)
#define UART_4_IBRD_80_MHZ_9600_BAUD                                       (520)
#define UART_4_FBRD_80_MHZ_9600_BAUD                                        (53)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI0
#define SPI_0_INST_IRQHandler                                   SPI0_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI0_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                       DL_GPIO_PIN_2
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM15)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM15_PF_SPI0_PICO
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                       DL_GPIO_PIN_3
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM16)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM16_PF_SPI0_SCLK
/* Defines for SPI_1 */
#define SPI_1_INST                                                         SPI1
#define SPI_1_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_1_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_1_PICO_PORT                                              GPIOB
#define GPIO_SPI_1_PICO_PIN                                      DL_GPIO_PIN_15
#define GPIO_SPI_1_IOMUX_PICO                                   (IOMUX_PINCM32)
#define GPIO_SPI_1_IOMUX_PICO_FUNC                   IOMUX_PINCM32_PF_SPI1_PICO
#define GPIO_SPI_1_POCI_PORT                                              GPIOB
#define GPIO_SPI_1_POCI_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_1_IOMUX_POCI                                   (IOMUX_PINCM31)
#define GPIO_SPI_1_IOMUX_POCI_FUNC                   IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for SPI_1 */
#define GPIO_SPI_1_SCLK_PORT                                              GPIOB
#define GPIO_SPI_1_SCLK_PIN                                      DL_GPIO_PIN_16
#define GPIO_SPI_1_IOMUX_SCLK                                   (IOMUX_PINCM33)
#define GPIO_SPI_1_IOMUX_SCLK_FUNC                   IOMUX_PINCM33_PF_SPI1_SCLK



/* Defines for ADC0 */
#define ADC0_INST                                                           ADC0
#define ADC0_INST_IRQHandler                                     ADC0_IRQHandler
#define ADC0_INST_INT_IRQN                                       (ADC0_INT_IRQn)
#define ADC0_ADCMEM_0                                         DL_ADC12_MEM_IDX_0
#define ADC0_ADCMEM_0_REF                  DL_ADC12_REFERENCE_VOLTAGE_VDDA_VREFM
#define ADC0_ADCMEM_1                                         DL_ADC12_MEM_IDX_1
#define ADC0_ADCMEM_1_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC0_ADCMEM_2                                         DL_ADC12_MEM_IDX_2
#define ADC0_ADCMEM_2_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC0_ADCMEM_3                                         DL_ADC12_MEM_IDX_3
#define ADC0_ADCMEM_3_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define GPIO_ADC0_C0_PORT                                                  GPIOA
#define GPIO_ADC0_C0_PIN                                          DL_GPIO_PIN_27
#define GPIO_ADC0_IOMUX_C0                                       (IOMUX_PINCM60)
#define GPIO_ADC0_IOMUX_C0_FUNC                   (IOMUX_PINCM60_PF_UNCONNECTED)
#define GPIO_ADC0_C1_PORT                                                  GPIOA
#define GPIO_ADC0_C1_PIN                                          DL_GPIO_PIN_26
#define GPIO_ADC0_IOMUX_C1                                       (IOMUX_PINCM59)
#define GPIO_ADC0_IOMUX_C1_FUNC                   (IOMUX_PINCM59_PF_UNCONNECTED)
#define GPIO_ADC0_C2_PORT                                                  GPIOA
#define GPIO_ADC0_C2_PIN                                          DL_GPIO_PIN_25
#define GPIO_ADC0_IOMUX_C2                                       (IOMUX_PINCM55)
#define GPIO_ADC0_IOMUX_C2_FUNC                   (IOMUX_PINCM55_PF_UNCONNECTED)
#define GPIO_ADC0_C3_PORT                                                  GPIOA
#define GPIO_ADC0_C3_PIN                                          DL_GPIO_PIN_24
#define GPIO_ADC0_IOMUX_C3                                       (IOMUX_PINCM54)
#define GPIO_ADC0_IOMUX_C3_FUNC                   (IOMUX_PINCM54_PF_UNCONNECTED)

/* Defines for ADC1 */
#define ADC1_INST                                                           ADC1
#define ADC1_INST_IRQHandler                                     ADC1_IRQHandler
#define ADC1_INST_INT_IRQN                                       (ADC1_INT_IRQn)
#define ADC1_ADCMEM_0                                         DL_ADC12_MEM_IDX_0
#define ADC1_ADCMEM_0_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC1_ADCMEM_1                                         DL_ADC12_MEM_IDX_1
#define ADC1_ADCMEM_1_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC1_ADCMEM_2                                         DL_ADC12_MEM_IDX_2
#define ADC1_ADCMEM_2_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define ADC1_ADCMEM_3                                         DL_ADC12_MEM_IDX_3
#define ADC1_ADCMEM_3_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define GPIO_ADC1_C4_PORT                                                  GPIOB
#define GPIO_ADC1_C4_PIN                                          DL_GPIO_PIN_17
#define GPIO_ADC1_IOMUX_C4                                       (IOMUX_PINCM43)
#define GPIO_ADC1_IOMUX_C4_FUNC                   (IOMUX_PINCM43_PF_UNCONNECTED)
#define GPIO_ADC1_C5_PORT                                                  GPIOB
#define GPIO_ADC1_C5_PIN                                          DL_GPIO_PIN_18
#define GPIO_ADC1_IOMUX_C5                                       (IOMUX_PINCM44)
#define GPIO_ADC1_IOMUX_C5_FUNC                   (IOMUX_PINCM44_PF_UNCONNECTED)
#define GPIO_ADC1_C6_PORT                                                  GPIOB
#define GPIO_ADC1_C6_PIN                                          DL_GPIO_PIN_19
#define GPIO_ADC1_IOMUX_C6                                       (IOMUX_PINCM45)
#define GPIO_ADC1_IOMUX_C6_FUNC                   (IOMUX_PINCM45_PF_UNCONNECTED)
#define GPIO_ADC1_C8_PORT                                                  GPIOB
#define GPIO_ADC1_C8_PIN                                          DL_GPIO_PIN_21
#define GPIO_ADC1_IOMUX_C8                                       (IOMUX_PINCM49)
#define GPIO_ADC1_IOMUX_C8_FUNC                   (IOMUX_PINCM49_PF_UNCONNECTED)


/* Defines for VREF */
#define VREF_VOLTAGE_MV                                                     3300
#define GPIO_VREF_VREFPOS_PORT                                             GPIOA
#define GPIO_VREF_VREFPOS_PIN                                     DL_GPIO_PIN_23
#define GPIO_VREF_IOMUX_VREFPOS                                  (IOMUX_PINCM53)
#define GPIO_VREF_IOMUX_VREFPOS_FUNC                IOMUX_PINCM53_PF_UNCONNECTED
#define GPIO_VREF_VREFNEG_PORT                                             GPIOA
#define GPIO_VREF_VREFNEG_PIN                                     DL_GPIO_PIN_21
#define GPIO_VREF_IOMUX_VREFNEG                                  (IOMUX_PINCM46)
#define GPIO_VREF_IOMUX_VREFNEG_FUNC                IOMUX_PINCM46_PF_UNCONNECTED




/* Port definition for Pin Group key */
#define key_PORT                                                         (GPIOB)

/* Defines for user: GPIOB.31 with pinCMx 68 on package pin 27 */
// pins affected by this interrupt request:["user"]
#define key_INT_IRQN                                            (GPIOB_INT_IRQn)
#define key_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define key_user_IIDX                                       (DL_GPIO_IIDX_DIO31)
#define key_user_PIN                                            (DL_GPIO_PIN_31)
#define key_user_IOMUX                                           (IOMUX_PINCM68)
/* Port definition for Pin Group IMU */
#define IMU_PORT                                                         (GPIOB)

/* Defines for CS_IMU: GPIOB.12 with pinCMx 29 on package pin 36 */
#define IMU_CS_IMU_PIN                                          (DL_GPIO_PIN_12)
#define IMU_CS_IMU_IOMUX                                         (IOMUX_PINCM29)
/* Port definition for Pin Group W25Q64 */
#define W25Q64_PORT                                                      (GPIOB)

/* Defines for CS_W25Q64: GPIOB.13 with pinCMx 30 on package pin 37 */
#define W25Q64_CS_W25Q64_PIN                                    (DL_GPIO_PIN_13)
#define W25Q64_CS_W25Q64_IOMUX                                   (IOMUX_PINCM30)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for L1: GPIOA.14 with pinCMx 36 on package pin 43 */
#define LED_L1_PIN                                              (DL_GPIO_PIN_14)
#define LED_L1_IOMUX                                             (IOMUX_PINCM36)
/* Defines for L2: GPIOA.17 with pinCMx 39 on package pin 54 */
#define LED_L2_PIN                                              (DL_GPIO_PIN_17)
#define LED_L2_IOMUX                                             (IOMUX_PINCM39)
/* Port definition for Pin Group keyboard */
#define keyboard_PORT                                                    (GPIOB)

/* Defines for H1: GPIOB.6 with pinCMx 23 on package pin 30 */
#define keyboard_H1_PIN                                          (DL_GPIO_PIN_6)
#define keyboard_H1_IOMUX                                        (IOMUX_PINCM23)
/* Defines for H2: GPIOB.7 with pinCMx 24 on package pin 31 */
#define keyboard_H2_PIN                                          (DL_GPIO_PIN_7)
#define keyboard_H2_IOMUX                                        (IOMUX_PINCM24)
/* Defines for H3: GPIOB.8 with pinCMx 25 on package pin 32 */
#define keyboard_H3_PIN                                          (DL_GPIO_PIN_8)
#define keyboard_H3_IOMUX                                        (IOMUX_PINCM25)
/* Defines for H4: GPIOB.9 with pinCMx 26 on package pin 33 */
#define keyboard_H4_PIN                                          (DL_GPIO_PIN_9)
#define keyboard_H4_IOMUX                                        (IOMUX_PINCM26)
/* Defines for V1: GPIOB.20 with pinCMx 48 on package pin 67 */
#define keyboard_V1_PIN                                         (DL_GPIO_PIN_20)
#define keyboard_V1_IOMUX                                        (IOMUX_PINCM48)
/* Defines for V2: GPIOB.24 with pinCMx 52 on package pin 71 */
#define keyboard_V2_PIN                                         (DL_GPIO_PIN_24)
#define keyboard_V2_IOMUX                                        (IOMUX_PINCM52)
/* Defines for V3: GPIOB.25 with pinCMx 56 on package pin 75 */
#define keyboard_V3_PIN                                         (DL_GPIO_PIN_25)
#define keyboard_V3_IOMUX                                        (IOMUX_PINCM56)
/* Defines for V4: GPIOB.27 with pinCMx 58 on package pin 77 */
#define keyboard_V4_PIN                                         (DL_GPIO_PIN_27)
#define keyboard_V4_IOMUX                                        (IOMUX_PINCM58)
/* Defines for RES: GPIOB.23 with pinCMx 51 on package pin 70 */
#define OLED_RES_PORT                                                    (GPIOB)
#define OLED_RES_PIN                                            (DL_GPIO_PIN_23)
#define OLED_RES_IOMUX                                           (IOMUX_PINCM51)
/* Defines for DC: GPIOC.8 with pinCMx 86 on package pin 65 */
#define OLED_DC_PORT                                                     (GPIOC)
#define OLED_DC_PIN                                              (DL_GPIO_PIN_8)
#define OLED_DC_IOMUX                                            (IOMUX_PINCM86)
/* Defines for CS: GPIOC.9 with pinCMx 87 on package pin 66 */
#define OLED_CS_PORT                                                     (GPIOC)
#define OLED_CS_PIN                                              (DL_GPIO_PIN_9)
#define OLED_CS_IOMUX                                            (IOMUX_PINCM87)
/* Port definition for Pin Group MOTOR */
#define MOTOR_PORT                                                       (GPIOC)

/* Defines for PH2: GPIOC.3 with pinCMx 77 on package pin 51 */
#define MOTOR_PH2_PIN                                            (DL_GPIO_PIN_3)
#define MOTOR_PH2_IOMUX                                          (IOMUX_PINCM77)
/* Defines for PH1: GPIOC.5 with pinCMx 79 on package pin 53 */
#define MOTOR_PH1_PIN                                            (DL_GPIO_PIN_5)
#define MOTOR_PH1_IOMUX                                          (IOMUX_PINCM79)
/* Defines for I1: GPIOB.26 with pinCMx 57 on package pin 76 */
#define Infrared_I1_PORT                                                 (GPIOB)
#define Infrared_I1_PIN                                         (DL_GPIO_PIN_26)
#define Infrared_I1_IOMUX                                        (IOMUX_PINCM57)
/* Defines for I2: GPIOB.28 with pinCMx 65 on package pin 24 */
#define Infrared_I2_PORT                                                 (GPIOB)
#define Infrared_I2_PIN                                         (DL_GPIO_PIN_28)
#define Infrared_I2_IOMUX                                        (IOMUX_PINCM65)
/* Defines for I3: GPIOA.12 with pinCMx 34 on package pin 41 */
#define Infrared_I3_PORT                                                 (GPIOA)
#define Infrared_I3_PIN                                         (DL_GPIO_PIN_12)
#define Infrared_I3_IOMUX                                        (IOMUX_PINCM34)
/* Defines for I4: GPIOA.13 with pinCMx 35 on package pin 42 */
#define Infrared_I4_PORT                                                 (GPIOA)
#define Infrared_I4_PIN                                         (DL_GPIO_PIN_13)
#define Infrared_I4_IOMUX                                        (IOMUX_PINCM35)
/* Defines for I5: GPIOA.16 with pinCMx 38 on package pin 45 */
#define Infrared_I5_PORT                                                 (GPIOA)
#define Infrared_I5_PIN                                         (DL_GPIO_PIN_16)
#define Infrared_I5_IOMUX                                        (IOMUX_PINCM38)
/* Defines for I6: GPIOA.22 with pinCMx 47 on package pin 62 */
#define Infrared_I6_PORT                                                 (GPIOA)
#define Infrared_I6_PIN                                         (DL_GPIO_PIN_22)
#define Infrared_I6_IOMUX                                        (IOMUX_PINCM47)
/* Defines for I7: GPIOA.28 with pinCMx 3 on package pin 3 */
#define Infrared_I7_PORT                                                 (GPIOA)
#define Infrared_I7_PIN                                         (DL_GPIO_PIN_28)
#define Infrared_I7_IOMUX                                         (IOMUX_PINCM3)
/* Defines for I8: GPIOA.31 with pinCMx 6 on package pin 7 */
#define Infrared_I8_PORT                                                 (GPIOA)
#define Infrared_I8_PIN                                         (DL_GPIO_PIN_31)
#define Infrared_I8_IOMUX                                         (IOMUX_PINCM6)


/* Defines for TRNG */
/*
 * The TRNG interrupt is part of INT_GROUP1. Refer to the TRM for more details
 * on interrupt grouping
 */
#define TRNG_INT_IRQN                                            (TRNG_INT_IRQn)
#define TRNG_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_TRNG)




/* Defines for DAC12 */
#define DAC12_IRQHandler                                         DAC0_IRQHandler
#define DAC12_INT_IRQN                                           (DAC0_INT_IRQn)
#define GPIO_DAC12_OUT_PORT                                                GPIOA
#define GPIO_DAC12_OUT_PIN                                        DL_GPIO_PIN_15
#define GPIO_DAC12_IOMUX_OUT                                     (IOMUX_PINCM37)
#define GPIO_DAC12_IOMUX_OUT_FUNC                   IOMUX_PINCM37_PF_UNCONNECTED


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_TimerG6_PWM_init(void);
void SYSCFG_DL_TimerG0_PWM_init(void);
void SYSCFG_DL_TimerA0_PWM_init(void);
void SYSCFG_DL_QEI_M1_init(void);
void SYSCFG_DL_QEI_M2_init(void);
void SYSCFG_DL_TimerA1_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_4_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_SPI_1_init(void);
void SYSCFG_DL_ADC0_init(void);
void SYSCFG_DL_ADC1_init(void);
void SYSCFG_DL_VREF_init(void);

void SYSCFG_DL_TRNG_init(void);
void SYSCFG_DL_DAC12_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
