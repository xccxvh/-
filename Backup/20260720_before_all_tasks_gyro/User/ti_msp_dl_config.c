/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gTimerG6_PWMBackup;
DL_TimerA_backupConfig gTimerA0_PWMBackup;
DL_TimerG_backupConfig gQEI_M1Backup;
DL_TimerA_backupConfig gTimerA1Backup;
DL_SPI_backupConfig gSPI_0Backup;
DL_SPI_backupConfig gSPI_1Backup;
DL_TRNG_backupConfig gTRNGBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_TimerG6_PWM_init();
    SYSCFG_DL_TimerG0_PWM_init();
    SYSCFG_DL_TimerA0_PWM_init();
    SYSCFG_DL_QEI_M1_init();
    SYSCFG_DL_QEI_M2_init();
    SYSCFG_DL_TimerA1_init();
    SYSCFG_DL_I2C_0_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_UART_1_init();
    SYSCFG_DL_UART_4_init();
    SYSCFG_DL_SPI_0_init();
    SYSCFG_DL_SPI_1_init();
    SYSCFG_DL_ADC0_init();
    SYSCFG_DL_ADC1_init();
    SYSCFG_DL_VREF_init();
    SYSCFG_DL_TRNG_init();
    SYSCFG_DL_DAC12_init();
    /* Ensure backup structures have no valid state */
	gTimerG6_PWMBackup.backupRdy 	= false;
	gTimerA0_PWMBackup.backupRdy 	= false;
	gQEI_M1Backup.backupRdy 	= false;
	gTimerA1Backup.backupRdy 	= false;

	gSPI_0Backup.backupRdy 	= false;
	gSPI_1Backup.backupRdy 	= false;
	gTRNGBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(TimerG6_PWM_INST, &gTimerG6_PWMBackup);
	retStatus &= DL_TimerA_saveConfiguration(TimerA0_PWM_INST, &gTimerA0_PWMBackup);
	retStatus &= DL_TimerG_saveConfiguration(QEI_M1_INST, &gQEI_M1Backup);
	retStatus &= DL_TimerA_saveConfiguration(TimerA1_INST, &gTimerA1Backup);
	retStatus &= DL_SPI_saveConfiguration(SPI_0_INST, &gSPI_0Backup);
	retStatus &= DL_SPI_saveConfiguration(SPI_1_INST, &gSPI_1Backup);
	retStatus &= DL_TRNG_saveConfiguration(TRNG, &gTRNGBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(TimerG6_PWM_INST, &gTimerG6_PWMBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TimerA0_PWM_INST, &gTimerA0_PWMBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(QEI_M1_INST, &gQEI_M1Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TimerA1_INST, &gTimerA1Backup, false);
	retStatus &= DL_SPI_restoreConfiguration(SPI_0_INST, &gSPI_0Backup);
	retStatus &= DL_SPI_restoreConfiguration(SPI_1_INST, &gSPI_1Backup);
	retStatus &= DL_TRNG_restoreConfiguration(TRNG, &gTRNGBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_reset(GPIOC);
    DL_TimerG_reset(TimerG6_PWM_INST);
    DL_TimerG_reset(TimerG0_PWM_INST);
    DL_TimerA_reset(TimerA0_PWM_INST);
    DL_TimerG_reset(QEI_M1_INST);
    DL_TimerG_reset(QEI_M2_INST);
    DL_TimerA_reset(TimerA1_INST);
    DL_I2C_reset(I2C_0_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_reset(UART_1_INST);
    DL_UART_Main_reset(UART_4_INST);
    DL_SPI_reset(SPI_0_INST);
    DL_SPI_reset(SPI_1_INST);
    DL_ADC12_reset(ADC0_INST);
    DL_ADC12_reset(ADC1_INST);
    DL_VREF_reset(VREF);
    DL_TRNG_reset(TRNG);
    DL_DAC12_reset(DAC0);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_GPIO_enablePower(GPIOC);
    DL_TimerG_enablePower(TimerG6_PWM_INST);
    DL_TimerG_enablePower(TimerG0_PWM_INST);
    DL_TimerA_enablePower(TimerA0_PWM_INST);
    DL_TimerG_enablePower(QEI_M1_INST);
    DL_TimerG_enablePower(QEI_M2_INST);
    DL_TimerA_enablePower(TimerA1_INST);
    DL_I2C_enablePower(I2C_0_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_UART_Main_enablePower(UART_1_INST);
    DL_UART_Main_enablePower(UART_4_INST);
    DL_SPI_enablePower(SPI_0_INST);
    DL_SPI_enablePower(SPI_1_INST);
    DL_ADC12_enablePower(ADC0_INST);
    DL_ADC12_enablePower(ADC1_INST);
    DL_VREF_enablePower(VREF);
    DL_TRNG_enablePower(TRNG);
    DL_DAC12_enablePower(DAC0);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_TimerG6_PWM_C0_IOMUX,GPIO_TimerG6_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TimerG6_PWM_C0_PORT, GPIO_TimerG6_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TimerG6_PWM_C1_IOMUX,GPIO_TimerG6_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TimerG6_PWM_C1_PORT, GPIO_TimerG6_PWM_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TimerG0_PWM_C0_IOMUX,GPIO_TimerG0_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TimerG0_PWM_C0_PORT, GPIO_TimerG0_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TimerG0_PWM_C1_IOMUX,GPIO_TimerG0_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TimerG0_PWM_C1_PORT, GPIO_TimerG0_PWM_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TimerA0_PWM_C0_IOMUX,GPIO_TimerA0_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TimerA0_PWM_C0_PORT, GPIO_TimerA0_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TimerA0_PWM_C1_IOMUX,GPIO_TimerA0_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TimerA0_PWM_C1_PORT, GPIO_TimerA0_PWM_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_M1_PHA_IOMUX,GPIO_QEI_M1_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_M1_PHB_IOMUX,GPIO_QEI_M1_PHB_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_M2_PHA_IOMUX,GPIO_QEI_M2_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_M2_PHB_IOMUX,GPIO_QEI_M2_PHB_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_1_IOMUX_TX, GPIO_UART_1_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_1_IOMUX_RX, GPIO_UART_1_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_4_IOMUX_TX, GPIO_UART_4_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_4_IOMUX_RX, GPIO_UART_4_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_0_IOMUX_SCLK, GPIO_SPI_0_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_0_IOMUX_PICO, GPIO_SPI_0_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_1_IOMUX_SCLK, GPIO_SPI_1_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_1_IOMUX_PICO, GPIO_SPI_1_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_1_IOMUX_POCI, GPIO_SPI_1_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalInputFeatures(key_user_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(IMU_CS_IMU_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(W25Q64_CS_W25Q64_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(LED_L1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(LED_L2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(keyboard_H1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(keyboard_H2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(keyboard_H3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(keyboard_H4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalInputFeatures(keyboard_V1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(keyboard_V2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(keyboard_V3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(keyboard_V4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(OLED_RES_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(OLED_DC_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(OLED_CS_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(MOTOR_PH2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(MOTOR_PH1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Infrared_I8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, LED_L2_PIN);
    DL_GPIO_setPins(GPIOA, LED_L1_PIN);
    DL_GPIO_enableOutput(GPIOA, LED_L1_PIN |
		LED_L2_PIN);
    DL_GPIO_clearPins(GPIOB, IMU_CS_IMU_PIN |
		W25Q64_CS_W25Q64_PIN |
		keyboard_H1_PIN |
		keyboard_H2_PIN |
		keyboard_H3_PIN |
		keyboard_H4_PIN |
		OLED_RES_PIN);
    DL_GPIO_enableOutput(GPIOB, IMU_CS_IMU_PIN |
		W25Q64_CS_W25Q64_PIN |
		keyboard_H1_PIN |
		keyboard_H2_PIN |
		keyboard_H3_PIN |
		keyboard_H4_PIN |
		OLED_RES_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_31_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, key_user_PIN);
    DL_GPIO_enableInterrupt(GPIOB, key_user_PIN);
    DL_GPIO_clearPins(GPIOC, OLED_DC_PIN |
		OLED_CS_PIN);
    DL_GPIO_setPins(GPIOC, MOTOR_PH2_PIN |
		MOTOR_PH1_PIN);
    DL_GPIO_enableOutput(GPIOC, OLED_DC_PIN |
		OLED_CS_PIN |
		MOTOR_PH2_PIN |
		MOTOR_PH1_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_HFCLK,
	.qDiv                   = 3,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ,100, false);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setHFCLKDividerForMFPCLK(DL_SYSCTL_HFCLK_MFPCLK_DIVIDER_10);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_enableMFPCLK();
	DL_SYSCTL_setMFPCLKSource(DL_SYSCTL_MFPCLK_SOURCE_HFCLK);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);

}


/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTimerG6_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gTimerG6_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 100,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TimerG6_PWM_init(void) {

    DL_TimerG_setClockConfig(
        TimerG6_PWM_INST, (DL_TimerG_ClockConfig *) &gTimerG6_PWMClockConfig);

    DL_TimerG_initPWMMode(
        TimerG6_PWM_INST, (DL_TimerG_PWMConfig *) &gTimerG6_PWMConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(TimerG6_PWM_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(TimerG6_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TimerG6_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 50, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(TimerG6_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TimerG6_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 95, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(TimerG6_PWM_INST);


    DL_TimerG_enableInterrupt(TimerG6_PWM_INST , DL_TIMER_INTERRUPT_LOAD_EVENT);

    DL_TimerG_setCCPDirection(TimerG6_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 8 (5000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   50000 Hz = 5000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerG_ClockConfig gTimerG0_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 99U
};

static const DL_TimerG_PWMConfig gTimerG0_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TimerG0_PWM_init(void) {

    DL_TimerG_setClockConfig(
        TimerG0_PWM_INST, (DL_TimerG_ClockConfig *) &gTimerG0_PWMClockConfig);

    DL_TimerG_initPWMMode(
        TimerG0_PWM_INST, (DL_TimerG_PWMConfig *) &gTimerG0_PWMConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(TimerG0_PWM_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(TimerG0_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_HIGH,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TimerG0_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TimerG0_PWM_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(TimerG0_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_HIGH,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TimerG0_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(TimerG0_PWM_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(TimerG0_PWM_INST);


    
    DL_TimerG_setCCPDirection(TimerG0_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 8 (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 10000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerA_ClockConfig gTimerA0_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 9U
};

static const DL_TimerA_PWMConfig gTimerA0_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TimerA0_PWM_init(void) {

    DL_TimerA_setClockConfig(
        TimerA0_PWM_INST, (DL_TimerA_ClockConfig *) &gTimerA0_PWMClockConfig);

    DL_TimerA_initPWMMode(
        TimerA0_PWM_INST, (DL_TimerA_PWMConfig *) &gTimerA0_PWMConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(TimerA0_PWM_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(TimerA0_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(TimerA0_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 1000, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(TimerA0_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(TimerA0_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 1000, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(TimerA0_PWM_INST);


    
    DL_TimerA_setCCPDirection(TimerA0_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}


static const DL_TimerG_ClockConfig gQEI_M1ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_M1_init(void) {

    DL_TimerG_setClockConfig(
        QEI_M1_INST, (DL_TimerG_ClockConfig *) &gQEI_M1ClockConfig);

    DL_TimerG_configQEI(QEI_M1_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_M1_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_M1_INST, 65535);
    DL_TimerG_enableClock(QEI_M1_INST);
    DL_TimerG_startCounter(QEI_M1_INST);
}
static const DL_TimerG_ClockConfig gQEI_M2ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_M2_init(void) {

    DL_TimerG_setClockConfig(
        QEI_M2_INST, (DL_TimerG_ClockConfig *) &gQEI_M2ClockConfig);

    DL_TimerG_configQEI(QEI_M2_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_M2_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_M2_INST, 65535);
    DL_TimerG_enableClock(QEI_M2_INST);
    DL_TimerG_startCounter(QEI_M2_INST);
}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   100000 Hz = 10000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerA_ClockConfig gTimerA1ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 99U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TimerA1_INST_LOAD_VALUE = (100us * 100000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTimerA1TimerConfig = {
    .period     = TimerA1_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TimerA1_init(void) {

    DL_TimerA_setClockConfig(TimerA1_INST,
        (DL_TimerA_ClockConfig *) &gTimerA1ClockConfig);

    DL_TimerA_initTimerMode(TimerA1_INST,
        (DL_TimerA_TimerConfig *) &gTimerA1TimerConfig);
    DL_TimerA_enableInterrupt(TimerA1_INST , DL_TIMERA_INTERRUPT_LOAD_EVENT);
    DL_TimerA_enableClock(TimerA1_INST);





}


static const DL_I2C_ClockConfig gI2C_0ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_8,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_0_init(void) {

    DL_I2C_setClockConfig(I2C_0_INST,
        (DL_I2C_ClockConfig *) &gI2C_0ClockConfig);
    DL_I2C_disableAnalogGlitchFilter(I2C_0_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_0_INST, 4);
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_disableControllerClockStretching(I2C_0_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_0_INST);


}

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_2
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115273.78
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_20_MHZ_115200_BAUD, UART_0_FBRD_20_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_0_INST);
}
static const DL_UART_Main_ClockConfig gUART_1ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_1Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_1_init(void)
{
    DL_UART_Main_setClockConfig(UART_1_INST, (DL_UART_Main_ClockConfig *) &gUART_1ClockConfig);

    DL_UART_Main_init(UART_1_INST, (DL_UART_Main_Config *) &gUART_1Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, UART_1_IBRD_40_MHZ_115200_BAUD, UART_1_FBRD_40_MHZ_115200_BAUD);



    DL_UART_Main_enable(UART_1_INST);
}
static const DL_UART_Main_ClockConfig gUART_4ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_4Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_4_init(void)
{
    DL_UART_Main_setClockConfig(UART_4_INST, (DL_UART_Main_ClockConfig *) &gUART_4ClockConfig);

    DL_UART_Main_init(UART_4_INST, (DL_UART_Main_Config *) &gUART_4Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.1
     */
    DL_UART_Main_setOversampling(UART_4_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_4_INST, UART_4_IBRD_80_MHZ_9600_BAUD, UART_4_FBRD_80_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_4_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_4_INST);
}

static const DL_SPI_Config gSPI_0_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_0_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_0_init(void) {
    DL_SPI_setClockConfig(SPI_0_INST, (DL_SPI_ClockConfig *) &gSPI_0_clockConfig);

    DL_SPI_init(SPI_0_INST, (DL_SPI_Config *) &gSPI_0_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     20000000 = (80000000)/((1 + 1) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_0_INST, 1);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_0_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(SPI_0_INST);
}
static const DL_SPI_Config gSPI_1_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_1_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_1_init(void) {
    DL_SPI_setClockConfig(SPI_1_INST, (DL_SPI_ClockConfig *) &gSPI_1_clockConfig);

    DL_SPI_init(SPI_1_INST, (DL_SPI_Config *) &gSPI_1_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     200000 = (80000000)/((1 + 199) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_1_INST, 199);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_1_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(SPI_1_INST);
}

/* ADC0 Initialization */
static const DL_ADC12_ClockConfig gADC0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC0_init(void)
{
    DL_ADC12_setClockConfig(ADC0_INST, (DL_ADC12_ClockConfig *) &gADC0ClockConfig);

    DL_ADC12_initSeqSample(ADC0_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_03, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VREFM, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_1,
        DL_ADC12_INPUT_CHAN_1, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_2,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC0_INST, ADC0_ADCMEM_3,
        DL_ADC12_INPUT_CHAN_3, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(ADC0_INST,32000);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC0_INST,(DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED));
    DL_ADC12_enableInterrupt(ADC0_INST,(DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED));
    DL_ADC12_enableConversions(ADC0_INST);
}
/* ADC1 Initialization */
static const DL_ADC12_ClockConfig gADC1ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC1_init(void)
{
    DL_ADC12_setClockConfig(ADC1_INST, (DL_ADC12_ClockConfig *) &gADC1ClockConfig);

    DL_ADC12_initSeqSample(ADC1_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_03, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC1_INST, ADC1_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_4, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC1_INST, ADC1_ADCMEM_1,
        DL_ADC12_INPUT_CHAN_5, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC1_INST, ADC1_ADCMEM_2,
        DL_ADC12_INPUT_CHAN_6, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(ADC1_INST, ADC1_ADCMEM_3,
        DL_ADC12_INPUT_CHAN_8, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_enableConversions(ADC1_INST);
}


static const DL_VREF_Config gVREFConfig = {
    .vrefEnable     = DL_VREF_ENABLE_DISABLE,
    .bufConfig      = DL_VREF_BUFCONFIG_OUTPUT_2_5V,
    .shModeEnable   = DL_VREF_SHMODE_DISABLE,
    .holdCycleCount = DL_VREF_HOLD_MIN,
    .shCycleCount   = DL_VREF_SH_MIN,
};

SYSCONFIG_WEAK void SYSCFG_DL_VREF_init(void) {
    DL_VREF_configReference(VREF,
        (DL_VREF_Config *) &gVREFConfig);
}


SYSCONFIG_WEAK void SYSCFG_DL_TRNG_init(void)
{
    DL_TRNG_setClockDivider(TRNG, DL_TRNG_CLOCK_DIVIDE_6);

    DL_TRNG_sendCommand(TRNG, DL_TRNG_CMD_NORM_FUNC);
    while (!DL_TRNG_isCommandDone(TRNG))
        ;
    DL_TRNG_clearInterruptStatus(TRNG, DL_TRNG_INTERRUPT_CMD_DONE_EVENT);

    DL_TRNG_setDecimationRate(TRNG, DL_TRNG_DECIMATION_RATE_5);
}

static const DL_DAC12_Config gDAC12Config = {
    .outputEnable              = DL_DAC12_OUTPUT_ENABLED,
    .resolution                = DL_DAC12_RESOLUTION_12BIT,
    .representation            = DL_DAC12_REPRESENTATION_BINARY,
    .voltageReferenceSource    = DL_DAC12_VREF_SOURCE_VEREFP_VEREFN,
    .amplifierSetting          = DL_DAC12_AMP_ON,
    .fifoEnable                = DL_DAC12_FIFO_DISABLED,
    .fifoTriggerSource         = DL_DAC12_FIFO_TRIGGER_SAMPLETIMER,
    .dmaTriggerEnable          = DL_DAC12_DMA_TRIGGER_DISABLED,
    .dmaTriggerThreshold       = DL_DAC12_FIFO_THRESHOLD_ONE_QTR_EMPTY,
    .sampleTimeGeneratorEnable = DL_DAC12_SAMPLETIMER_DISABLE,
    .sampleRate                = DL_DAC12_SAMPLES_PER_SECOND_500,
};
SYSCONFIG_WEAK void SYSCFG_DL_DAC12_init(void)
{
    DL_DAC12_init(DAC0, (DL_DAC12_Config *) &gDAC12Config);
    DL_DAC12_output12(DAC0, 1555);
    DL_DAC12_enable(DAC0);
}

