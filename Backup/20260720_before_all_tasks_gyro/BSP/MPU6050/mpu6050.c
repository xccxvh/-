#include "MPU6050/mpu6050.h"

/* MPU6050 在 AD0 悬空或接地时的 7 位地址。 */
#define MPU6050_I2C_ADDRESS             (0x68U)

/* 本驱动使用到的寄存器地址，均来自 MPU-6000/MPU-6050 寄存器映射。 */
#define MPU6050_REG_SMPLRT_DIV          (0x19U)
#define MPU6050_REG_CONFIG              (0x1AU)
#define MPU6050_REG_GYRO_CONFIG         (0x1BU)
#define MPU6050_REG_GYRO_ZOUT_H         (0x47U)
#define MPU6050_REG_PWR_MGMT_1          (0x6BU)
#define MPU6050_REG_WHO_AM_I            (0x75U)
#define MPU6050_WHO_AM_I_VALUE          (0x68U)

/*
 * 轮询超时用于防止传感器未上电、接线断开或总线被拉低时程序永久卡死。
 * 该数值按 80 MHz CPU 留出充足余量，不承担精确计时职责。
 */
#define MPU6050_I2C_POLL_TIMEOUT        (200000UL)

static bool mpu6050_wait_controller_idle(void)
{
    uint32_t timeout = MPU6050_I2C_POLL_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
        if ((DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
        {
            return false;
        }

        if (timeout-- == 0U)
        {
            return false;
        }
    }

    return true;
}

static bool mpu6050_wait_controller_done(void)
{
    uint32_t timeout = MPU6050_I2C_POLL_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U)
    {
        if (timeout-- == 0U)
        {
            return false;
        }
    }

    return ((DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_ERROR) == 0U);
}

/*
 * 写单个寄存器。发送缓存只有“寄存器地址+数据”两个字节，不会超过控制器 FIFO；
 * 每次传输前清空旧数据并等待总线空闲，避免上一次异常传输污染本次配置命令。
 */
static bool mpu6050_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg, value};

    if (!mpu6050_wait_controller_idle())
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, tx_data, 2U);
    DL_I2C_startControllerTransfer(I2C_0_INST,
                                   MPU6050_I2C_ADDRESS,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   2U);

    if (!mpu6050_wait_controller_done())
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return false;
    }

    return true;
}

/*
 * 先写入内部寄存器指针，再发起读传输。MPU6050 允许在两个阶段之间产生 STOP，
 * 因此可直接使用 DriverLib 的两次控制器传输；接收阶段逐字节排空 FIFO，既支持
 * WHO_AM_I 单字节读取，也支持陀螺仪连续寄存器读取。
 */
static bool mpu6050_read_registers(uint8_t start_reg,
                                   uint8_t *data,
                                   uint8_t length)
{
    uint8_t index;
    uint32_t timeout;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    if (!mpu6050_wait_controller_idle())
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &start_reg, 1U);
    DL_I2C_startControllerTransfer(I2C_0_INST,
                                   MPU6050_I2C_ADDRESS,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1U);
    if (!mpu6050_wait_controller_done() ||
        !mpu6050_wait_controller_idle())
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_startControllerTransfer(I2C_0_INST,
                                   MPU6050_I2C_ADDRESS,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   length);

    for (index = 0U; index < length; index++)
    {
        timeout = MPU6050_I2C_POLL_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST))
        {
            if ((DL_I2C_getControllerStatus(I2C_0_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
            {
                DL_I2C_resetControllerTransfer(I2C_0_INST);
                return false;
            }

            if (timeout-- == 0U)
            {
                DL_I2C_resetControllerTransfer(I2C_0_INST);
                return false;
            }
        }

        data[index] = DL_I2C_receiveControllerData(I2C_0_INST);
    }

    if (!mpu6050_wait_controller_done())
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return false;
    }

    return true;
}

bool mpu6050_init(void)
{
    uint8_t who_am_i = 0U;

    /* 上电后留出稳定时间，再读取身份寄存器，防止模块稳压器尚未稳定造成误判。 */
    delay_ms(100U);
    if (!mpu6050_read_registers(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) ||
        (who_am_i != MPU6050_WHO_AM_I_VALUE))
    {
        return false;
    }

    /*
     * 使用 X 轴陀螺 PLL 作为时钟并退出睡眠；数字低通设为约 44 Hz，1 kHz 内部
     * 采样经 4 分频得到 200 Hz。±500 dps 可覆盖小车原地转向且保留足够分辨率。
     */
    if (!mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x01U) ||
        !mpu6050_write_register(MPU6050_REG_CONFIG, 0x03U) ||
        !mpu6050_write_register(MPU6050_REG_SMPLRT_DIV, 0x04U) ||
        !mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, 0x08U))
    {
        return false;
    }

    delay_ms(50U);
    return true;
}

bool mpu6050_read_gyro_z_raw(int16_t *gyro_z_raw)
{
    uint8_t raw_data[2];

    if ((gyro_z_raw == NULL) ||
        !mpu6050_read_registers(MPU6050_REG_GYRO_ZOUT_H, raw_data, 2U))
    {
        return false;
    }

    /* MPU6050 输出为高字节在前的 16 位补码，必须显式拼接后再解释为有符号数。 */
    *gyro_z_raw = (int16_t)(((uint16_t)raw_data[0] << 8U) |
                           (uint16_t)raw_data[1]);
    return true;
}

bool mpu6050_calibrate_gyro_z(uint16_t sample_count,
                              uint16_t sample_period_ms,
                              int32_t *bias_raw)
{
    int16_t gyro_z_raw;
    int32_t sum = 0;
    uint16_t sample;

    if ((bias_raw == NULL) || (sample_count == 0U))
    {
        return false;
    }

    /* 校准期间小车必须静止；平均多个样本可抑制单次噪声，结果保留原始 LSB 单位。 */
    for (sample = 0U; sample < sample_count; sample++)
    {
        if (!mpu6050_read_gyro_z_raw(&gyro_z_raw))
        {
            return false;
        }

        sum += gyro_z_raw;
        delay_ms(sample_period_ms);
    }

    *bias_raw = sum / (int32_t)sample_count;
    return true;
}
