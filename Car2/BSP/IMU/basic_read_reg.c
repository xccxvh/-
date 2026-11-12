#include "bsp_common.h"
#include "SPI1/spi1.h"
#include "IMU/LSM6DSV16X/lsm6dsv16x_reg.h"

/*
 * LSM6DSV IMU 底层寄存器适配层。
 *
 * 职责：
 * - 将 ST 官方 LSM6DSV16X 寄存器驱动的 read_reg/write_reg 回调接到本工程 SPI1。
 * - 按例程中的配置完成 WHO_AM_I 校验、软复位、量程、ODR 和滤波链设置。
 * - 向上层 IMU.c 提供与旧驱动兼容的 setup_imu()/bsp_IcmGetRawData() 接口，
 *   这样 main.c 以及姿态解算层不需要因为芯片型号变化而改调用方式。
 *
 * 当前硬件连接：
 * - SPI1：PB16(SCLK)、PB15(MOSI/PICO)、PB14(MISO/POCI)，由 SysConfig 初始化。
 * - CS：普通 GPIO，当前工程由 SysConfig 生成 IMU_PORT/IMU_CS_PIN，空闲时保持高电平。
 *
 * 时序约束：
 * - LSM6DSV 上电后需要等待 BOOT_TIME，再读取 WHO_AM_I。
 * - SPI 读事务地址最高位置 1，写事务地址最高位保持 0。
 * - 多字节读写期间 CS 必须保持低电平，否则芯片会把后续字节当作新事务。
 */

#define LSM6DSV_BOOT_TIME_MS          (10U)
#define LSM6DSV_RESET_TIMEOUT_MS      (100U)
#define LSM6DSV_STARTUP_SAMPLE_MS     (20U)
#define LSM6DSV_SPI_READ_BIT          (0x80U)
#define LSM6DSV_OK                    (0)
#define LSM6DSV_ERROR                 (-1)

static stmdev_ctx_t lsm6dsv_ctx;
static bool lsm6dsv_ctx_ready = false;
static bool lsm6dsv_sample_valid = false;
static uint8_t lsm6dsv_last_whoami = 0U;
static float lsm6dsv_last_accel_mg[3] = {0.0f, 0.0f, 0.0f};
static float lsm6dsv_last_gyro_dps[3] = {0.0f, 0.0f, 0.0f};
static float lsm6dsv_last_temp_degc = 25.0f;

static int32_t lsm6dsv_platform_write(void *handle, uint8_t reg,
                                      const uint8_t *bufp, uint16_t len);
static int32_t lsm6dsv_platform_read(void *handle, uint8_t reg,
                                     uint8_t *bufp, uint16_t len);
static void lsm6dsv_platform_delay(uint32_t ms);

/*
 * 绑定 ST 官方驱动上下文。
 *
 * 输入输出：
 * - 无显式输入；写入静态 lsm6dsv_ctx。
 * - 绑定后，所有 lsm6dsv16x_*() API 都会通过本文件的 SPI1 回调访问芯片。
 *
 * 实现原因：
 * - 官方驱动只依赖函数指针，不直接知道 MCU 外设；统一在这里适配，可以避免
 *   BSP 其它模块头文件直接包含聚合头 bsp.h。
 */
static void lsm6dsv_bind_context(void)
{
    if (lsm6dsv_ctx_ready) {
        return;
    }

    memset(&lsm6dsv_ctx, 0, sizeof(lsm6dsv_ctx));
    lsm6dsv_ctx.write_reg = lsm6dsv_platform_write;
    lsm6dsv_ctx.read_reg = lsm6dsv_platform_read;
    lsm6dsv_ctx.mdelay = lsm6dsv_platform_delay;
    lsm6dsv_ctx.handle = NULL;
    lsm6dsv_ctx_ready = true;
}

/*
 * SPI 写寄存器回调。
 *
 * 输入：
 * - reg：起始寄存器地址，写操作最高位必须为 0。
 * - bufp/len：待写入的数据缓冲区和长度。
 *
 * 输出：
 * - 返回 0 表示 SPI 事务已完成；返回 -1 表示调用参数非法。
 *
 * 硬件约束：
 * - 写入地址字节和数据字节必须处在同一次 CS 拉低窗口中。
 * - SPI1 的 CPOL/CPHA、位宽、速率由 SysConfig 先完成初始化。
 */
static int32_t lsm6dsv_platform_write(void *handle, uint8_t reg,
                                      const uint8_t *bufp, uint16_t len)
{
    uint16_t i;

    (void)handle;

    if ((bufp == NULL) && (len != 0U)) {
        return LSM6DSV_ERROR;
    }

    SPI1_CS_IMU(0);
    if (!spi1_transfer_byte((uint8_t)(reg & (uint8_t)(~LSM6DSV_SPI_READ_BIT)), NULL)) {
        SPI1_CS_IMU(1);
        return LSM6DSV_ERROR;
    }

    for (i = 0U; i < len; i++) {
        if (!spi1_transfer_byte(bufp[i], NULL)) {
            SPI1_CS_IMU(1);
            return LSM6DSV_ERROR;
        }
    }

    SPI1_CS_IMU(1);
    return LSM6DSV_OK;
}

/*
 * SPI 读寄存器回调。
 *
 * 输入：
 * - reg：起始寄存器地址，读操作由本函数自动置最高位。
 * - bufp/len：接收缓冲区和期望读取长度。
 *
 * 输出：
 * - bufp 被填入连续寄存器数据；返回 0 表示成功。
 *
 * 时序约束：
 * - 读地址发送后，需要继续发送 dummy 字节驱动 SCLK，芯片才会从 MISO 输出数据。
 * - 多字节读取依赖 CTRL3.IF_INC 自动地址递增，setup_imu() 会显式打开该功能。
 */
static int32_t lsm6dsv_platform_read(void *handle, uint8_t reg,
                                     uint8_t *bufp, uint16_t len)
{
    uint16_t i;

    (void)handle;

    if ((bufp == NULL) && (len != 0U)) {
        return LSM6DSV_ERROR;
    }

    SPI1_CS_IMU(0);
    if (!spi1_transfer_byte((uint8_t)(reg | LSM6DSV_SPI_READ_BIT), NULL)) {
        SPI1_CS_IMU(1);
        return LSM6DSV_ERROR;
    }

    for (i = 0U; i < len; i++) {
        if (!spi1_transfer_byte(0x00U, &bufp[i])) {
            SPI1_CS_IMU(1);
            return LSM6DSV_ERROR;
        }
    }

    SPI1_CS_IMU(1);
    return LSM6DSV_OK;
}

/*
 * 官方驱动毫秒延时回调。
 *
 * 输入：
 * - ms：需要阻塞等待的毫秒数。
 *
 * 说明：
 * - 使用 UART0 模块提供的 delay_ms()，该函数在 SYSCFG 初始化后可用。
 * - LSM6DSV 软复位、上电和首批数据稳定都需要毫秒级等待。
 */
static void lsm6dsv_platform_delay(uint32_t ms)
{
    delay_ms(ms);
}

/*
 * 统一打印寄存器驱动错误，避免初始化失败时只停在静默状态。
 *
 * 输入：
 * - stage：当前配置阶段说明。
 * - rc：ST 官方驱动返回值，0 为成功。
 *
 * 输出：
 * - 返回原始 rc，便于调用方继续组合判断。
 */
static int32_t lsm6dsv_report_if_error(const char *stage, int32_t rc)
{
    if (rc != 0) {
        printf("LSM6DSV %s failed, rc=%ld\r\n", stage, (long)rc);
    }

    return rc;
}

/*
 * 初始化 LSM6DSV。
 *
 * 输入：
 * - use_ln：兼容旧 ICM 接口的参数，LSM6DSV 当前按例程的普通高性能采样配置处理。
 * - accel_en：非 0 时打开加速度计。
 * - gyro_en：非 0 时打开陀螺仪。
 *
 * 输出：
 * - 返回 0 表示 WHO_AM_I、软复位、量程、ODR、滤波配置全部成功。
 *
 * 关键配置：
 * - WHO_AM_I 期望值采用例程驱动中的 LSM6DSV16X_ID，LSM6DSV/LSM6DSV16X 系列例程值为 0x70。
 * - 加速度量程 4g，输出换算单位为 mg。
 * - 陀螺仪量程 1000dps，输出换算单位为 dps。
 * - ODR 使用例程的 60Hz，主循环 100ms 打印一次时可读到稳定的新样本。
 */
int setup_imu(int use_ln, int accel_en, int gyro_en)
{
    int32_t rc;
    uint32_t retry;
    uint8_t whoami = 0U;
    lsm6dsv16x_reset_t rst = LSM6DSV16X_GLOBAL_RST;
    lsm6dsv16x_filt_settling_mask_t filt_settling_mask;

    (void)use_ln;

    lsm6dsv_bind_context();
    lsm6dsv_sample_valid = false;
    SPI1_CS_IMU(1);

    delay_ms(LSM6DSV_BOOT_TIME_MS);

    rc = lsm6dsv16x_device_id_get(&lsm6dsv_ctx, &whoami);
    if (lsm6dsv_report_if_error("WHO_AM_I read", rc) != 0) {
        return LSM6DSV_ERROR;
    }

    lsm6dsv_last_whoami = whoami;
    if (whoami != LSM6DSV16X_ID) {
        printf("LSM6DSV WHO_AM_I=0x%02x, expect 0x%02x\r\n",
               (unsigned)whoami, (unsigned)LSM6DSV16X_ID);
        return LSM6DSV_ERROR;
    }

    rc = lsm6dsv16x_reset_set(&lsm6dsv_ctx, LSM6DSV16X_RESTORE_CTRL_REGS);
    if (lsm6dsv_report_if_error("reset set", rc) != 0) {
        return LSM6DSV_ERROR;
    }

    for (retry = 0U; retry < LSM6DSV_RESET_TIMEOUT_MS; retry++) {
        rc = lsm6dsv16x_reset_get(&lsm6dsv_ctx, &rst);
        if (rc != 0) {
            (void)lsm6dsv_report_if_error("reset get", rc);
            return LSM6DSV_ERROR;
        }

        if (rst == LSM6DSV16X_READY) {
            break;
        }

        delay_ms(1U);
    }

    if (rst != LSM6DSV16X_READY) {
        printf("LSM6DSV reset timeout\r\n");
        return LSM6DSV_ERROR;
    }

    /*
     * 复位后立即固定串行接口行为：
     * - 自动地址递增用于 6 字节原始数据连续读取。
     * - 4 线 SPI 对应当前 PB16/PB15/PB14 硬件连接。
     * - 禁用 I2C/I3C 可减少共享引脚误触发，当前板卡只使用 SPI 通信。
     */
    rc = 0;
    rc += lsm6dsv16x_auto_increment_set(&lsm6dsv_ctx, PROPERTY_ENABLE);
    rc += lsm6dsv16x_spi_mode_set(&lsm6dsv_ctx, LSM6DSV16X_SPI_4_WIRE);
    rc += lsm6dsv16x_ui_i2c_i3c_mode_set(&lsm6dsv_ctx, LSM6DSV16X_I2C_I3C_DISABLE);
    rc += lsm6dsv16x_block_data_update_set(&lsm6dsv_ctx, PROPERTY_ENABLE);
    if (lsm6dsv_report_if_error("bus/basic config", rc) != 0) {
        return LSM6DSV_ERROR;
    }

    rc = 0;
    rc += lsm6dsv16x_xl_full_scale_set(&lsm6dsv_ctx, LSM6DSV16X_4g);
    rc += lsm6dsv16x_gy_full_scale_set(&lsm6dsv_ctx, LSM6DSV16X_1000dps);
    rc += lsm6dsv16x_xl_data_rate_set(&lsm6dsv_ctx,
                                      (accel_en != 0) ? LSM6DSV16X_ODR_AT_60Hz : LSM6DSV16X_ODR_OFF);
    rc += lsm6dsv16x_gy_data_rate_set(&lsm6dsv_ctx,
                                      (gyro_en != 0) ? LSM6DSV16X_ODR_AT_60Hz : LSM6DSV16X_ODR_OFF);
    if (lsm6dsv_report_if_error("range/odr config", rc) != 0) {
        return LSM6DSV_ERROR;
    }

    /*
     * 滤波链沿用例程中的轻量滤波：
     * - settling mask 让滤波器未稳定时先屏蔽 DRDY，避免姿态初始阶段读到过渡样本。
     * - 陀螺仪 LP1、加速度 LP2 使用 VERY_LIGHT，保留运动响应，同时压低高频噪声。
     */
    memset(&filt_settling_mask, 0, sizeof(filt_settling_mask));
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = (accel_en != 0) ? PROPERTY_ENABLE : PROPERTY_DISABLE;
    filt_settling_mask.irq_g = (gyro_en != 0) ? PROPERTY_ENABLE : PROPERTY_DISABLE;

    rc = 0;
    rc += lsm6dsv16x_filt_settling_mask_set(&lsm6dsv_ctx, filt_settling_mask);
    rc += lsm6dsv16x_filt_gy_lp1_set(&lsm6dsv_ctx, (gyro_en != 0) ? PROPERTY_ENABLE : PROPERTY_DISABLE);
    rc += lsm6dsv16x_filt_gy_lp1_bandwidth_set(&lsm6dsv_ctx, LSM6DSV16X_GY_VERY_LIGHT);
    rc += lsm6dsv16x_filt_xl_lp2_set(&lsm6dsv_ctx, (accel_en != 0) ? PROPERTY_ENABLE : PROPERTY_DISABLE);
    rc += lsm6dsv16x_filt_xl_lp2_bandwidth_set(&lsm6dsv_ctx, LSM6DSV16X_XL_VERY_LIGHT);
    if (lsm6dsv_report_if_error("filter config", rc) != 0) {
        return LSM6DSV_ERROR;
    }

    delay_ms(LSM6DSV_STARTUP_SAMPLE_MS);
    return LSM6DSV_OK;
}

/*
 * 仅读取 WHO_AM_I。
 *
 * 输入：
 * - whoami：输出指针。
 *
 * 输出：
 * - 返回 0 表示读取成功，*whoami 为芯片 ID。
 *
 * 用途：
 * - IMU_init() 失败后调用本函数，可以快速判断是 SPI/CS 通信问题，还是后续配置阶段失败。
 */
int IMU_probe_whoami(uint8_t *whoami)
{
    int32_t rc;

    if (whoami == NULL) {
        return LSM6DSV_ERROR;
    }

    lsm6dsv_bind_context();
    SPI1_CS_IMU(1);
    delay_ms(LSM6DSV_BOOT_TIME_MS);

    rc = lsm6dsv16x_device_id_get(&lsm6dsv_ctx, whoami);
    if (rc == 0) {
        lsm6dsv_last_whoami = *whoami;
    }

    SPI1_CS_IMU(1);
    return (rc == 0) ? LSM6DSV_OK : LSM6DSV_ERROR;
}

/*
 * 读取并换算 IMU 原始数据。
 *
 * 输入：
 * - accel_mg：长度至少 3 的数组，接收 X/Y/Z 加速度，单位 mg。
 * - gyro_dps：长度至少 3 的数组，接收 X/Y/Z 角速度，单位 dps。
 * - temp_degc：接收温度，单位摄氏度。
 *
 * 输出：
 * - 返回 0 表示读取成功；返回 -1 表示参数非法或寄存器访问失败。
 *
 * 说明：
 * - 函数名保留 bsp_IcmGetRawData 是为了兼容旧 IMU.c 调用点；内部实现已经切换为 LSM6DSV。
 * - 若本次没有新的 DRDY 标志，仍返回最近一次有效样本，避免上层姿态解算因短暂轮询空档得到全零。
 */
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc)
{
    int32_t rc;
    int16_t data_raw_acceleration[3] = {0, 0, 0};
    int16_t data_raw_angular_rate[3] = {0, 0, 0};
    int16_t data_raw_temperature = 0;
    lsm6dsv16x_data_ready_t drdy;

    if ((accel_mg == NULL) || (gyro_dps == NULL) || (temp_degc == NULL)) {
        return LSM6DSV_ERROR;
    }

    lsm6dsv_bind_context();
    memset(&drdy, 0, sizeof(drdy));

    rc = lsm6dsv16x_flag_data_ready_get(&lsm6dsv_ctx, &drdy);
    if (lsm6dsv_report_if_error("data-ready read", rc) != 0) {
        return LSM6DSV_ERROR;
    }

    if ((drdy.drdy_xl != 0U) || (!lsm6dsv_sample_valid)) {
        rc = lsm6dsv16x_acceleration_raw_get(&lsm6dsv_ctx, data_raw_acceleration);
        if (lsm6dsv_report_if_error("acc read", rc) != 0) {
            return LSM6DSV_ERROR;
        }

        lsm6dsv_last_accel_mg[0] = lsm6dsv16x_from_fs4_to_mg(data_raw_acceleration[0]);
        lsm6dsv_last_accel_mg[1] = lsm6dsv16x_from_fs4_to_mg(data_raw_acceleration[1]);
        lsm6dsv_last_accel_mg[2] = lsm6dsv16x_from_fs4_to_mg(data_raw_acceleration[2]);
    }

    if ((drdy.drdy_gy != 0U) || (!lsm6dsv_sample_valid)) {
        rc = lsm6dsv16x_angular_rate_raw_get(&lsm6dsv_ctx, data_raw_angular_rate);
        if (lsm6dsv_report_if_error("gyro read", rc) != 0) {
            return LSM6DSV_ERROR;
        }

        lsm6dsv_last_gyro_dps[0] = lsm6dsv16x_from_fs1000_to_mdps(data_raw_angular_rate[0]) / 1000.0f;
        lsm6dsv_last_gyro_dps[1] = lsm6dsv16x_from_fs1000_to_mdps(data_raw_angular_rate[1]) / 1000.0f;
        lsm6dsv_last_gyro_dps[2] = lsm6dsv16x_from_fs1000_to_mdps(data_raw_angular_rate[2]) / 1000.0f;
    }

    if ((drdy.drdy_temp != 0U) || (!lsm6dsv_sample_valid)) {
        rc = lsm6dsv16x_temperature_raw_get(&lsm6dsv_ctx, &data_raw_temperature);
        if (lsm6dsv_report_if_error("temp read", rc) != 0) {
            return LSM6DSV_ERROR;
        }

        lsm6dsv_last_temp_degc = lsm6dsv16x_from_lsb_to_celsius(data_raw_temperature);
    }

    lsm6dsv_sample_valid = true;
    accel_mg[0] = lsm6dsv_last_accel_mg[0];
    accel_mg[1] = lsm6dsv_last_accel_mg[1];
    accel_mg[2] = lsm6dsv_last_accel_mg[2];
    gyro_dps[0] = lsm6dsv_last_gyro_dps[0];
    gyro_dps[1] = lsm6dsv_last_gyro_dps[1];
    gyro_dps[2] = lsm6dsv_last_gyro_dps[2];
    *temp_degc = lsm6dsv_last_temp_degc;

    return LSM6DSV_OK;
}
