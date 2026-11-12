#include "MYI2C1/myi2c.h"

/*
 * 软件 I2C 位操作实现。
 *
 * 实现原因：
 * - 同一硬件上，参考灰度通信例程使用本方式可稳定通信，而 I2C0 硬件控制器读寄存器
 *   在当前板卡与模块组合下始终 NACK/超时。
 * - 时序与参考例程 CORE_TEST_SIMPLE_PROTOCOL/BSP/MYI2C1 保持一致：每位约 8 us，
 *   对应约 100 kHz 量级。
 *
 * 输入输出：
 * - MyI2C_Init()：释放总线并将 PA0/PA1 设为默认高电平。
 * - MyI2C_Start/Stop/SendByte/ReceiveByte/SendAck/ReceiveAck：标准 I2C 主机原语。
 */

#define MYI2C_BIT_DELAY_US          (8U)

static void myi2c_sda_out(void)
{
    DL_GPIO_initDigitalOutput(GPIO_I2C_SW_IOMUX_SDA);
    DL_GPIO_setPins(GPIO_I2C_SW_SDA_PORT, GPIO_I2C_SW_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_SW_SDA_PORT, GPIO_I2C_SW_SDA_PIN);
}

static void myi2c_scl_out(void)
{
    DL_GPIO_initDigitalOutput(GPIO_I2C_SW_IOMUX_SCL);
    DL_GPIO_setPins(GPIO_I2C_SW_SCL_PORT, GPIO_I2C_SW_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_SW_SCL_PORT, GPIO_I2C_SW_SCL_PIN);
}

static void myi2c_sda_in(void)
{
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_SW_IOMUX_SDA,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
}

static void myi2c_write_scl(uint8_t bit_value)
{
    if (bit_value != 0U) {
        DL_GPIO_setPins(GPIO_I2C_SW_SCL_PORT, GPIO_I2C_SW_SCL_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_I2C_SW_SCL_PORT, GPIO_I2C_SW_SCL_PIN);
    }
    delay_us(MYI2C_BIT_DELAY_US);
}

static void myi2c_write_sda(uint8_t bit_value)
{
    myi2c_sda_out();
    if (bit_value != 0U) {
        DL_GPIO_setPins(GPIO_I2C_SW_SDA_PORT, GPIO_I2C_SW_SDA_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_I2C_SW_SDA_PORT, GPIO_I2C_SW_SDA_PIN);
    }
    delay_us(MYI2C_BIT_DELAY_US);
}

static uint8_t myi2c_read_sda(void)
{
    uint32_t pin_level;

    myi2c_sda_in();
    pin_level = DL_GPIO_readPins(GPIO_I2C_SW_SDA_PORT, GPIO_I2C_SW_SDA_PIN);
    delay_us(MYI2C_BIT_DELAY_US);
    return (pin_level != 0U) ? 1U : 0U;
}

void MyI2C_Init(void)
{
    /*
     * 初始化软件 I2C 总线空闲态。
     * 必须在 SYSCFG_DL_init() 之后调用，以便先完成系统时钟配置再切换 GPIO 模式。
     */
    myi2c_scl_out();
    myi2c_sda_out();
    DL_GPIO_setPins(GPIO_I2C_SW_SDA_PORT, GPIO_I2C_SW_SDA_PIN | GPIO_I2C_SW_SCL_PIN);
}

void MyI2C_Start(void)
{
    myi2c_sda_out();
    myi2c_write_sda(1U);
    myi2c_write_scl(1U);
    myi2c_write_sda(0U);
    myi2c_write_scl(0U);
}

void MyI2C_Stop(void)
{
    myi2c_sda_out();
    myi2c_write_sda(0U);
    myi2c_write_scl(1U);
    myi2c_write_sda(1U);
}

void MyI2C_SendByte(uint8_t byte_value)
{
    uint8_t bit_index;

    myi2c_sda_out();
    myi2c_write_scl(0U);
    for (bit_index = 0U; bit_index < 8U; bit_index++) {
        myi2c_write_sda((uint8_t)((byte_value & (0x80U >> bit_index)) != 0U));
        myi2c_write_scl(1U);
        delay_us(1U);
        myi2c_write_scl(0U);
        delay_us(1U);
    }
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t bit_index;
    uint8_t byte_value = 0U;

    myi2c_write_sda(1U);
    for (bit_index = 0U; bit_index < 8U; bit_index++) {
        myi2c_write_scl(1U);
        if (myi2c_read_sda() != 0U) {
            byte_value |= (uint8_t)(0x80U >> bit_index);
        }
        myi2c_write_scl(0U);
    }

    return byte_value;
}

void MyI2C_SendAck(uint8_t ack_bit)
{
    myi2c_sda_out();
    myi2c_write_sda(ack_bit);
    myi2c_write_scl(1U);
    myi2c_write_scl(0U);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t ack_bit;

    myi2c_write_sda(1U);
    myi2c_write_scl(1U);
    ack_bit = myi2c_read_sda();
    myi2c_write_scl(0U);
    return ack_bit;
}
