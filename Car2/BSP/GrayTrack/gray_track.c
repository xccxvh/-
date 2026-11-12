#include "GrayTrack/gray_track.h"

/*
 * 8 路数字灰度驱动（替代原 I2C 智能模块驱动）。
 *
 * 用户实际接线：
 *   OUT1 -> PA1      OUT2 -> PA0      OUT3 -> PB16     OUT4 -> PB14
 *   OUT5 -> PB15     OUT6 -> PB27     OUT7 -> PB26     OUT8 -> PC8
 *
 * 这些引脚在厂家 Car2 工程里原本分别属于 I2C_GRAY、IMU 的 SPI1、
 * KEYBoard 矩阵，因此：
 * 1. ti_msp_dl_config.c 已把它们改成 GPIO 上拉输入；
 * 2. line_following_app_init() 不再初始化 IMU/SPI1；
 * 3. 本文件用 DL_GPIO_readPins() 读 8 路电平，组成 compare_bits。
 *
 * 有效电平：本车底盘较低，实测白地板时 OUT 为低、灰线位于探头下方时
 * OUT 为高（旧逻辑电平=0 时 LCD 显示白地板 1000、灰线 0000）。
 * 循迹需要“压线 = 有效”，因此这里改为高电平有效：
 * 灰线时对应位置 1、显示 1000；白地板位置 0、显示 0000。
 *
 * 原 I2C 版本已备份为 gray_track.c.i2c.bak。
 */

#define GRAY_DIGITAL_BLACK_LEVEL       (1U)

/* 8 路灰度引脚定义：顺序必须与 OUT1~OUT8 的实际接线一致。 */
typedef struct
{
    GPIO_Regs *port;
    uint32_t pin;
    uint32_t iomux;
} GrayDigitalPin;

static const GrayDigitalPin s_gray_pins[GRAY_TRACK_CHANNEL_COUNT] = {
    {GPIOA, DL_GPIO_PIN_1,  IOMUX_PINCM2},   /* OUT1: PA1  */
    {GPIOA, DL_GPIO_PIN_0,  IOMUX_PINCM1},   /* OUT2: PA0  */
    {GPIOB, DL_GPIO_PIN_16, IOMUX_PINCM33},  /* OUT3: PB16 */
    {GPIOB, DL_GPIO_PIN_14, IOMUX_PINCM31},  /* OUT4: PB14 */
    {GPIOB, DL_GPIO_PIN_15, IOMUX_PINCM32},  /* OUT5: PB15 */
    /*
     * 实测物理探头 6/7/8 分别落在原 G8/G6/G7 上，
     * 这里按实际顺序重排为 G6=PC8、G7=PB27、G8=PB26。
     */
    {GPIOC, DL_GPIO_PIN_8,  IOMUX_PINCM86},  /* G6: 物理传感器6（PC8） */
    {GPIOB, DL_GPIO_PIN_27, IOMUX_PINCM58},  /* G7: 物理传感器7（PB27）*/
    {GPIOB, DL_GPIO_PIN_26, IOMUX_PINCM57},  /* G8: 物理传感器8（PB26）*/
};

static uint8_t s_heartbeat;
static uint8_t s_sample_counter;
static uint16_t s_threshold = GRAY_TRACK_DEFAULT_LED_THRESHOLD;
static gray_track_result_t s_last_result = GRAY_TRACK_RESULT_OK;

/*
 * 读取 8 路原始电平并归一化成黑线位图。
 * bit0~bit7 依次对应 OUT1~OUT8；1 表示压黑线。
 */
static uint8_t gray_digital_read_mask(void)
{
    uint8_t mask = 0U;
    uint8_t i;

    for (i = 0U; i < GRAY_TRACK_CHANNEL_COUNT; i++)
    {
        uint8_t level = (DL_GPIO_readPins(s_gray_pins[i].port,
                                          s_gray_pins[i].pin) != 0U) ? 1U : 0U;

        if (level == (uint8_t)GRAY_DIGITAL_BLACK_LEVEL)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

/*
 * 初始化 8 路 GPIO 为上拉输入。
 * SysConfig 生成的代码已经把冲突的 I2C/SPI1/KEYBoard 配置改掉了，
 * 这里再配置一次，保证调用顺序变化时也不会把引脚变成输出。
 */
void gray_track_init(void)
{
    uint8_t i;

    for (i = 0U; i < GRAY_TRACK_CHANNEL_COUNT; i++)
    {
        DL_GPIO_initDigitalInputFeatures(s_gray_pins[i].iomux,
                                         DL_GPIO_INVERSION_DISABLE,
                                         DL_GPIO_RESISTOR_PULL_UP,
                                         DL_GPIO_HYSTERESIS_DISABLE,
                                         DL_GPIO_WAKEUP_DISABLE);
    }

    s_heartbeat = 0U;
    s_sample_counter = 0U;
    s_last_result = GRAY_TRACK_RESULT_OK;
}

/*
 * 数字灰度没有可写的 LED 阈值，这里只记录应用层传入的值，
 * 保持 line_following_app_init() 的“写阈值并回读”流程可用。
 */
bool gray_track_set_led_threshold(uint16_t threshold)
{
    if (threshold > GRAY_TRACK_LED_THRESHOLD_MAX)
    {
        threshold = GRAY_TRACK_LED_THRESHOLD_MAX;
    }

    s_threshold = threshold;
    s_last_result = GRAY_TRACK_RESULT_OK;
    return true;
}

bool gray_track_get_led_threshold(uint16_t *threshold)
{
    if (threshold == NULL)
    {
        s_last_result = GRAY_TRACK_RESULT_PARAM_ERROR;
        return false;
    }

    *threshold = s_threshold;
    s_last_result = GRAY_TRACK_RESULT_OK;
    return true;
}

bool gray_track_read_compare_bits(uint8_t *compare_bits)
{
    if (compare_bits == NULL)
    {
        s_last_result = GRAY_TRACK_RESULT_PARAM_ERROR;
        return false;
    }

    *compare_bits = gray_digital_read_mask();
    s_last_result = GRAY_TRACK_RESULT_OK;
    return true;
}

/*
 * LCD 的 G1~G8 原本显示 ADC 原始值，这里用 0/1000 表示该路是否压线，
 * 方便现场直接看出哪几路有效。
 */
bool gray_track_read_adc(uint16_t adc[GRAY_TRACK_CHANNEL_COUNT])
{
    uint8_t mask;
    uint8_t i;

    if (adc == NULL)
    {
        s_last_result = GRAY_TRACK_RESULT_PARAM_ERROR;
        return false;
    }

    mask = gray_digital_read_mask();
    for (i = 0U; i < GRAY_TRACK_CHANNEL_COUNT; i++)
    {
        adc[i] = ((mask & (uint8_t)(1U << i)) != 0U) ? 1000U : 0U;
    }

    s_last_result = GRAY_TRACK_RESULT_OK;
    return true;
}

/*
 * 状态区读取：用实时 GPIO 位图填充 compare_bits；
 * heartbeat/sample_counter 每次调用递增，LCD 上能看到 Hb 在跳，
 * 证明灰度数据在刷新。
 */
bool gray_track_read_status(gray_track_snapshot_t *snapshot)
{
    uint8_t mask;
    uint8_t i;

    if (snapshot == NULL)
    {
        s_last_result = GRAY_TRACK_RESULT_PARAM_ERROR;
        return false;
    }

    mask = gray_digital_read_mask();

    snapshot->proto_version = GRAY_TRACK_PROTO_VERSION;
    snapshot->device_id = GRAY_TRACK_DEVICE_ID;
    snapshot->heartbeat = s_heartbeat++;
    snapshot->sample_counter = s_sample_counter++;
    snapshot->compare_bits = mask;

    for (i = 0U; i < GRAY_TRACK_CHANNEL_COUNT; i++)
    {
        snapshot->adc[i] = ((mask & (uint8_t)(1U << i)) != 0U) ? 1000U : 0U;
    }

    snapshot->online = true;
    snapshot->result = GRAY_TRACK_RESULT_OK;
    s_last_result = GRAY_TRACK_RESULT_OK;
    return true;
}

bool gray_track_read_snapshot(gray_track_snapshot_t *snapshot)
{
    return gray_track_read_status(snapshot);
}

/* 数字输入没有 I2C 错误计数，固定返回 0。 */
uint32_t gray_track_get_error_count(void)
{
    return 0U;
}

gray_track_result_t gray_track_get_last_result(void)
{
    return s_last_result;
}

const char *gray_track_result_text(gray_track_result_t result)
{
    switch (result)
    {
    case GRAY_TRACK_RESULT_OK:
        return "OK";
    case GRAY_TRACK_RESULT_PARAM_ERROR:
        return "PARAM";
    case GRAY_TRACK_RESULT_TIMEOUT:
        return "TIMEOUT";
    case GRAY_TRACK_RESULT_BUS_ERROR:
        return "BUS";
    case GRAY_TRACK_RESULT_ID_MISMATCH:
        return "ID";
    default:
        return "UNKNOWN";
    }
}
