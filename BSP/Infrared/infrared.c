#include "Infrared/infrared.h"

/*
 * 八路灰度循迹模块实现。
 *
 * 设计原因：
 * 1. 红外灰度模块的输出在黑白边缘会出现毫秒级抖动，直接读单次 GPIO 会让
 *    循迹误差在相邻控制周期之间来回跳变，因此这里统一提供“多次采样投票”的
 *    稳定化接口，由上层决定采样次数和命中阈值。
 * 2. 不同批次模块的黑线有效电平可能相反，模块内部统一按
 *    INF_BLACK_LINE_LEVEL 归一化，上层只处理“1 = 黑线”。
 * 3. 加权偏差的符号约定与工程历史保持一致：正值表示黑线在右侧。
 */

/* 兼容旧接口 read_inf_mask() 的默认采样参数：7 次采样、命中 4 次判有效。 */
#define INF_DEFAULT_SAMPLE_COUNT     (7U)
#define INF_DEFAULT_SAMPLE_THRESHOLD (4U)

/*
 * 读取一次原始电平并归一化成黑线位图。
 * 逐路比较而不是拼端口，是为了兼容八路分散在不同 GPIO 端口的实际布线。
 */
uint8_t inf_read_mask_raw(void)
{
    uint8_t mask = 0U;

    if ((INF_1 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x01U; }
    if ((INF_2 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x02U; }
    if ((INF_3 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x04U; }
    if ((INF_4 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x08U; }
    if ((INF_5 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x10U; }
    if ((INF_6 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x20U; }
    if ((INF_7 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x40U; }
    if ((INF_8 != 0U) == (INF_BLACK_LINE_LEVEL != 0U)) { mask |= 0x80U; }

    return mask;
}

/*
 * 多次采样投票消抖。
 * 每一路独立计数，只有命中次数达到阈值才置位。相邻采样之间插入 1 ms 延时，
 * 让采样点覆盖红外模块输出可能的抖动窗口。
 * 注意：本函数是阻塞的，总耗时约 (samples - 1) ms，调用者必须把这段时间
 * 计入控制周期；本工程按 5 次采样（约 4 ms）使用。
 */
uint8_t inf_read_mask_stable(uint8_t samples, uint8_t threshold)
{
    uint8_t hit_count[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t sample;
    uint8_t mask = 0U;
    uint8_t i;
    uint8_t bit;

    /* 非法参数收敛，避免除零或无意义的多路命中判据。 */
    if (samples == 0U)
    {
        samples = 1U;
    }
    if (threshold == 0U)
    {
        threshold = 1U;
    }
    if (threshold > samples)
    {
        threshold = samples;
    }

    for (i = 0U; i < samples; i++)
    {
        sample = inf_read_mask_raw();
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((sample & (uint8_t)(1U << bit)) != 0U)
            {
                hit_count[bit]++;
            }
        }

        /* 最后一次采样后不再等待，缩短整个采样窗口。 */
        if ((uint8_t)(i + 1U) < samples)
        {
            delay_ms(1U);
        }
    }

    for (bit = 0U; bit < 8U; bit++)
    {
        if (hit_count[bit] >= threshold)
        {
            mask |= (uint8_t)(1U << bit);
        }
    }

    return mask;
}

/* 统计位图中置 1 的位数，用于识别横跨传感器阵列的启停线。 */
uint8_t inf_count_bits(uint8_t mask)
{
    uint8_t count = 0U;

    while (mask != 0U)
    {
        count = (uint8_t)(count + (mask & 1U));
        mask >>= 1U;
    }

    return count;
}

/*
 * 位图转加权偏差。
 * I1~I8 的权重为 -7、-5、-3、-1、+1、+3、+5、+7，取命中探头的平均值后取反，
 * 得到工程统一的“正值 = 黑线在右”符号约定。
 * 完全没有命中时返回 INF_LINE_LOST_VALUE，由上层进入找线/停车流程。
 */
int inf_error_from_mask(uint8_t mask)
{
    static const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int sum = 0;
    int count = 0;
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((mask & (uint8_t)(1U << bit)) != 0U)
        {
            sum += weight[bit];
            count++;
        }
    }

    if (count == 0)
    {
        return INF_LINE_LOST_VALUE;
    }

    return -(sum / count);
}

/*
 * 兼容旧接口：使用默认 7 次采样、4 次命中阈值。
 * 该参数与旧版 infrared.c 完全一致，避免影响工程中其他仍在使用旧接口的代码。
 */
uint8_t read_inf_mask(void)
{
    return inf_read_mask_stable(INF_DEFAULT_SAMPLE_COUNT,
                                INF_DEFAULT_SAMPLE_THRESHOLD);
}

/* 兼容旧接口：返回加权偏差，丢线时返回 INF_LINE_LOST_VALUE。 */
int read_inf(void)
{
    return inf_error_from_mask(read_inf_mask());
}
