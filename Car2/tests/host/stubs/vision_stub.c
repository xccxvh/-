#include "VISION/vision_link.h"

/*
 * 主机测试用的视觉链路替身。
 *
 * ROLL 模式不使用视觉数据，因此这里只需要保证链接通过：
 * - Vision_Update/GetLatest/GetStatus 返回“无有效帧”；
 * - 遥测组包函数保持和真实实现相同的字节序，便于检查 telemetry 负载。
 */

void Vision_Init(uint32_t baud)
{
    (void)baud;
}

void Vision_Update(void)
{
}

bool Vision_GetLatest(BallMeasurement *out, uint32_t *age_ms)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->valid = false;
    }
    if (age_ms != NULL) {
        *age_ms = 0xFFFFFFFFU;
    }
    return false;
}

bool Vision_IsFresh(void)
{
    return false;
}

void Vision_GetStatus(vision_link_status_t *status)
{
    if (status != NULL) {
        memset(status, 0, sizeof(*status));
    }
}

bool Vision_SendFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    (void)type;
    (void)payload;
    (void)len;
    return true;
}

void Vision_WriteU32LE(uint8_t *dst, uint32_t value)
{
    if (dst == NULL) {
        return;
    }
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

void Vision_WriteF32LE(uint8_t *dst, float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    Vision_WriteU32LE(dst, raw);
}
