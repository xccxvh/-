#include "VISION/vision_link.h"
#include "SystemTime/system_time.h"

/*
 * 主机测试用的视觉链路替身。
 *
 * ROLL 模式不使用视觉数据，因此这里只需要保证链接通过：
 * - Vision_Update/GetLatest/GetStatus 返回“无有效帧”；
 * - 遥测组包函数保持和真实实现相同的字节序，便于检查 telemetry 负载。
 */

static ServoCommand s_host_cmd;
static bool s_host_have_cmd = false;
static uint32_t s_host_cmd_rx_ms = 0U;

static ControlCommand s_host_control;
static bool s_host_have_control = false;
static uint32_t s_host_control_rx_ms = 0U;

/* 测试用：注入一帧 0x02 远程舵机命令。 */
void host_vision_set_servo_command(uint16_t ticks, uint16_t seq)
{
    s_host_cmd.target_pwm_ticks = ticks;
    s_host_cmd.seq = seq;
    s_host_cmd.valid = true;
    s_host_have_cmd = true;
    s_host_cmd_rx_ms = system_time_get_tick_ms();
}

/* 测试用：注入一条 0x03 控制指令。 */
void host_vision_set_control_command(uint8_t opcode, uint8_t mode,
                                     float arg0, float arg1, uint16_t seq)
{
    s_host_control.seq = seq;
    s_host_control.opcode = opcode;
    s_host_control.mode = mode;
    s_host_control.arg0 = arg0;
    s_host_control.arg1 = arg1;
    s_host_control.valid = true;
    s_host_have_control = true;
    s_host_control_rx_ms = system_time_get_tick_ms();
}

void Vision_Init(uint32_t baud)
{
    (void)baud;
    memset(&s_host_cmd, 0, sizeof(s_host_cmd));
    s_host_have_cmd = false;
    s_host_cmd_rx_ms = 0U;
    memset(&s_host_control, 0, sizeof(s_host_control));
    s_host_have_control = false;
    s_host_control_rx_ms = 0U;
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

bool Vision_GetServoCommand(ServoCommand *out, uint32_t *age_ms)
{
    uint32_t now = system_time_get_tick_ms();

    if (out != NULL) {
        *out = s_host_cmd;
    }
    if (age_ms != NULL) {
        *age_ms = s_host_have_cmd ?
                  (uint32_t)(now - s_host_cmd_rx_ms) : 0xFFFFFFFFU;
    }

    return s_host_have_cmd;
}

bool Vision_GetControlCommand(ControlCommand *out, uint32_t *age_ms)
{
    uint32_t now = system_time_get_tick_ms();

    if (out != NULL) {
        *out = s_host_control;
    }
    if (age_ms != NULL) {
        *age_ms = s_host_have_control ?
                  (uint32_t)(now - s_host_control_rx_ms) : 0xFFFFFFFFU;
    }

    return s_host_have_control;
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
