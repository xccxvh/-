/*
 * 视觉链路协议解析测试。
 *
 * 通过替身 UART FIFO 把 Python 工具生成的帧喂给真实的 vision_link.c，
 * 验证：
 * - 帧同步、长度、CRC 校验；
 * - 坐标/置信度/未标定标志的语义过滤；
 * - 150 ms 超时；
 * - 遥测帧的组包长度、序号和 CRC。
 *
 * 参考帧由 tools/ball_link.py 生成：
 *   a5 5a 01 01 0c 34 12 cd ab 00 00 00 00 60 40 5a 03 00 00 ce 68
 * 对应 seq=0x1234、frame_time=0xABCD、x=3.5 cm、confidence=90、flags=0x03。
 */

#include "VISION/vision_link.h"
#include "SystemTime/system_time.h"

#include <stdio.h>
#include <string.h>

/* 真实的 UART4 中断入口在 vision_link.c 中定义，这里显式声明以便测试调用。 */
void UART_WF_INST_IRQHandler(void);

static int g_checks = 0;
static int g_failures = 0;

static const uint8_t REFERENCE_FRAME[] = {
    0xA5, 0x5A, 0x01, 0x01, 0x0C, 0x34, 0x12, 0xCD, 0xAB,
    0x00, 0x00, 0x00, 0x00, 0x60, 0x40, 0x5A, 0x03, 0x00, 0x00, 0xCE, 0x68
};

static void check_true(bool condition, const char *name)
{
    g_checks++;
    if (condition) {
        printf("[PASS] %s\n", name);
    } else {
        g_failures++;
        printf("[FAIL] %s\n", name);
    }
}

static void check_near(float actual, float expect, float tol, const char *name)
{
    g_checks++;
    if ((actual >= expect - tol) && (actual <= expect + tol)) {
        printf("[PASS] %s (actual=%.3f)\n", name, actual);
    } else {
        g_failures++;
        printf("[FAIL] %s (actual=%.3f expect=%.3f)\n", name, actual, expect);
    }
}

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 按协议在 C 侧独立组帧，用于构造不同 seq/坐标/标志的测试帧。 */
static uint16_t build_ball_frame(uint8_t *out, uint16_t seq, uint32_t frame_time,
                                 float x_cm, uint8_t confidence, uint8_t flags)
{
    uint8_t payload[12];

    Vision_WriteU32LE(&payload[0], frame_time);
    Vision_WriteF32LE(&payload[4], x_cm);
    payload[8] = confidence;
    payload[9] = flags;
    payload[10] = 0U;
    payload[11] = 0U;

    out[0] = 0xA5U;
    out[1] = 0x5AU;
    out[2] = 0x01U;
    out[3] = 0x01U;
    out[4] = 12U;
    out[5] = (uint8_t)(seq & 0xFFU);
    out[6] = (uint8_t)(seq >> 8);
    memcpy(&out[7], payload, 12U);

    {
        uint16_t crc = crc16_ccitt(&out[2], 5U + 12U);
        out[19] = (uint8_t)(crc & 0xFFU);
        out[20] = (uint8_t)(crc >> 8);
    }
    return 21U;
}

static void uart_push(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    for (i = 0U; i < len; i++) {
        host_uart_rx[host_uart_rx_head] = data[i];
        host_uart_rx_head = (uint16_t)((host_uart_rx_head + 1U) % HOST_UART_RX_SIZE);
    }
}

static void uart_service(void)
{
    /*
     * 第一次中断把 RX FIFO 搬进 vision_link 的内部环形缓冲；
     * 第二次中断用于 TX 队列（若为空则是空转）。
     */
    UART_WF_INST_IRQHandler();
    UART_WF_INST_IRQHandler();
    Vision_Update();
}

static void prepare(void)
{
    host_time_reset();
    host_uart_rx_head = 0U;
    host_uart_rx_tail = 0U;
    host_uart_tx_len = 0U;
    Vision_Init(115200U);
}

static void test_reference_frame(void)
{
    BallMeasurement measurement;
    uint32_t age = 0U;
    vision_link_status_t status;

    prepare();

    uart_push(REFERENCE_FRAME, sizeof(REFERENCE_FRAME));
    uart_service();

    check_true(Vision_GetLatest(&measurement, &age) == true, "reference frame accepted");
    check_near(measurement.x_cm, 3.5f, 0.001f, "x_cm parsed");
    check_true(measurement.seq == 0x1234U, "seq parsed");
    check_true(measurement.frame_time_ms == 0x0000ABCDU, "Jetson timestamp parsed");
    check_near(measurement.confidence, 90.0f, 0.01f, "confidence parsed");
    check_true(age <= 1U, "age is fresh");

    Vision_GetStatus(&status);
    check_true(status.have_frame, "status.have_frame set");
    check_true(status.frames_ok == 1U, "frames_ok == 1");
}

static void test_timeout(void)
{
    BallMeasurement measurement;
    uint32_t age = 0U;
    uint8_t frame[21];

    prepare();

    uart_push(REFERENCE_FRAME, sizeof(REFERENCE_FRAME));
    uart_service();
    (void)Vision_GetLatest(&measurement, &age);
    check_true(age <= 1U, "age before timeout");

    host_time_advance_ms(151U);
    check_true(Vision_GetLatest(&measurement, &age) == false, "151 ms -> stale");
    check_true(measurement.valid == false, "stale frame reported invalid");
    check_true(Vision_IsFresh() == false, "Vision_IsFresh false after timeout");

    /* 新帧应恢复新鲜；seq 递增。 */
    host_time_advance_ms(10U);
    uart_push(frame, build_ball_frame(frame, 0x1235U, 100U, -1.25f, 88U, 0x03U));
    uart_service();
    check_true(Vision_GetLatest(&measurement, &age) == true, "new frame restores freshness");
    check_near(measurement.x_cm, -1.25f, 0.001f, "second frame x parsed");
}

static void test_crc_error(void)
{
    uint8_t frame[21];
    BallMeasurement measurement;
    uint32_t age = 0U;
    vision_link_status_t status;

    prepare();

    uart_push(REFERENCE_FRAME, sizeof(REFERENCE_FRAME));
    uart_service();

    memcpy(frame, REFERENCE_FRAME, sizeof(frame));
    frame[12] ^= 0xFFU;            /* 破坏 x 数据，CRC 必然不匹配 */
    uart_push(frame, sizeof(frame));
    uart_service();

    Vision_GetStatus(&status);
    check_true(status.frames_crc_error >= 1U, "CRC error counted");
    (void)Vision_GetLatest(&measurement, &age);
    check_near(measurement.x_cm, 3.5f, 0.001f, "bad frame does not overwrite position");
}

static void test_uncalibrated_rejected(void)
{
    uint8_t frame[21];
    BallMeasurement measurement;
    uint32_t age = 0U;
    vision_link_status_t status;

    prepare();

    /* flags 只有 bit0（有效）、没有 bit1（厘米标定完成）。 */
    uart_push(frame, build_ball_frame(frame, 1U, 0U, 2.0f, 95U, 0x01U));
    uart_service();

    Vision_GetStatus(&status);
    check_true(status.frames_rejected >= 1U, "uncalibrated frame rejected");
    check_true(Vision_GetLatest(&measurement, &age) == false, "uncalibrated not fresh");
    check_true(measurement.valid == false, "uncalibrated reported invalid");
}

static void test_low_confidence_and_range(void)
{
    uint8_t frame[21];
    BallMeasurement measurement;
    uint32_t age = 0U;
    vision_link_status_t status;

    prepare();

    uart_push(frame, build_ball_frame(frame, 1U, 0U, 2.0f, 5U, 0x03U));
    uart_service();
    check_true(Vision_GetLatest(&measurement, &age) == false, "low confidence rejected");

    prepare();
    uart_push(frame, build_ball_frame(frame, 2U, 0U, 50.0f, 95U, 0x03U));
    uart_service();
    Vision_GetStatus(&status);
    check_true(Vision_GetLatest(&measurement, &age) == false, "out-of-range x rejected");
    check_true(status.frames_rejected >= 1U, "range rejection counted");
}

static void test_telemetry_frame(void)
{
    uint8_t payload[32];
    uint16_t crc;
    uint16_t len;

    prepare();

    Vision_WriteU32LE(&payload[0], 123456U);
    Vision_WriteF32LE(&payload[4], 5.0f);
    Vision_WriteF32LE(&payload[8], 3.25f);
    Vision_WriteF32LE(&payload[12], -1.5f);
    Vision_WriteF32LE(&payload[16], 1.75f);
    Vision_WriteF32LE(&payload[20], 1.2f);
    payload[24] = (uint8_t)(1655U & 0xFFU);
    payload[25] = (uint8_t)(1655U >> 8);
    payload[26] = 2U;
    payload[27] = 0x03U;
    Vision_WriteF32LE(&payload[28], 1.1f);

    check_true(Vision_SendFrame(VISION_LINK_TYPE_TELEMETRY, payload, 32U) == true,
               "telemetry enqueued");

    UART_WF_INST_IRQHandler();
    len = host_uart_tx_len;

    check_true(len == 41U, "telemetry frame length = 41");
    check_true(host_uart_tx[0] == 0xA5U, "telemetry sync0");
    check_true(host_uart_tx[1] == 0x5AU, "telemetry sync1");
    check_true(host_uart_tx[2] == 0x01U, "telemetry version");
    check_true(host_uart_tx[3] == 0x81U, "telemetry type");
    check_true(host_uart_tx[4] == 32U, "telemetry payload len");
    check_true((host_uart_tx[5] == 0U) && (host_uart_tx[6] == 0U), "telemetry seq starts at 0");

    crc = crc16_ccitt(&host_uart_tx[2], 5U + 32U);
    check_true((host_uart_tx[39] == (uint8_t)(crc & 0xFFU)) &&
               (host_uart_tx[40] == (uint8_t)(crc >> 8)),
               "telemetry CRC valid");
}

int main(void)
{
    printf("=== vision_link protocol host tests ===\n");

    test_reference_frame();
    test_timeout();
    test_crc_error();
    test_uncalibrated_rejected();
    test_low_confidence_and_range();
    test_telemetry_frame();

    printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
