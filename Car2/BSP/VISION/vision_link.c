#include "VISION/vision_link.h"
#include "SystemTime/system_time.h"

/*
 * 视觉链路实现。
 *
 * 数据流：
 *   UART4 RX 中断 -> 256 字节接收环形缓冲 -> Vision_Update() 逐字节解析
 *   -> CRC/语义校验 -> BallMeasurement -> 上层球控制。
 *
 *   Vision_SendFrame() -> 512 字节发送环形缓冲 -> UART4 TX 中断 -> Jetson/PC。
 *
 * 设计原因：
 * 1. 中断里只搬字节，不做 CRC 和浮点解析，避免 30 Hz 数据在中断里堆积；
 * 2. 发送走软件缓冲 + TX 中断，主循环永远不会因为串口阻塞；
 * 3. 速度计算只用 G3519 本地接收时刻，Jetson 时间戳仅作诊断，
 *    防止两端绝对时钟混用。
 *
 * 协议细节见 vision_link.h 顶部注释和 docs/BALL_CONTROL.md。
 */

#define VISION_RX_RING_SIZE             (256U)
#define VISION_RX_RING_MASK             (VISION_RX_RING_SIZE - 1U)
#define VISION_TX_RING_SIZE             (512U)
#define VISION_TX_RING_MASK             (VISION_TX_RING_SIZE - 1U)

/* 完整帧长度 = 同步 2 + 版本/类型/长度/序号 5 + 负载 + CRC 2。 */
#define VISION_FRAME_OVERHEAD           (9U)
#define VISION_FRAME_BUF_SIZE           (VISION_LINK_MAX_PAYLOAD_LEN + VISION_FRAME_OVERHEAD)

/* 视觉位置的物理合理范围：摆杆有效行程约 ±25 cm，留一点余量。 */
#define VISION_X_LIMIT_CM               (30.0f)
#define VISION_CONFIDENCE_MIN           (20U)

/* CRC 覆盖范围从“版本”字节开始，跳过两个同步字节。 */
#define VISION_CRC_START_INDEX          (2U)

static volatile uint8_t  s_rx_ring[VISION_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

static volatile uint8_t  s_tx_ring[VISION_TX_RING_SIZE];
static volatile uint16_t s_tx_head = 0U;
static volatile uint16_t s_tx_tail = 0U;

static uint8_t  s_frame_buf[VISION_FRAME_BUF_SIZE];
static uint16_t s_frame_index = 0U;

static BallMeasurement s_latest;
static vision_link_status_t s_status;
static bool     s_have_frame = false;
static uint16_t s_tx_seq = 0U;

/* 速度/调试用的最近一次接收时刻由 s_status.last_rx_ms 统一维护。 */

/*
 * CRC16-CCITT-FALSE：初值 0xFFFF，多项式 0x1021，不做输出取反。
 *
 * 采用逐位实现，代码小、无需查表；18~40 字节的帧在 80 MHz 上耗时可以忽略。
 */
static uint16_t vision_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static float vision_read_f32_le(const uint8_t *src)
{
    uint32_t raw = (uint32_t)src[0] |
                   ((uint32_t)src[1] << 8) |
                   ((uint32_t)src[2] << 16) |
                   ((uint32_t)src[3] << 24);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

void Vision_WriteF32LE(uint8_t *dst, float value)
{
    uint32_t raw;

    if (dst == NULL) {
        return;
    }

    memcpy(&raw, &value, sizeof(raw));
    Vision_WriteU32LE(dst, raw);
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

/*
 * 记录一次解码错误。
 *
 * 连续错误达到阈值后置 link_fault，由球控制器映射为 FAULT 状态；
 * 收到任意一个 CRC 正确的帧后会清零。
 */
static void vision_note_decode_error(void)
{
    if (s_status.consecutive_errors < 0xFFU) {
        s_status.consecutive_errors++;
    }

    if (s_status.consecutive_errors >= VISION_LINK_FAULT_ERROR_COUNT) {
        s_status.link_fault = true;
    }
}

/*
 * 处理一个 CRC 正确、长度和类型符合要求的视觉位置帧。
 *
 * 语义无效（未完成厘米标定、置信度低、坐标越界/NaN）不会直接丢帧，
 * 而是把 valid 置 false 并立即让上层感知，从而尽快退出 AUTO。
 */
static void vision_process_ball_frame(const uint8_t *frame, uint8_t payload_len)
{
    const uint8_t *payload = &frame[7];
    uint32_t now = system_time_get_tick_ms();
    uint16_t seq = (uint16_t)((uint16_t)frame[5] | ((uint16_t)frame[6] << 8));
    uint8_t flags = payload[9];
    uint8_t confidence = payload[8];
    float x_cm = vision_read_f32_le(&payload[4]);
    bool valid;

    /*
     * 重复帧处理：seq 与上一帧相同说明 Jetson 重发或总线回环，
     * 不刷新 last_rx_ms，避免“只有重复帧”时视觉被误判为新鲜。
     */
    if (s_have_frame && (seq == s_status.last_seq)) {
        s_status.frames_duplicate++;
        return;
    }

    s_status.frames_ok++;
    s_status.consecutive_errors = 0U;
    s_status.link_fault = false;
    s_status.last_seq = seq;
    s_status.last_rx_ms = now;
    s_latest.seq = seq;
    s_latest.frame_time_ms = (uint32_t)payload[0] |
                             ((uint32_t)payload[1] << 8) |
                             ((uint32_t)payload[2] << 16) |
                             ((uint32_t)payload[3] << 24);
    s_latest.x_cm = x_cm;
    s_latest.confidence = (float)confidence;

    /*
     * 有效性判定：
     * - bit0：Jetson 认为本帧检测有效；
     * - bit1：Jetson 已完成厘米标定，禁止用像素坐标冒充厘米；
     * - 置信度不低于 20；
     * - 坐标在物理行程内且不是 NaN/Inf（NaN 会使两个比较都为 false）。
     */
    valid = ((flags & 0x01U) != 0U) &&
            ((flags & 0x02U) != 0U) &&
            (confidence >= VISION_CONFIDENCE_MIN) &&
            (x_cm >= -VISION_X_LIMIT_CM) &&
            (x_cm <= VISION_X_LIMIT_CM);

    if (!valid) {
        s_status.frames_rejected++;
        s_latest.valid = false;
    } else {
        s_latest.valid = true;
    }

    s_have_frame = true;
    s_status.have_frame = true;

    (void)payload_len;
}

/*
 * 帧后处理：校验版本、类型、负载长度和 CRC，再分发。
 */
static void vision_finish_frame(const uint8_t *frame)
{
    uint8_t payload_len = frame[4];
    uint16_t expected_crc;
    uint16_t received_crc;

    if ((frame[2] != VISION_LINK_VERSION) ||
        (frame[3] != VISION_LINK_TYPE_BALL) ||
        (payload_len != VISION_LINK_BALL_PAYLOAD_LEN)) {
        s_status.frames_format_error++;
        vision_note_decode_error();
        return;
    }

    expected_crc = vision_crc16(&frame[VISION_CRC_START_INDEX],
                                (uint32_t)(5U + payload_len));
    received_crc = (uint16_t)((uint16_t)frame[7U + payload_len] |
                              ((uint16_t)frame[8U + payload_len] << 8));

    if (expected_crc != received_crc) {
        s_status.frames_crc_error++;
        vision_note_decode_error();
        return;
    }

    vision_process_ball_frame(frame, payload_len);
}

/*
 * 逐字节状态机。
 *
 * 状态含义：
 * - index 0：等待 SYNC0；
 * - index 1：等待 SYNC1（再次收到 SYNC0 时保持 index 1）；
 * - index >=2：持续收进 s_frame_buf，收满 9+len 后校验。
 *
 * 任意阶段发现长度超限都会立刻复位状态机，避免坏帧导致长时间无法重新同步。
 */
static void vision_parse_byte(uint8_t byte)
{
    uint8_t payload_len;
    uint16_t total_len;

    if (s_frame_index == 0U) {
        if (byte == VISION_LINK_SYNC0) {
            s_frame_buf[0] = byte;
            s_frame_index = 1U;
        }
        return;
    }

    if (s_frame_index == 1U) {
        if (byte == VISION_LINK_SYNC1) {
            s_frame_buf[1] = byte;
            s_frame_index = 2U;
        } else if (byte != VISION_LINK_SYNC0) {
            s_frame_index = 0U;
        }
        return;
    }

    if (s_frame_index >= VISION_FRAME_BUF_SIZE) {
        s_status.frames_format_error++;
        vision_note_decode_error();
        s_frame_index = 0U;
        return;
    }

    s_frame_buf[s_frame_index++] = byte;

    /* 收到长度字段后立即计算总长，超限帧直接丢弃。 */
    if (s_frame_index >= 5U) {
        payload_len = s_frame_buf[4];
        if (payload_len > VISION_LINK_MAX_PAYLOAD_LEN) {
            s_status.frames_format_error++;
            vision_note_decode_error();
            s_frame_index = 0U;
            return;
        }

        total_len = (uint16_t)(VISION_FRAME_OVERHEAD + payload_len);
        if (s_frame_index == total_len) {
            s_frame_index = 0U;
            vision_finish_frame(s_frame_buf);
        }
    }
}

/*
 * 发送中断服务：把软件环形缓冲里的数据尽量填进 UART TX FIFO。
 *
 * 队列空时关闭 TX 中断，避免 TX FIFO 半空事件造成中断风暴；
 * 上层每次入队后会重新打开 TX 中断。
 */
static void vision_tx_drain(void)
{
    while ((s_tx_tail != s_tx_head) &&
           (DL_UART_Main_isTXFIFOFull(UART_WF_INST) == false)) {
        uint8_t byte = s_tx_ring[s_tx_tail];

        s_tx_tail = (uint16_t)((s_tx_tail + 1U) & VISION_TX_RING_MASK);
        DL_UART_Main_transmitData(UART_WF_INST, byte);
    }

    if (s_tx_tail == s_tx_head) {
        DL_UART_Main_disableInterrupt(UART_WF_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

/*
 * UART4 中断入口。
 *
 * 同一入口同时处理 RX 和 TX：
 * - RX：把 FIFO 读空并压入接收环形缓冲，缓冲满时丢弃并计数；
 * - TX：调用 vision_tx_drain() 续发。
 *
 * getPendingInterrupt() 会清除本次返回的中断源；若 RX/TX 同时挂起，
 * 会在下一次中断入口处理另一个来源。
 */
void UART_WF_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_WF_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (DL_UART_Main_isRXFIFOEmpty(UART_WF_INST) == false) {
            uint8_t byte = DL_UART_Main_receiveData(UART_WF_INST);
            uint16_t next = (uint16_t)((s_rx_head + 1U) & VISION_RX_RING_MASK);

            if (next != s_rx_tail) {
                s_rx_ring[s_rx_head] = byte;
                s_rx_head = next;
            } else {
                s_status.rx_overflow++;
            }
        }
        break;

    case DL_UART_MAIN_IIDX_TX:
        vision_tx_drain();
        break;

    default:
        break;
    }
}

void Vision_Init(uint32_t baud)
{
    if (baud == 0U) {
        baud = VISION_LINK_DEFAULT_BAUD;
    }

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_frame_index = 0U;
    s_tx_seq = 0U;

    memset(&s_latest, 0, sizeof(s_latest));
    memset(&s_status, 0, sizeof(s_status));
    s_have_frame = false;

    /*
     * UART4 在 SysConfig 里已经分配了 PB10/PB11 并初始化过；
     * 这里按目标波特率重新配置，随后打开 FIFO 和收发中断。
     * 写法和 UART0 调试串口保持一致，便于维护。
     */
    DL_UART_Main_changeConfig(UART_WF_INST);
    DL_UART_Main_configBaudRate(UART_WF_INST, UART_WF_INST_FREQUENCY, baud);
    DL_UART_Main_enableFIFOs(UART_WF_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_WF_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_WF_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_UART_Main_enableInterrupt(UART_WF_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(UART_WF_INST);

    /* 清掉上电期间可能残留的 RX 数据，避免把噪声当成帧头。 */
    while (DL_UART_Main_isRXFIFOEmpty(UART_WF_INST) == false) {
        (void)DL_UART_Main_receiveData(UART_WF_INST);
    }

    NVIC_ClearPendingIRQ(UART_WF_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_WF_INST_INT_IRQN);
}

void Vision_Update(void)
{
    /*
     * 单生产者（中断）单消费者（主循环）环形缓冲：
     * 主循环每次取一个字节后推进 tail，解析过程可能较长，但不会阻塞中断。
     */
    while (s_rx_tail != s_rx_head) {
        uint8_t byte = s_rx_ring[s_rx_tail];

        s_rx_tail = (uint16_t)((s_rx_tail + 1U) & VISION_RX_RING_MASK);
        vision_parse_byte(byte);
    }
}

bool Vision_GetLatest(BallMeasurement *out, uint32_t *age_ms)
{
    uint32_t now;
    uint32_t age;
    bool fresh;

    if (out == NULL) {
        return false;
    }

    now = system_time_get_tick_ms();
    age = (uint32_t)(now - s_status.last_rx_ms);
    fresh = s_have_frame && s_latest.valid && (age <= VISION_LINK_TIMEOUT_MS);

    *out = s_latest;
    out->valid = fresh;

    if (age_ms != NULL) {
        *age_ms = age;
    }

    return fresh;
}

bool Vision_IsFresh(void)
{
    uint32_t now = system_time_get_tick_ms();
    uint32_t age = (uint32_t)(now - s_status.last_rx_ms);

    return s_have_frame && s_latest.valid && (age <= VISION_LINK_TIMEOUT_MS);
}

void Vision_GetStatus(vision_link_status_t *status)
{
    if (status == NULL) {
        return;
    }

    *status = s_status;
}

/*
 * 把一帧完整数据放入发送环形缓冲。
 *
 * 入队前先检查剩余空间，空间不足时整帧丢弃并计数，绝不写半帧，
 * 否则 Jetson 侧会因为流错位而连续 CRC 失败。
 */
static bool vision_enqueue_frame(const uint8_t *frame, uint16_t total_len)
{
    uint16_t used;
    uint16_t free_bytes;
    uint16_t i;

    used = (uint16_t)((s_tx_head - s_tx_tail) & VISION_TX_RING_MASK);
    free_bytes = (uint16_t)(VISION_TX_RING_SIZE - 1U - used);

    if (free_bytes < total_len) {
        s_status.tx_dropped++;
        return false;
    }

    for (i = 0U; i < total_len; i++) {
        s_tx_ring[s_tx_head] = frame[i];
        s_tx_head = (uint16_t)((s_tx_head + 1U) & VISION_TX_RING_MASK);
    }

    /* 队列非空，打开 TX 中断让 ISR 续发。 */
    DL_UART_Main_enableInterrupt(UART_WF_INST, DL_UART_MAIN_INTERRUPT_TX);

    return true;
}

bool Vision_SendFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[VISION_FRAME_BUF_SIZE];
    uint16_t crc;
    uint16_t total_len;

    if (len > VISION_LINK_MAX_PAYLOAD_LEN) {
        return false;
    }
    if ((payload == NULL) && (len != 0U)) {
        return false;
    }

    frame[0] = VISION_LINK_SYNC0;
    frame[1] = VISION_LINK_SYNC1;
    frame[2] = VISION_LINK_VERSION;
    frame[3] = type;
    frame[4] = len;
    frame[5] = (uint8_t)(s_tx_seq & 0xFFU);
    frame[6] = (uint8_t)((s_tx_seq >> 8) & 0xFFU);

    if (len != 0U) {
        memcpy(&frame[7], payload, len);
    }

    crc = vision_crc16(&frame[VISION_CRC_START_INDEX], (uint32_t)(5U + len));
    frame[7U + len] = (uint8_t)(crc & 0xFFU);
    frame[8U + len] = (uint8_t)((crc >> 8) & 0xFFU);

    total_len = (uint16_t)(VISION_FRAME_OVERHEAD + len);

    if (!vision_enqueue_frame(frame, total_len)) {
        return false;
    }

    s_tx_seq++;
    return true;
}
