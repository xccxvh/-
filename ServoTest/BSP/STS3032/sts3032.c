#include "STS3032/sts3032.h"

/*
 * 飞特 STS/SMS 协议常量。
 * 指令帧格式：FF FF + ID + Length + Instruction + Params + Checksum。
 * 状态帧格式：FF FF + ID + Length + Status + Params + Checksum。
 * Length 统计 Instruction/Status、参数区和 Checksum，因此等于参数长度 + 2。
 * Checksum 为 ID、Length、Instruction/Status、参数区逐字节求和后取低 8 位再按位取反。
 */
#define STS3032_FRAME_HEAD                 (0xFFU)
#define STS3032_INST_PING                  (0x01U)
#define STS3032_INST_READ                  (0x02U)
#define STS3032_INST_WRITE                 (0x03U)
#define STS3032_INST_SYNC_WRITE            (0x83U)

/* STS/SMS 协议表中本工程用到的 RAM 地址。 */
#define STS3032_REG_TORQUE_ENABLE          (0x28U)
#define STS3032_REG_GOAL_ACC               (0x29U)
#define STS3032_REG_GOAL_POSITION          (0x2AU)
#define STS3032_REG_PRESENT_POSITION       (0x38U)
#define STS3032_REG_OPERATION_MODE         (0x4DU)
#define STS3032_REG_RUNNING_SPEED          (0x4EU)

#define STS3032_TX_BUFFER_SIZE             (128U)
#define STS3032_RX_RING_SIZE               (128U)
#define STS3032_RX_PARAM_BUFFER_SIZE       (64U)
#define STS3032_MIN_STATUS_LENGTH          (2U)
#define STS3032_SYNC_TORQUE_PARAM_SIZE     (2U + (STS3032_SYNC_MAX_TARGETS * 2U))
#define STS3032_SYNC_POSITION_PARAM_SIZE   (2U + (STS3032_SYNC_MAX_TARGETS * 7U))
#define STS3032_SYNC_MODE_PARAM_SIZE       (2U + (STS3032_SYNC_MAX_TARGETS * 2U))
#define STS3032_SYNC_WHEEL_PARAM_SIZE      (2U + (STS3032_SYNC_MAX_TARGETS * 3U))
#define STS3032_WHEEL_REVERSE_FLAG         (0x8000U)

#define STS3032_RX_WAIT_HEAD0              (0U)
#define STS3032_RX_WAIT_HEAD1              (1U)
#define STS3032_RX_WAIT_ID                 (2U)
#define STS3032_RX_WAIT_LENGTH             (3U)
#define STS3032_RX_WAIT_PAYLOAD            (4U)
#define STS3032_RX_WAIT_CHECKSUM           (5U)

static bool sts3032_initialized = false;
static Sts3032Error sts3032_last_error = STS3032_ERROR_NOT_READY;
static uint8_t sts3032_last_device_status = 0U;
static uint32_t sts3032_configured_baud_rate = 0U;
static uint32_t sts3032_tx_packet_count = 0U;
static uint32_t sts3032_rx_packet_count = 0U;
static volatile uint8_t sts3032_rx_ring[STS3032_RX_RING_SIZE];
static volatile uint16_t sts3032_rx_head = 0U;
static volatile uint16_t sts3032_rx_tail = 0U;
static volatile uint32_t sts3032_rx_overrun_count = 0U;

static bool Sts3032_IsServoIdValid(uint8_t id)
{
    return (id <= STS3032_ID_MAX);
}

static bool Sts3032_IsPacketIdValid(uint8_t id)
{
    return ((id <= STS3032_ID_MAX) || (id == STS3032_BROADCAST_ID));
}

static bool Sts3032_IsPositionValid(uint16_t position)
{
    return (position <= STS3032_POSITION_MAX);
}

static bool Sts3032_IsSpeedValid(uint16_t speed)
{
    return (speed <= STS3032_SPEED_MAX);
}

static bool Sts3032_IsModeValid(Sts3032OperationMode mode)
{
    return ((mode == STS3032_MODE_POSITION) ||
            (mode == STS3032_MODE_WHEEL) ||
            (mode == STS3032_MODE_PWM) ||
            (mode == STS3032_MODE_STEP));
}

/*
 * 校验恒速模式速度。
 * 协议表规定 0 为停止，转动时有效速度范围为 50~3400；方向由 bit15 表示。
 */
static bool Sts3032_IsWheelSpeedValid(int16_t speed)
{
    uint16_t magnitude;

    if (speed == 0)
    {
        return true;
    }

    magnitude = (speed < 0) ? (uint16_t)(-speed) : (uint16_t)speed;
    return ((magnitude >= STS3032_WHEEL_SPEED_MIN) &&
            (magnitude <= STS3032_WHEEL_SPEED_MAX));
}

static void Sts3032_WriteU16LE(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value & 0x00FFU);
    buffer[1] = (uint8_t)((value >> 8) & 0x00FFU);
}

static uint16_t Sts3032_ReadU16LE(const uint8_t *buffer)
{
    return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

/*
 * 将有符号速度编码为协议表的恒速速度字段。
 * 正数保持 bit15=0，负数在绝对值基础上置 bit15；这样调用层只需要用正负表达方向。
 */
static uint16_t Sts3032_EncodeWheelSpeed(int16_t speed)
{
    if (speed < 0)
    {
        return (uint16_t)((uint16_t)(-speed) | STS3032_WHEEL_REVERSE_FLAG);
    }

    return (uint16_t)speed;
}

static int16_t Sts3032_DecodeWheelSpeed(uint16_t raw_speed)
{
    uint16_t magnitude = (uint16_t)(raw_speed & (uint16_t)(~STS3032_WHEEL_REVERSE_FLAG));

    if ((raw_speed & STS3032_WHEEL_REVERSE_FLAG) != 0U)
    {
        return (int16_t)(-(int16_t)magnitude);
    }

    return (int16_t)magnitude;
}

static uint8_t Sts3032_MakeChecksum(const uint8_t *buffer, uint8_t length)
{
    uint8_t index;
    uint16_t sum = 0U;

    for (index = 0U; index < length; index++)
    {
        sum = (uint16_t)(sum + buffer[index]);
    }

    return (uint8_t)(~((uint8_t)sum));
}

static uint16_t Sts3032_RxNextIndex(uint16_t index)
{
    index++;
    if (index >= STS3032_RX_RING_SIZE)
    {
        index = 0U;
    }

    return index;
}

/*
 * 清空 UART6 接收路径。
 * 职责：同时清空软件环形缓冲和硬件 RX FIFO，避免上一帧残留数据污染下一次响应解析。
 * 时序约束：复位 head/tail 时临时关闭 UART6 NVIC，避免中断同时写入环形缓冲。
 */
static void Sts3032_ClearRxBuffer(void)
{
    NVIC_DisableIRQ(UART_6_INST_INT_IRQN);
    sts3032_rx_head = 0U;
    sts3032_rx_tail = 0U;

    while (DL_UART_Main_isRXFIFOEmpty(UART_6_INST) == false)
    {
        (void)DL_UART_Main_receiveData(UART_6_INST);
    }

    DL_UART_Main_clearInterruptStatus(UART_6_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_6_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_6_INST_INT_IRQN);
}

/*
 * 从 UART6 接收环形缓冲读取 1 字节。
 * 设计原因：中断只做搬运，协议状态机在任务上下文里解析，避免中断中执行复杂逻辑。
 */
static bool Sts3032_ReadRxByte(uint8_t *data)
{
    if ((data == 0) || (sts3032_rx_tail == sts3032_rx_head))
    {
        return false;
    }

    *data = sts3032_rx_ring[sts3032_rx_tail];
    sts3032_rx_tail = Sts3032_RxNextIndex(sts3032_rx_tail);
    return true;
}

/*
 * 发送单字节到 UART6。
 * 关键点：写 TXDATA 后等待 UART busy 清零，确保该字节的起始位、数据位和停止位
 * 已经从 PB22 完整移出，再发送下一个字节。STS3032 帧很短，阻塞等待成本可接受。
 */
static void Sts3032_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_6_INST) != false)
    {
    }

    DL_UART_Main_transmitData(UART_6_INST, data);
    while (DL_UART_Main_isBusy(UART_6_INST) != false)
    {
    }
}

/*
 * 发送 STS/SMS 协议指令帧。
 * 输入：id 为普通 ID 或广播 ID；instruction 为协议指令；params 为参数区。
 * 输出：成功时整帧已从 PB22 完整发送；失败时记录参数、初始化或缓冲区错误。
 */
static bool Sts3032_SendPacket(uint8_t id,
                               uint8_t instruction,
                               const uint8_t *params,
                               uint8_t param_length)
{
    uint8_t frame[STS3032_TX_BUFFER_SIZE];
    uint8_t frame_length;
    uint8_t index;
    uint8_t checksum_area_length;

    if ((sts3032_initialized == false) ||
        (Sts3032_IsPacketIdValid(id) == false) ||
        ((params == 0) && (param_length != 0U)))
    {
        sts3032_last_error = (sts3032_initialized == false) ?
                             STS3032_ERROR_NOT_READY :
                             STS3032_ERROR_PARAM;
        return false;
    }

    frame_length = (uint8_t)(param_length + 6U);
    if (frame_length > STS3032_TX_BUFFER_SIZE)
    {
        sts3032_last_error = STS3032_ERROR_BUFFER;
        return false;
    }

    frame[0] = STS3032_FRAME_HEAD;
    frame[1] = STS3032_FRAME_HEAD;
    frame[2] = id;
    frame[3] = (uint8_t)(param_length + 2U);
    frame[4] = instruction;

    for (index = 0U; index < param_length; index++)
    {
        frame[(uint8_t)(5U + index)] = params[index];
    }

    checksum_area_length = (uint8_t)(param_length + 3U);
    frame[(uint8_t)(5U + param_length)] =
        Sts3032_MakeChecksum(&frame[2], checksum_area_length);

    Sts3032_ClearRxBuffer();
    for (index = 0U; index < frame_length; index++)
    {
        Sts3032_SendByte(frame[index]);
    }

    sts3032_tx_packet_count++;
    sts3032_last_error = STS3032_ERROR_NONE;
    return true;
}

/*
 * 等待并解析一个状态包。
 * expected_id 用于过滤其它 ID；ignored_instruction 用于丢弃 TX/RX 短接调试时可能
 * 收到的本机发送回显。timeout_ms 为阻塞等待上限，适合初始化和简单测试流程。
 */
static bool Sts3032_WaitStatus(uint8_t expected_id,
                               uint8_t ignored_instruction,
                               uint8_t *params,
                               uint8_t params_buffer_size,
                               uint8_t *params_length,
                               uint32_t timeout_ms)
{
    uint8_t state = STS3032_RX_WAIT_HEAD0;
    uint8_t packet_id = 0U;
    uint8_t packet_length = 0U;
    uint8_t payload[STS3032_RX_PARAM_BUFFER_SIZE];
    uint8_t payload_index = 0U;
    uint8_t checksum_sum = 0U;
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms <= timeout_ms)
    {
        uint8_t data;

        while (Sts3032_ReadRxByte(&data) != false)
        {
            switch (state)
            {
                case STS3032_RX_WAIT_HEAD0:
                    if (data == STS3032_FRAME_HEAD)
                    {
                        state = STS3032_RX_WAIT_HEAD1;
                    }
                    break;

                case STS3032_RX_WAIT_HEAD1:
                    state = (data == STS3032_FRAME_HEAD) ?
                            STS3032_RX_WAIT_ID :
                            STS3032_RX_WAIT_HEAD0;
                    break;

                case STS3032_RX_WAIT_ID:
                    packet_id = data;
                    checksum_sum = data;
                    state = STS3032_RX_WAIT_LENGTH;
                    break;

                case STS3032_RX_WAIT_LENGTH:
                    packet_length = data;
                    checksum_sum = (uint8_t)(checksum_sum + data);
                    payload_index = 0U;
                    if ((packet_length < STS3032_MIN_STATUS_LENGTH) ||
                        ((uint8_t)(packet_length - 1U) > STS3032_RX_PARAM_BUFFER_SIZE))
                    {
                        state = STS3032_RX_WAIT_HEAD0;
                    }
                    else
                    {
                        state = STS3032_RX_WAIT_PAYLOAD;
                    }
                    break;

                case STS3032_RX_WAIT_PAYLOAD:
                    payload[payload_index] = data;
                    checksum_sum = (uint8_t)(checksum_sum + data);
                    payload_index++;
                    if (payload_index >= (uint8_t)(packet_length - 1U))
                    {
                        state = STS3032_RX_WAIT_CHECKSUM;
                    }
                    break;

                case STS3032_RX_WAIT_CHECKSUM:
                    if (data == (uint8_t)(~checksum_sum))
                    {
                        uint8_t status = payload[0];
                        uint8_t received_param_length =
                            (uint8_t)(packet_length - STS3032_MIN_STATUS_LENGTH);

                        if ((packet_id == expected_id) &&
                            !((ignored_instruction != 0U) && (status == ignored_instruction)))
                        {
                            if (status != 0U)
                            {
                                sts3032_last_device_status = status;
                                sts3032_last_error = STS3032_ERROR_DEVICE_STATUS;
                                return false;
                            }

                            if (received_param_length > params_buffer_size)
                            {
                                sts3032_last_error = STS3032_ERROR_BUFFER;
                                return false;
                            }

                            if ((params != 0) && (received_param_length != 0U))
                            {
                                memcpy(params, &payload[1], received_param_length);
                            }
                            if (params_length != 0)
                            {
                                *params_length = received_param_length;
                            }

                            sts3032_rx_packet_count++;
                            sts3032_last_device_status = 0U;
                            sts3032_last_error = STS3032_ERROR_NONE;
                            return true;
                        }
                    }
                    else
                    {
                        sts3032_last_error = STS3032_ERROR_CHECKSUM;
                    }

                    state = STS3032_RX_WAIT_HEAD0;
                    break;

                default:
                    state = STS3032_RX_WAIT_HEAD0;
                    break;
            }
        }

        delay_ms(1U);
        elapsed_ms++;
    }

    sts3032_last_error = STS3032_ERROR_TIMEOUT;
    return false;
}

/*
 * 接入 SysConfig 已初始化好的 UART6。
 * 本函数不重新初始化 UART6，也不修改波特率；它只校验 SysConfig 生成结果，并打开
 * NVIC 中断入口，保证 UART6_IRQHandler 能接收舵机状态包。
 */
bool sts3032_init(uint32_t baud_rate)
{
    if (baud_rate == 0U)
    {
        baud_rate = UART_6_BAUD_RATE;
    }

    if (baud_rate != UART_6_BAUD_RATE)
    {
        sts3032_initialized = false;
        sts3032_last_error = STS3032_ERROR_PARAM;
        sts3032_configured_baud_rate = 0U;
        return false;
    }

    if ((DL_UART_Main_isEnabled(UART_6_INST) == false) ||
        (DL_UART_Main_getIntegerBaudRateDivisor(UART_6_INST) == 0U))
    {
        sts3032_initialized = false;
        sts3032_last_error = STS3032_ERROR_NOT_READY;
        sts3032_configured_baud_rate = 0U;
        return false;
    }

    sts3032_initialized = true;
    sts3032_last_error = STS3032_ERROR_NONE;
    sts3032_last_device_status = 0U;
    sts3032_configured_baud_rate = baud_rate;
    sts3032_tx_packet_count = 0U;
    sts3032_rx_packet_count = 0U;
    sts3032_rx_overrun_count = 0U;
    Sts3032_ClearRxBuffer();

    return true;
}

bool sts3032_ping(uint8_t id, uint32_t timeout_ms)
{
    uint8_t response_length = 0U;

    if (Sts3032_IsServoIdValid(id) == false)
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    if (Sts3032_SendPacket(id, STS3032_INST_PING, 0, 0U) == false)
    {
        return false;
    }

    return Sts3032_WaitStatus(id,
                              STS3032_INST_PING,
                              0,
                              0U,
                              &response_length,
                              timeout_ms);
}

bool sts3032_write_register(uint8_t id,
                            uint8_t start_addr,
                            const uint8_t *data,
                            uint8_t length,
                            uint32_t timeout_ms)
{
    uint8_t params[STS3032_RX_PARAM_BUFFER_SIZE];

    if ((Sts3032_IsPacketIdValid(id) == false) ||
        (data == 0) ||
        (length == 0U) ||
        ((uint8_t)(length + 1U) > STS3032_RX_PARAM_BUFFER_SIZE))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    params[0] = start_addr;
    memcpy(&params[1], data, length);

    if (Sts3032_SendPacket(id,
                           STS3032_INST_WRITE,
                           params,
                           (uint8_t)(length + 1U)) == false)
    {
        return false;
    }

    if (id == STS3032_BROADCAST_ID)
    {
        return true;
    }

    return Sts3032_WaitStatus(id, STS3032_INST_WRITE, 0, 0U, 0, timeout_ms);
}

bool sts3032_read_register(uint8_t id,
                           uint8_t start_addr,
                           uint8_t length,
                           uint8_t *buffer,
                           uint8_t buffer_size,
                           uint32_t timeout_ms)
{
    uint8_t params[2];
    uint8_t received_length = 0U;

    if ((Sts3032_IsServoIdValid(id) == false) ||
        (length == 0U) ||
        (buffer == 0) ||
        (buffer_size < length))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    params[0] = start_addr;
    params[1] = length;

    if (Sts3032_SendPacket(id, STS3032_INST_READ, params, (uint8_t)sizeof(params)) == false)
    {
        return false;
    }

    if (Sts3032_WaitStatus(id,
                           STS3032_INST_READ,
                           buffer,
                           buffer_size,
                           &received_length,
                           timeout_ms) == false)
    {
        return false;
    }

    if (received_length != length)
    {
        sts3032_last_error = STS3032_ERROR_FRAME;
        return false;
    }

    return true;
}

bool sts3032_set_torque(uint8_t id, bool enable, uint32_t timeout_ms)
{
    uint8_t value = enable ? 1U : 0U;

    return sts3032_write_register(id,
                                  STS3032_REG_TORQUE_ENABLE,
                                  &value,
                                  (uint8_t)sizeof(value),
                                  timeout_ms);
}

bool sts3032_sync_write_torque(const uint8_t *ids, uint8_t count, bool enable)
{
    uint8_t params[STS3032_SYNC_TORQUE_PARAM_SIZE];
    uint8_t index;

    if ((ids == 0) || (count == 0U) || (count > STS3032_SYNC_MAX_TARGETS))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    params[0] = STS3032_REG_TORQUE_ENABLE;
    params[1] = 1U;
    for (index = 0U; index < count; index++)
    {
        if (Sts3032_IsServoIdValid(ids[index]) == false)
        {
            sts3032_last_error = STS3032_ERROR_PARAM;
            return false;
        }

        params[(uint8_t)(2U + index * 2U)] = ids[index];
        params[(uint8_t)(3U + index * 2U)] = enable ? 1U : 0U;
    }

    return Sts3032_SendPacket(STS3032_BROADCAST_ID,
                              STS3032_INST_SYNC_WRITE,
                              params,
                              (uint8_t)(2U + count * 2U));
}

bool sts3032_set_operation_mode(uint8_t id,
                                Sts3032OperationMode mode,
                                uint32_t timeout_ms)
{
    uint8_t value = (uint8_t)mode;

    if (Sts3032_IsModeValid(mode) == false)
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    return sts3032_write_register(id,
                                  STS3032_REG_OPERATION_MODE,
                                  &value,
                                  (uint8_t)sizeof(value),
                                  timeout_ms);
}

bool sts3032_sync_write_operation_mode(const uint8_t *ids,
                                       uint8_t count,
                                       Sts3032OperationMode mode)
{
    uint8_t params[STS3032_SYNC_MODE_PARAM_SIZE];
    uint8_t index;

    if ((ids == 0) ||
        (count == 0U) ||
        (count > STS3032_SYNC_MAX_TARGETS) ||
        (Sts3032_IsModeValid(mode) == false))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    /*
     * 同步写格式：广播 ID + 指令 0x83 + 起始地址 0x4D + 每个 ID 写 1 字节模式。
     * 广播同步写没有状态返回，调用者应在必要时单独 PING 或读寄存器确认链路。
     */
    params[0] = STS3032_REG_OPERATION_MODE;
    params[1] = 1U;
    for (index = 0U; index < count; index++)
    {
        if (Sts3032_IsServoIdValid(ids[index]) == false)
        {
            sts3032_last_error = STS3032_ERROR_PARAM;
            return false;
        }

        params[(uint8_t)(2U + index * 2U)] = ids[index];
        params[(uint8_t)(3U + index * 2U)] = (uint8_t)mode;
    }

    return Sts3032_SendPacket(STS3032_BROADCAST_ID,
                              STS3032_INST_SYNC_WRITE,
                              params,
                              (uint8_t)(2U + count * 2U));
}

bool sts3032_set_position(uint8_t id,
                          uint16_t position,
                          uint16_t time,
                          uint16_t speed,
                          uint32_t timeout_ms)
{
    uint8_t data[6];

    if ((Sts3032_IsServoIdValid(id) == false) ||
        (Sts3032_IsPositionValid(position) == false) ||
        (Sts3032_IsSpeedValid(speed) == false))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    Sts3032_WriteU16LE(&data[0], position);
    Sts3032_WriteU16LE(&data[2], time);
    Sts3032_WriteU16LE(&data[4], speed);

    return sts3032_write_register(id,
                                  STS3032_REG_GOAL_POSITION,
                                  data,
                                  (uint8_t)sizeof(data),
                                  timeout_ms);
}

bool sts3032_set_position_with_acc(uint8_t id,
                                   uint8_t acceleration,
                                   uint16_t position,
                                   uint16_t time,
                                   uint16_t speed,
                                   uint32_t timeout_ms)
{
    uint8_t data[7];

    if ((Sts3032_IsServoIdValid(id) == false) ||
        (Sts3032_IsPositionValid(position) == false) ||
        (Sts3032_IsSpeedValid(speed) == false))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    data[0] = acceleration;
    Sts3032_WriteU16LE(&data[1], position);
    Sts3032_WriteU16LE(&data[3], time);
    Sts3032_WriteU16LE(&data[5], speed);

    return sts3032_write_register(id,
                                  STS3032_REG_GOAL_ACC,
                                  data,
                                  (uint8_t)sizeof(data),
                                  timeout_ms);
}

bool sts3032_sync_write_positions(const Sts3032PositionTarget *targets,
                                  uint8_t count)
{
    uint8_t params[STS3032_SYNC_POSITION_PARAM_SIZE];
    uint8_t index;

    if ((targets == 0) || (count == 0U) || (count > STS3032_SYNC_MAX_TARGETS))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    params[0] = STS3032_REG_GOAL_POSITION;
    params[1] = 6U;
    for (index = 0U; index < count; index++)
    {
        uint8_t offset = (uint8_t)(2U + index * 7U);

        if ((Sts3032_IsServoIdValid(targets[index].id) == false) ||
            (Sts3032_IsPositionValid(targets[index].position) == false) ||
            (Sts3032_IsSpeedValid(targets[index].speed) == false))
        {
            sts3032_last_error = STS3032_ERROR_PARAM;
            return false;
        }

        params[offset] = targets[index].id;
        Sts3032_WriteU16LE(&params[(uint8_t)(offset + 1U)], targets[index].position);
        Sts3032_WriteU16LE(&params[(uint8_t)(offset + 3U)], targets[index].time);
        Sts3032_WriteU16LE(&params[(uint8_t)(offset + 5U)], targets[index].speed);
    }

    return Sts3032_SendPacket(STS3032_BROADCAST_ID,
                              STS3032_INST_SYNC_WRITE,
                              params,
                              (uint8_t)(2U + count * 7U));
}

bool sts3032_set_wheel_speed(uint8_t id, int16_t speed, uint32_t timeout_ms)
{
    uint8_t data[2];
    uint16_t encoded_speed;

    if ((Sts3032_IsServoIdValid(id) == false) ||
        (Sts3032_IsWheelSpeedValid(speed) == false))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    /*
     * 恒速模式下写运行速度寄存器 78(0x4E)。
     * 0=停止，正数为顺时针，负数置 bit15 为逆时针；字节序沿用 STS/SMS 小端格式。
     */
    encoded_speed = Sts3032_EncodeWheelSpeed(speed);
    Sts3032_WriteU16LE(data, encoded_speed);

    return sts3032_write_register(id,
                                  STS3032_REG_RUNNING_SPEED,
                                  data,
                                  (uint8_t)sizeof(data),
                                  timeout_ms);
}

bool sts3032_sync_write_wheel_speeds(const Sts3032WheelTarget *targets,
                                     uint8_t count)
{
    uint8_t params[STS3032_SYNC_WHEEL_PARAM_SIZE];
    uint8_t index;

    if ((targets == 0) || (count == 0U) || (count > STS3032_SYNC_MAX_TARGETS))
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    /*
     * 同步写恒速速度：起始地址 0x4E，每个 ID 写 2 字节速度。
     * 该接口用于多 ID 同时开始、停止或改变方向，广播帧不会等待单个舵机状态包。
     */
    params[0] = STS3032_REG_RUNNING_SPEED;
    params[1] = 2U;
    for (index = 0U; index < count; index++)
    {
        uint8_t offset = (uint8_t)(2U + index * 3U);
        uint16_t encoded_speed;

        if ((Sts3032_IsServoIdValid(targets[index].id) == false) ||
            (Sts3032_IsWheelSpeedValid(targets[index].speed) == false))
        {
            sts3032_last_error = STS3032_ERROR_PARAM;
            return false;
        }

        encoded_speed = Sts3032_EncodeWheelSpeed(targets[index].speed);
        params[offset] = targets[index].id;
        Sts3032_WriteU16LE(&params[(uint8_t)(offset + 1U)], encoded_speed);
    }

    return Sts3032_SendPacket(STS3032_BROADCAST_ID,
                              STS3032_INST_SYNC_WRITE,
                              params,
                              (uint8_t)(2U + count * 3U));
}

bool sts3032_read_operation_mode(uint8_t id,
                                 Sts3032OperationMode *mode,
                                 uint32_t timeout_ms)
{
    uint8_t value;

    if (mode == 0)
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    if (sts3032_read_register(id,
                              STS3032_REG_OPERATION_MODE,
                              (uint8_t)sizeof(value),
                              &value,
                              (uint8_t)sizeof(value),
                              timeout_ms) == false)
    {
        return false;
    }

    if ((value > (uint8_t)STS3032_MODE_STEP))
    {
        sts3032_last_error = STS3032_ERROR_FRAME;
        return false;
    }

    *mode = (Sts3032OperationMode)value;
    return true;
}

bool sts3032_read_wheel_speed(uint8_t id, int16_t *speed, uint32_t timeout_ms)
{
    uint8_t data[2];
    uint16_t raw_speed;

    if (speed == 0)
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    if (sts3032_read_register(id,
                              STS3032_REG_RUNNING_SPEED,
                              (uint8_t)sizeof(data),
                              data,
                              (uint8_t)sizeof(data),
                              timeout_ms) == false)
    {
        return false;
    }

    /*
     * 回读速度字段用于现场判断：如果 mode=1 但 speed 仍为 0，说明速度写入没有生效；
     * 如果 speed 有值但电机不动，则应优先检查供电、扭矩或机械负载。
     */
    raw_speed = Sts3032_ReadU16LE(data);
    *speed = Sts3032_DecodeWheelSpeed(raw_speed);
    return true;
}

bool sts3032_read_position(uint8_t id, uint16_t *position, uint32_t timeout_ms)
{
    uint8_t data[2];

    if (position == 0)
    {
        sts3032_last_error = STS3032_ERROR_PARAM;
        return false;
    }

    if (sts3032_read_register(id,
                              STS3032_REG_PRESENT_POSITION,
                              (uint8_t)sizeof(data),
                              data,
                              (uint8_t)sizeof(data),
                              timeout_ms) == false)
    {
        return false;
    }

    *position = Sts3032_ReadU16LE(data);
    return true;
}

/*
 * UART6 接收中断。
 * 职责：在 1Mbps 字节流到达时尽快读空 RX FIFO，并写入软件环形缓冲。
 * 该函数名与 SysConfig 生成的 UART_6_INST_IRQHandler 对应，启动文件会引用本符号。
 */
void UART6_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_6_INST) == DL_UART_MAIN_IIDX_RX)
    {
        while (DL_UART_Main_isRXFIFOEmpty(UART_6_INST) == false)
        {
            uint8_t data = DL_UART_Main_receiveData(UART_6_INST);
            uint16_t next_head = Sts3032_RxNextIndex(sts3032_rx_head);

            if (next_head != sts3032_rx_tail)
            {
                sts3032_rx_ring[sts3032_rx_head] = data;
                sts3032_rx_head = next_head;
            }
            else
            {
                sts3032_rx_overrun_count++;
            }
        }
    }
}

Sts3032Error sts3032_get_last_error(void)
{
    return sts3032_last_error;
}

uint8_t sts3032_get_last_device_status(void)
{
    return sts3032_last_device_status;
}

uint32_t sts3032_get_tx_packet_count(void)
{
    return sts3032_tx_packet_count;
}

uint32_t sts3032_get_rx_packet_count(void)
{
    return sts3032_rx_packet_count;
}

uint32_t sts3032_get_rx_overrun_count(void)
{
    return sts3032_rx_overrun_count;
}

uint32_t sts3032_get_configured_baud_rate(void)
{
    return sts3032_configured_baud_rate;
}

bool sts3032_is_uart_enabled(void)
{
    return DL_UART_Main_isEnabled(UART_6_INST);
}

uint32_t sts3032_get_uart_integer_divisor(void)
{
    return DL_UART_Main_getIntegerBaudRateDivisor(UART_6_INST);
}

uint32_t sts3032_get_uart_fractional_divisor(void)
{
    return DL_UART_Main_getFractionalBaudRateDivisor(UART_6_INST);
}

const char *sts3032_get_error_string(Sts3032Error error)
{
    switch (error)
    {
        case STS3032_ERROR_NONE:
            return "none";

        case STS3032_ERROR_NOT_READY:
            return "not_ready";

        case STS3032_ERROR_PARAM:
            return "param";

        case STS3032_ERROR_TIMEOUT:
            return "timeout";

        case STS3032_ERROR_CHECKSUM:
            return "checksum";

        case STS3032_ERROR_DEVICE_STATUS:
            return "device_status";

        case STS3032_ERROR_FRAME:
            return "frame";

        case STS3032_ERROR_BUFFER:
            return "buffer";

        default:
            return "unknown";
    }
}
