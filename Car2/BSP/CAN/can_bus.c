#include "CAN/can_bus.h"

/*
 * MCAN 驱动实现说明：
 * 1. SysConfig 已经完成 CANFD0 的时钟、PA12/PA13 复用和消息 RAM 地址分配。
 * 2. 本文件只在应用启动阶段把 MCAN 短暂切到 SW_INIT，补充“接收所有标准/扩展数据帧到 FIFO0”
 *    的滤波策略，然后恢复 NORMAL 模式并打开 FIFO0 新消息中断。
 * 3. 接收中断不能直接 printf，否则 UART0 阻塞发送会拉长中断时间；ISR 只搬运帧到环形队列，
 *    主循环通过 can_bus_poll_rx() 取出后再打印。
 */

#define CAN_BUS_STD_ID_MAX             (0x7FFU)
#define CAN_BUS_EXT_ID_MAX             (0x1FFFFFFFU)
#define CAN_BUS_TX_BUFFER_INDEX        (0U)
#define CAN_BUS_RX_QUEUE_SIZE          (8U)
#define CAN_BUS_MODE_WAIT_LOOP         (100000UL)
#define CAN_BUS_STD_ID_SHIFT           (18U)
#define CAN_BUS_STD_ID_MASK_IN_RX      (0x1FFC0000UL)

#define CAN_BUS_FILTER_STORE_FIFO0     (1U)
#define CAN_BUS_FILTER_RANGE           (0U)

/*
 * 中断掩码只打开“有新帧”和关键错误。
 * 未打开 PEA/PED 协议错误：当总线上暂时没有其它节点 ACK 时，MCAN 可能连续产生协议错误，
 * 若每次都从 UART0 打印会淹没真正的接收日志；TX busy 和 bus warning/passive/off 已足够定位问题。
 */
#define CAN_BUS_IRQ_RX_MASK            (DL_MCAN_INTR_SRC_RX_FIFO0_NEW_MSG)
#define CAN_BUS_IRQ_DRAIN_MASK         (DL_MCAN_INTR_SRC_RX_FIFO0_NEW_MSG | \
                                        DL_MCAN_INTR_SRC_RX_FIFO0_FULL)
#define CAN_BUS_IRQ_ERROR_MASK         (DL_MCAN_INTR_SRC_RX_FIFO0_FULL | \
                                        DL_MCAN_INTR_SRC_RX_FIFO0_MSG_LOST | \
                                        DL_MCAN_INTR_SRC_BUS_OFF_STATUS | \
                                        DL_MCAN_INTR_SRC_WARNING_STATUS | \
                                        DL_MCAN_INTR_SRC_ERR_PASSIVE | \
                                        DL_MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE)
#define CAN_BUS_IRQ_MASK               (CAN_BUS_IRQ_RX_MASK | CAN_BUS_IRQ_ERROR_MASK)

#ifdef MCAN0_INST_INT_IRQN
#define CAN_BUS_IRQN                   (MCAN0_INST_INT_IRQN)
#else
#define CAN_BUS_IRQN                   (CANFD0_INT_IRQn)
#endif

static volatile bool s_can_ready = false;
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static volatile uint32_t s_rx_count = 0U;
static volatile uint32_t s_rx_drop_count = 0U;
static volatile uint32_t s_tx_count = 0U;
static volatile uint32_t s_tx_busy_count = 0U;
static volatile uint32_t s_error_count = 0U;
static volatile uint32_t s_error_irq_status = 0U;
static CanBusFrame s_rx_queue[CAN_BUS_RX_QUEUE_SIZE];

/*
 * 将 DLC 转换为真实数据字节数。
 * 经典 CAN 只会使用 0~8；CAN FD 报文可能使用 9~15，本驱动当前只暴露前 8 字节，
 * 但仍保留 DLC 与 truncated 标志，方便调试串口判断是否收到 FD 长帧。
 */
static uint8_t can_bus_dlc_to_length(uint8_t dlc)
{
    static const uint8_t dlc_length_table[16] = {
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
        8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
    };

    return dlc_length_table[dlc & 0x0FU];
}

static uint8_t can_bus_next_queue_index(uint8_t index)
{
    index++;
    if (index >= CAN_BUS_RX_QUEUE_SIZE) {
        index = 0U;
    }

    return index;
}

/*
 * 等待 MCAN 进入目标工作模式。
 * 输入：mode 为 DL_MCAN_OPERATION_MODE_xxx；输出：true 表示在有限循环内进入目标状态。
 * 这里不做无限等待，避免外设异常时主程序卡死在启动阶段。
 */
static bool can_bus_wait_operation_mode(uint32_t mode)
{
    uint32_t wait_loop = CAN_BUS_MODE_WAIT_LOOP;

    while (wait_loop > 0U) {
        if (DL_MCAN_getOpMode(MCAN0_INST) == mode) {
            return true;
        }
        wait_loop--;
    }

    return false;
}

/*
 * 配置全局接收策略与消息 ID 滤波表。
 * 原因：SysConfig 负责分配消息 RAM，但当前工程需要“接收到总线上任意帧都能打印”，
 * 因此启动时将非匹配帧、标准 ID 范围 0x000~0x7FF、扩展 ID 范围 0x00000000~0x1FFFFFFF
 * 都导入 RX FIFO0。远程帧不携带数据，这里直接拒绝，避免调试输出误解为空数据帧。
 */
static bool can_bus_config_accept_all_to_fifo0(void)
{
    static const DL_MCAN_ConfigParams config = {
        .monEnable = 0U,
        .asmEnable = 0U,
        .tsPrescalar = 0U,
        .tsSelect = 0U,
        .timeoutSelect = DL_MCAN_TIMEOUT_SELECT_CONT,
        .timeoutPreload = 0U,
        .timeoutCntEnable = 0U,
        .filterConfig = {
            .rrfe = 1U,
            .rrfs = 1U,
            .anfe = 0U,
            .anfs = 0U,
        },
    };
    static const DL_MCAN_StdMsgIDFilterElement std_filter = {
        .sfid2 = CAN_BUS_STD_ID_MAX,
        .sfid1 = 0U,
        .sfec = CAN_BUS_FILTER_STORE_FIFO0,
        .sft = CAN_BUS_FILTER_RANGE,
    };
    static const DL_MCAN_ExtMsgIDFilterElement ext_filter = {
        .efid1 = 0U,
        .efec = CAN_BUS_FILTER_STORE_FIFO0,
        .efid2 = CAN_BUS_EXT_ID_MAX,
        .eft = CAN_BUS_FILTER_RANGE,
    };

    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_SW_INIT);
    if (can_bus_wait_operation_mode(DL_MCAN_OPERATION_MODE_SW_INIT) == false) {
        return false;
    }

    if (DL_MCAN_config(MCAN0_INST, &config) != 0) {
        return false;
    }

#if (MCAN0_INST_MCAN_STD_ID_FILTER_NUM > 0)
    DL_MCAN_addStdMsgIDFilter(MCAN0_INST, 0U, &std_filter);
#endif

#if (MCAN0_INST_MCAN_EXT_ID_FILTER_NUM > 0)
    DL_MCAN_addExtMsgIDFilter(MCAN0_INST, 0U, &ext_filter);
#endif
    (void)DL_MCAN_setExtIDAndMask(MCAN0_INST, CAN_BUS_EXT_ID_MAX);

    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);
    return can_bus_wait_operation_mode(DL_MCAN_OPERATION_MODE_NORMAL);
}

/*
 * 打开 MCAN 内核中断、TI wrapper 中断和 NVIC。
 * FIFO0 新消息进入 Line1，错误状态也进入 Line1；主循环可通过
 * can_bus_take_error_status() 获取聚合后的错误标志，便于判断无 ACK、总线关闭等问题。
 */
static void can_bus_enable_interrupts(void)
{
    DL_MCAN_enableIntr(MCAN0_INST, DL_MCAN_INTR_MASK_ALL, false);
    DL_MCAN_clearIntrStatus(MCAN0_INST, DL_MCAN_INTR_MASK_ALL, DL_MCAN_INTR_SRC_MCAN_LINE_1);

    DL_MCAN_selectIntrLine(MCAN0_INST, CAN_BUS_IRQ_MASK, DL_MCAN_INTR_LINE_NUM_1);
    DL_MCAN_enableIntr(MCAN0_INST, CAN_BUS_IRQ_MASK, true);
    DL_MCAN_enableIntrLine(MCAN0_INST, DL_MCAN_INTR_LINE_NUM_1, true);

    DL_MCAN_clearInterruptStatus(MCAN0_INST, DL_MCAN_MSP_INTERRUPT_LINE1);
    DL_MCAN_enableInterrupt(MCAN0_INST, DL_MCAN_MSP_INTERRUPT_LINE1);

    NVIC_ClearPendingIRQ(CAN_BUS_IRQN);
    NVIC_EnableIRQ(CAN_BUS_IRQN);
}

static void can_bus_reset_runtime_state(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    s_rx_drop_count = 0U;
    s_tx_count = 0U;
    s_tx_busy_count = 0U;
    s_error_count = 0U;
    s_error_irq_status = 0U;
}

/*
 * 将 DriverLib 的接收对象转换为上层通用帧结构。
 * 标准帧 ID 在 MCAN RAM 中位于 ID[28:18]，扩展帧 ID 右对齐存放；这是 TI 示例里的固定格式。
 */
static void can_bus_decode_rx_frame(const DL_MCAN_RxBufElement *rx_msg, CanBusFrame *frame)
{
    uint8_t payload_length;
    uint8_t copy_length;
    uint8_t i;

    if ((rx_msg == NULL) || (frame == NULL)) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->extended = (rx_msg->xtd != 0U);
    frame->remote = (rx_msg->rtr != 0U);
    frame->fd_format = (rx_msg->fdf != 0U);
    frame->bit_rate_switch = (rx_msg->brs != 0U);
    frame->dlc = (uint8_t)(rx_msg->dlc & 0x0FU);
    frame->id = frame->extended ?
                (rx_msg->id & CAN_BUS_EXT_ID_MAX) :
                ((rx_msg->id & CAN_BUS_STD_ID_MASK_IN_RX) >> CAN_BUS_STD_ID_SHIFT);

    payload_length = can_bus_dlc_to_length(frame->dlc);
    copy_length = payload_length;
    if (copy_length > CAN_BUS_MAX_DATA_LENGTH) {
        copy_length = CAN_BUS_MAX_DATA_LENGTH;
        frame->truncated = true;
    }
    if (frame->remote != false) {
        copy_length = 0U;
    }

    frame->length = copy_length;
    for (i = 0U; i < copy_length; i++) {
        frame->data[i] = rx_msg->data[i];
    }
}

static void can_bus_push_rx_frame(const CanBusFrame *frame)
{
    uint8_t next_head;

    if (frame == NULL) {
        return;
    }

    next_head = can_bus_next_queue_index(s_rx_head);
    if (next_head == s_rx_tail) {
        s_rx_drop_count++;
        return;
    }

    s_rx_queue[s_rx_head] = *frame;
    s_rx_head = next_head;
    s_rx_count++;
}

/*
 * 从 RX FIFO0 搬运所有当前可见的帧。
 * guard 用 FIFO 深度加 1 限制循环次数，避免异常状态下 ISR 长时间占用 CPU。
 */
static void can_bus_drain_rx_fifo0(void)
{
    DL_MCAN_RxFIFOStatus rx_fifo_status;
    DL_MCAN_RxBufElement rx_msg;
    CanBusFrame frame;
    uint8_t guard = (uint8_t)(MCAN0_INST_MCAN_FIFO_0_NUM + 1U);

    while (guard > 0U) {
        memset(&rx_fifo_status, 0, sizeof(rx_fifo_status));
        rx_fifo_status.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &rx_fifo_status);
        if (rx_fifo_status.fillLvl == 0U) {
            break;
        }

        memset(&rx_msg, 0, sizeof(rx_msg));
        DL_MCAN_readMsgRam(MCAN0_INST,
                           DL_MCAN_MEM_TYPE_FIFO,
                           0U,
                           rx_fifo_status.num,
                           &rx_msg);
        (void)DL_MCAN_writeRxFIFOAck(MCAN0_INST,
                                     rx_fifo_status.num,
                                     rx_fifo_status.getIdx);

        can_bus_decode_rx_frame(&rx_msg, &frame);
        can_bus_push_rx_frame(&frame);
        guard--;
    }
}

bool can_bus_init(void)
{
    s_can_ready = false;
    can_bus_reset_runtime_state();

    if (can_bus_config_accept_all_to_fifo0() == false) {
        s_error_count++;
        return false;
    }

    can_bus_enable_interrupts();
    s_can_ready = true;
    return true;
}

bool can_bus_send(uint32_t id, bool extended, const uint8_t *data, uint8_t length)
{
    DL_MCAN_TxBufElement tx_msg;
    uint8_t i;

    if ((s_can_ready == false) || (length > CAN_BUS_MAX_DATA_LENGTH)) {
        return false;
    }
    if ((length > 0U) && (data == NULL)) {
        return false;
    }
    if (((extended == false) && (id > CAN_BUS_STD_ID_MAX)) ||
        ((extended != false) && (id > CAN_BUS_EXT_ID_MAX))) {
        return false;
    }
    if (DL_MCAN_getOpMode(MCAN0_INST) != DL_MCAN_OPERATION_MODE_NORMAL) {
        return false;
    }
    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & (1UL << CAN_BUS_TX_BUFFER_INDEX)) != 0U) {
        s_tx_busy_count++;
        return false;
    }

    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.id = (extended != false) ? id : (id << CAN_BUS_STD_ID_SHIFT);
    tx_msg.rtr = 0U;
    tx_msg.xtd = (extended != false) ? 1U : 0U;
    tx_msg.esi = 0U;
    tx_msg.dlc = length;
    tx_msg.brs = 0U;
    tx_msg.fdf = 0U;
    tx_msg.efc = 0U;
    tx_msg.mm = 0U;
    for (i = 0U; i < length; i++) {
        tx_msg.data[i] = data[i];
    }

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, CAN_BUS_TX_BUFFER_INDEX, &tx_msg);
    if (DL_MCAN_TXBufAddReq(MCAN0_INST, CAN_BUS_TX_BUFFER_INDEX) != 0) {
        s_error_count++;
        return false;
    }

    s_tx_count++;
    return true;
}

bool can_bus_send_standard(uint16_t id, const uint8_t *data, uint8_t length)
{
    return can_bus_send((uint32_t)id, false, data, length);
}

bool can_bus_poll_rx(CanBusFrame *frame)
{
    bool has_frame = false;

    if (frame == NULL) {
        return false;
    }

    NVIC_DisableIRQ(CAN_BUS_IRQN);
    if (s_rx_tail != s_rx_head) {
        *frame = s_rx_queue[s_rx_tail];
        s_rx_tail = can_bus_next_queue_index(s_rx_tail);
        has_frame = true;
    }
    NVIC_EnableIRQ(CAN_BUS_IRQN);

    return has_frame;
}

bool can_bus_take_error_status(uint32_t *irq_status)
{
    uint32_t status;

    if (irq_status == NULL) {
        return false;
    }

    NVIC_DisableIRQ(CAN_BUS_IRQN);
    status = s_error_irq_status;
    s_error_irq_status = 0U;
    NVIC_EnableIRQ(CAN_BUS_IRQN);

    *irq_status = status;
    return (status != 0U);
}

uint32_t can_bus_get_rx_count(void)
{
    return s_rx_count;
}

uint32_t can_bus_get_rx_drop_count(void)
{
    return s_rx_drop_count;
}

uint32_t can_bus_get_tx_count(void)
{
    return s_tx_count;
}

uint32_t can_bus_get_tx_busy_count(void)
{
    return s_tx_busy_count;
}

uint32_t can_bus_get_error_count(void)
{
    return s_error_count;
}

bool can_bus_is_ready(void)
{
    return s_can_ready;
}

void CANFD0_IRQHandler(void)
{
    DL_MCAN_IIDX pending = DL_MCAN_getPendingInterrupt(MCAN0_INST);
    uint32_t intr_status;

    switch (pending) {
    case DL_MCAN_IIDX_LINE1:
        intr_status = DL_MCAN_getIntrStatus(MCAN0_INST) & CAN_BUS_IRQ_MASK;
        DL_MCAN_clearIntrStatus(MCAN0_INST, intr_status, DL_MCAN_INTR_SRC_MCAN_LINE_1);
        DL_MCAN_clearInterruptStatus(MCAN0_INST, DL_MCAN_MSP_INTERRUPT_LINE1);

        if ((intr_status & CAN_BUS_IRQ_ERROR_MASK) != 0U) {
            s_error_irq_status |= (intr_status & CAN_BUS_IRQ_ERROR_MASK);
            s_error_count++;
        }
        if ((intr_status & CAN_BUS_IRQ_DRAIN_MASK) != 0U) {
            can_bus_drain_rx_fifo0();
        }
        break;

    default:
        break;
    }
}
