#ifndef CAN_BUS_H
#define CAN_BUS_H

/*
 * PA12/PA13 外接 CAN 收发器的板级驱动接口。
 *
 * 硬件与 SysConfig 约束：
 * - User/config.syscfg 中的 MCAN0/CANFD0 负责完成外设时钟、消息 RAM、PA12=CANTX、
 *   PA13=CANRX 的初始化，本驱动不重新初始化管脚和时钟。
 * - 当前测试按经典 CAN 数据帧工作，默认仲裁速率在 SysConfig 中配置为 500 kbit/s。
 * - CAN 是多主总线，发送帧需要总线上至少有一个正常节点应答 ACK；若只有本板一个节点，
 *   控制器会持续重发或上报错误，这是 CAN 协议的正常表现。
 *
 * 软件职责：
 * - can_bus_init() 在 SysConfig 初始化之后补充接收滤波和 MCAN 中断使能。
 * - ISR 只负责把收到的帧搬到环形队列，printf 等耗时操作必须放在主循环中完成。
 * - BSP 头文件只包含 bsp_common.h，不直接包含聚合头 bsp.h。
 */
#include "bsp_common.h"

#define CAN_BUS_DEFAULT_BITRATE_KBPS (500U)
#define CAN_BUS_TEST_TX_ID           (0x351U)
#define CAN_BUS_TX_PERIOD_MS         (1000U)
#define CAN_BUS_MAX_DATA_LENGTH      (8U)

typedef struct {
    /*
     * id：标准帧为 11 bit，扩展帧为 29 bit，均以右对齐形式提供给上层。
     * dlc：原始 CAN DLC 字段；length：本驱动实际拷贝到 data[] 的字节数，经典 CAN 最大 8。
     */
    uint32_t id;
    uint8_t dlc;
    uint8_t length;
    uint8_t data[CAN_BUS_MAX_DATA_LENGTH];
    bool extended;
    bool remote;
    bool fd_format;
    bool bit_rate_switch;
    bool truncated;
} CanBusFrame;

/*
 * 初始化 PA12/PA13 CAN 总线驱动。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后调用，MCAN0 的时钟、管脚和消息 RAM 已由 SysConfig 配好。
 *
 * 行为：
 * - 补充接收所有标准/扩展数据帧到 FIFO0 的滤波规则。
 * - 打开 MCAN FIFO0 新消息中断，并初始化软件接收队列与统计计数。
 *
 * 返回值：
 * - true：MCAN 进入正常工作模式。
 * - false：模式切换或寄存器配置超时。
 */
bool can_bus_init(void);

/*
 * 发送一帧 CAN 数据帧。
 *
 * 参数：
 * - id：标准帧使用 11 bit，扩展帧使用 29 bit，均右对齐传入。
 * - extended：true 发送扩展帧，false 发送标准帧。
 * - data：数据缓冲区，length 为 0 时可为 NULL。
 * - length：数据长度，经典 CAN 最大 8 字节。
 *
 * 返回值：
 * - true：帧已成功写入 TX buffer 并请求发送。
 * - false：驱动未初始化、参数非法或 TX buffer 正忙。
 */
bool can_bus_send(uint32_t id, bool extended, const uint8_t *data, uint8_t length);

/*
 * 发送标准 11 bit CAN 数据帧。
 *
 * 参数：
 * - id：标准帧 ID，范围 0x000~0x7FF。
 * - data：数据缓冲区，length 为 0 时可为 NULL。
 * - length：数据长度，经典 CAN 最大 8 字节。
 *
 * 返回值：
 * - true：发送请求提交成功。
 * - false：参数非法或硬件暂不可发送。
 */
bool can_bus_send_standard(uint16_t id, const uint8_t *data, uint8_t length);

/*
 * 从软件接收队列取出一帧 CAN 报文。
 *
 * 参数：
 * - frame：输出帧结构体指针，不可为 NULL。
 *
 * 返回值：
 * - true：成功取出一帧。
 * - false：队列为空或参数无效。
 *
 * 中断约束：
 * - CANFD0_IRQHandler 只负责把硬件 FIFO 搬入该软件队列；打印和业务处理应在主循环中轮询本函数。
 */
bool can_bus_poll_rx(CanBusFrame *frame);

/*
 * 取走最近一次 CAN 中断错误状态。
 *
 * 参数：
 * - irq_status：输出 MCAN 中断状态位，可为 NULL 仅清除标志。
 *
 * 返回值：
 * - true：存在未消费的错误/告警状态。
 * - false：没有新的错误状态。
 */
bool can_bus_take_error_status(uint32_t *irq_status);

/*
 * 获取累计接收帧计数。
 *
 * 返回值：
 * - 自 can_bus_init() 后成功入队的 CAN 帧数量。
 */
uint32_t can_bus_get_rx_count(void);

/*
 * 获取接收队列溢出丢帧计数。
 *
 * 返回值：
 * - 软件环形队列满时被丢弃的 CAN 帧数量。
 */
uint32_t can_bus_get_rx_drop_count(void);

/*
 * 获取累计发送请求计数。
 *
 * 返回值：
 * - 成功提交到 MCAN TX buffer 的帧数量。
 */
uint32_t can_bus_get_tx_count(void);

/*
 * 获取 TX buffer 忙计数。
 *
 * 返回值：
 * - 调用发送接口时发现硬件发送缓冲仍忙的次数。
 */
uint32_t can_bus_get_tx_busy_count(void);

/*
 * 获取 CAN 错误/告警中断累计次数。
 *
 * 返回值：
 * - CAN FIFO 满、丢帧、Bus-Off、Error Passive 等错误状态累计次数。
 */
uint32_t can_bus_get_error_count(void);

/*
 * 查询 CAN 驱动是否已经完成初始化。
 *
 * 返回值：
 * - true：can_bus_init() 成功，允许收发。
 * - false：尚未初始化或初始化失败。
 */
bool can_bus_is_ready(void);

#endif
