#ifndef STS3032_H
#define STS3032_H

/*
 * STS3032 串口总线舵机 BSP 驱动。
 *
 * 职责：
 * 1. 使用 SysConfig 中名为 UART_6 的 UART6 外设与 STS3032 通信。
 * 2. UART6 的外设时钟、PB21/PB22 引脚复用、1000000 8N1 波特率、FIFO 和 RX
 *    中断均由 User/config.syscfg 生成，本模块不重复初始化 UART 寄存器。
 * 3. 提供 PING、扭矩使能、位置控制、当前位置读取和多 ID 同步写接口。
 *
 * 硬件约束：
 * - PB22 为 UART6_TX，连接舵机接收端或总线转换电路 TX 输入。
 * - PB21 为 UART6_RX，连接舵机返回端或总线转换电路 RX 输出。
 * - MCU 与舵机电源必须共地，舵机 ID 和波特率必须与本模块配置一致。
 */
#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STS3032_DEFAULT_BAUD_RATE          (1000000U)
#define STS3032_RESPONSE_TIMEOUT_MS        (20U)

#define STS3032_ID_MAX                     (253U)
#define STS3032_BROADCAST_ID               (254U)
#define STS3032_POSITION_MAX               (4095U)
#define STS3032_SPEED_MAX                  (3400U)
#define STS3032_WHEEL_SPEED_MIN            (50U)
#define STS3032_WHEEL_SPEED_MAX            (3400U)
#define STS3032_SYNC_MAX_TARGETS           (16U)

typedef enum
{
    STS3032_ERROR_NONE = 0,
    STS3032_ERROR_NOT_READY,
    STS3032_ERROR_PARAM,
    STS3032_ERROR_TIMEOUT,
    STS3032_ERROR_CHECKSUM,
    STS3032_ERROR_DEVICE_STATUS,
    STS3032_ERROR_FRAME,
    STS3032_ERROR_BUFFER
} Sts3032Error;

/*
 * STS/SMS 运行模式。
 * 这些数值直接写入协议表中的“工作模式”寄存器 77(0x4D)：
 * 0 位置伺服；1 恒速旋转；2 PWM；3 步进。本工程当前主要使用位置与恒速模式。
 */
typedef enum
{
    STS3032_MODE_POSITION = 0,
    STS3032_MODE_WHEEL = 1,
    STS3032_MODE_PWM = 2,
    STS3032_MODE_STEP = 3
} Sts3032OperationMode;

/*
 * 多 ID 同步位置目标。
 * id       : 舵机 ID，范围 0~253，不能使用广播 ID。
 * position : 目标位置，范围 0~4095，2048 约为中位。
 * time     : 协议表中位置控制的运行时间字段，简单位置模式通常填 0。
 * speed    : 目标速度，范围 0~3400。
 */
typedef struct
{
    uint8_t id;
    uint16_t position;
    uint16_t time;
    uint16_t speed;
} Sts3032PositionTarget;

/*
 * 多 ID 同步恒速目标。
 * speed : 有符号速度；0 表示停止，正数按协议表的“顺时针转”，负数会置 bit15
 *         作为“逆时针转”。非 0 速度的绝对值范围为 50~3400。
 */
typedef struct
{
    uint8_t id;
    int16_t speed;
} Sts3032WheelTarget;

/*
 * 初始化 STS3032 串口总线驱动的软件状态。
 *
 * 参数：
 * - baud_rate：期望通信波特率；当前硬件默认 1000000。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后调用，UART6/PB21/PB22 配置由 SysConfig 完成。
 *
 * 返回值：
 * - true：UART6 已处于可用状态，驱动内部状态初始化完成。
 * - false：UART6 未使能或波特率配置不符合预期。
 */
bool sts3032_init(uint32_t baud_rate);

/*
 * 向指定 ID 舵机发送 PING 指令。
 *
 * 参数：
 * - id：目标舵机 ID，范围 0~253。
 * - timeout_ms：等待状态帧的超时时间，单位 ms。
 *
 * 返回值：
 * - true：收到目标 ID 返回的状态帧且校验正确。
 * - false：超时、校验错误、设备状态错误或参数非法。
 */
bool sts3032_ping(uint8_t id, uint32_t timeout_ms);

/*
 * 向舵机连续写入寄存器数据。
 *
 * 参数：
 * - id：目标舵机 ID，可为广播 ID 254；广播写不会等待状态帧。
 * - start_addr：起始寄存器地址。
 * - data：待写入数据缓冲区。
 * - length：写入字节数。
 * - timeout_ms：非广播写等待状态帧的超时时间。
 *
 * 返回值：
 * - true：写命令发送成功，并在需要时收到正常状态帧。
 * - false：参数非法、发送失败、超时或设备返回错误状态。
 */
bool sts3032_write_register(uint8_t id,
                            uint8_t start_addr,
                            const uint8_t *data,
                            uint8_t length,
                            uint32_t timeout_ms);

/*
 * 从舵机连续读取寄存器数据。
 *
 * 参数：
 * - id：目标舵机 ID，不能使用广播 ID。
 * - start_addr：起始寄存器地址。
 * - length：期望读取字节数。
 * - buffer：输出缓冲区。
 * - buffer_size：输出缓冲区容量，必须不小于 length。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：读取成功并拷贝到 buffer。
 * - false：参数非法、超时、校验错误或设备返回错误状态。
 */
bool sts3032_read_register(uint8_t id,
                           uint8_t start_addr,
                           uint8_t length,
                           uint8_t *buffer,
                           uint8_t buffer_size,
                           uint32_t timeout_ms);

/*
 * 设置单个舵机扭矩使能。
 *
 * 参数：
 * - id：目标舵机 ID，可为广播 ID。
 * - enable：true 打开扭矩，false 关闭扭矩。
 * - timeout_ms：非广播写等待状态帧的超时时间。
 *
 * 返回值：
 * - true：命令执行成功。
 * - false：写寄存器失败或设备返回错误状态。
 */
bool sts3032_set_torque(uint8_t id, bool enable, uint32_t timeout_ms);

/*
 * 使用同步写批量设置多个舵机扭矩使能。
 *
 * 参数：
 * - ids：目标舵机 ID 数组，不能包含广播 ID。
 * - count：目标数量，范围 1~STS3032_SYNC_MAX_TARGETS。
 * - enable：true 打开扭矩，false 关闭扭矩。
 *
 * 返回值：
 * - true：同步写帧成功发送。
 * - false：参数非法或发送失败。
 *
 * 说明：
 * - SYNC_WRITE 按协议不返回状态帧。
 */
bool sts3032_sync_write_torque(const uint8_t *ids, uint8_t count, bool enable);

/*
 * 设置单个舵机工作模式。
 *
 * 参数：
 * - id：目标舵机 ID。
 * - mode：位置、恒速、PWM 或步进模式。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：模式写入成功。
 * - false：参数非法、超时或设备返回错误状态。
 */
bool sts3032_set_operation_mode(uint8_t id,
                                Sts3032OperationMode mode,
                                uint32_t timeout_ms);

/*
 * 使用同步写批量设置多个舵机工作模式。
 *
 * 参数：
 * - ids：目标舵机 ID 数组。
 * - count：目标数量，范围 1~STS3032_SYNC_MAX_TARGETS。
 * - mode：要写入的工作模式。
 *
 * 返回值：
 * - true：同步写帧发送成功。
 * - false：参数非法或发送失败。
 */
bool sts3032_sync_write_operation_mode(const uint8_t *ids,
                                       uint8_t count,
                                       Sts3032OperationMode mode);

/*
 * 发送位置模式控制命令。
 *
 * 参数：
 * - id：目标舵机 ID，可为广播 ID。
 * - position：目标位置，范围 0~4095。
 * - time：运行时间字段，简单速度控制通常填 0。
 * - speed：目标速度，范围 0~3400。
 * - timeout_ms：非广播写等待状态帧的超时时间。
 *
 * 返回值：
 * - true：目标位置写入成功。
 * - false：参数非法、超时或设备返回错误状态。
 */
bool sts3032_set_position(uint8_t id,
                          uint16_t position,
                          uint16_t time,
                          uint16_t speed,
                          uint32_t timeout_ms);

/*
 * 发送带加速度字段的位置模式控制命令。
 *
 * 参数：
 * - id：目标舵机 ID。
 * - acceleration：目标加速度字段，对应协议表 GOAL_ACC。
 * - position：目标位置，范围 0~4095。
 * - time：运行时间字段。
 * - speed：目标速度，范围 0~3400。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：命令写入成功。
 * - false：参数非法、超时或设备返回错误状态。
 */
bool sts3032_set_position_with_acc(uint8_t id,
                                   uint8_t acceleration,
                                   uint16_t position,
                                   uint16_t time,
                                   uint16_t speed,
                                   uint32_t timeout_ms);

/*
 * 使用同步写批量发送多个舵机的位置目标。
 *
 * 参数：
 * - targets：位置目标数组，每个元素包含 id、position、time、speed。
 * - count：目标数量，范围 1~STS3032_SYNC_MAX_TARGETS。
 *
 * 返回值：
 * - true：同步写位置帧发送成功。
 * - false：参数非法或发送失败。
 */
bool sts3032_sync_write_positions(const Sts3032PositionTarget *targets,
                                  uint8_t count);

/*
 * 设置单个舵机恒速旋转速度。
 *
 * 参数：
 * - id：目标舵机 ID。
 * - speed：有符号速度；正负表示方向，0 停止，非 0 绝对值范围 50~3400。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：速度写入成功。
 * - false：参数非法、超时或设备返回错误状态。
 */
bool sts3032_set_wheel_speed(uint8_t id, int16_t speed, uint32_t timeout_ms);

/*
 * 使用同步写批量设置多个舵机恒速速度。
 *
 * 参数：
 * - targets：恒速目标数组，每个元素包含 id 和有符号 speed。
 * - count：目标数量，范围 1~STS3032_SYNC_MAX_TARGETS。
 *
 * 返回值：
 * - true：同步写恒速帧发送成功。
 * - false：参数非法或发送失败。
 */
bool sts3032_sync_write_wheel_speeds(const Sts3032WheelTarget *targets,
                                     uint8_t count);

/*
 * 读取单个舵机当前工作模式。
 *
 * 参数：
 * - id：目标舵机 ID。
 * - mode：输出工作模式指针，不可为 NULL。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：读取成功。
 * - false：参数非法、超时、校验错误或返回值不是有效模式。
 */
bool sts3032_read_operation_mode(uint8_t id,
                                 Sts3032OperationMode *mode,
                                 uint32_t timeout_ms);

/*
 * 读取单个舵机当前恒速速度寄存器。
 *
 * 参数：
 * - id：目标舵机 ID。
 * - speed：输出有符号速度指针；负数表示反向。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：读取成功。
 * - false：参数非法、超时或校验失败。
 */
bool sts3032_read_wheel_speed(uint8_t id, int16_t *speed, uint32_t timeout_ms);

/*
 * 读取单个舵机当前位置。
 *
 * 参数：
 * - id：目标舵机 ID。
 * - position：输出当前位置指针，范围 0~4095。
 * - timeout_ms：等待状态帧的超时时间。
 *
 * 返回值：
 * - true：读取成功。
 * - false：参数非法、超时或校验失败。
 */
bool sts3032_read_position(uint8_t id, uint16_t *position, uint32_t timeout_ms);

/*
 * 获取最近一次驱动错误码。
 *
 * 返回值：
 * - Sts3032Error 枚举值，可配合 sts3032_get_error_string() 转为字符串。
 */
Sts3032Error sts3032_get_last_error(void);

/*
 * 获取最近一次舵机状态帧中的设备状态字节。
 *
 * 返回值：
 * - 设备返回的 status/error 字段；0 表示设备未报告错误。
 */
uint8_t sts3032_get_last_device_status(void);

/*
 * 获取累计发送协议帧数量。
 *
 * 返回值：
 * - 自 sts3032_init() 后成功写入 UART6 的指令帧数量。
 */
uint32_t sts3032_get_tx_packet_count(void);

/*
 * 获取累计接收并校验通过的状态帧数量。
 *
 * 返回值：
 * - 自 sts3032_init() 后成功解析的状态帧数量。
 */
uint32_t sts3032_get_rx_packet_count(void);

/*
 * 获取 UART6 接收环形缓冲溢出计数。
 *
 * 返回值：
 * - ISR 接收字节时因软件环形缓冲满而丢弃的次数。
 */
uint32_t sts3032_get_rx_overrun_count(void);

/*
 * 获取驱动记录的 UART6 目标波特率。
 *
 * 返回值：
 * - sts3032_init() 传入并通过检查的波特率，单位 baud。
 */
uint32_t sts3032_get_configured_baud_rate(void);

/*
 * 查询 UART6 是否处于使能状态。
 *
 * 返回值：
 * - true：UART6 外设被使能。
 * - false：UART6 未使能，通常表示 SysConfig 初始化未完成。
 */
bool sts3032_is_uart_enabled(void);

/*
 * 获取 UART6 当前整数分频寄存器值。
 *
 * 返回值：
 * - UART IBRD 寄存器值，用于调试确认 1000000 baud 配置。
 */
uint32_t sts3032_get_uart_integer_divisor(void);

/*
 * 获取 UART6 当前小数分频寄存器值。
 *
 * 返回值：
 * - UART FBRD 寄存器值，用于调试确认 1000000 baud 配置。
 */
uint32_t sts3032_get_uart_fractional_divisor(void);

/*
 * 将 STS3032 驱动错误码转换为可打印字符串。
 *
 * 参数：
 * - error：错误码。
 *
 * 返回值：
 * - 指向静态字符串的指针，供 UART0/LCD 状态显示使用。
 */
const char *sts3032_get_error_string(Sts3032Error error);

#ifdef __cplusplus
}
#endif

#endif
