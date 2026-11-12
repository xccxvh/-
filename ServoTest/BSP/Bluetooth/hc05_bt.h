#ifndef HC05_BT_H
#define HC05_BT_H

/*
 * HC-05 经典蓝牙模块（AT 指令）简易驱动头文件。
 *
 * 硬件连接（SysConfig UART_BL / UART3）：
 * - MCU PB12 (TX) → HC-05 RXD
 * - MCU PB13 (RX) ← HC-05 TXD
 * - 8N1，出厂 AT 模式默认 9600
 *
 * 使用约束：
 * - 模块 KEY/EN 脚在**上电或复位时拉高**才进入 AT 命令模式；仅接 VCC/GND/TX/RX
 *   时可能处于透传模式，导致 AT 无应答。
 * - 本驱动采用轮询收发，不占用 UART3 中断；自检结果通过 UART0 printf 输出。
 */

#include "bsp_common.h"

/** HC-05 AT 交互结果 */
typedef enum {
    HC05_BT_RESULT_OK = 0,           /**< 应答中包含 OK */
    HC05_BT_RESULT_NO_RESPONSE,      /**< 超时无字节 */
    HC05_BT_RESULT_PARTIAL,          /**< 有数据但未匹配 OK */
    HC05_BT_RESULT_ERROR             /**< 参数错误或发送失败 */
} hc05_bt_result_t;

/*
 * 初始化 HC-05 蓝牙模块串口侧软件状态。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 后调用，UART3/PB12/PB13 的硬件配置由 SysConfig 完成。
 *
 * 时序约束：
 * - HC-05 上电后需要稳定时间，本函数内部会阻塞等待约 500ms 并清空 RX FIFO。
 */
void hc05_bt_init(void);

/*
 * 清空 UART3 接收 FIFO。
 *
 * 用途：
 * - 发送新的 AT 命令前丢弃透传数据、上电杂散字节或上一条命令残留。
 */
void hc05_bt_flush_rx(void);

/*
 * 发送一条 HC-05 AT 命令。
 *
 * 参数：
 * - cmd：以 '\0' 结尾的 AT 字符串，例如 "AT" 或 "AT+NAME?"。
 *
 * 返回值：
 * - true：命令已通过 UART3 阻塞发送完成。
 * - false：参数为空或命令长度超过内部临时缓冲。
 *
 * 说明：
 * - 若 cmd 末尾没有 "\r\n"，函数会自动追加 AT 协议换行。
 */
bool hc05_bt_send_cmd(const char *cmd);

/*
 * 在超时时间内轮询读取 HC-05 应答。
 *
 * 参数：
 * - buf：输出缓冲区。
 * - buf_len：缓冲区容量，包含结尾 '\0'。
 * - timeout_ms：最长等待时间，单位 ms。
 *
 * 返回值：
 * - 实际写入字节数，不包含结尾 '\0'。
 *
 * 接收规则：
 * - 收到至少 1 字节后，若线路空闲超过内部 idle gap，则认为一帧应答结束。
 */
uint32_t hc05_bt_read(uint8_t *buf, uint32_t buf_len, uint32_t timeout_ms);

/*
 * 执行 HC-05 AT 自检。
 *
 * 行为：
 * - 依次发送 "AT"、"AT+VERSION?"、"AT+NAME?"，并把原始应答打印到 UART0。
 *
 * 返回值：
 * - HC05_BT_RESULT_OK：收到包含 OK 的预期应答。
 * - 其它值：无响应、部分响应或发送参数错误。
 */
hc05_bt_result_t hc05_bt_self_test(void);

#endif /* HC05_BT_H */
