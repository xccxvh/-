#ifndef WB2_WIFI_H
#define WB2_WIFI_H

/*
 * 安信可 WB2-01S WiFi 模块（AT 指令）简易驱动头文件。
 *
 * 硬件连接（SysConfig UART_WF / UART4）：
 * - MCU PB10 (TX) → 模块 RX
 * - MCU PB11 (RX) ← 模块 TX
 * - 8N1，默认 115200（与模块出厂 AT 波特率一致）
 *
 * 说明：
 * - 本驱动采用轮询收发，不占用 UART4 中断，避免与灰度传感器等同口 ISR 冲突。
 * - 调试结果通过 UART0 printf 输出，请在 PC 串口助手查看。
 */

#include "bsp_common.h"

/** 自检 / AT 交互结果 */
typedef enum {
    WB2_WIFI_RESULT_OK = 0,          /**< 收到预期应答（含 OK） */
    WB2_WIFI_RESULT_NO_RESPONSE,     /**< 超时无任何字节 */
    WB2_WIFI_RESULT_PARTIAL,         /**< 有数据但未匹配 OK */
    WB2_WIFI_RESULT_ERROR            /**< 参数或内部状态错误 */
} wb2_wifi_result_t;

/*
 * 初始化 WB2 WiFi 模块串口侧软件状态。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 后调用，UART4/PB10/PB11 的硬件配置由 SysConfig 完成。
 *
 * 时序约束：
 * - WB2 模块上电后需要时间启动固件，本函数内部会阻塞等待约 800ms 并清空 RX FIFO。
 */
void wb2_wifi_init(void);

/*
 * 清空 UART4 接收 FIFO。
 *
 * 用途：
 * - 发送新的 AT 命令前丢弃上电杂散字节或上一条命令残留，避免误判应答内容。
 */
void wb2_wifi_flush_rx(void);

/*
 * 发送一条 AT 命令。
 *
 * 参数：
 * - cmd：以 '\0' 结尾的 AT 字符串，例如 "AT" 或 "AT+GMR"。
 *
 * 返回值：
 * - true：命令已通过 UART4 阻塞发送完成。
 * - false：参数为空或命令长度超过内部临时缓冲。
 *
 * 说明：
 * - 若 cmd 末尾没有 "\r\n"，函数会自动追加 AT 协议换行。
 */
bool wb2_wifi_send_cmd(const char *cmd);

/*
 * 在超时时间内轮询读取 WB2 应答。
 *
 * 参数：
 * - buf：输出缓冲区。
 * - buf_len：缓冲区容量，包含结尾 '\0'。
 * - timeout_ms：最长等待时间，单位 ms。
 *
 * 返回值：
 * - 实际写入 buf 的字节数，不包含结尾 '\0'。
 *
 * 接收规则：
 * - 收到至少 1 字节后，若线路空闲超过内部 idle gap，则认为一帧应答结束。
 */
uint32_t wb2_wifi_read(uint8_t *buf, uint32_t buf_len, uint32_t timeout_ms);

/*
 * 执行 WB2 AT 自检。
 *
 * 行为：
 * - 依次发送 "AT" 和 "AT+GMR"，并把原始应答打印到 UART0。
 *
 * 返回值：
 * - WB2_WIFI_RESULT_OK：收到包含 OK 的预期应答。
 * - 其它值：无响应、部分响应或发送参数错误。
 */
wb2_wifi_result_t wb2_wifi_self_test(void);

#endif /* WB2_WIFI_H */
