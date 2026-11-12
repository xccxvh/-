/*
 * 安信可 WB2-01S WiFi 模块简易 AT 驱动（轮询方式）。
 *
 * 职责：
 * 1. 通过 UART4（PB10/PB11）与模块交换 AT 指令，验证 MCU 能否控制/通信。
 * 2. 提供上电自检，将原始应答打印到调试串口 UART0。
 *
 * 时序约束：
 * - WB2 系列模块上电后需数百毫秒完成启动，自检前必须预留启动时间。
 * - AT 指令以 \\r\\n 结尾；应答通常含 "OK" 或 "ERROR"。
 */

#include "wb2_wifi.h"
#include "SystemTime/system_time.h"

/** 模块上电至可接受 AT 的保守等待时间（ms） */
#define WB2_WIFI_BOOT_DELAY_MS        (800U)

/** 单次 AT 等待应答总超时（ms） */
#define WB2_WIFI_AT_TIMEOUT_MS        (2000U)

/** 连续无新字节则认为一帧应答结束（ms） */
#define WB2_WIFI_RX_IDLE_GAP_MS       (50U)

/** 自检读缓冲上限 */
#define WB2_WIFI_RX_BUF_SIZE          (384U)

/** 判断字符串是否以 suffix 结尾（用于识别是否已有 \\r\\n） */
static bool wb2_wifi_ends_with_crlf(const char *s)
{
    size_t len;

    if (s == NULL) {
        return false;
    }

    len = strlen(s);
    if (len < 2U) {
        return false;
    }

    return (s[len - 2U] == '\r' && s[len - 1U] == '\n');
}

void wb2_wifi_flush_rx(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_WF_INST)) {
        (void)DL_UART_receiveData(UART_WF_INST);
    }
}

void wb2_wifi_init(void)
{
    /*
     * 引脚与波特率已在 SYSCFG_DL_init() -> SYSCFG_DL_UART_WF_init() 中配置。
     * 此处仅等待模块就绪并清空可能存在的上电杂散字节。
     */
    delay_ms(WB2_WIFI_BOOT_DELAY_MS);
    wb2_wifi_flush_rx();
}

bool wb2_wifi_send_cmd(const char *cmd)
{
    const char *p;
    char line[64];
    size_t len;
    size_t i;

    if (cmd == NULL) {
        return false;
    }

    if (wb2_wifi_ends_with_crlf(cmd)) {
        p = cmd;
    } else {
        len = strlen(cmd);
        if (len >= (sizeof(line) - 3U)) {
            return false;
        }
        memcpy(line, cmd, len);
        line[len] = '\r';
        line[len + 1U] = '\n';
        line[len + 2U] = '\0';
        p = line;
    }

    for (i = 0U; p[i] != '\0'; i++) {
        DL_UART_Main_transmitDataBlocking(UART_WF_INST, (uint8_t)p[i]);
    }

    return true;
}

uint32_t wb2_wifi_read(uint8_t *buf, uint32_t buf_len, uint32_t timeout_ms)
{
    uint32_t count = 0U;
    uint32_t start_ms;
    uint32_t last_rx_ms;

    if ((buf == NULL) || (buf_len < 2U)) {
        return 0U;
    }

    start_ms = system_time_get_tick_ms();
    last_rx_ms = start_ms;

    while (count < (buf_len - 1U)) {
        if (!DL_UART_isRXFIFOEmpty(UART_WF_INST)) {
            buf[count++] = DL_UART_receiveData(UART_WF_INST);
            last_rx_ms = system_time_get_tick_ms();
            continue;
        }

        /* 已收到数据且线路空闲一段时间，认为模块应答结束 */
        if ((count > 0U) &&
            system_time_elapsed_ms(&last_rx_ms, WB2_WIFI_RX_IDLE_GAP_MS)) {
            break;
        }

        if (system_time_elapsed_ms(&start_ms, timeout_ms)) {
            break;
        }
    }

    buf[count] = '\0';
    return count;
}

static wb2_wifi_result_t wb2_wifi_wait_ok(const char *cmd_label, const char *cmd)
{
    static uint8_t rx_buf[WB2_WIFI_RX_BUF_SIZE];
    uint32_t n;
    const char *ok_ptr;

    wb2_wifi_flush_rx();

    if (!wb2_wifi_send_cmd(cmd)) {
        printf("[WB2] %s send fail\r\n", cmd_label);
        return WB2_WIFI_RESULT_ERROR;
    }

    n = wb2_wifi_read(rx_buf, sizeof(rx_buf), WB2_WIFI_AT_TIMEOUT_MS);

    if (n == 0U) {
        printf("[WB2] %s: no response (check PB10/PB11, baud %u)\r\n",
               cmd_label, (unsigned)UART_WF_BAUD_RATE);
        return WB2_WIFI_RESULT_NO_RESPONSE;
    }

    printf("[WB2] %s rx (%lu B):\r\n", cmd_label, (unsigned long)n);
    printf("%s\r\n", (const char *)rx_buf);

    ok_ptr = strstr((const char *)rx_buf, "OK");
    if (ok_ptr == NULL) {
        return WB2_WIFI_RESULT_PARTIAL;
    }

    return WB2_WIFI_RESULT_OK;
}

wb2_wifi_result_t wb2_wifi_self_test(void)
{
    wb2_wifi_result_t r;

    printf("[WB2] self test start, UART4 %u 8N1\r\n",
           (unsigned)UART_WF_BAUD_RATE);

    r = wb2_wifi_wait_ok("AT", "AT");
    if (r != WB2_WIFI_RESULT_OK) {
        printf("[WB2] FAIL: basic AT\r\n");
        return r;
    }

    r = wb2_wifi_wait_ok("AT+GMR", "AT+GMR");
    if (r == WB2_WIFI_RESULT_OK) {
        printf("[WB2] PASS: module responds OK\r\n");
    } else if (r == WB2_WIFI_RESULT_PARTIAL) {
        printf("[WB2] WARN: AT OK but GMR no OK (may still work)\r\n");
    } else {
        printf("[WB2] FAIL: AT+GMR\r\n");
    }

    return r;
}
