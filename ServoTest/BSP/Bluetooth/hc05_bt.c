/*
 * HC-05 蓝牙模块简易 AT 驱动（轮询方式，UART3 / PB12-PB13）。
 *
 * 职责：
 * 1. 通过 UART_BL 与 HC-05 交换 AT 指令，验证 MCU 与模块串口链路。
 * 2. 上电自检将原始应答打印到 UART0，便于与 USB 串口助手对照。
 *
 * 时序约束：
 * - HC-05 出厂 AT 波特率为 9600；若模块曾被 AT+UART 改为其它速率，需同步修改 SysConfig。
 * - AT 应答以 OK/ERROR 结尾；接收采用“线路空闲”判定帧结束，避免固定 2s 阻塞。
 */

#include "hc05_bt.h"
#include "SystemTime/system_time.h"

/** HC-05 上电至可接受 AT 的保守等待（ms） */
#define HC05_BT_BOOT_DELAY_MS         (500U)

/** 单次 AT 总超时（ms） */
#define HC05_BT_AT_TIMEOUT_MS         (2000U)

/** 连续无新字节认为应答结束（ms） */
#define HC05_BT_RX_IDLE_GAP_MS        (80U)

/** 自检接收缓冲上限 */
#define HC05_BT_RX_BUF_SIZE           (256U)

static bool hc05_bt_ends_with_crlf(const char *s)
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

void hc05_bt_flush_rx(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(UART_BL_INST) == false) {
        (void)DL_UART_Main_receiveData(UART_BL_INST);
    }
}

void hc05_bt_init(void)
{
    /*
     * 引脚与 9600 波特率已在 SYSCFG_DL_UART_BL_init() 中配置。
     * 此处等待模块就绪并丢弃上电杂散字节。
     */
    delay_ms(HC05_BT_BOOT_DELAY_MS);
    hc05_bt_flush_rx();
}

bool hc05_bt_send_cmd(const char *cmd)
{
    const char *p;
    char line[64];
    size_t len;
    size_t i;

    if (cmd == NULL) {
        return false;
    }

    if (hc05_bt_ends_with_crlf(cmd)) {
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
        DL_UART_Main_transmitDataBlocking(UART_BL_INST, (uint8_t)p[i]);
    }

    return true;
}

uint32_t hc05_bt_read(uint8_t *buf, uint32_t buf_len, uint32_t timeout_ms)
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
        if (DL_UART_Main_isRXFIFOEmpty(UART_BL_INST) == false) {
            buf[count++] = DL_UART_Main_receiveData(UART_BL_INST);
            last_rx_ms = system_time_get_tick_ms();
            continue;
        }

        if ((count > 0U) &&
            system_time_elapsed_ms(&last_rx_ms, HC05_BT_RX_IDLE_GAP_MS)) {
            break;
        }

        if (system_time_elapsed_ms(&start_ms, timeout_ms)) {
            break;
        }
    }

    buf[count] = '\0';
    return count;
}

static hc05_bt_result_t hc05_bt_wait_ok(const char *cmd_label, const char *cmd)
{
    static uint8_t rx_buf[HC05_BT_RX_BUF_SIZE];
    uint32_t n;

    hc05_bt_flush_rx();

    if (hc05_bt_send_cmd(cmd) == false) {
        printf("[HC05] %s send fail\r\n", cmd_label);
        return HC05_BT_RESULT_ERROR;
    }

    n = hc05_bt_read(rx_buf, sizeof(rx_buf), HC05_BT_AT_TIMEOUT_MS);

    if (n == 0U) {
        printf("[HC05] %s: no response (PB12/PB13, baud %u, KEY high?)\r\n",
               cmd_label, (unsigned)UART_BL_BAUD_RATE);
        return HC05_BT_RESULT_NO_RESPONSE;
    }

    printf("[HC05] %s rx (%lu B):\r\n", cmd_label, (unsigned long)n);
    printf("%s\r\n", (const char *)rx_buf);

    if (strstr((const char *)rx_buf, "OK") == NULL) {
        return HC05_BT_RESULT_PARTIAL;
    }

    return HC05_BT_RESULT_OK;
}

hc05_bt_result_t hc05_bt_self_test(void)
{
    hc05_bt_result_t r;

    printf("[HC05] self test, UART3 %u 8N1 PB12/PB13\r\n",
           (unsigned)UART_BL_BAUD_RATE);

    r = hc05_bt_wait_ok("AT", "AT");
    if (r != HC05_BT_RESULT_OK) {
        printf("[HC05] FAIL: basic AT\r\n");
        return r;
    }

    r = hc05_bt_wait_ok("AT+VERSION?", "AT+VERSION?");
    if (r != HC05_BT_RESULT_OK) {
        printf("[HC05] WARN: AT OK, version query no OK\r\n");
    }

    r = hc05_bt_wait_ok("AT+NAME?", "AT+NAME?");
    if (r == HC05_BT_RESULT_OK) {
        printf("[HC05] PASS: module responds OK\r\n");
    } else if (r == HC05_BT_RESULT_PARTIAL) {
        printf("[HC05] WARN: name query no OK (module may still work)\r\n");
    } else {
        printf("[HC05] FAIL: AT+NAME?\r\n");
    }

    return r;
}
