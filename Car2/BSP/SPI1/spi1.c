#include "SPI1/spi1.h"

/*
 * SPI1 单字节全双工传输。
 *
 * 运行约束：
 * - SPI1 当前由 SysConfig 配置为控制器模式、8bit、Mode 3、2MHz。
 * - LSM6DSV 读取寄存器时，发送地址字节也会同步产生一个无效 RX 字节；随后每发送一个
 *   dummy 字节才能读回一个有效数据字节。
 * - 发送前需要确认 TX FIFO 未满，接收前需要确认 RX FIFO 非空；只等待 BUSY 清零会在
 *   个别状态下读到旧 FIFO 数据，导致 WHO_AM_I 判断失败。
 *
 * 故障保护：
 * - 使用有限循环等待，避免 SPI 外设异常时卡死在初始化阶段。
 * - 兼容旧接口 spi1_read_write_byte()，它在失败时返回 0xFF；新驱动优先使用
 *   spi1_transfer_byte() 获取明确的成功/失败状态。
 */

#define SPI1_TRANSFER_TIMEOUT_LOOP (100000U)

static bool spi1_wait_tx_ready(void)
{
    uint32_t timeout = SPI1_TRANSFER_TIMEOUT_LOOP;

    while (DL_SPI_isTXFIFOFull(SPI_1_INST)) {
        if (timeout == 0U) {
            return false;
        }
        timeout--;
    }

    return true;
}

static bool spi1_wait_rx_ready(void)
{
    uint32_t timeout = SPI1_TRANSFER_TIMEOUT_LOOP;

    while (DL_SPI_isRXFIFOEmpty(SPI_1_INST)) {
        if (timeout == 0U) {
            return false;
        }
        timeout--;
    }

    return true;
}

static bool spi1_wait_idle(void)
{
    uint32_t timeout = SPI1_TRANSFER_TIMEOUT_LOOP;

    while (DL_SPI_isBusy(SPI_1_INST)) {
        if (timeout == 0U) {
            return false;
        }
        timeout--;
    }

    return true;
}

/*
 * 清空 RX FIFO 中可能残留的旧字节。
 *
 * 实现原因：
 * - 本工程使用阻塞式单字节 API，每次调用理论上只会产生并消费 1 个 RX 字节。
 * - 如果上一次传输在调试中断点、片选异常或外设复位附近被打断，RX FIFO 可能残留旧数据；
 *   进入下一次寄存器事务前先清掉，可以避免 WHO_AM_I 读到前一次的无效字节。
 */
static void spi1_flush_rx_fifo(void)
{
    uint32_t guard = 16U;

    while (!DL_SPI_isRXFIFOEmpty(SPI_1_INST) && (guard > 0U)) {
        (void)DL_SPI_receiveData8(SPI_1_INST);
        guard--;
    }
}

/*
 * 带状态返回的 SPI1 单字节传输。
 *
 * 输入：
 * - tx_data：需要写入 MOSI/PICO 的字节。
 * - rx_data：接收 MISO/POCI 字节的指针；允许为 NULL，表示调用方只关心写入时钟。
 *
 * 输出：
 * - true：TX/RX 均在超时时间内完成，rx_data 已更新。
 * - false：TX FIFO、RX FIFO 或 BUSY 状态等待超时，调用方应终止当前片选事务。
 */
bool spi1_transfer_byte(uint8_t tx_data, uint8_t *rx_data)
{
    uint8_t dummy_rx;

    if (rx_data == NULL) {
        rx_data = &dummy_rx;
    }

    if (!spi1_wait_idle()) {
        return false;
    }

    spi1_flush_rx_fifo();

    if (!spi1_wait_tx_ready()) {
        return false;
    }

    DL_SPI_transmitData8(SPI_1_INST, tx_data);

    if (!spi1_wait_rx_ready()) {
        return false;
    }

    *rx_data = DL_SPI_receiveData8(SPI_1_INST);

    return spi1_wait_idle();
}

/*
 * 旧代码兼容接口。
 *
 * 说明：
 * - W25Q64 等旧模块仍使用返回字节的简单接口。
 * - 运行异常时返回 0xFF；需要判断失败原因的新代码应使用 spi1_transfer_byte()。
 */
uint8_t spi1_read_write_byte(uint8_t dat)
{
    uint8_t data = 0xFFU;

    (void)spi1_transfer_byte(dat, &data);

    return data;
}
