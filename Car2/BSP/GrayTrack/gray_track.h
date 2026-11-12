#ifndef GRAY_TRACK_H
#define GRAY_TRACK_H

/*
 * 8 路灰度循迹模块 I2C 主机接口。
 *
 * 硬件连接：
 * - MCU PA0 -> SDA，MCU PA1 -> SCL，对接外部 8 路灰度模块。
 * - 灰度模块 7 位 I2C 地址固定为 0x58（8 位写地址 0xB0）。
 *
 * 实现说明：
 * - 与已验证的灰度通信例程一致，使用软件 I2C 位操作，而非 I2C0 硬件控制器。
 * - gray_track_init() 会关闭 I2C0 控制器并将 PA0/PA1 切为 GPIO 驱动。
 *
 * 协议摘要：
 * - 读寄存器：先写 1 字节寄存器地址，再重复起始读取数据。
 * - 0x00~0x04：协议版本、设备 ID、心跳、采样计数、灰度比较位图。
 * - 0x20~0x2F：8 路灰度 ADC 原始值，小端排列。
 *
 * 约束：
 * - 本文件属于 BSP 模块头文件，只包含 BSP/bsp_common.h，不包含聚合头 bsp.h。
 * - 调用 gray_track_init() 前必须先调用 SYSCFG_DL_init() 完成系统时钟初始化。
 */
#include "bsp_common.h"

#define GRAY_TRACK_CHANNEL_COUNT       (8U)
#define GRAY_TRACK_I2C_ADDRESS_7BIT    (0x58U)
#define GRAY_TRACK_I2C_ADDR_8BIT       (0xB0U)
#define GRAY_TRACK_PROTO_VERSION       (0x07U)
#define GRAY_TRACK_DEVICE_ID           (0x42U)
#define GRAY_TRACK_LED_THRESHOLD_MAX   (4095U)
#define GRAY_TRACK_DEFAULT_LED_THRESHOLD (180U)

/* 0x13/0x14：LED 比较阈值，16 位小端，写入后模块 compare_bits 按此阈值更新。 */
#define GRAY_TRACK_REG_LED_THRESHOLD_L (0x13U)
#define GRAY_TRACK_REG_LED_THRESHOLD_H (0x14U)

typedef enum {
    GRAY_TRACK_RESULT_OK = 0,
    GRAY_TRACK_RESULT_PARAM_ERROR,
    GRAY_TRACK_RESULT_TIMEOUT,
    GRAY_TRACK_RESULT_BUS_ERROR,
    GRAY_TRACK_RESULT_ID_MISMATCH
} gray_track_result_t;

typedef struct {
    uint8_t proto_version;                         /* 0x00：协议版本，当前期望 0x07。 */
    uint8_t device_id;                             /* 0x01：设备 ID，当前期望 0x42。 */
    uint8_t heartbeat;                             /* 0x02：模块 100ms 心跳计数。 */
    uint8_t sample_counter;                        /* 0x03：灰度采样递增计数。 */
    uint8_t compare_bits;                          /* 0x04：bit0~bit7 对应灰度 1~8。 */
    uint16_t adc[GRAY_TRACK_CHANNEL_COUNT];        /* 0x20~0x2F：8 路小端 ADC 原始值。 */
    bool online;                                   /* 最近一次快照是否通信成功且 ID 匹配。 */
    gray_track_result_t result;                    /* 最近一次读取结果，便于 LCD/UART 显示。 */
} gray_track_snapshot_t;

/*
 * 初始化灰度模块驱动状态。
 *
 * 行为：
 * - 关闭 I2C0 硬件控制器，初始化软件 I2C 总线。
 * - 清除软件错误计数和最近一次快照。
 * - 不修改模块阈值；阈值由应用层调用 gray_track_set_led_threshold() 写入。
 */
void gray_track_init(void);

/*
 * 将 LED 比较阈值写入灰度模块 0x13/0x14。
 *
 * 参数：
 * - threshold：0~4095；超过 4095 会被钳位。
 *
 * 返回值：
 * - true：I2C 写入成功。
 * - false：参数错误或总线 NACK。
 */
bool gray_track_set_led_threshold(uint16_t threshold);

/*
 * 回读灰度模块当前 LED 比较阈值。
 */
bool gray_track_get_led_threshold(uint16_t *threshold);

/*
 * 快速读取 0x00~0x04 状态区（单次 I2C 读事务）。
 *
 * 用途：
 * - 10 ms 循迹控制周期内只读 compare_bits 与心跳，不读 0x20 ADC 块，缩短阻塞时间。
 *
 * 返回值：
 * - true：通信成功且协议版本/设备 ID 匹配；snapshot->adc 内容不会被更新。
 */
bool gray_track_read_status(gray_track_snapshot_t *snapshot);

/*
 * 读取灰度模块完整快照。
 *
 * 参数：
 * - snapshot：输出快照指针，不能为 NULL。
 *
 * 返回值：
 * - true：通信成功，且协议版本/设备 ID 与文档匹配。
 * - false：参数错误、I2C 超时/NACK/总线错误，或模块 ID 不匹配。
 *
 * 时序约束：
 * - 内部使用阻塞式 I2C 轮询，主循环周期调用即可，不应在中断服务函数中调用。
 */
bool gray_track_read_snapshot(gray_track_snapshot_t *snapshot);

/*
 * 仅读取灰度比较位图。
 *
 * 参数：
 * - compare_bits：输出 bit0~bit7 的比较结果。
 *
 * 用途：
 * - 控制周期很短且只需要阈值结果时可调用；当前循迹应用使用完整快照显示原始值。
 */
bool gray_track_read_compare_bits(uint8_t *compare_bits);

/*
 * 读取 8 路灰度 ADC 原始值。
 *
 * 参数：
 * - adc：长度至少为 8 的输出数组。
 *
 * 返回值：
 * - true：成功读到 16 字节并完成小端转换。
 * - false：I2C 通信失败或参数无效。
 */
bool gray_track_read_adc(uint16_t adc[GRAY_TRACK_CHANNEL_COUNT]);

/*
 * 读取最近一次驱动层错误计数。
 *
 * 说明：
 * - 只统计 I2C/ID 级失败次数，便于判断线缆接触、地址错误或模块掉电。
 */
uint32_t gray_track_get_error_count(void);

/*
 * 读取最近一次操作结果。
 *
 * 返回值：
 * - GRAY_TRACK_RESULT_OK 表示最近一次操作成功。
 * - 其它枚举值用于 LCD/UART 简短诊断。
 */
gray_track_result_t gray_track_get_last_result(void);

/*
 * 把结果码转换成短 ASCII 文本。
 *
 * 用途：
 * - LCD 字库当前主要显示 ASCII，应用层可直接展示该短文本。
 */
const char *gray_track_result_text(gray_track_result_t result);

#endif
