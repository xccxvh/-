#ifndef BSP_VISION_VISION_LINK_H
#define BSP_VISION_VISION_LINK_H

/*
 * Jetson Nano 视觉位置数据链路。
 *
 * 硬件连接（默认使用 Car2 空闲的 UART4）：
 * - G3519 PB11 = UART4_RX，接 Jetson 的 TX；
 * - G3519 PB10 = UART4_TX，接 Jetson 的 RX；
 * - 双方必须共地，电平 3.3 V（若 Jetson 侧为 5 V TTL，必须加电平转换）。
 * - 默认波特率 115200，8N1。
 *
 * 资源说明：
 * - UART4 在旧工程里分配给 WiFi 模块（BSP/WiFi/wb2_wifi.c）。
 * - 滚球控制工程不再同时使用 WiFi；若必须保留 WiFi，应改用其它 UART 并
 *   修改 VISION_LINK_INST 相关宏，二者不能共用一个串口。
 *
 * 通信约束：
 * - 位置帧约 30 Hz，单帧 21 字节，115200 下占用不到 1%；
 * - 接收完全中断 + 环形缓冲，非阻塞；
 * - 发送遥测使用软件环形缓冲 + TX 中断，主循环不会因为串口变慢而阻塞。
 *
 * 头文件规则：
 * - 只包含 BSP 公共基础头 bsp_common.h，不包含聚合头 bsp.h。
 */
#include "bsp_common.h"

/*
 * 帧格式（所有多字节字段均为小端）：
 *
 *   偏移  长度  内容
 *   0     1     同步字节 0
 *   1     1     同步字节 1
 *   2     1     协议版本，当前 0x01
 *   3     1     帧类型：0x01=视觉位置，0x81=G3519 遥测
 *   4     1     负载长度 len
 *   5     2     帧序号 seq（uint16）
 *   7     len   负载
 *   7+len 2     CRC16-CCITT-FALSE（初值 0xFFFF，多项式 0x1021）
 *              覆盖范围：版本、类型、长度、序号、负载（偏移 2 ~ 6+len）
 *
 * 视觉位置帧（类型 0x01，len=12）负载：
 *   0   4     Jetson 单调时钟帧时间戳，单位 ms（uint32）
 *   4   4     钢珠位置 x_cm（float32，必须已完成厘米标定）
 *   8   1     检测置信度 0~100（uint8）
 *   9   1     标志位：bit0=数据有效，bit1=厘米标定完成
 *   10  2     保留，必须填 0
 *
 * 遥测帧（类型 0x81，len=32）负载：
 *   0   4     G3519 本地毫秒时间戳
 *   4   4     目标位置 target_cm（float32）
 *   8   4     钢珠位置 ball_x_cm（float32）
 *   12  4     钢珠速度 ball_v_cm_s（float32）
 *   16  4     位置误差 position_error_cm（float32）
 *   20  4     摆杆倾角指令 beam_angle_cmd_deg（float32）
 *   24  2     舵机脉宽 servo_pwm_us（uint16）
 *   26  1     控制状态 control_state（见 ball_control_state_t）
 *   27  1     标志位：bit0=视觉有效，bit1=自动控制使能，bit2=故障
 *   28  4     经过斜率限制后的实际摆杆倾角 beam_actual_deg（float32）
 *
 * 说明：beam_actual_deg 可用于 PC/Jetson 做“半实物仿真”——
 * 接收遥测里的实际倾角，积分钢珠动力学，再把位置帧发回 G3519。
 *
 * 详细协议说明和 Jetson 端示例见 docs/BALL_CONTROL.md 与 tools/。
 */

#define VISION_LINK_SYNC0               (0xA5U)
#define VISION_LINK_SYNC1               (0x5AU)
#define VISION_LINK_VERSION             (0x01U)
#define VISION_LINK_TYPE_BALL           (0x01U)
#define VISION_LINK_TYPE_TELEMETRY      (0x81U)
#define VISION_LINK_BALL_PAYLOAD_LEN    (12U)
#define VISION_LINK_MAX_PAYLOAD_LEN     (32U)
#define VISION_LINK_DEFAULT_BAUD        (115200U)

/* 视觉有效期：超过该时间没有新的有效帧，就认为视觉丢失。 */
#define VISION_LINK_TIMEOUT_MS          (150U)

/* 连续解码错误达到该次数后置 link_fault，要求上层进入 FAULT 保护。 */
#define VISION_LINK_FAULT_ERROR_COUNT   (20U)

/*
 * 钢珠测量结果（与题目约定一致）。
 *
 * 字段说明：
 * - x_cm：Jetson 完成厘米标定后的钢珠位置，单位 cm；未标定时必须置
 *   valid=false，禁止用像素坐标冒充厘米位置。
 * - frame_time_ms：Jetson 单调时钟时间戳，仅用于排序和诊断；
 *   G3519 的速度计算使用本地接收时刻，绝不与本地绝对时钟混算。
 * - seq：帧序号，用于识别重复帧和丢帧。
 * - confidence：检测置信度 0~100。
 * - valid：本帧是否可用于控制。
 */
typedef struct {
    float    x_cm;
    uint32_t frame_time_ms;
    uint16_t seq;
    float    confidence;
    bool     valid;
} BallMeasurement;

/*
 * 链路健康状态，供上层调试和故障判定使用。
 */
typedef struct {
    uint32_t frames_ok;           /* CRC 正确且格式正确的帧数 */
    uint32_t frames_crc_error;    /* CRC 错误帧数 */
    uint32_t frames_format_error; /* 同步/长度/版本错误帧数 */
    uint32_t frames_rejected;     /* 帧正确但语义无效（未标定/低置信度/超范围） */
    uint32_t frames_duplicate;    /* seq 与上一帧相同的重复帧 */
    uint32_t rx_overflow;         /* 接收环形缓冲溢出字节数 */
    uint32_t tx_dropped;          /* 遥测因发送缓冲满而丢弃的帧数 */
    uint32_t last_rx_ms;          /* 最近一次收到完整位置帧的本地时间 */
    uint16_t last_seq;
    uint8_t  consecutive_errors;  /* 连续解码错误次数 */
    bool     have_frame;          /* 上电后是否至少收到过一帧完整位置帧 */
    bool     link_fault;          /* 连续错误过多，链路判定为故障 */
} vision_link_status_t;

/*
 * 初始化视觉链路。
 *
 * 参数：
 * - baud：目标波特率，传 0 时使用 VISION_LINK_DEFAULT_BAUD。
 *
 * 行为：
 * - 清空收发缓冲和统计；
 * - 按 baud 重新配置 UART4，开启 FIFO、RX 中断和 TX 中断对应的 NVIC；
 * - 不发送任何数据，也不阻塞。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后、system_time_init() 之后调用。
 */
void Vision_Init(uint32_t baud);

/*
 * 处理接收环形缓冲中的字节（主循环调用，非阻塞）。
 *
 * 说明：
 * - 中断只负责把字节搬进环形缓冲，真正的协议解析放在本函数；
 * - 建议在主循环每圈都调用一次，30 Hz 数据量下开销可以忽略；
 * - 解析成功的有效帧会更新 Vision_GetLatest() 的结果。
 */
void Vision_Update(void);

/*
 * 取最近一帧测量结果。
 *
 * 参数：
 * - out：输出缓冲区，不能为 NULL；
 * - age_ms：可选输出，返回该帧距今的毫秒数，可为 NULL。
 *
 * 返回值：
 * - true：存在最近帧且仍处于有效期内（<=150 ms）且帧本身有效；
 * - false：从未收到帧、已超时或最近帧被判定为无效。
 *
 * 注意：返回 false 时 out 仍会被填充（valid=false），便于调试显示。
 */
bool Vision_GetLatest(BallMeasurement *out, uint32_t *age_ms);

/* 视觉数据当前是否新鲜且有效。 */
bool Vision_IsFresh(void);

/* 读取链路统计。status 为 NULL 时直接返回。 */
void Vision_GetStatus(vision_link_status_t *status);

/*
 * 非阻塞发送一帧遥测。
 *
 * 参数：
 * - type：帧类型，遥测固定传 VISION_LINK_TYPE_TELEMETRY；
 * - payload：负载指针，可为 NULL（此时 len 必须为 0）；
 * - len：负载长度，最大 VISION_LINK_MAX_PAYLOAD_LEN。
 *
 * 返回值：
 * - true：已完整放入发送环形缓冲；
 * - false：参数非法或缓冲空间不足，本帧被丢弃（不会阻塞、不会破坏已有帧）。
 */
bool Vision_SendFrame(uint8_t type, const uint8_t *payload, uint8_t len);

/*
 * 把 float 按小端写入 4 字节缓冲区。
 * 供上层组包和单元测试复用，避免各处重复写 union/memcpy。
 */
void Vision_WriteF32LE(uint8_t *dst, float value);

/* 把 uint32 按小端写入 4 字节缓冲区。 */
void Vision_WriteU32LE(uint8_t *dst, uint32_t value);

#endif
