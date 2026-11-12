#ifndef DRV8874_H
#define DRV8874_H

/*
 * DRV8874 四路直流电机驱动接口，工作在 PH/EN 模式。
 *
 * 当前硬件连接关系：
 * - MCU MOTOR_PWM Cx（PA29/PA30/PA28/PB1）-> DRV8874 EN/IN1：速度 PWM，
 *   占空比 0~1000 对应 0~100%。
 * - MCU MOTOR_PHx（PA31/PB0/PB2/PB3）-> DRV8874 PH/IN2：方向 GPIO，
 *   1=正转，0=反转。
 * - nSLEEP：外部硬件控制，MCU 只读取；低电平表示驱动器睡眠。
 * - nFAULT：低有效故障输入，MCU 只读取；低电平表示驱动器报错。
 *
 * speed_permille：正数正转、负数反转，绝对值 0~1000 表示 PWM 千分比。
 *
 * 约束：BSP 模块头文件只包含 bsp_common.h，不直接包含聚合头 bsp.h。
 */
#include "bsp_common.h"

typedef enum {
    MOTOR_M1 = 0,
    MOTOR_M2 = 1,
    MOTOR_M3 = 2,
    MOTOR_M4 = 3,
    MOTOR_COUNT
} motor_id_t;

typedef struct {
    bool enabled;           /* 软件层是否正在输出：方向已选且 EN PWM 非零。 */
    bool asleep;            /* nSLEEP 低电平后的语义状态，true 表示硬件睡眠。 */
    bool fault;             /* nFAULT 低电平后的语义状态，true 表示故障有效。 */
    bool nsleep_high;       /* nSLEEP 原始电平：1=未睡眠，0=睡眠。 */
    bool nfault_high;       /* nFAULT 原始电平：1=无故障，0=故障有效。 */
    int16_t speed_permille; /* 当前命令速度，正负号表示方向，绝对值为千分比。 */
    uint16_t adc_raw;       /* 最近一次 ADC 刷新得到的原始 0~4095 计数。 */
    uint16_t current_ma;    /* 由 IPROPI 估算的电流值，后续可按实测参数标定。 */
} motor_status_t;

/*
 * 初始化四路 DRV8874 电机驱动的软件状态、PWM 输出和 ADC 电流采样通道。
 *
 * 调用时机：
 * - 必须在 SYSCFG_DL_init() 之后调用；MOTOR_PWM、MOTOR_PH、ADC、VREF 均由 SysConfig 配置。
 *
 * 安全约束：
 * - 初始化后所有电机默认停止输出，避免上电瞬间误转。
 */
void motor_init(void);

/*
 * 使能或关闭单路电机输出。
 *
 * 参数：
 * - id：电机编号 MOTOR_M1~MOTOR_M4。
 * - enable：true 允许按当前速度输出；false 强制该路 EN PWM 为 0。
 */
void motor_enable(motor_id_t id, bool enable);

/*
 * 设置单路电机睡眠状态。
 *
 * 参数：
 * - id：电机编号。
 * - sleep：true 请求睡眠，false 请求唤醒。
 *
 * 说明：
 * - 当前硬件 nSLEEP 主要由外部电路控制，函数保留软件接口用于兼容和后续硬件扩展。
 */
void motor_sleep(motor_id_t id, bool sleep);

/*
 * 设置单路电机速度与方向。
 *
 * 参数：
 * - id：电机编号。
 * - speed_permille：速度千分比，范围建议 -1000~1000；正数正转，负数反转，0 停止。
 *
 * 硬件约束：
 * - 方向由 MOTOR_PHx GPIO 输出，速度由 MOTOR_PWM EN 通道占空比输出。
 */
void motor_set_speed(motor_id_t id, int16_t speed_permille);

/*
 * 读取单路电机的综合状态快照。
 *
 * 参数：
 * - id：电机编号。
 * - status：输出状态结构体指针，不可为 NULL。
 *
 * 返回值：
 * - true：读取成功。
 * - false：参数无效。
 */
bool motor_get_status(motor_id_t id, motor_status_t *status);

/*
 * 读取单路电机估算电流。
 *
 * 参数：
 * - id：电机编号。
 *
 * 返回值：
 * - 最近一次 ADC 刷新后换算得到的电流值，单位 mA。
 */
uint16_t motor_read_current_ma(motor_id_t id);

/*
 * 读取单路电机电流采样 ADC 原始值。
 *
 * 参数：
 * - id：电机编号。
 *
 * 返回值：
 * - 0~4095 的 ADC 原始计数；若 id 无效返回 0。
 */
uint16_t motor_read_adc_raw(motor_id_t id);

/*
 * 查询单路电机是否处于故障状态。
 *
 * 参数：
 * - id：电机编号。
 *
 * 返回值：
 * - true：nFAULT 低电平，驱动器报告故障。
 * - false：无故障或参数无效。
 */
bool motor_is_fault(motor_id_t id);

/*
 * 查询单路电机是否处于睡眠状态。
 *
 * 参数：
 * - id：电机编号。
 *
 * 返回值：
 * - true：nSLEEP 低电平。
 * - false：未睡眠或参数无效。
 */
bool motor_is_asleep(motor_id_t id);

/*
 * 停止全部电机输出。
 *
 * 行为：
 * - 四路电机速度命令清零，EN PWM 输出置 0，PH GPIO 保持安全低电平。
 */
void motor_stop_all(void);

/*
 * 刷新四路电机电流 ADC 缓存。
 *
 * 调用约束：
 * - 应在主循环周期调用，不要在中断中长时间阻塞。
 * - motor_get_status()/motor_read_current_ma() 读取的是最近一次刷新后的缓存值。
 */
void motor_refresh_adc(void);

/*
 * 查询是否存在任意一路电机故障。
 *
 * 返回值：
 * - true：至少一路 nFAULT 有效。
 * - false：所有电机均未报告故障。
 */
bool motor_any_fault(void);

/*
 * 查询是否存在任意一路电机睡眠。
 *
 * 返回值：
 * - true：至少一路 nSLEEP 为低。
 * - false：所有电机均处于非睡眠状态。
 */
bool motor_any_asleep(void);

/*
 * 查询是否存在任意一路电机正在输出。
 *
 * 返回值：
 * - true：至少一路软件速度命令非 0 且输出已使能。
 * - false：所有电机停止。
 */
bool motor_any_running(void);

/*
 * 清除单路电机故障的软件记录。
 *
 * 参数：
 * - id：电机编号。
 *
 * 说明：
 * - DRV8874 的实际故障解除仍由硬件状态决定；该函数只用于清理软件侧暂存状态。
 */
void motor_clear_fault(motor_id_t id);

#endif
