#ifndef APP_BALL_TEST_H
#define APP_BALL_TEST_H

/*
 * 滚球控制测试应用（整机入口）。
 *
 * 本应用与 line_following 并列，由 User/main.c 的编译期宏选择，二者不会同时
 * 运行，也不会修改循迹代码。
 *
 * 支持四种测试模式（可在运行时用 K1+K2 循环切换）：
 * - MANUAL：不启用 PD，手动以 0.1° 步长微调摆杆，用于平衡点标定（任务一）；
 * - ROLL：开环 5°/7.5°/10° 自动左右滚动，按 K1 启动后会循环执行
 *   +θ -> 0 -> -θ -> 0，直到再按一次 K1 停止（任务二）；
 * - SIM：用模拟钢珠位置验证 PD 控制逻辑，无需 Jetson；
 * - UART：接收 Jetson 通过 UART0（Type-C/DAPLink，PA10/PA11）发来的真实厘米位置。
 *
 * 按键操作：
 * - MANUAL：K1 = +BALL_TEST_MANUAL_STEP_DEG，K2 = -BALL_TEST_MANUAL_STEP_DEG；
 * - ROLL：K1 = 启动连续左右滚动（运行中再按 = 立即停止并回平衡点），
 *         K2 = 切换 5°/7.5°/10° 档位（运行中忽略）；
 * - SIM/UART：K2 = 切换场景/目标位置；
 *   K1 = 使能/关闭 AUTO，但仅当 BALL_TEST_ALLOW_AUTO=1 时生效；
 *   本版本默认 1（静态滚球联调允许 K1 进入闭环）；设为 0 则只监测。
 * - 任意模式：先按住 K1 再按 K2 = 切换到下一个模式。
 *
 * 显示与日志：
 * - LCD 采用逐行增量刷新：只有内容变化的行才重写，且两行之间至少间隔
 *   50 ms，避免 8 MHz SPI 整屏刷新阻塞约 60 ms、打乱 30 Hz 控制节拍；
 * - ROLL 模式不使用视觉数据，也不改变 UART 协议；
 * - UART0 以 100 ms 周期发送二进制遥测帧，由 Jetson/PC 记录；
 * - 调试输出全部非阻塞，不会拖慢 30 Hz 控制节拍。
 *
 * 头文件规则：应用层头文件只包含公共基础头，不包含聚合头 bsp.h。
 */
#include "bsp_common.h"

/* 测试模式。 */
typedef enum {
    BALL_TEST_MODE_MANUAL = 0,
    BALL_TEST_MODE_SIM = 1,
    BALL_TEST_MODE_UART = 2,
    BALL_TEST_MODE_ROLL = 3
} ball_test_mode_t;

/*
 * 开环滚动的阶段。
 *
 * 依次为：平衡点 -> 正倾角 -> 回中 -> 反倾角 -> 回中 -> 结束。
 * 每个阶段都先等实际倾角到位，再按配置时间保持。
 */
typedef enum {
    BALL_ROLL_IDLE = 0,       /* 未运行，摆杆保持在平衡点 */
    BALL_ROLL_NEUTRAL,        /* 回到平衡点并稳定，准备正向 */
    BALL_ROLL_POSITIVE,       /* 正倾角 +θ 保持 */
    BALL_ROLL_RETURN_POS,     /* 正向结束，回平衡点 */
    BALL_ROLL_NEGATIVE,       /* 反倾角 -θ 保持 */
    BALL_ROLL_RETURN_NEG,     /* 反向结束，回平衡点 */
    BALL_ROLL_FINISH          /* 到位超时中止；摆杆回平衡点 */
} ball_test_roll_state_t;

/*
 * 开环滚动状态的只读快照，供 LCD、串口日志和主机测试使用。
 */
typedef struct {
    bool     running;      /* 是否正在执行自动流程 */
    bool     aborted;      /* 是否因到位超时中止 */
    uint8_t  level;        /* 0..2 -> 5/7.5/10° */
    float    theta_deg;    /* 当前档位角度，单位 ° */
    ball_test_roll_state_t state;
} ball_roll_status_t;

/*
 * 是否允许 SIM/UART 模式下按 K1 进入 AUTO。
 *
 * 本次静态滚球联调使用 UART 监测 + K1 启动 AUTO，因此设为 1；
 * 设为 0 时 UART/SIM 下 K1 只监测、不进入 PD 闭环。
 */
#ifndef BALL_TEST_ALLOW_AUTO
#define BALL_TEST_ALLOW_AUTO            (1U)
#endif

/* 上电默认模式；本次任务要求默认仍由 main.c 选择循迹，不自动进入滚球测试。 */
#ifndef BALL_TEST_DEFAULT_MODE
#define BALL_TEST_DEFAULT_MODE          (BALL_TEST_MODE_UART)
#endif

/*
 * MANUAL 模式单次按键的倾角步长（°）。
 *
 * 平衡点标定建议先用 0.2° 快速接近，再改为 0.1° 精调；
 * 修改本宏后重新编译即可。
 */
#ifndef BALL_TEST_MANUAL_STEP_DEG
#define BALL_TEST_MANUAL_STEP_DEG       (0.1f)
#endif

/*
 * 开环滚动的倾角档位（摆杆实际目标倾角，非舵机轴角度）。
 *
 * 当前档位：5° / 10° / 15°，每个档位取 ±对应角度，
 * 均相对“标定后的平衡点”计算。
 */
#define BALL_TEST_ROLL_LEVEL_COUNT      (3U)

/*
 * 各阶段保持时间（ms），均为“舵机实际到达目标后”再开始计时。
 *
 * 当前配置为 5°/7.5°/10°，正/负保持时间分别为 500 ms / 200 ms，
 * 都比原来 2°/3°/4°/5° 的 800 ms 短，避免钢珠直接撞到轨道端部。
 * 必须通过真实钢珠测试逐档调整：
 * - 5° 可以适当加长观察滚动；
 * - 10°/15° 先从更短时间开始，确认钢珠不撞端再逐步加长。
 */
#ifndef BALL_TEST_ROLL_POSITIVE_HOLD_MS
#define BALL_TEST_ROLL_POSITIVE_HOLD_MS  (500U)
#endif
#ifndef BALL_TEST_ROLL_NEGATIVE_HOLD_MS
#define BALL_TEST_ROLL_NEGATIVE_HOLD_MS  (200U)
#endif
#ifndef BALL_TEST_ROLL_CENTER_HOLD_MS
#define BALL_TEST_ROLL_CENTER_HOLD_MS    (400U)
#endif

/*
 * 到达判定与超时（仅用于开环测试）：
 * - 实际倾角与目标相差不超过 0.2° 才认为“到位”，随后开始保持计时；
 * - 3 s 仍未到位则判定机械卡滞/舵机异常，立即中止并回平衡点。
 */
#define BALL_TEST_ROLL_SETTLE_TOL_DEG    (0.2f)
#define BALL_TEST_ROLL_SETTLE_TIMEOUT_MS (3000U)

/*
 * 开环测试允许的最大摆杆倾角（°）。
 *
 * 只在 K1 启动后的连续 ROLL 运行期间把 Servo 的倾角上限提高到这里；
 * 停止、正常结束、到位超时中止或切换模式时，都会立即恢复
 * SERVO_BEAM_LIMIT_DEFAULT_DEG（当前 5.0°）。
 * PD 闭环的 ±2° 限幅不受影响。
 */
#ifndef BALL_TEST_ROLL_BEAM_LIMIT_DEG
#define BALL_TEST_ROLL_BEAM_LIMIT_DEG    (15.0f)
#endif

/*
 * 初始化测试应用。
 *
 * 调用顺序（由 main.c 保证）：
 *   SYSCFG_DL_init -> uart0_init -> system_time_init -> BallTest_AppInit
 *
 * 内部会依次初始化 LCD、按键、舵机、视觉链路和 PD 控制器，
 * 然后按 BALL_TEST_DEFAULT_MODE 进入对应模式，状态为 IDLE（未使能 AUTO）。
 */
void BallTest_AppInit(void);

/*
 * 主循环任务：解析视觉、处理按键、推进手动/开环/PD 状态机、
 * 发送遥测和刷新 LCD。
 *
 * 约束：
 * - 非阻塞；LCD 逐行增量刷新，遥测 100 ms 一次；
 * - 不得在中断中调用。
 */
void BallTest_AppTask(void);

/*
 * 运行时切换测试模式。
 *
 * 参数：
 * - mode：目标模式。
 *
 * 行为：
 * - 先停止正在运行的开环滚动、关闭 AUTO，并限速回中；
 * - 所有模式切换都会把 Servo 倾角上限恢复为默认 5.0°；
 * - ROLL 的 15° 上限只在 K1 启动后的连续运行期间生效；
 * - 进入 MANUAL 时摆杆回水平，手动微调量清零；
 * - 进入 SIM 时重置模拟钢珠状态；
 * - 进入 UART 时重置帧转发序号。
 */
void BallTest_SetMode(ball_test_mode_t mode);

/*
 * 读取开环滚动状态快照。
 *
 * 参数：
 * - status：输出缓冲区，不能为 NULL。
 */
void BallTest_GetRollStatus(ball_roll_status_t *status);

#endif
