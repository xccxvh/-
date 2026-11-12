/*
 * 2026 年全国大学生电子设计竞赛 H 题 要求 2：
 * 小车置于 A 点，按键启动后沿黑线顺时针行驶一圈并停到 A 点，
 * 计时停止并显示行驶总时间；要求行驶总时间不超过 20 s，停车偏差不超过 2 cm。
 *
 * 一、场地约束（题目说明 1、2）
 *   AB、CD 为 1.5 m 直道，BC、DA 为半径 0.5 m 的半圆，线宽 1.8±0.2 cm，
 *   一圈总长约 1.5*2 + pi*0.5*2 = 6.14 m。要在 20 s 内完成，平均速度必须
 *   不低于 0.31 m/s，因此直道需要较高基础 PWM，而弯道必须主动降速，
 *   否则半径仅 0.5 m 的半圆会因离心趋势过大而丢线。
 *   A 点启停线是长 5 cm、线宽 1.8 cm、与环路垂直的横线，它会让传感器阵列
 *   在瞬间同时覆盖更多探头，是识别“回到 A 点”的物理特征。
 *
 * 二、本文件用到的硬件
 *   - 八路红外灰度：BSP/Infrared 模块，输出黑线位图与加权横向偏差；
 *   - 左右直流电机：BSP/MOTOR 模块，set_motor_speed(left, right)，
 *     PWM 量程 -1000~1000，正值为前进；
 *   - 启动按键：PB31 是底板上的独立“用户按键”，SysConfig 已配置为内部上拉
 *     输入、下降沿中断；按键另一端接 GND，未按下为高电平、按下为低电平。
 *     注意 PB20 是 4x4 矩阵键盘接口的 V1 列线，本车没有矩阵键盘，不能当按键用；
 *   - OLED：显示实时计时、运行状态和传感器调试信息；
 *   - TimerA1：SysConfig 配置为 100 us 周期中断，提供 32 位单调节拍。
 *
 * 三、控制时序
 *   100 Hz（10 ms）控制周期。每个周期先做约 4 ms 的八路灰度投票采样，
 *   随后计算 PID 差速、刷新显示，剩余时间用 WFE 等待下一个 10 ms 时隙，
 *   以 TimerA1 节拍对齐，避免采样和显示耗时把控制周期越拖越长。
 *
 * 四、安全策略
 *   1. 起点本身就压在启停线上，必须先确认车辆已经驶离横线才允许识别终点；
 *   2. 终点识别还要求已经行驶足够长时间（8 s），防止起步瞬间误判；
 *   3. 超过 20 s 未完成或连续丢线 1 s，立即制动停车并提示，防止冲出场地。
 *
 * 说明：本文件是要求 2 的独立应用入口，旧版测试程序 main.c 仍保留在工程中，
 *       但当前 Keil 工程只编译 main_h2.c，不会出现重复 main 符号。
 */
#include "ti_msp_dl_config.h"
#include "bsp.h"

/*
 * 串口调试输出总开关。
 *
 * 实车调试发现：UART0 写入一个字节后，STAT.BUSY 状态位不会清零，
 * printf 会永久停在 BSP/UART0/uart0.c 的 fputc() 内部：
 *     while (DL_UART_Main_isBusy(UART0)) { }
 * 这样 main 无法进入 OLED 待机画面和循迹流程，表现为“OLED 不亮、电机乱转”。
 * 要求 2 的计时和运行状态本来就显示在 OLED 上，串口输出只是辅助调试手段，
 * 因此默认关闭，保证 UART 异常时不会拖死主控制流程。
 * 等 UART0 的时钟/引脚/BUSY 问题排查清楚后，把本宏改回 1 即可恢复串口打印。
 */
#define H2_ENABLE_UART_DEBUG         (0)

/*
 * 电机引脚映射诊断开关。
 * 1 = 上电后进入逐通道点动测试：用 LED2 的闪烁次数标识当前相位，
 *     每个相位只驱动一路 PWM 持续 2 秒，用来确认 TIMG6/TIMA0 的
 *     四个通道分别控制左轮(M1)还是右轮(M2)，以及转向。
 * 0 = 正常运行要求 2 的循迹程序。
 * 完成电机映射确认后必须改回 0。
 */
#define H2_MOTOR_PROBE              (1)

/* ============================ 灰度采样参数 ============================ */

/*
 * 每个控制周期连续采样 5 次、间隔 1 ms，某一路命中 3 次才判为稳定压线。
 * 采样窗口约 4 ms：0.5 m/s 时车辆只前进 2 mm，不会明显拖慢响应，
 * 同时能滤除红外模块在黑白边缘的毫秒级抖动。
 */
#define H2_INF_SAMPLE_COUNT          (5U)
#define H2_INF_HIT_THRESHOLD         (3U)

/* ============================ 控制周期参数 ============================ */

/* 10 ms 控制周期 = 100 Hz。TimerA1 一个节拍是 100 us，因此周期为 100 ticks。 */
#define H2_CTRL_PERIOD_MS            (10U)
#define H2_CTRL_PERIOD_TICKS         (H2_CTRL_PERIOD_MS * 10U)

/* ============================== 速度参数 ============================== */

/*
 * 直道基础 PWM。PWM 量程沿用 motor.c 的 -1000~1000。
 * 6.14 m 一圈要在 20 s 内完成，直道需要较高速度；若实车偏慢，
 * 优先提高本值，但不要超过 H2_MAX_FORWARD_PWM。
 */
#define H2_BASE_PWM                  (260)

/*
 * 弯道降速：基础 PWM = 直道基础值 - |偏差| * H2_CURVE_SLOW_SLOPE，
 * 并限制不低于 H2_CURVE_MIN_PWM。
 * 偏差越大说明偏离黑线越多，通常也意味着正在急弯，此时主动降速
 * 可以显著降低 0.5 m 半径半圆的丢线概率。
 * 实车若在 BC、DA 半圆频繁丢线，优先降低 H2_CURVE_MIN_PWM 或提高斜率；
 * 若整圈耗时接近 20 s，则相反方向调整。
 */
#define H2_CURVE_SLOW_SLOPE          (20)
#define H2_CURVE_MIN_PWM             (170)

/* 基础速度每周期最大变化量，抑制直道/弯道切换时的速度突变。 */
#define H2_BASE_SLEW_STEP            (8)

/*
 * 单轮正转 PWM 的上下限。
 * 下限取 40 而不是 0：motor.c 在 pwm<=0 时走反向分支，pwm=0 会得到
 * 比较值 1000（即 100% 反向占空比），并不是停车。循迹过程中必须让两个
 * 轮子始终处在明确的正转区间，否则内侧轮会在弯道被误判成反转。
 */
#define H2_MIN_FORWARD_PWM           (40)
#define H2_MAX_FORWARD_PWM           (520)

/* =============================== PID 参数 =============================== */

/*
 * 定点整数 PID：所有增益先放大 16 倍，运算后再除以 16，避免 M0+ 上使用浮点。
 * 比例项 18.0：偏差 1 对应约 18 PWM 差速；
 * 积分项 0.125：补偿左右电机和轮胎的静态不对称；
 * 微分项 3.0：抑制进弯/出弯时的超调，采样周期 10 ms，噪声放大有限。
 * 积分限幅防止长时间单侧偏差导致积分饱和。
 */
#define H2_PID_SCALE                 (16)
#define H2_PID_KP                    (288)
#define H2_PID_KI                    (2)
#define H2_PID_KD                    (48)
#define H2_PID_INTEGRAL_LIMIT        (300)

/* 差速修正量限幅。基础 260 时，修正 160 对应外轮 420、内轮 100。 */
#define H2_MAX_CORRECTION_PWM        (160)

/* ============================== 丢线处理 ============================== */

/*
 * 八路全部看不到黑线时，按最后一次有效偏差的方向低速找线：
 * 基础 PWM 降到 150，固定差速 90，保证小半径回线时不会因速度过高而冲出去。
 * 连续丢线 100 个周期（约 1 s）仍未重新见线则判定失败并停车。
 */
#define H2_LOST_BASE_PWM             (150)
#define H2_LOST_CORRECTION           (90)
#define H2_LOST_STOP_SAMPLES         (100U)

/* ============================ 启停线识别参数 ============================ */

/*
 * 启停线识别依据：正常的 1.8 cm 循迹线通常只会压住 1~2 路探头，
 * 而 5 cm 长的横向启停线会让 4 路及以上探头同时压黑。
 * 该阈值与传感器阵列的探头间距直接相关，必须在实车上用 OLED 的
 * “S:-------- N:x” 实时显示核对：
 *   正常循迹时 N 的最大值应明显小于 H2_FINISH_SENSOR_MIN，
 *   车辆人工推过 A 点横线时 N 应达到或超过该阈值。
 * 如果实车 N 达不到 4，可下调到 3，但必须确认过弯时不会误触发。
 */
#define H2_FINISH_SENSOR_MIN         (4U)

/* 连续 2 个周期都满足才确认终点，滤除单周期噪声。 */
#define H2_FINISH_CONFIRM_SAMPLES    (2U)

/*
 * 起点脱离判定：传感器位图连续 10 个周期（约 100 ms）都不超过 2 路压黑，
 * 才认为车辆已经驶离起点横线，之后才允许识别终点。
 */
#define H2_START_CLEAR_MAX_SENSORS   (2U)
#define H2_START_CLEAR_SAMPLES       (10U)

/*
 * 终点最短时间门槛 8 s。
 * 按 0.5 m/s 估算，8 s 只走了约 4 m，仍明显小于一圈 6.14 m，
 * 因此不会与真正的终点混淆，同时能避开起步阶段的任何干扰。
 */
#define H2_FINISH_MIN_TIME_MS        (8000U)
#define H2_FINISH_MIN_TIME_TICKS     (H2_FINISH_MIN_TIME_MS * 10U)

/* ============================== 安全与停车 ============================== */

/* 题目要求整圈不超过 20 s，超过即本次测试失败，立即停车防止冲出场地。 */
#define H2_SAFETY_TIMEOUT_MS         (20000U)
#define H2_SAFETY_TIMEOUT_TICKS      (H2_SAFETY_TIMEOUT_MS * 10U)

/*
 * 主动制动：检测到终点后先反向给 50 ms、PWM 200 的制动量，再进入停止状态。
 * 若只切断动力，车辆会以约 0.5 m/s 的初速度滑行数厘米，难以满足 2 cm
 * 的停车偏差要求。制动参数需要按实车质量、轮胎摩擦和电池电压微调：
 * 制动时间过短会冲过 A 点，过长则会倒退。
 *
 * 停车点标定方法：
 * 题目允许在车身中心轴线上指定唯一测试位置。该位置不必放在传感器阵列处，
 * 可以整体后移“检测延迟 + 制动滑行距离”，使车辆停稳后测试点正好落在 A 点。
 * 若测得车辆总是冲过 A 点，就把测试点后移同样距离或加大制动量；
 * 若车辆在 A 点前停住，则把测试点前移或减小制动量。
 */
#define H2_BRAKE_PWM                 (200)
#define H2_BRAKE_TIME_MS             (50U)

/*
 * 停止 PWM 沿用本工程既有实车约定：set_motor_speed(1, 1)。
 * 注意不能传入 0：motor.c 在 pwm<=0 分支里写入 1000 + pwm 作为比较值，
 * pwm=0 时 PH 方向脚为低、比较值为 1000，等效 100% 反向占空比，
 * 会让车在停车指令下反而全速后退。1/1000 占空比才是不转动的安全值。
 */
#define H2_STOP_PWM                  (1)

/* ============================== 按键参数 ============================== */

/*
 * 启动按键：PB31 是底板上的独立用户按键，SysConfig 中名为 key/user，
 * 已配置为内部上拉输入，按下时与 GND 导通、读到低电平。
 * 之所以不用 PB20：PB20 属于 4x4 矩阵键盘接口的 V1 列线，
 * 本车未安装矩阵键盘，该引脚悬空，无法作为启动键使用。
 * 若更换按键引脚，只修改这两个宏即可；
 * 连续 5 次、每次间隔 5 ms 都检测到按下才确认，即 25 ms 消抖。
 */
#define H2_START_KEY_PORT            (GPIOB)
#define H2_START_KEY_PIN             (DL_GPIO_PIN_31)
#define H2_START_KEY_PRESSED() \
    (DL_GPIO_readPins(H2_START_KEY_PORT, H2_START_KEY_PIN) == 0U)
#define H2_KEY_DEBOUNCE_SAMPLES      (5U)
#define H2_KEY_SAMPLE_MS             (5U)

/* 显示刷新周期：100 ms，兼顾可读性和控制周期占用。 */
#define H2_DISPLAY_PERIOD_TICKS      (1000U)

/* ================================ 类型 ================================ */

/* 循迹 PID 的运行时状态。 */
typedef struct
{
    int last_error;  /* 上一次有效偏差，用于微分项 */
    int integral;    /* 误差积分，带限幅 */
} H2_LinePid;

/* 单圈运行结果，决定结束画面提示内容。 */
typedef enum
{
    H2_RUN_FINISHED = 0,  /* 识别到 A 点启停线，正常完成 */
    H2_RUN_TIMEOUT,       /* 超过 20 s 仍未完成 */
    H2_RUN_LINE_LOST      /* 连续丢线超时，判定失败 */
} H2_RunResult;

/* ============================== 工具函数 ============================== */

/* 通用整数限幅。 */
static int h2_clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/* 整数绝对值，避免为一次取绝对值引入浮点库。 */
static int h2_abs_int(int value)
{
    return (value >= 0) ? value : -value;
}

/*
 * 按实车既有约定停车。
 * 调用前应确保已经完成制动；本函数只负责把两路 PWM 拉到停止值。
 */
static void h2_stop_motor(void)
{
    set_motor_speed(H2_STOP_PWM, H2_STOP_PWM);
}

/*
 * 主动制动后停车。
 * 先反向输出 H2_BRAKE_PWM 持续 H2_BRAKE_TIME_MS，利用电机反电动势和
 * 轮胎摩擦快速消耗动能，再切换到停止状态。
 * 该函数是阻塞的，只在终点或故障停车时调用，不影响正常控制周期。
 */
static void h2_brake_and_stop(void)
{
    if (H2_BRAKE_TIME_MS > 0U)
    {
        set_motor_speed(-H2_BRAKE_PWM, -H2_BRAKE_PWM);
        delay_ms(H2_BRAKE_TIME_MS);
    }

    h2_stop_motor();
}

/* ============================== PID 控制 ============================== */

/*
 * 计算一次离散 PID 差速修正量。
 * 输入 error：-7~+7 的横向偏差，正值表示黑线偏右；
 * 返回：限幅后的差速修正，正值表示需要提高左轮、降低右轮。
 * 积分项单独限幅，长弯或短暂丢线后不会因为积分饱和而出现大幅回头。
 */
static int h2_line_pid_step(H2_LinePid *pid, int error)
{
    int derivative = error - pid->last_error;
    int correction;

    pid->integral = h2_clamp_int(pid->integral + error,
                                 -H2_PID_INTEGRAL_LIMIT,
                                 H2_PID_INTEGRAL_LIMIT);
    correction = (H2_PID_KP * error +
                  H2_PID_KI * pid->integral +
                  H2_PID_KD * derivative) / H2_PID_SCALE;
    pid->last_error = error;

    return h2_clamp_int(correction,
                        -H2_MAX_CORRECTION_PWM,
                        H2_MAX_CORRECTION_PWM);
}

/*
 * 根据偏差大小计算本周期的基础速度。
 * 直道（偏差接近 0）使用 H2_BASE_PWM 保证 20 s 内跑完全程；
 * 偏差越大越接近急弯，基础速度按斜率下调，最低到 H2_CURVE_MIN_PWM。
 */
static int h2_curve_base_pwm(int error)
{
    int base = H2_BASE_PWM - h2_abs_int(error) * H2_CURVE_SLOW_SLOPE;

    if (base < H2_CURVE_MIN_PWM)
    {
        base = H2_CURVE_MIN_PWM;
    }

    return base;
}

/*
 * 让 current 以每周期 step 的步长逼近 target。
 * 用于基础速度平滑，避免弯道降速和出弯提速时产生明显冲击。
 * 在 10 ms 周期下，step=8 对应最大 800 PWM/s 的变化率。
 */
static int h2_slew_toward(int current, int target, int step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

/* ============================== 显示函数 ============================== */

/*
 * 显示两位十进制数，固定补零。
 * 直接调用 OLED_ShowNum 会隐藏前导零，导致计时显示宽度跳动，
 * 因此秒和百分秒都使用本函数固定占两格。
 */
static void h2_show_two_digits(uint8_t x, uint8_t y, uint32_t value)
{
    value %= 100U;
    OLED_ShowChar(x, y, (u8)('0' + (value / 10U)));
    OLED_ShowChar((uint8_t)(x + 8U), y, (u8)('0' + (value % 10U)));
}

/*
 * 把 100 us 节拍显示为“T=秒.百分秒s”。
 * 计时本身保留 100 us 分辨率，显示只做 0.01 s 截断，最终锁存值不受影响。
 */
static void h2_show_time(uint32_t elapsed_ticks)
{
    uint32_t centiseconds = elapsed_ticks / 100U;
    uint32_t seconds = centiseconds / 100U;
    uint32_t fraction = centiseconds % 100U;

    OLED_ShowString(0, 2, (u8 *)"T=");
    h2_show_two_digits(16, 2, seconds);
    OLED_ShowChar(32, 2, '.');
    h2_show_two_digits(40, 2, fraction);
    OLED_ShowString(56, 2, (u8 *)"s   ");
}

/*
 * 实时显示八路传感器状态，供启动前检查接线和极性。
 * “*”表示该路压在黑线上，“-”表示白底，N 为压黑线的总路数。
 * 该画面只在等待启动时刷新，不影响行驶中的控制周期。
 */
static void h2_show_sensor_bar(void)
{
    uint8_t mask = inf_read_mask_stable(H2_INF_SAMPLE_COUNT,
                                        H2_INF_HIT_THRESHOLD);
    uint8_t count = inf_count_bits(mask);
    u8 text[9];
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        text[i] = ((mask & (uint8_t)(1U << i)) != 0U) ? (u8)'*' : (u8)'-';
    }
    text[8] = '\0';

    OLED_ShowString(0, 6, (u8 *)"S:");
    OLED_ShowString(16, 6, text);
    OLED_ShowString(88, 6, (u8 *)"N:");
    OLED_ShowChar(104, 6, (u8)('0' + count));
}

/* 等待启动时的固定提示画面。 */
static void h2_show_idle_screen(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"H2 REQUIREMENT2");
    OLED_ShowString(0, 2, (u8 *)"PRESS START KEY");
    OLED_ShowString(0, 4, (u8 *)"LIMIT 20.00s");
    h2_show_sensor_bar();
}

/*
 * 行驶中的实时画面：运行状态、计时、当前压黑路数和偏差。
 * N 和 E 是现场调参的关键依据：
 *   N 的常态值用于确认 H2_FINISH_SENSOR_MIN 是否合理；
 *   E 的符号用于确认误差方向是否与转向修正一致。
 */
static void h2_show_running(uint32_t elapsed_ticks,
                            uint8_t black_count,
                            int error)
{
    OLED_ShowString(0, 0, (u8 *)"RUN ");
    h2_show_time(elapsed_ticks);

    OLED_ShowString(0, 4, (u8 *)"N=");
    OLED_ShowChar(16, 4, (u8)('0' + black_count));
    OLED_ShowString(32, 4, (u8 *)"E=");
    if (error < 0)
    {
        OLED_ShowChar(48, 4, '-');
        OLED_ShowChar(56, 4, (u8)('0' + (-error)));
    }
    else
    {
        OLED_ShowChar(48, 4, '+');
        OLED_ShowChar(56, 4, (u8)('0' + error));
    }
}

/* 结束画面：显示本次结果和锁存的最终用时。 */
static void h2_show_result(H2_RunResult result, uint32_t elapsed_ticks)
{
    OLED_Clear();

    if (result == H2_RUN_FINISHED)
    {
        OLED_ShowString(0, 0, (u8 *)"FINISHED");
    }
    else if (result == H2_RUN_TIMEOUT)
    {
        OLED_ShowString(0, 0, (u8 *)"TIMEOUT");
    }
    else
    {
        OLED_ShowString(0, 0, (u8 *)"LINE LOST");
    }

    h2_show_time(elapsed_ticks);
    OLED_ShowString(0, 4, (u8 *)"PRESS TO RETRY");
    /* 结束画面同样保留八路状态，便于失败时判断是停车过线还是传感器问题。 */
    h2_show_sensor_bar();
}

/* ============================== 按键处理 ============================== */

/*
 * 等待启动按键按下并返回计时起点。
 *
 * 时序说明：
 * 1. 先等待按键释放，避免上一次测试结束后手指仍按着按键直接触发下一圈；
 * 2. 连续 H2_KEY_DEBOUNCE_SAMPLES 次采样都按下才确认；
 * 3. 计时起点取“第一次检测到按下”的节拍，而不是消抖完成后的节拍，
 *    这样 25 ms 消抖时间不会计入成绩；
 * 4. 等待期间每 100 ms 刷新一次传感器状态，方便现场检查八路接线。
 */
static uint32_t h2_wait_start_key(void)
{
    uint8_t stable_count = 0U;
    uint32_t press_tick = 0U;
    uint32_t last_display_tick = TimerA1_get_tick_100us();

    while (H2_START_KEY_PRESSED())
    {
        delay_ms(H2_KEY_SAMPLE_MS);
    }

    while (1)
    {
        if (H2_START_KEY_PRESSED())
        {
            if (stable_count == 0U)
            {
                press_tick = TimerA1_get_tick_100us();
            }
            stable_count++;

            if (stable_count >= H2_KEY_DEBOUNCE_SAMPLES)
            {
                return press_tick;
            }
        }
        else
        {
            /* 中途抖动一次就重新计时，避免把噪声当成启动。 */
            stable_count = 0U;
        }

        if ((TimerA1_get_tick_100us() - last_display_tick) >=
            H2_DISPLAY_PERIOD_TICKS)
        {
            last_display_tick = TimerA1_get_tick_100us();
            h2_show_sensor_bar();
        }

        delay_ms(H2_KEY_SAMPLE_MS);
    }
}

/* ============================== 单圈控制 ============================== */

/*
 * 执行一圈循迹并计时。
 *
 * 参数：
 *   start_tick         —— 按键确认瞬间的 TimerA1 节拍，作为 0 时刻；
 *   final_elapsed_ticks —— 输出：本次结束（或失败）时锁存的用时，单位 100 us。
 *
 * 返回：正常回到 A 点、超时或丢线失败，见 H2_RunResult。
 *
 * 状态推进：
 *   1) 车辆从起点横线上出发，此时传感器会同时压住多路，不能识别终点；
 *   2) 连续 10 个周期压黑路数不超过 2，判定已驶离起点，打开终点识别；
 *   3) 已驶离起点、时间超过 8 s、连续 2 个周期压黑路数达到阈值，
 *      判定到达 A 点，锁存时间并制动停车；
 *   4) 任意时刻超过 20 s 或连续丢线 1 s，立即制动停车。
 */
static H2_RunResult h2_run_one_lap(uint32_t start_tick,
                                   uint32_t *final_elapsed_ticks)
{
    H2_LinePid pid = {0, 0};
    uint8_t mask;
    uint8_t black_count;
    uint8_t start_clear_count = 0U;
    uint8_t finish_confirm_count = 0U;
    uint8_t lost_count = 0U;
    bool finish_armed = false;
    int error = 0;
    int last_error = 0;
    int correction;
    int base_pwm = H2_BASE_PWM;
    int left_pwm;
    int right_pwm;
    uint32_t last_display_ticks = 0U;

    OLED_Clear();

    while (1)
    {
        uint32_t loop_start_tick = TimerA1_get_tick_100us();
        uint32_t elapsed_ticks = loop_start_tick - start_tick;

        /* ---------- 安全兜底：超过 20 s 立即停车 ---------- */
        if (elapsed_ticks >= H2_SAFETY_TIMEOUT_TICKS)
        {
            *final_elapsed_ticks = TimerA1_get_tick_100us() - start_tick;
            h2_brake_and_stop();
            return H2_RUN_TIMEOUT;
        }

        /* ---------- 灰度采样（约 4 ms，阻塞） ---------- */
        mask = inf_read_mask_stable(H2_INF_SAMPLE_COUNT,
                                    H2_INF_HIT_THRESHOLD);
        black_count = inf_count_bits(mask);

        /* ---------- 起点脱离判定 ---------- */
        if (!finish_armed)
        {
            if (black_count <= H2_START_CLEAR_MAX_SENSORS)
            {
                start_clear_count++;
                if (start_clear_count >= H2_START_CLEAR_SAMPLES)
                {
                    finish_armed = true;
                }
            }
            else
            {
                start_clear_count = 0U;
            }
        }

        /* ---------- 终点识别 ---------- */
        if (finish_armed &&
            (elapsed_ticks >= H2_FINISH_MIN_TIME_TICKS) &&
            (black_count >= H2_FINISH_SENSOR_MIN))
        {
            finish_confirm_count++;
            if (finish_confirm_count >= H2_FINISH_CONFIRM_SAMPLES)
            {
                /* 锁存到达 A 点的时刻，制动过程不计入成绩。 */
                *final_elapsed_ticks = TimerA1_get_tick_100us() - start_tick;
                h2_brake_and_stop();
                return H2_RUN_FINISHED;
            }
        }
        else
        {
            finish_confirm_count = 0U;
        }

        /* ---------- 循迹控制 ---------- */
        if (black_count > 0U)
        {
            /* 见线：用加权偏差做 PID 差速，并按照偏差幅度调整基础速度。 */
            error = inf_error_from_mask(mask);
            last_error = error;
            lost_count = 0U;

            correction = h2_line_pid_step(&pid, error);
            base_pwm = h2_slew_toward(base_pwm,
                                      h2_curve_base_pwm(error),
                                      H2_BASE_SLEW_STEP);
        }
        else
        {
            /*
             * 全白丢线：保持最后一次有效偏差的方向低速找线，
             * 避免在弯道丢线后直行冲出场地。
             * 积分项同步衰减，重新见线时不会因为丢线期间没有误差积累
             * 而保留一个过大的历史修正量。
             */
            lost_count++;
            if (lost_count >= H2_LOST_STOP_SAMPLES)
            {
                *final_elapsed_ticks = TimerA1_get_tick_100us() - start_tick;
                h2_brake_and_stop();
                return H2_RUN_LINE_LOST;
            }

            base_pwm = H2_LOST_BASE_PWM;
            pid.integral = (pid.integral * 3) / 4;
            correction = (last_error >= 0) ? H2_LOST_CORRECTION :
                                             -H2_LOST_CORRECTION;
        }

        left_pwm = h2_clamp_int(base_pwm + correction,
                                H2_MIN_FORWARD_PWM,
                                H2_MAX_FORWARD_PWM);
        right_pwm = h2_clamp_int(base_pwm - correction,
                                 H2_MIN_FORWARD_PWM,
                                 H2_MAX_FORWARD_PWM);
        set_motor_speed(left_pwm, right_pwm);

        /* ---------- 显示刷新（100 ms 一次，避免占用控制周期） ---------- */
        if ((elapsed_ticks - last_display_ticks) >= H2_DISPLAY_PERIOD_TICKS)
        {
            last_display_ticks = elapsed_ticks;
            h2_show_running(elapsed_ticks, black_count, last_error);
        }

        /*
         * 以 TimerA1 节拍补齐 10 ms 控制周期：
         * 采样和显示已经消耗的时间会被扣除，而不是无条件再延时 10 ms。
         * WFE 在等待期间进入低功耗，TimerA1 的 100 us 中断负责唤醒。
         */
        while ((TimerA1_get_tick_100us() - loop_start_tick) <
               H2_CTRL_PERIOD_TICKS)
        {
            __WFE();
        }
    }
}

/* ============================ 电机引脚诊断 ============================ */

#if H2_MOTOR_PROBE

/*
 * 用 LED2 闪烁 phase 次，提示当前即将测试第几路 PWM。
 * 闪烁间隔 150 ms，并且刻意不用 LED1：LED1 由 TimerA1 每 500 ms 翻转一次，
 * 会和相位提示混在一起，LED2 在本程序中只用于诊断提示，不会互相干扰。
 */
static void h2_probe_show_phase(uint8_t phase)
{
    uint8_t i;

    for (i = 0U; i < phase; i++)
    {
        LED2(1);
        delay_ms(150U);
        LED2(0);
        delay_ms(150U);
    }
}

/*
 * 电机 PWM 通道映射诊断。
 *
 * 使用方法：
 * 1. 把车架空，保证左右轮都能自由转动；
 * 2. 确认底板左下角的 Motor-EN 开关在 ON、主电源已打开；
 * 3. 上电后 LED2 先闪 N 下，表示接下来测试第 N 路；
 * 4. 随后 2 秒内只驱动这一路 PWM，观察是左轮还是右轮、往哪个方向转；
 * 5. 每路之间停 1 秒，然后进入下一路，四路循环重复。
 *
 * 测试顺序（LED2 闪烁次数对应）：
 *   1 闪 → TIMG6 CC0，周期 100，比较值 50（约 50%）
 *   2 闪 → TIMG6 CC1，周期 100，比较值 50（约 50%）
 *   3 闪 → TIMA0 CC0，周期 1000，比较值 300（约 30%）
 *   4 闪 → TIMA0 CC1，周期 1000，比较值 300（约 30%）
 *
 * 进入测试前先把四路 PWM 全部清零，避免 SysConfig 上电默认的
 * TIMG6 50%/5% 占空比干扰判断；同时确保两个定时器都在计数。
 * 方向脚 PH1/PH2 先统一给高电平，观察到的转向直接记录，
 * 后续正式写电机驱动时再按结果决定是否取反。
 */
static void h2_motor_pin_probe(void)
{
    /* 四路 PWM 全部清零，先让两个轮子静止。 */
    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);

    /* 方向脚统一拉高，后续观察到的转向以这个电平为准。 */
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_PH1_PIN | MOTOR_PH2_PIN);

    /* 确保两个定时器都在计数，旧诊断代码可能停过 TIMG6。 */
    DL_TimerG_startCounter(TimerG6_PWM_INST);
    DL_TimerA_startCounter(TimerA0_PWM_INST);

    while (1)
    {
        /* 第 1 路：TIMG6 CC0。 */
        h2_probe_show_phase(1U);
        DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 50U, DL_TIMER_CC_0_INDEX);
        delay_ms(2000U);
        DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
        delay_ms(1000U);

        /* 第 2 路：TIMG6 CC1。 */
        h2_probe_show_phase(2U);
        DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 50U, DL_TIMER_CC_1_INDEX);
        delay_ms(2000U);
        DL_TimerG_setCaptureCompareValue(TimerG6_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
        delay_ms(1000U);

        /* 第 3 路：TIMA0 CC0。 */
        h2_probe_show_phase(3U);
        DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 300U, DL_TIMER_CC_0_INDEX);
        delay_ms(2000U);
        DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
        delay_ms(1000U);

        /* 第 4 路：TIMA0 CC1。 */
        h2_probe_show_phase(4U);
        DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 300U, DL_TIMER_CC_1_INDEX);
        delay_ms(2000U);
        DL_TimerA_setCaptureCompareValue(TimerA0_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
        delay_ms(1000U);
    }
}

#endif /* H2_MOTOR_PROBE */

/* ================================ 入口 ================================ */

int main(void)
{
    uint32_t start_tick;
    uint32_t final_elapsed_ticks;
    H2_RunResult result;

    SYSCFG_DL_init();
    uart0_init(115200U);
    TimerA1_init();
    OLED_Init();
    OLED_Clear();
    h2_stop_motor();

#if H2_MOTOR_PROBE
    /*
     * 诊断模式：上电直接进入 PWM 通道映射测试，不会返回。
     * 测完四路后把 H2_MOTOR_PROBE 改回 0，重新编译即可恢复循迹程序。
     */
    h2_motor_pin_probe();
#endif

#if H2_ENABLE_UART_DEBUG
    /* 上电先把调试信息发到串口，便于确认复位和参数版本。 */
    printf("H2 requirement 2 ready, limit 20.00s\r\n");
#endif

    while (1)
    {
        h2_show_idle_screen();

        /* 等待启动键，返回按键确认瞬间的时间戳。 */
        start_tick = h2_wait_start_key();

        result = h2_run_one_lap(start_tick, &final_elapsed_ticks);

        /* 结束画面保持显示最终用时，直到下一次按键启动。 */
        h2_show_result(result, final_elapsed_ticks);

#if H2_ENABLE_UART_DEBUG
        printf("H2 result=%s, ticks_100us=%lu, time=%lu.%02lus\r\n",
               (result == H2_RUN_FINISHED) ? "finished" :
               ((result == H2_RUN_TIMEOUT) ? "timeout" : "line_lost"),
               (unsigned long)final_elapsed_ticks,
               (unsigned long)(final_elapsed_ticks / 10000U),
               (unsigned long)((final_elapsed_ticks / 100U) % 100U));
#endif

        /*
         * 等待按键释放后再进入下一轮等待，防止按住按键时立即重新触发。
         * 释放后再停留 100 ms，给操作者留出抬手时间。
         */
        while (H2_START_KEY_PRESSED())
        {
            delay_ms(H2_KEY_SAMPLE_MS);
        }
        delay_ms(100U);
    }
}
