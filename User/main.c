/*------------------------------------------------------------------------------
 * Project: core_test
 * Target : TI MSPM0G3519
 * 
 * Contest task 1:
 * Put the car at point A. The car drives straight to point B, stops, then gives
 * a sound and light indication. The target distance is controlled by the two
 * wheel encoder counts. Straight sections use encoder feedback to trim PWM.
 *----------------------------------------------------------------------------*/
#include "ti_msp_dl_config.h"
#include "bsp.h"

/*
 * ============================================================================
 * 测试2、测试3、测试4可调参数说明
 * ============================================================================
 *
 * 单位约定
 * ----------
 * 1. 名称带 COUNT：编码器累计计数。距离不够时增大，超过目标时减小。
 * 2. 名称带 PWM：电机输出。增大通常更快，但惯性和冲线距离也会增大。
 * 3. 名称带 MD：毫度，1000 MD = 1°；正值为顺时针，负值为逆时针。
 * 4. 名称带 MS：毫秒；名称带 TICKS：TimerA1的100us节拍。
 * 5. LINE_KP：灰度比例系数。增大纠偏更强，过大会摇摆；减小更平滑，
 *    过小则可能压不住偏差而冲出轨迹。
 *
 * 调参原则
 * ----------
 * 1. 先调距离和转角，再调速度，最后调灰度Kp及丢线确认。
 * 2. 一次只改一个参数，并保存对应日志，避免无法判断变化来源。
 * 3. 降低基础PWM后，一般也应重新检查Kp；同一个Kp在低速下可能显得过强。
 *
 * 测试1、测试2：A→B直线
 * ----------
 *   两项测试共用陀螺仪直线闭环和灰度横线停车逻辑；编码器计数只提供
 *   最短识别距离与最大安全距离，不再作为测试一的正常停车目标。
 * TASK1_BASE_PWM
 *   A→B的基础速度，同时也是测试1速度。增大更快，减小更稳。
 * TASK1_MIN_PWM / TASK1_MAX_PWM
 *   所有limit_pwm输出的下限/上限，通常不作为第一调节项。
 * TASK1_LOOP_DELAY_MS
 *   编码器/陀螺仪直线控制周期。减小响应更快，但CPU和串口负担增加。
 * STRAIGHT_ENCODER_KP_DIV
 *   左右编码器直线差速修正的除数。减小=纠偏增强；增大=纠偏减弱。
 * STRAIGHT_ENCODER_MAX_CORRECTION
 *   编码器直线修正最大PWM，限制左右轮最大速度差。
 * STRAIGHT_LINE_MIN_COUNT
 *   A→B允许识别终点灰度线前的最短距离，增大可防止途中误判。
 * STRAIGHT_LINE_MAX_COUNT（当前等于TASK1_TARGET_COUNT）
 *   A→B找不到终点线时的安全最大距离；增大跑得更远才强制结束。
 * STRAIGHT_LINE_HIT_LIMIT
 *   终点灰度线连续命中次数；增大更抗干扰，但停车位置更靠后。
 *
 * 测试2：B→C灰度弧线
 * ----------
 * TASK2_ARC_BASE_PWM
 *   B→C和测试2 D→A的灰度循迹基础速度。增大更快，减小更稳。
 * TASK2_LINE_KP
 *   灰度P控制系数；增大纠偏更强，也更容易左右摇摆。
 * TASK2_ARC_MIN_COUNT
 *   到达此最低轮计数前不允许把丢线判作弧线终点。
 * TASK2_BC_LINE_LOST_LIMIT
 *   B→C终点连续丢线确认帧数；增大更不易误停，但会多冲出一段。
 * TASK2_LINE_SEEN_LIMIT
 *   起步后连续见线确认帧数；增大可防止瞬时灰度被当作有效轨迹。
 * TASK2_ARC_LOOP_DELAY_MS
 *   测试2灰度循环附加延时；增大动作变慢且丢线确认距离变长。
 * TASK2_BC_ACQUIRE_ERROR
 *   B→C刚进入弧线时使用的初始灰度误差，正负决定初始修正方向。
 * TASK23_SEARCH_HALF_ANGLE_MD
 *   尚未达到最低编码器距离便丢线时，左右摆头找线的单侧最大角度。
 * TASK23_SEARCH_TIMEOUT_MS
 *   摆头找线最长时间，超时后停止，防止车辆永久旋转。
 * TASK23_SEARCH_LINE_HIT_LIMIT
 *   找线时连续见线次数；增大更可靠，但重新起步稍慢。
 *
 * 测试2：C点处理与C→D直线
 * ----------
 * TASK_SENSOR_TO_AXLE_ALIGN_COUNT
 *   测试2中灰度传感器中心先到C后，轮轴中心继续前进的补偿距离。
 *   增大表示轮轴继续前进更远；测试3、4已使用各自独立参数。
 * TASK2_C_HEADING_COMPENSATE_MD
 *   C点回正到180°后的航向补偿；当前-10000表示逆时针补偿10°，最终航向170°。
 * TASK2_CD_BASE_PWM
 *   C→D直线速度，也用于C点轮轴前移补偿。
 * TASK2_CD_LINE_HIT_LIMIT
 *   C→D终点灰度线连续命中次数；增大更抗干扰但停车更晚。
 * TASK2_CD_LINE_MAX_COUNT
 *   C→D未检测到D点时的最大安全距离。
 *
 * 测试2：D→A灰度弧线
 * ----------
 * TASK3_REF_ARC_BASE_PWM / TASK3_REF_LINE_KP
 *   当前测试2 D→A直接复用测试3的纯灰度P控制速度与比例系数。
 * TASK3_REF_ARC_MIN_COUNT / TASK3_REF_LINE_LOST_LIMIT
 *   当前测试2 D→A也复用测试3最低距离和连续丢线结束条件。
 * TASK3_REF_LINE_SEEN_LIMIT / TASK3_REF_ARC_LOOP_DELAY_MS
 *   当前测试2 D→A复用测试3见线确认帧数与循环延时，不再使用测试2旧DA参数。
 *
 * 测试3：A→E、B→F直线与转角
 * ----------
 * TASK3_STAGE_HEADING_MD
 *   A→E相对局部0°的目标角，同时B→F使用其相反方向。
 *   增大绝对值会加大斜线角度；先确认正负方向再修改。
 * TASK3_AE_STAGE_COUNT
 *   A→E轮轴行驶距离。增大E点更靠近C，减小则提前停车。
 * TASK3_BF_STAGE_COUNT
 *   B→F轮轴行驶距离。增大F点更靠近D，减小则提前停车。
 * TASK3_ROUTE_STRAIGHT_PWM
 *   测试3 A→E、B点轮轴补偿和B→F的直线基础速度。
 * TASK3_STAGE_SETTLE_MS
 *   到E/F后的静止等待时间；增大可让车身和陀螺仪更稳定。
 * TASK3_B_AXLE_ALIGN_COUNT
 *   C→B灰度传感器到达B后，轮轴继续到B的前进补偿。
 * TASK3_LOCAL_ZERO_HEADING_MD
 *   C点和D点进入灰度弧线前的局部基准方向，当前为0°。
 * TASK3_B_RETURN_HEADING_MD
 *   C→B结束后B点回正目标，当前为180°。
 * TASK3_B_RIGHT_WHEEL_COMPENSATE_MD
 *   B点回正后仅驱动右轮的附加角度；绝对值增大，附加修正角增大。
 * TASK3_B_RIGHT_WHEEL_PWM
 *   B点单右轮补偿速度；增大更快，但更容易角度过冲。
 * TASK3_B_RIGHT_WHEEL_TOLERANCE_MD
 *   B点单右轮补偿允许误差；减小更精确，但可能耗时或抖动。
 * TASK3_B_RIGHT_WHEEL_TIMEOUT_TICKS
 *   B点单右轮补偿安全超时。
 *
 * 测试3：C→B、D→A纯灰度循迹
 * ----------
 * TASK3_REF_ARC_BASE_PWM
 *   两段弧线的基础速度，是调整测试3循迹快慢的首选参数。
 * TASK3_REF_LINE_KP
 *   两段弧线的灰度P系数；增大纠偏更猛，减小运行更平滑。
 * TASK3_REF_ARC_MIN_COUNT
 *   两轮达到此最低计数后，才允许连续丢线判定终点。
 * TASK3_REF_LINE_LOST_LIMIT
 *   终点连续丢线帧数；增大更抗中途丢线，但在B/A点冲出更多。
 * TASK3_REF_LINE_SEEN_LIMIT
 *   起步连续见线确认帧数；增大更抗误识别。
 * TASK3_REF_ARC_LOOP_DELAY_MS
 *   灰度控制循环延时；减小响应更快，增大运行可能出现卡顿且冲线更远。
 *   该循迹函数不使用陀螺仪修正、最低时间或摆头找线参数。
 *
 * 测试4：独立路线参数
 * ----------
 * TASK4_ROUTE_STAGE_HEADING_MD
 *   测试4每圈A→E斜线角度及B→F相反方向角度。
 * TASK4_ROUTE_AE_STAGE_COUNT_FIRST / TASK4_ROUTE_BF_STAGE_COUNT_FIRST
 *   测试4第一圈A→E、B→F的独立编码器距离。
 * TASK4_ROUTE_AE_STAGE_COUNT_LATER / TASK4_ROUTE_BF_STAGE_COUNT_LATER
 *   测试四第2～4圈A→E、B→F的编码器距离，可与第一圈分别调整。
 * TASK4_ROUTE_STAGE_SETTLE_MS
 *   测试4每圈到E/F后的静止等待时间。
 * TASK4_ROUTE_STRAIGHT_PWM
 *   测试4每圈A→E、B点轮轴补偿、B→F的直线速度。
 * TASK4_ROUTE_B_AXLE_ALIGN_COUNT
 *   测试4每圈灰度传感器到B后，轮轴继续到B的补偿距离。
 * TASK4_ROUTE_LOCAL_ZERO_HEADING_MD
 *   测试4每圈C点和D点进入灰度弧线前的局部基准方向。
 * TASK4_ROUTE_B_RETURN_HEADING_MD
 *   测试4每圈C→B结束后的B点回正目标。
 * TASK4_ROUTE_B_RIGHT_COMPENSATE_MD
 *   测试4每圈B点单右轮附加角度补偿。
 * TASK4_ROUTE_B_RIGHT_WHEEL_PWM
 *   测试4每圈B点单右轮补偿速度。
 * TASK4_ROUTE_B_RIGHT_TOLERANCE_MD
 *   测试4每圈B点单右轮补偿允许角度误差。
 * TASK4_ROUTE_B_RIGHT_TIMEOUT_TICKS
 *   测试4每圈B点单右轮补偿安全超时。
 * TASK4_ROUTE_A_RETURN_HEADING_MD
 *   测试4每圈D→A循迹结束后的A点回正目标；当前相对D点局部方向为180°。
 *   增大或减小该值可修正下一圈从A出发前的车头角度。
 * TASK4_ROUTE_A_FORWARD_COUNT
 *   测试4第1、2、3圈回到A后，为下一圈准备的短距离直行计数。
 *   增大则下一圈起点前移更多；第4圈结束后不会执行该动作。
 *
 * 测试4：独立灰度循迹参数
 * ----------
 * TASK4_REF_ARC_BASE_PWM
 *   测试4每圈C→B、D→A基础速度。
 * TASK4_REF_LINE_KP
 *   测试4每圈灰度P系数；增大纠偏更强，也更容易摇摆。
 * TASK4_REF_ARC_MIN_COUNT
 *   测试4每圈达到该最低计数后才允许按连续丢线结束。
 * TASK4_REF_LINE_LOST_LIMIT
 *   测试4每圈终点连续丢线帧数；增大更抗误停，但冲出更多。
 * TASK4_REF_LINE_SEEN_LIMIT
 *   测试4每圈起步连续见线确认帧数。
 * TASK4_REF_ARC_LOOP_DELAY_MS
 *   测试4每圈灰度控制循环延时。
 *   这些数值当前与TASK3_REF_*一致，但两组宏没有引用关系，可以独立修改。
 *
 * 测试2、测试3、测试4共用的陀螺仪底层直线/转向控制
 * ----------
 * TASK4_TURN_TOLERANCE_MD
 *   原地转向允许角度误差；减小更准，但更容易在目标附近反复修正。
 * TASK4_TURN_STABLE_SAMPLES
 *   连续落入角度误差范围的确认次数；增大更稳但停止更慢。
 * TASK4_TURN_MIN_PWM / TASK4_TURN_MAX_PWM
 *   原地转向最小/最大PWM。最小值过低可能电机不转，过高容易过冲。
 * TASK4_TURN_KP_DIV
 *   转向比例控制除数；减小转向更强，增大转向更柔和。
 * TASK4_TURN_LOOP_DELAY_MS
 *   转向控制周期。
 * TASK4_TURN_TIMEOUT_TICKS
 *   转向安全超时。
 * TASK4_STRAIGHT_KP_DIV
 *   陀螺仪直行修正除数；减小航向纠偏更强，过小会左右摆。
 * TASK4_STRAIGHT_MAX_CORRECTION
 *   陀螺仪直行允许的最大左右轮PWM差。
 * TASK23_HEADING_AB_MD / TASK23_HEADING_CD_MD
 *   测试2 AB/CD边基准航向，通常保持0°/180°，不要作为位置微调参数。
 *
 * 当前测试3、测试4不生效的旧参数
 * ----------
 * TASK3_STRAIGHT_COUNT、TASK3_CB_TRACK_LOOP_DELAY_MS、
 * TASK3_DA_TRACK_LOOP_DELAY_MS、TASK3_DA_LINE_LOST_LIMIT、
 * TASK3_ARC_MIN_COUNT、TASK3_ARC_EXTRA_TURN_COUNT、
 * TASK3_ARC_END_TURN_COUNT、TASK3_C_TURN_COUNT、TASK3_D_TURN_COUNT、
 * TASK3_EXTRA_TURN_PWM、TASK3_POINT_FORWARD_MS、TASK3_DA_ENTRY_RIGHT_TURN_MD、
 * TASK3_DA_END_MIN_COUNT、TASK3_DA_TRACK_MIN_TIME_MS。
 * 以及TASK34_AC_LINE_MIN_COUNT、TASK34_BD_LINE_MIN_COUNT、TASK34_CB_*、
 * TASK34_DA_*、TASK4_DA_TRACK_MIN_TIME_MS、TASK4_A_LEFT_COMPENSATE_MD。
 * 这些是旧版测试3/4流程遗留量，当前task34_drive_route_internal没有使用；
 * 调整它们不会改变当前测试3或测试4运行结果。
 * ============================================================================
 */

#if defined(__clang__)
#define NOINLINE __attribute__((noinline))
#else
#define NOINLINE
#endif

#define TASK1_STOP_PWM 1
#define TASK1_BASE_PWM 260
#define MOTOR_TEST_PWM 300
#define TASK1_MIN_PWM 140
#define TASK1_MAX_PWM 520
#define TASK1_LOOP_DELAY_MS 5
#define STRAIGHT_ENCODER_KP_DIV 80
#define STRAIGHT_ENCODER_MAX_CORRECTION 45
#define STRAIGHT_LEFT_QEI_INDEX 1U
#define STRAIGHT_RIGHT_QEI_INDEX 0U

/*
 * 测试一、测试二的A→B现在共用灰度横线停车逻辑。该编码器计数不再是正常停车点，
 * 仅作为灰度传感器未识别到B点横线时的最大安全行驶距离；正常情况下车辆会在
 * 连续识别到B点黑线后立即停车。
 */
#define TASK1_TARGET_COUNT 280000U

/* 测试一至四的通用编码器直行动作在安全距离末端使用该单级减速参数。 */
#define TASK1_SLOWDOWN_COUNT 2500U
#define TASK1_SLOWDOWN_PWM 190
#define TASK1_SIGNAL_HZ 2000
#define TASK1_SIGNAL_DUTY 50
#define TASK1_SIGNAL_MS 180

#define TASK3_STRAIGHT_COUNT 400000U
#define TASK34_STRAIGHT_PWM 210
/*
 * 任务3根据传感器中心位于轮轴前方154mm重新定义过渡点：
 * E距C 15.4cm，F距D 15.4cm。轮轴停在E/F并转向CD方向后，灰度中心
 * 恰好落到C/D点，因此不再继续前进到C/D。
 * 理论角atan(80/84.6)=43.399°。A→E和B→F均已按实车结果独立标定，
 * 实际距离直接由下方TASK3_AE_STAGE_COUNT与TASK3_BF_STAGE_COUNT决定。
 */
#define TASK3_STAGE_HEADING_MD              43399L
#define TASK3_AE_STAGE_COUNT                293200U
#define TASK3_BF_STAGE_COUNT                244100U
#define TASK3_STAGE_SETTLE_MS               200U
#define TASK3_ROUTE_STRAIGHT_PWM            210
#define TASK3_B_AXLE_ALIGN_COUNT            40000U
#define TASK3_LOCAL_ZERO_HEADING_MD         0L
#define TASK3_B_RETURN_HEADING_MD           180000L
#define TASK3_B_RIGHT_WHEEL_COMPENSATE_MD   (-1000L)
#define TASK3_B_RIGHT_WHEEL_PWM             100
#define TASK3_B_RIGHT_WHEEL_TOLERANCE_MD    300L
#define TASK3_B_RIGHT_WHEEL_TIMEOUT_TICKS   15000U

/*
 * 测试4独立路线参数，不通过宏别名引用测试3。
 * 第一圈和第2～4圈的A→E、B→F距离也分别保存；当前两组先设为相同值。
 */
#define TASK4_ROUTE_STAGE_HEADING_MD        43399L
#define TASK4_ROUTE_AE_STAGE_COUNT_FIRST    285160U
#define TASK4_ROUTE_BF_STAGE_COUNT_FIRST    236060U
#define TASK4_ROUTE_AE_STAGE_COUNT_LATER    269080U
#define TASK4_ROUTE_BF_STAGE_COUNT_LATER    244100U
#define TASK4_ROUTE_STAGE_SETTLE_MS         200U
#define TASK4_ROUTE_STRAIGHT_PWM            210
#define TASK4_ROUTE_B_AXLE_ALIGN_COUNT      40000U
#define TASK4_ROUTE_LOCAL_ZERO_HEADING_MD   0L
#define TASK4_ROUTE_B_RETURN_HEADING_MD     180000L
#define TASK4_ROUTE_B_RIGHT_COMPENSATE_MD   (-1000L)
#define TASK4_ROUTE_B_RIGHT_WHEEL_PWM       100
#define TASK4_ROUTE_B_RIGHT_TOLERANCE_MD    300L
#define TASK4_ROUTE_B_RIGHT_TIMEOUT_TICKS   15000U
#define TASK4_ROUTE_A_RETURN_HEADING_MD     180000L
#define TASK4_ROUTE_A_FORWARD_COUNT         20000U
#define TASK3_CB_TRACK_LOOP_DELAY_MS         20U
#define TASK3_DA_TRACK_LOOP_DELAY_MS         40U
#define TASK3_DA_LINE_LOST_LIMIT             3U
#define STRAIGHT_LINE_MIN_COUNT 80000U
/*
 * 第三、第四题的A→C与B→D都是约128cm对角线。按任务1中100cm约280000计数估算，
 * 对角线理论值约358000。多次实测A→C有效计数主要落在292000~307000，而
 * 282260处仍出现过早识别，因此A→C门槛提高到290000。加入B点轮轴补偿后，
 * B→D正常一次在290767识别，而251786几乎刚过旧门槛，判为途中干扰；因此
 * B→D门槛提高到280000，仍给实际终点保留约10%的余量。
 * 两个参数均独立于任务2的直线路段。
 */
#define TASK34_AC_LINE_MIN_COUNT 290000U
#define TASK34_BD_LINE_MIN_COUNT 280000U
/* 任务2直线找线的最大距离直接复用任务1目标，避免维护等价的中间别名。 */
#define STRAIGHT_LINE_MAX_COUNT TASK1_TARGET_COUNT
#define STRAIGHT_LINE_HIT_LIMIT 4
/* 测试2的C→D单独降速并缩短横线确认窗口，提高D点识别灵敏度。 */
#define TASK2_CD_BASE_PWM 200
#define TASK2_CD_LINE_HIT_LIMIT 2U
#define TASK2_CD_LINE_MAX_COUNT 350000U
#define TASK_DIAGNOSTIC_COUNT_STEP 50000U
/* 恢复此前实车使用的低速循迹参数：基础PWM 150，比例修正系数19。 */
#define TASK2_ARC_BASE_PWM 150
/* 降速后把两轮最低循迹距离由31000提高到80000，阻止弧线前段丢线时提前退出。 */
#define TASK2_ARC_MIN_COUNT 80000U
#define TASK3_ARC_MIN_COUNT 80000U
#define TASK34_CB_END_MIN_COUNT             242000U
#define TASK34_CB_FORCE_END_COUNT           0U
#define TASK34_CB_LINE_LOST_CONFIRM         4U
#define TASK_SENSOR_TO_AXLE_ALIGN_COUNT     60000U
#define TASK34_B_AXLE_ALIGN_COUNT           30000U
#define TASK2_BC_ACQUIRE_ERROR              2
#define TASK34_CB_ACQUIRE_ERROR             (-2)
#define TASK34_CB_RECOVERY_LOST_LIMIT        3U
#define TASK34_CB_BASE_PWM                  145
#define TASK34_CB_LINE_KP                   7
/*
 * C→B是方向固定的左圆弧：右轮基础PWM始终高于左轮。灰度只在该基础弧线
 * 上微调，最大正修正只能把两轮拉到同速，不能让车辆持续反向右转。
 */
#define TASK34_CB_LEFT_TURN_BIAS             8
#define TASK34_CB_MAX_GRAY_CORRECTION        28
#define TASK34_CB_CORRECTION_SLEW_STEP       4
#define TASK34_CB_LOST_GRAY_CORRECTION       (-4)
#define TASK34_CB_SAFETY_COUNT               380000U
#define TASK2_LINE_KP 19
#define TASK2_LINE_LOST_VALUE 100
/*
 * 循迹结束采用“连续丢线次数”消抖：B 键的 B→C 左转需要更早结束，
 * 因此使用较小阈值；其他弧线保留较大阈值，避免传感器瞬时漏检导致误停车。
 * 循迹循环周期约为 TASK2_ARC_LOOP_DELAY_MS，减小阈值会缩短离开黑线后的转动距离。
 */
/*
 * 测试二B→C除连续丢线帧数外，还要求全白状态持续600ms。细黑线落在相邻探头
 * 之间造成短暂全白时，小车继续循迹并有机会重新压线；重新见线会立即清零计时。
 */
#define TASK2_BC_LINE_LOST_LIMIT 16U
#define TASK2_BC_LINE_LOST_DELAY_MS 600U
#define TASK_OTHER_LINE_LOST_LIMIT 20U
#define TASK2_LINE_SEEN_LIMIT 5U
/*
 * 原始测试2退出条件按连续丢线窗口计数，测试2附加延时使用20ms。
 * 当前测试3、4不使用TASK_OTHER_ARC_LOOP_DELAY_MS，而分别使用各自的
 * TASK3_REF_ARC_LOOP_DELAY_MS与TASK4_REF_ARC_LOOP_DELAY_MS。
 */
#define TASK2_ARC_LOOP_DELAY_MS 20U
#define TASK_OTHER_ARC_LOOP_DELAY_MS 13U
#define TASK2_ARC_EXTRA_COUNT 0U
/*
 * 测试3独立灰度循迹参数。修改这些值不会影响测试2或测试4。
 */
#define TASK3_REF_ARC_BASE_PWM              120
#define TASK3_REF_ARC_MIN_COUNT             36000U
#define TASK3_REF_LINE_KP                   22
#define TASK3_REF_LINE_LOST_LIMIT           25U
#define TASK3_REF_LINE_SEEN_LIMIT           5U
#define TASK3_REF_ARC_LOOP_DELAY_MS          2U

/* 测试4独立灰度参数：当前与测试3数值相同，但后续可单独调整。 */
#define TASK4_REF_ARC_BASE_PWM              120
#define TASK4_REF_ARC_MIN_COUNT             36000U
#define TASK4_REF_LINE_KP                   20
#define TASK4_REF_LINE_LOST_LIMIT           30U
#define TASK4_REF_LINE_SEEN_LIMIT           5U
#define TASK4_REF_ARC_LOOP_DELAY_MS          2U
/*
 * 测试四C→B的终点丢线延时确认时间。细黑线可能恰好落在相邻灰度探头之间，
 * 此时8路输入会短暂全部为白；只有全白状态连续保持到该时间，并且同时满足
 * 最低编码器计数和连续丢线帧数，才允许判定到达B点。单位为毫秒。
 */
#define TASK4_CB_LINE_LOST_DELAY_MS         600U
/*
 * 测试四从E/F转回局部0°后，以低速直行主动寻找C/D点黑线。连续命中次数用于
 * 滤除细黑线和数字输入抖动；最大编码器计数与超时时间共同限制找线距离，防止
 * 灰度传感器失效或车轮堵转时持续行驶。超时时间单位为毫秒。
 */
#define TASK4_POINT_GRAY_HIT_LIMIT            2U
#define TASK4_POINT_GRAY_SEEK_PWM             150
#define TASK4_POINT_GRAY_MAX_COUNT         100000U
#define TASK4_POINT_GRAY_TIMEOUT_MS         5000U
#define TASK4_POINT_GRAY_TIMEOUT_TICKS \
    (TASK4_POINT_GRAY_TIMEOUT_MS * 10U)
#define TASK3_ARC_EXTRA_TURN_COUNT 25000U
/* C 键到达 B 点后的左转编码器目标值；增大该值可加大进入 B→D 前的左转角度。 */
#define TASK3_ARC_END_TURN_COUNT 25000U
#define TASK3_C_TURN_COUNT 25000U
#define TASK3_D_TURN_COUNT 25000U
#define TASK3_EXTRA_TURN_PWM 220
#define TASK3_POINT_FORWARD_MS 100U
/* 测试3到达D点后原地顺时针右转30°，再直接进入D→A纯灰度循迹。 */
#define TASK3_DA_ENTRY_RIGHT_TURN_MD 30000L
/*
 * 以下TASK34_DA_*为旧版测试4的D→A专用PD状态机参数。
 * 当前测试3、4均使用各自的TASK3_REF_* / TASK4_REF_*纯灰度P控制，
 * 因此修改这组旧参数不会影响当前测试3或测试4。
 */
#define TASK34_DA_BASE_PWM                  140
#define TASK34_DA_APPROACH_PWM              140
#define TASK34_DA_LINE_KP                   17
#define TASK34_DA_LINE_KD                   3
#define TASK34_DA_MAX_CORRECTION            90
#define TASK34_DA_END_MIN_COUNT             200000U
#define TASK34_A_CENTER_ERROR_MAX           5
#define TASK34_A_CENTER_RECOVERY_LIMIT       1U
#define TASK34_A_EDGE_RECOVERY_LIMIT         1U
#define TASK34_A_LOST_CONFIRM_COUNT         4U
#define TASK34_DA_PRELOST_LOG_MIN_TIME_MS    3000U
#define TASK34_DA_PRELOST_LOG_MIN_TIME_TICKS \
    (TASK34_DA_PRELOST_LOG_MIN_TIME_MS * 10U)
#define TASK34_A_SETTLE_MS                  120U
#define TASK34_DA_MAX_COUNT                 600000U
#define TASK34_DA_MAX_TIME_MS               11000U
#define TASK34_DA_MAX_TIME_TICKS            (TASK34_DA_MAX_TIME_MS * 10U)
#define TASK34_DA_STATE_TRACKING            0U
#define TASK34_DA_STATE_END_WINDOW          1U
#define TASK34_DA_STATE_LINE_RECOVERY       2U
#define TASK34_DA_STATE_END_CONFIRM         3U
#define TASK34_DA_WINDOW_VALID_COUNT        2U
#define TASK34_DA_LOST_SEARCH_CORRECTION    60
#define TASK34_DA_LOST_GRACE_COUNT          3U
#define TASK34_DA_INF_THRESHOLD             5U
/* 测试4每圈回到A并对准0°后，再向左补偿5°；负角表示逆时针左转。 */
#define TASK4_A_LEFT_COMPENSATE_MD (-5000L)

/*
 * 测试2的B→C和D→A已取消最低运行时间门槛；中途丢线保护改由最低编码器
 * 距离承担。B→C在达到最低距离前持续丢线时仍会执行±10°摆头找线。
 */
/* 任务3回到A时独立使用更早的终点窗口，避免越过A后才进入强差速找线。 */
#define TASK3_DA_END_MIN_COUNT              180000U
#define TASK3_DA_TRACK_MIN_TIME_MS          (4000U)
#define TASK3_DA_TRACK_MIN_TIME_TICKS       (TASK3_DA_TRACK_MIN_TIME_MS * 10U)
/* 测试4的D→A至少运行4.5s并达到独立最低里程，才允许进入A点终点窗口。 */
#define TASK4_DA_TRACK_MIN_TIME_MS           (4500U)
#define TASK4_DA_TRACK_MIN_TIME_TICKS        (TASK4_DA_TRACK_MIN_TIME_MS * 10U)
#define TASK23_SEARCH_HALF_ANGLE_MD         (10000L)
#define TASK23_SEARCH_TIMEOUT_MS            (10000U)
#define TASK23_SEARCH_TIMEOUT_TICKS         (TASK23_SEARCH_TIMEOUT_MS * 10U)
#define TASK23_SEARCH_LINE_HIT_LIMIT        (3U)

/*
 * AB=100、AD=80，对角线相对纵向边的夹角 atan(80/100)=38.6598°。
 * 初始-Y定义为0°、顺时针为正：C→D切线为180°，A→C为38.66°，B→D为141.34°。
 */
#define TASK23_DIAGONAL_ANGLE_MD            (38660L)
#define TASK23_HEADING_AB_MD                (0L)
#define TASK23_HEADING_AC_MD                (TASK23_DIAGONAL_ANGLE_MD)
#define TASK23_HEADING_CD_MD                (180000L)
/*
 * 测试2在C点先回正到180°，再执行-10°补偿，最终以170°完成轮轴前移和C→D直线。
 * 负值表示从车顶向下观察逆时针左转；调整该值只影响测试2的C点出口与C→D航向。
 */
#define TASK2_C_HEADING_COMPENSATE_MD       (-5000L)
#define TASK2_C_EXIT_HEADING_MD \
    (TASK23_HEADING_CD_MD + TASK2_C_HEADING_COMPENSATE_MD)
/*
 * 测试3的B→D以理论38.66°为基准增加实车补偿。当前-1000表示减小1°，即左转
 * 37.66°；以后每增加1000就是把左转增大1°，每减少1000就是把左转减小1°。
 */
#define TASK3_BD_TURN_COMPENSATE_MD         (-1000L)
#define TASK3_BD_TURN_ANGLE_MD              \
    (TASK23_DIAGONAL_ANGLE_MD + TASK3_BD_TURN_COMPENSATE_MD)
#define TASK23_HEADING_BD_MD                (180000L - TASK3_BD_TURN_ANGLE_MD)

/*
 * 第四问陀螺仪航向约定：小车上电时的前进方向是全局 Y 轴负方向，定义为 0°；
 * 从车顶向下观察，顺时针（右转）为正角度。GY-6050 正面朝上安装时，芯片 Z 轴
 * 正方向对应逆时针，因此积分符号取 -1；若实物倒置，只需把该常量改为 +1。
 */
#define TASK4_GYRO_Z_CLOCKWISE_SIGN       (-1)
#define TASK4_GYRO_CALIBRATION_SAMPLES    (400U)
#define TASK4_GYRO_CALIBRATION_PERIOD_MS  (2U)
#define TASK4_GYRO_SAMPLE_MAX_TICKS       (300U)
#define TASK4_GYRO_SCALE_DIVISOR           (655LL)

#define TASK4_TURN_TOLERANCE_MD            (1500L)
#define TASK4_TURN_STABLE_SAMPLES          (4U)
#define TASK4_TURN_MIN_PWM                 (150)
#define TASK4_TURN_MAX_PWM                 (260)
#define TASK4_TURN_KP_DIV                  (300L)
#define TASK4_TURN_LOOP_DELAY_MS           (8U)
#define TASK4_TURN_TIMEOUT_TICKS           (50000UL)

/* 直线段以陀螺航向误差修正左右轮 PWM，编码器仍负责距离上限和点位搜索窗口。 */
#define TASK4_STRAIGHT_KP_DIV              (150L)
#define TASK4_STRAIGHT_MAX_CORRECTION      (70)

/*
 * 弧线循迹退出诊断信息：记录退出原因和当时两轮较小的累计编码器计数。
 * 该信息保留给前三问和调试使用，第四问的弧线同时更新陀螺仪累计航向。
 */
#define ARC_EXIT_NONE 0U
#define ARC_EXIT_LINE_LOST 1U
#define ARC_EXIT_MAX_COUNT 2U

static uint32_t qei_cnt[2] = {0U, 0U};
static uint32_t qei_base[2] = {0U, 0U};
static uint32_t qei_prev[2] = {0U, 0U};
static uint32_t qei_total[2] = {0U, 0U};
static uint32_t arc_exit_count = 0U;
static uint8_t arc_exit_reason = ARC_EXIT_NONE;

/*
 * 第四问航向状态。积分分子保留 65.5 LSB/(°/s) 换算产生的小数余量；TimerA1
 * 每个 tick 为 100 us，因此 raw_delta*tick/655 的结果正好是毫度。
 */
static int32_t task4_gyro_z_bias_raw = 0;
static int64_t task4_yaw_integral_numerator = 0LL;
static uint32_t task4_gyro_last_tick = 0U;
static uint8_t task4_gyro_fault = 0U;

/*
 * 测试2、3现在也会在流程中报告陀螺仪初始化、采样或转向故障，而统一的提示函数
 * 实现在任务4流程附近。此处只做文件内部前向声明，避免调用位置受函数排列顺序限制。
 */
static void task4_show_gyro_error(void);

static int limit_pwm(int pwm)
{
    if (pwm > TASK1_MAX_PWM)
    {
        return TASK1_MAX_PWM;
    }

    if (pwm < -TASK1_MAX_PWM)
    {
        return -TASK1_MAX_PWM;
    }

    if ((pwm > 0) && (pwm < TASK1_MIN_PWM))
    {
        return TASK1_MIN_PWM;
    }

    if ((pwm < 0) && (pwm > -TASK1_MIN_PWM))
    {
        return -TASK1_MIN_PWM;
    }

    return pwm;
}

/*
 * 循迹必须保留连续差速，不能把较慢一侧强制抬到普通运动的140最小PWM，否则
 * 小车在弧线上转向不足并更早丢线。该专用限幅只约束硬件最大值，允许较慢一侧
 * 按150基础速度和19比例系数计算出的低PWM运行。
 */
static int limit_arc_pwm(int pwm)
{
    if (pwm > TASK1_MAX_PWM)
    {
        return TASK1_MAX_PWM;
    }

    if (pwm < -TASK1_MAX_PWM)
    {
        return -TASK1_MAX_PWM;
    }

    return pwm;
}

static int32_t qei_signed_delta(uint32_t now, uint32_t base)
{
    int32_t delta = (int32_t)now - (int32_t)base;

    if (delta > 32767)
    {
        delta -= 65536;
    }
    else if (delta < -32768)
    {
        delta += 65536;
    }

    return delta;
}

static uint32_t qei_abs_delta(uint32_t now, uint32_t base)
{
    int32_t delta = qei_signed_delta(now, base);

    return (delta >= 0) ? (uint32_t)delta : (uint32_t)(-delta);
}

static void qei_zero_soft(void)
{
    get_motor_qei_cnt(qei_base);
    qei_prev[0] = qei_base[0];
    qei_prev[1] = qei_base[1];
    qei_total[0] = 0U;
    qei_total[1] = 0U;
}

static void show_task1_status(uint32_t m1_delta, uint32_t m2_delta)
{
    OLED_ShowNum(64, 0, m1_delta, 6, 16);
    OLED_ShowNum(64, 2, m2_delta, 6, 16);
}

static void stop_motor(void)
{
    set_motor_speed(TASK1_STOP_PWM, TASK1_STOP_PWM);
}

/* 将任意航向误差折叠到 [-180°, 180°]，保证转向总是选择较短方向。 */
static int32_t task4_normalize_angle_md(int32_t angle_md)
{
    while (angle_md > 180000L)
    {
        angle_md -= 360000L;
    }

    while (angle_md < -180000L)
    {
        angle_md += 360000L;
    }

    return angle_md;
}

/* 根据积分分子换算当前航向，返回值采用“0°=-Y、顺时针为正”的毫度坐标。 */
static int32_t task4_get_heading_md(void)
{
    return task4_normalize_angle_md(
        (int32_t)(task4_yaw_integral_numerator / TASK4_GYRO_SCALE_DIVISOR));
}

/*
 * 丢弃停车提示等非控制阶段经过的时间，只同步时间戳而不积分。若把一次新的速率
 * 样本乘到很长的旧时间间隔上，会产生虚假角度，因此每段运动开始前都调用本函数。
 */
static void task4_gyro_sync_time(void)
{
    task4_gyro_last_tick = TimerA1_get_tick_100us();
}

/*
 * 读取一次 Z 轴角速度并按真实 100 us 节拍积分。正常控制周期远小于 30 ms；若因
 * OLED、红外采样或中断阻塞造成更长间隔，则丢弃该段积分，防止单点速率外推过远。
 */
static bool task4_gyro_update(void)
{
    int16_t gyro_z_raw;
    int32_t corrected_raw;
    uint32_t now_tick;
    uint32_t elapsed_ticks;

    if (!mpu6050_read_gyro_z_raw(&gyro_z_raw))
    {
        task4_gyro_fault = 1U;
        return false;
    }

    now_tick = TimerA1_get_tick_100us();
    elapsed_ticks = now_tick - task4_gyro_last_tick;
    task4_gyro_last_tick = now_tick;

    if (elapsed_ticks > TASK4_GYRO_SAMPLE_MAX_TICKS)
    {
        return true;
    }

    corrected_raw = (int32_t)gyro_z_raw - task4_gyro_z_bias_raw;
    task4_yaw_integral_numerator +=
        (int64_t)TASK4_GYRO_Z_CLOCKWISE_SIGN *
        (int64_t)corrected_raw * (int64_t)elapsed_ticks;
    return true;
}

/*
 * 初始化第四问航向：校准时电机保持停止，完成后把当前车头方向置为 0°，即用户
 * 指定的全局 -Y。MPU6050 没有磁力计，运行中只能保持相对该启动方向的航向。
 */
static bool task4_gyro_prepare(void)
{
    stop_motor();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MPU6050 init");

    if (!mpu6050_init())
    {
        return false;
    }

    OLED_ShowString(0, 2, (u8 *)"Keep still...");
    if (!mpu6050_calibrate_gyro_z(TASK4_GYRO_CALIBRATION_SAMPLES,
                                  TASK4_GYRO_CALIBRATION_PERIOD_MS,
                                  &task4_gyro_z_bias_raw))
    {
        return false;
    }

    task4_yaw_integral_numerator = 0LL;
    task4_gyro_fault = 0U;
    task4_gyro_sync_time();
    return true;
}

/*
 * 按绝对航向原地闭环转向。大误差时提高 PWM，接近目标时降速，并要求连续多个
 * 样本进入容差带后才完成，以抑制车轮惯性造成的越界；超时或 I2C 错误立即停车。
 */
static bool task4_gyro_turn_to(int32_t target_heading_md)
{
    int32_t error_md;
    int32_t abs_error_md;
    int turn_pwm;
    uint8_t stable_samples = 0U;
    uint32_t start_tick = TimerA1_get_tick_100us();

    target_heading_md = task4_normalize_angle_md(target_heading_md);
    task4_gyro_sync_time();

    while ((TimerA1_get_tick_100us() - start_tick) < TASK4_TURN_TIMEOUT_TICKS)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        error_md = task4_normalize_angle_md(target_heading_md -
                                             task4_get_heading_md());
        abs_error_md = (error_md >= 0L) ? error_md : -error_md;

        if (abs_error_md <= TASK4_TURN_TOLERANCE_MD)
        {
            stop_motor();
            stable_samples++;
            if (stable_samples >= TASK4_TURN_STABLE_SAMPLES)
            {
                return true;
            }
        }
        else
        {
            stable_samples = 0U;
            turn_pwm = (int)(abs_error_md / TASK4_TURN_KP_DIV);
            if (turn_pwm < TASK4_TURN_MIN_PWM)
            {
                turn_pwm = TASK4_TURN_MIN_PWM;
            }
            else if (turn_pwm > TASK4_TURN_MAX_PWM)
            {
                turn_pwm = TASK4_TURN_MAX_PWM;
            }

            /* 正误差表示需要顺时针右转：左轮前进、右轮后退。 */
            if (error_md > 0L)
            {
                set_motor_speed(turn_pwm, -turn_pwm);
            }
            else
            {
                set_motor_speed(-turn_pwm, turn_pwm);
            }
        }

        delay_ms(TASK4_TURN_LOOP_DELAY_MS);
    }

    stop_motor();
    task4_gyro_fault = 1U;
    return false;
}

/* 以当前累计航向为基准执行相对转向，正角度右转、负角度左转。 */
static bool task4_gyro_turn_relative(int32_t delta_heading_md)
{
    return task4_gyro_turn_to(task4_get_heading_md() + delta_heading_md);
}

/*
 * 测试3、4在B点使用右轮单轮向前完成左偏补偿。目标角以进入函数时的真实航向为
 * 基准，动作期间持续积分Z轴角速度；进入允许误差或越过目标即停车。若右轮卡住
 * 或陀螺仪方向异常，超时后置故障并停车，防止单轮持续驱动。
 */
static bool compensate_with_right_wheel(int32_t compensate_md,
                                        int wheel_pwm,
                                        int32_t tolerance_md,
                                        uint32_t timeout_ticks)
{
    int32_t target_heading_md = task4_normalize_angle_md(
        task4_get_heading_md() + compensate_md);
    int32_t error_md;
    uint32_t start_tick = TimerA1_get_tick_100us();

    task4_gyro_sync_time();
    while ((TimerA1_get_tick_100us() - start_tick) <
           timeout_ticks)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        error_md = task4_normalize_angle_md(target_heading_md -
                                             task4_get_heading_md());
        if (error_md >= -tolerance_md)
        {
            stop_motor();
            return true;
        }

        /* 左轮停止、右轮向前，使车头逆时针左偏。 */
        set_motor_speed(TASK1_STOP_PWM, wheel_pwm);
        delay_ms(TASK4_TURN_LOOP_DELAY_MS);
    }

    stop_motor();
    task4_gyro_fault = 1U;
    return false;
}

static void set_straight_speed_closed_loop(int base_pwm)
{
    int32_t left_count = (int32_t)qei_total[STRAIGHT_LEFT_QEI_INDEX];
    int32_t right_count = (int32_t)qei_total[STRAIGHT_RIGHT_QEI_INDEX];
    int correction = (int)((right_count - left_count) / STRAIGHT_ENCODER_KP_DIV);

    if (correction > STRAIGHT_ENCODER_MAX_CORRECTION)
    {
        correction = STRAIGHT_ENCODER_MAX_CORRECTION;
    }
    else if (correction < -STRAIGHT_ENCODER_MAX_CORRECTION)
    {
        correction = -STRAIGHT_ENCODER_MAX_CORRECTION;
    }

    set_motor_speed(limit_pwm(base_pwm + correction),
                    limit_pwm(base_pwm - correction));
}

static void point_signal_once(void)
{
    LED1_toggle;
    LED2_toggle;
    set_RGB(0x00, 0xff, 0x00);
    set_led_buzzer(TASK1_SIGNAL_HZ, TASK1_SIGNAL_DUTY, TASK1_SIGNAL_DUTY);
    delay_ms(TASK1_SIGNAL_MS);

    set_RGB(0x00, 0x00, 0x00);
    set_led_buzzer(TASK1_SIGNAL_HZ, 100, 100);
}

static void buzzer_off(void)
{
    set_led_buzzer(TASK1_SIGNAL_HZ, 100, 100);
}

#define INF_COLLECT_SAMPLE_COUNT 12U
#define INF_COLLECT_THRESHOLD 7U

static void collect_inf_counts(uint8_t *hit_count)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        hit_count[i] = 0U;
    }

    for (i = 0U; i < INF_COLLECT_SAMPLE_COUNT; i++)
    {
        if (INF_1 != 0)
        {
            hit_count[0]++;
        }
        if (INF_2 != 0)
        {
            hit_count[1]++;
        }
        if (INF_3 != 0)
        {
            hit_count[2]++;
        }
        if (INF_4 != 0)
        {
            hit_count[3]++;
        }
        if (INF_5 != 0)
        {
            hit_count[4]++;
        }
        if (INF_6 != 0)
        {
            hit_count[5]++;
        }
        if (INF_7 != 0)
        {
            hit_count[6]++;
        }
        if (INF_8 != 0)
        {
            hit_count[7]++;
        }
        delay_ms(1);
    }
}

static int inf_error_from_mask(uint8_t mask)
{
    int sum = 0;
    int count = 0;

    if ((mask & 0x01U) != 0U)
    {
        sum -= 7;
        count++;
    }
    if ((mask & 0x02U) != 0U)
    {
        sum -= 5;
        count++;
    }
    if ((mask & 0x04U) != 0U)
    {
        sum -= 3;
        count++;
    }
    if ((mask & 0x08U) != 0U)
    {
        sum -= 1;
        count++;
    }
    if ((mask & 0x10U) != 0U)
    {
        sum += 1;
        count++;
    }
    if ((mask & 0x20U) != 0U)
    {
        sum += 3;
        count++;
    }
    if ((mask & 0x40U) != 0U)
    {
        sum += 5;
        count++;
    }
    if ((mask & 0x80U) != 0U)
    {
        sum += 7;
        count++;
    }

    if (count == 0)
    {
        return TASK2_LINE_LOST_VALUE;
    }

    return -(sum / count);
}

/*
 * 使用指定命中阈值完成12ms灰度滤波。普通路段继续使用全局7/12阈值；D→A因
 * 实测信号频繁在相邻采样间断开，使用5/12专用阈值提高连续性，但不影响其他路段。
 */
static int read_inf_delayed_with_threshold(uint8_t threshold)
{
    uint8_t hit_count[8];
    uint8_t mask = 0U;
    uint8_t i;

    collect_inf_counts(hit_count);

    for (i = 0U; i < 8U; i++)
    {
        if (hit_count[i] >= threshold)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return inf_error_from_mask(mask);
}

static int read_inf_delayed(void)
{
    return read_inf_delayed_with_threshold(INF_COLLECT_THRESHOLD);
}

static void show_signed_num(uint8_t x, uint8_t y, int value)
{
    if (value < 0)
    {
        OLED_ShowString(x, y, (u8 *)"-");
        OLED_ShowNum(x + 8U, y, (uint32_t)(-value), 3, 16);
    }
    else
    {
        OLED_ShowString(x, y, (u8 *)" ");
        OLED_ShowNum(x + 8U, y, (uint32_t)value, 3, 16);
    }
}

static NOINLINE void sensor_collect_mode(void)
{
    uint8_t hit_count[8];
    uint8_t mask;
    int error;

    stop_motor();

    while (1)
    {
        collect_inf_counts(hit_count);
        mask = read_inf_mask();
        error = inf_error_from_mask(mask);

        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"1-4:");
        OLED_ShowNum(40, 0, hit_count[0], 2, 16);
        OLED_ShowNum(64, 0, hit_count[1], 2, 16);
        OLED_ShowNum(88, 0, hit_count[2], 2, 16);
        OLED_ShowNum(112, 0, hit_count[3], 2, 16);

        OLED_ShowString(0, 2, (u8 *)"5-8:");
        OLED_ShowNum(40, 2, hit_count[4], 2, 16);
        OLED_ShowNum(64, 2, hit_count[5], 2, 16);
        OLED_ShowNum(88, 2, hit_count[6], 2, 16);
        OLED_ShowNum(112, 2, hit_count[7], 2, 16);

        OLED_ShowString(0, 4, (u8 *)"M:");
        OLED_ShowNum(16, 4, mask, 3, 16);
        OLED_ShowString(64, 4, (u8 *)"E:");
        show_signed_num(80, 4, error);

        OLED_ShowString(0, 6, (u8 *)"# exit");
        buzzer_off();

        if (get_keyboard_value() == '#')
        {
            while (get_keyboard_value() == '#')
            {
                buzzer_off();
                delay_ms(20);
            }
            break;
        }

        delay_ms(80);
    }

    stop_motor();
}
static void show_motor_test_menu(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A:L+ B:L-");
    OLED_ShowString(0, 2, (u8 *)"C:R+ D:R-");
    OLED_ShowString(0, 4, (u8 *)"1:++ 2:--");
    OLED_ShowString(0, 6, (u8 *)"#:Stop *:Exit");
}

static NOINLINE void motor_direction_test_mode(void)
{
    char key;
    char last_key = 0;

    stop_motor();
    show_motor_test_menu();

    while (1)
    {
        key = get_keyboard_value();

        if ((key != 0) && (key != last_key))
        {
            last_key = key;

            switch (key)
            {
            case 'A':
                set_motor_speed(MOTOR_TEST_PWM, TASK1_STOP_PWM);
                OLED_ShowString(72, 0, (u8 *)"L+");
                break;
            case 'B':
                set_motor_speed(-MOTOR_TEST_PWM, TASK1_STOP_PWM);
                OLED_ShowString(72, 0, (u8 *)"L-");
                break;
            case 'C':
                set_motor_speed(TASK1_STOP_PWM, MOTOR_TEST_PWM);
                OLED_ShowString(72, 2, (u8 *)"R+");
                break;
            case 'D':
                set_motor_speed(TASK1_STOP_PWM, -MOTOR_TEST_PWM);
                OLED_ShowString(72, 2, (u8 *)"R-");
                break;
            case '1':
                set_motor_speed(MOTOR_TEST_PWM, MOTOR_TEST_PWM);
                OLED_ShowString(72, 4, (u8 *)"++");
                break;
            case '2':
                set_motor_speed(-MOTOR_TEST_PWM, -MOTOR_TEST_PWM);
                OLED_ShowString(72, 4, (u8 *)"--");
                break;
            case '#':
                stop_motor();
                show_motor_test_menu();
                break;
            case '*':
                stop_motor();
                while (get_keyboard_value() == '*')
                {
                    buzzer_off();
                    delay_ms(20);
                }
                return;
            default:
                break;
            }
        }
        else if (key == 0)
        {
            last_key = 0;
        }

        buzzer_off();
        delay_ms(40);
    }
}
static void show_task_menu(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A: Task1");
    OLED_ShowString(0, 2, (u8 *)"B: Task2");
    OLED_ShowString(0, 4, (u8 *)"C: Task3");
    OLED_ShowString(0, 6, (u8 *)"D:T4 #:S *:M");
}

static NOINLINE char wait_task_key(void)
{
    char key;
    char last_key = 0;
    uint8_t stable_count = 0U;
    uint8_t release_count = 0U;

    while (release_count < 10U)
    {
        if (get_keyboard_value() == 0)
        {
            release_count++;
        }
        else
        {
            release_count = 0U;
        }

        buzzer_off();
        delay_ms(20);
    }

    while (1)
    {
        key = get_keyboard_value();

        if (((key >= 'A') && (key <= 'D')) || (key == '#') || (key == '*'))
        {
            if (key == last_key)
            {
                stable_count++;
            }
            else
            {
                last_key = key;
                stable_count = 1U;
            }

            if (stable_count >= 6U)
            {
                while (get_keyboard_value() == key)
                {
                    buzzer_off();
                    delay_ms(20);
                }

                buzzer_off();
                return key;
            }
        }
        else
        {
            last_key = 0;
            stable_count = 0U;
        }

        buzzer_off();
        delay_ms(20);
    }
}

static void task_not_ready(char key)
{
    stop_motor();
    buzzer_off();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Task not ready");
    OLED_ShowString(0, 2, (u8 *)"Key:");
    OLED_ShowString(48, 2, (key == 'B') ? (u8 *)"B" : ((key == 'C') ? (u8 *)"C" : (u8 *)"D"));
    point_signal_once();
    delay_ms(500);
}

static uint32_t update_qei_total(void)
{
    int32_t m1_step;
    int32_t m2_step;

    get_motor_qei_cnt(qei_cnt);
    m1_step = qei_signed_delta(qei_cnt[0], qei_prev[0]);
    m2_step = qei_signed_delta(qei_cnt[1], qei_prev[1]);
    qei_prev[0] = qei_cnt[0];
    qei_prev[1] = qei_cnt[1];

    qei_total[0] += (m1_step >= 0) ? (uint32_t)m1_step : (uint32_t)(-m1_step);
    qei_total[1] += (m2_step >= 0) ? (uint32_t)m2_step : (uint32_t)(-m2_step);

    return (qei_total[0] + qei_total[1]) / 2U;
}

static uint32_t qei_min_total(void)
{
    return (qei_total[0] < qei_total[1]) ? qei_total[0] : qei_total[1];
}

static void drive_straight_count(uint32_t target_count)
{
    uint32_t average_delta = 0U;
    uint32_t next_log_count = TASK_DIAGNOSTIC_COUNT_STEP;
    int base_pwm = TASK1_BASE_PWM;

    qei_zero_soft();
    printf("COUNT STRAIGHT START TARGET=%lu\r\n",
           (unsigned long)target_count);

    while (average_delta < target_count)
    {
        average_delta = update_qei_total();

        if ((target_count - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        if (average_delta >= next_log_count)
        {
            printf("COUNT STRAIGHT SAMPLE L=%lu R=%lu AVG=%lu PWM=%d\r\n",
                   (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                   (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                   (unsigned long)average_delta, base_pwm);
            next_log_count += TASK_DIAGNOSTIC_COUNT_STEP;
        }

        set_straight_speed_closed_loop(base_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    printf("COUNT STRAIGHT END L=%lu R=%lu AVG=%lu\r\n",
           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
           (unsigned long)average_delta);
}

static void drive_straight_until_line(uint32_t min_count, uint32_t max_count)
{
    uint32_t average_delta = 0U;
    int base_pwm = TASK1_BASE_PWM;
    uint8_t hit_count = 0U;

    qei_zero_soft();

    while (average_delta < max_count)
    {
        average_delta = update_qei_total();

        if ((max_count - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        if (average_delta > min_count)
        {
            if (read_inf() != TASK2_LINE_LOST_VALUE)
            {
                hit_count++;
                if (hit_count >= STRAIGHT_LINE_HIT_LIMIT)
                {
                    break;
                }
            }
            else
            {
                hit_count = 0U;
            }
        }

        set_straight_speed_closed_loop(base_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}

/*
 * 陀螺仪直线路段。调用者传入该路段的理论绝对航向，后续始终用该目标差速修正；
 * 不再把转向结束时允许范围内的残余误差锁定一整段，避免长距离横向偏移。
 * 灰度传感器连续命中line_hit_limit次后确认到达横线，编码器最大计数用于传感器
 * 异常时安全退出。drive_pwm允许测试2的C→D独立降速，不影响其他直线路段。
 */
static bool task4_drive_straight_until_line(const char *segment_label,
                                             uint32_t min_count,
                                             uint32_t max_count,
                                             int drive_pwm,
                                             uint8_t line_hit_limit,
                                             int32_t target_heading_md)
{
    uint32_t average_delta = 0U;
    uint32_t next_log_count = TASK_DIAGNOSTIC_COUNT_STEP;
    uint32_t start_tick = TimerA1_get_tick_100us();
    uint8_t hit_count = 0U;
    uint8_t line_detected = 0U;
    int line_error = TASK2_LINE_LOST_VALUE;
    int32_t heading_error_md;
    int correction;
    int base_pwm = drive_pwm;

    qei_zero_soft();
    task4_gyro_sync_time();

    while (average_delta < max_count)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        average_delta = update_qei_total();
        if ((max_count - average_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        if (average_delta > min_count)
        {
            line_error = read_inf();
            if (line_error != TASK2_LINE_LOST_VALUE)
            {
                hit_count++;
                if (hit_count >= line_hit_limit)
                {
                    line_detected = 1U;
                    break;
                }
            }
            else
            {
                hit_count = 0U;
            }
        }

        heading_error_md = task4_normalize_angle_md(target_heading_md -
                                                     task4_get_heading_md());
        correction = (int)(heading_error_md / TASK4_STRAIGHT_KP_DIV);
        if (correction > TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = TASK4_STRAIGHT_MAX_CORRECTION;
        }
        else if (correction < -TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = -TASK4_STRAIGHT_MAX_CORRECTION;
        }

        /* 正误差需要向右修正，因此提高左轮、降低右轮的前进PWM。 */
        /*
         * 每前进50000计数输出一次过程状态。TH为目标航向，H为实测航向，
         * HC为航向闭环修正，E/HIT用于判断灰度横线是何时开始出现的。
         */
        if (average_delta >= next_log_count)
        {
            printf("STRAIGHT SAMPLE %s L=%lu R=%lu AVG=%lu T=%lu E=%d HIT=%u TH=%ld H=%ld HC=%d PWM=%d\r\n",
                   segment_label,
                   (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                   (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                   (unsigned long)average_delta,
                   (unsigned long)((TimerA1_get_tick_100us() - start_tick) / 10U),
                   line_error, (unsigned int)hit_count,
                   (long)target_heading_md, (long)task4_get_heading_md(),
                   correction, base_pwm);
            next_log_count += TASK_DIAGNOSTIC_COUNT_STEP;
        }

        set_motor_speed(limit_pwm(base_pwm + correction),
                        limit_pwm(base_pwm - correction));
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    /*
     * 每段陀螺仪直线结束时输出完整诊断。R=LINE表示灰度横线确认退出，R=MAX表示
     * 达到最大编码器距离退出；左右计数顺序使用工程定义的真实左右轮索引。
     */
    printf("STRAIGHT %s EXIT=%s L=%lu R=%lu AVG=%lu T=%lu E=%d H=%ld\r\n",
           segment_label,
           (line_detected != 0U) ? "LINE" : "MAX",
           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
           (unsigned long)average_delta,
           (unsigned long)((TimerA1_get_tick_100us() - start_tick) / 10U),
           line_error, (long)task4_get_heading_md());
    return true;
}

/*
 * 灰度传感器位于轮轴前方154mm。C→B检测到B点后，先把车头对准180°切线，再用
 * 本函数按编码器前进约154mm，使轮轴中心而不是传感器中心落到B点。前进期间用
 * 传入的绝对航向闭环修正，避免固定时间动作受电池电压影响。返回false表示陀螺仪
 * 读取失败，调用者必须终止当前任务。
 */
static bool task4_forward_by_count(const char *point_label,
                                    uint32_t target_count,
                                    int drive_pwm,
                                    int32_t target_heading_md)
{
    uint32_t min_delta = 0U;
    int32_t heading_error_md;
    int correction;
    int base_pwm = drive_pwm;

    qei_zero_soft();
    task4_gyro_sync_time();
    printf("AXLE ALIGN %s START TARGET=%lu TH=%ld\r\n",
           point_label, (unsigned long)target_count,
           (long)target_heading_md);

    while (min_delta < target_count)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        update_qei_total();
        min_delta = qei_min_total();
        if (min_delta >= target_count)
        {
            break;
        }
        if ((target_count - min_delta) < TASK1_SLOWDOWN_COUNT)
        {
            base_pwm = TASK1_SLOWDOWN_PWM;
        }

        heading_error_md = task4_normalize_angle_md(target_heading_md -
                                                     task4_get_heading_md());
        correction = (int)(heading_error_md / TASK4_STRAIGHT_KP_DIV);
        if (correction > TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = TASK4_STRAIGHT_MAX_CORRECTION;
        }
        else if (correction < -TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = -TASK4_STRAIGHT_MAX_CORRECTION;
        }

        set_motor_speed(limit_pwm(base_pwm + correction),
                        limit_pwm(base_pwm - correction));
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    printf("AXLE ALIGN %s END L=%lu R=%lu MIN=%lu TH=%ld H=%ld\r\n",
           point_label,
           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
           (unsigned long)min_delta, (long)target_heading_md,
           (long)task4_get_heading_md());
    return true;
}

static void task3_turn_right_by_count(uint32_t turn_count)
{
    uint32_t turn_delta = 0U;

    if (turn_count == 0U)
    {
        return;
    }

    qei_zero_soft();
    while (turn_delta < turn_count)
    {
        update_qei_total();
        turn_delta = qei_total[0] + qei_total[1];
        set_motor_speed(TASK3_EXTRA_TURN_PWM, -TASK3_EXTRA_TURN_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}
static void task3_turn_opposite_by_count(uint32_t turn_count)
{
    uint32_t turn_delta = 0U;

    if (turn_count == 0U)
    {
        return;
    }

    qei_zero_soft();
    while (turn_delta < turn_count)
    {
        update_qei_total();
        turn_delta = qei_total[0] + qei_total[1];
        set_motor_speed(-TASK3_EXTRA_TURN_PWM, TASK3_EXTRA_TURN_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}
static void task3_back_by_count(uint32_t back_count)
{
    uint32_t back_delta = 0U;

    if (back_count == 0U)
    {
        return;
    }

    qei_zero_soft();
    while (back_delta < back_count)
    {
        update_qei_total();
        back_delta = qei_total[0] + qei_total[1];
        set_motor_speed(-TASK3_EXTRA_TURN_PWM, -TASK3_EXTRA_TURN_PWM);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
}
static void task3_forward_by_time(uint32_t forward_ms)
{
    if (forward_ms == 0U)
    {
        return;
    }

    set_motor_speed(TASK1_BASE_PWM, TASK1_BASE_PWM);
    delay_ms(forward_ms);
    stop_motor();
}

/*
 * 沿黑线行驶至线段末端。lost_limit 是连续丢线确认次数，允许每个路段
 * 独立调整停止时刻，避免校准某一处转弯时改变其他循迹路段的动作。gyro_tracking
 * 仅由第四问置 1，使弧线运动产生的实际转角也进入绝对航向积分。loop_delay_ms
 * 允许测试2单独增加灰度循迹延时，而不改变测试3、4的响应周期。min_time_ticks
 * 为0时关闭时间门槛，非0时必须达到指定100us节拍数才允许按丢线条件结束。
 */
static void follow_arc_to_line_end_config(uint32_t min_count, uint32_t extra_count,
                                          uint32_t extra_turn_count, uint8_t stop_after,
                                          uint8_t lost_limit, uint32_t slow_count,
                                           uint32_t max_count, int slow_pwm,
                                           int lost_left_pwm, int lost_right_pwm,
                                           uint8_t gyro_tracking,
                                           uint32_t loop_delay_ms,
                                           uint32_t min_time_ticks)
{
    uint32_t average_delta = 0U;
    uint32_t min_delta = 0U;
    uint32_t extra_target = 0U;
    int line_error = 0;
    int last_error = 0;
    int correction;
    int base_pwm = TASK2_ARC_BASE_PWM;
    int left_pwm = TASK2_ARC_BASE_PWM;
    int right_pwm = TASK2_ARC_BASE_PWM;
    uint8_t lost_count = 0U;
    uint8_t seen_count = 0U;
    uint8_t line_confirmed = 0U;
    uint32_t start_tick = TimerA1_get_tick_100us();

    qei_zero_soft();
    arc_exit_reason = ARC_EXIT_NONE;
    arc_exit_count = 0U;

    while (1)
    {
        if ((gyro_tracking != 0U) && !task4_gyro_update())
        {
            stop_motor();
            return;
        }

        average_delta = update_qei_total();
        min_delta = qei_min_total();

        /*
         * slow_count 和 max_count 为 0 时关闭对应功能，保持 B、C 键原有行为。
         * D 键进入 A 点附近后先降速，以缩短传感器确认和电机惯性造成的越界距离。
         */
        if ((slow_count > 0U) && (min_delta >= slow_count))
        {
            base_pwm = slow_pwm;
        }

        /* 红外异常时以编码器最大距离强制结束，防止 D 键越过 A 点后继续前进。 */
        if ((max_count > 0U) && (min_delta >= max_count))
        {
            arc_exit_reason = ARC_EXIT_MAX_COUNT;
            arc_exit_count = min_delta;
            break;
        }

        line_error = read_inf_delayed();

        if (line_error == TASK2_LINE_LOST_VALUE)
        {
            /*
             * 在尚未进入终点窗口时可能长时间检测不到黑线，计数必须饱和而不能溢出。
             * 进入窗口后若仍处于丢线状态，可立即按设定阈值结束循迹。
             */
            if (lost_count < lost_limit)
            {
                lost_count++;
            }
            /*
             * D→A进入A点终点窗口后，第一次全白便立即停车并在原地累计确认。
             * 不能继续沿用last_error驱动左右轮，否则最后一次较大的单边误差会在
             * 20轮确认期间形成持续差速，表现为到达A点后原地右转或左转。
             */
            if ((line_confirmed != 0U) &&
                (min_delta >= min_count) &&
                ((min_time_ticks == 0U) ||
                 ((TimerA1_get_tick_100us() - start_tick) >= min_time_ticks)))
            {
                stop_motor();
                if (lost_count >= lost_limit)
                {
                    arc_exit_reason = ARC_EXIT_LINE_LOST;
                    arc_exit_count = min_delta;
                    break;
                }

                show_task1_status(qei_total[0], qei_total[1]);
                delay_ms(loop_delay_ms);
                continue;
            }

            line_error = last_error;

            /*
             * D 键在终点前短暂丢线时，以两个正 PWM 低速向右搜索黑线；两个参数
             * 为 0 时保持普通 B、C 键的原有 PID 丢线处理，不改变其他任务行为。
             */
            if ((lost_left_pwm > 0) && (lost_right_pwm > 0))
            {
                set_motor_speed(lost_left_pwm, lost_right_pwm);
                show_task1_status(qei_total[0], qei_total[1]);
                delay_ms(loop_delay_ms);
                continue;
            }
        }
        else
        {
            lost_count = 0U;
            if (seen_count < TASK2_LINE_SEEN_LIMIT)
            {
                seen_count++;
                if (seen_count >= TASK2_LINE_SEEN_LIMIT)
                {
                    line_confirmed = 1U;
                }
            }
            last_error = line_error;
        }

        correction = line_error * TASK2_LINE_KP;
        left_pwm = limit_arc_pwm(base_pwm + correction);
        right_pwm = limit_arc_pwm(base_pwm - correction);
        set_motor_speed(left_pwm, right_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(loop_delay_ms);
    }

    if (extra_count > 0U)
    {
        extra_target = average_delta + extra_count;
        while (average_delta < extra_target)
        {
            if ((gyro_tracking != 0U) && !task4_gyro_update())
            {
                stop_motor();
                return;
            }

            average_delta = update_qei_total();
            set_motor_speed(left_pwm, right_pwm);
            show_task1_status(qei_total[0], qei_total[1]);
            delay_ms(loop_delay_ms);
        }
    }

    task3_turn_opposite_by_count(extra_turn_count);

    if (stop_after != 0U)
    {
        stop_motor();
    }
}

/* 普通循迹路段不启用末端减速和最大距离限制，维持 B、C 键原来的运行方式。 */
static void follow_arc_to_line_end(uint32_t min_count, uint32_t extra_count,
                                   uint32_t extra_turn_count, uint8_t stop_after,
                                   uint8_t lost_limit, uint32_t loop_delay_ms,
                                   uint32_t min_time_ticks)
{
    follow_arc_to_line_end_config(min_count, extra_count, extra_turn_count, stop_after,
                                  lost_limit, 0U, 0U, TASK2_ARC_BASE_PWM, 0, 0, 0U,
                                  loop_delay_ms, min_time_ticks);
}

/*
 * 测试3、测试4共用的纯灰度P控制实现。两项任务分别传入各自独立参数；
 * 电机只由灰度误差控制，陀螺仪仅记录B点回正所需航向，不参与循迹PWM。
 */
static void task34_follow_arc_reference(int base_pwm,
                                        uint32_t min_count,
                                        int line_kp,
                                        uint8_t lost_limit,
                                        uint8_t seen_limit,
                                        uint32_t loop_delay_ms,
                                        uint32_t lost_delay_ms,
                                        uint8_t stop_after)
{
    uint32_t min_delta = 0U;
    uint32_t start_tick = TimerA1_get_tick_100us();
    uint32_t lost_start_tick = 0U;
    uint32_t lost_elapsed_ticks = 0U;
    int line_error = 0;
    int last_error = 0;
    int correction;
    int left_pwm = base_pwm;
    int right_pwm = base_pwm;
    uint8_t lost_count = 0U;
    uint8_t seen_count = 0U;
    uint8_t line_confirmed = 0U;
    uint8_t lost_timing = 0U;

    qei_zero_soft();

    while (1)
    {
        /*
         * 陀螺仪在该函数中只被动记录航向，左右轮PWM仍只由灰度误差计算，
         * 避免航向闭环与弧线灰度纠偏相互竞争。
         */
        task4_gyro_update();
        update_qei_total();
        min_delta = qei_min_total();
        line_error = read_inf_delayed();

        if (line_error == TASK2_LINE_LOST_VALUE)
        {
            /*
             * 第一次读到全白时记录时刻，后续持续全白才累计丢线时间。
             * 中间只要重新检测到一次黑线就立即清零，避免细线位于两个探头
             * 之间产生的瞬时盲区被误认为轨迹终点。系统节拍单位为100us。
             */
            if (lost_timing == 0U)
            {
                lost_start_tick = TimerA1_get_tick_100us();
                lost_timing = 1U;
            }
            lost_elapsed_ticks = TimerA1_get_tick_100us() - lost_start_tick;
            lost_count++;
            if ((line_confirmed != 0U) &&
                (min_delta >= min_count) &&
                (lost_count >= lost_limit) &&
                (lost_elapsed_ticks >= (lost_delay_ms * 10U)))
            {
                break;
            }

            line_error = last_error;
        }
        else
        {
            lost_count = 0U;
            lost_timing = 0U;
            lost_elapsed_ticks = 0U;
            if (seen_count < seen_limit)
            {
                seen_count++;
                if (seen_count >= seen_limit)
                {
                    line_confirmed = 1U;
                }
            }
            last_error = line_error;
        }

        correction = line_error * line_kp;
        left_pwm = limit_pwm(base_pwm + correction);
        right_pwm = limit_pwm(base_pwm - correction);
        set_motor_speed(left_pwm, right_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(loop_delay_ms);
    }

    if (stop_after != 0U)
    {
        stop_motor();
    }

    printf("T3 TRACK END L=%lu R=%lu MIN=%lu T=%lu E=%d LOST=%u\r\n",
           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
           (unsigned long)min_delta,
           (unsigned long)((TimerA1_get_tick_100us() - start_tick) / 10U),
           last_error, (unsigned int)lost_count);
}

/*
 * 测试3、4的D→A专用居中循迹与A点停车控制。
 *
 * 输入：
 *   min_count      两个驱动轮都必须达到的最低编码器累计数；
 *   min_time_ticks D→A必须经过的最低时间，单位为TimerA1的100us节拍。
 *
 * 控制策略：
 *   1. 灰度加权误差的目标值为0，即让黑线尽量位于8路传感器中心；
 *   2. 使用受限PD差速，D项只在连续有效灰度样本之间计算，恢复见线的第一帧
 *      不计算D项，避免停车恢复时产生瞬时转向冲击；
 *   3. 只有当前连续2帧检测到灰度、两个轮子达到最低里程且达到最低时间后，
 *      才开启A点终点检测；
 *   4. 终点窗口第一次丢线立即停车但不直接结束，下一帧重新见线便恢复；
 *   5. 停车后持续无灰度才进入静止确认，连续4次丢线后最终确认A点；最大里程
 *      或最大时间用于传感器故障时的强制安全停车。
 *
 * 该函数不执行倒车、编码器位置归一化或陀螺仪转向。退出时车辆保持停车，
 * 调用者可将此处的实际切线方向重新定义为下一圈的航向零点。
 */
static void task34_follow_arc_to_a_centered(uint32_t min_count,
                                             uint32_t min_time_ticks)
{
    uint32_t min_delta = 0U;
    uint32_t next_log_count = TASK_DIAGNOSTIC_COUNT_STEP;
    uint32_t start_tick = TimerA1_get_tick_100us();
    uint32_t elapsed_ticks = 0U;
    int line_error = 0;
    int last_error = 0;
    int derivative = 0;
    int correction = 0;
    int base_pwm = TASK34_DA_BASE_PWM;
    int left_pwm;
    int right_pwm;
    uint8_t line_confirmed = 0U;
    uint8_t line_was_lost = 0U;
    uint8_t state = TASK34_DA_STATE_TRACKING;
    uint8_t recovery_count = 0U;
    uint8_t recovery_limit = TASK34_A_CENTER_RECOVERY_LIMIT;
    uint8_t confirm_count = 0U;
    uint8_t prewindow_lost_reported = 0U;
    uint8_t prewindow_lost_active = 0U;
    uint8_t valid_count = 0U;
    uint8_t force_search_correction = 0U;
    uint8_t tracking_lost_count = 0U;
    int search_direction = 1;

    qei_zero_soft();
    arc_exit_reason = ARC_EXIT_NONE;
    arc_exit_count = 0U;
    printf("DA START min=%lu time=%lu\r\n",
           (unsigned long)min_count, (unsigned long)min_time_ticks);

    while (1)
    {
        force_search_correction = 0U;
        update_qei_total();
        min_delta = qei_min_total();
        elapsed_ticks = TimerA1_get_tick_100us() - start_tick;

        /*
         * 最大里程和最大时间只作为传感器异常时的安全边界，不参与正常A点定位。
         * 到达任一边界都停车并输出诊断数据，避免灰度断线后电机永久运行。
         */
        if ((min_delta >= TASK34_DA_MAX_COUNT) ||
            (elapsed_ticks >= TASK34_DA_MAX_TIME_TICKS))
        {
            arc_exit_reason = ARC_EXIT_MAX_COUNT;
            arc_exit_count = min_delta;
            printf("DA SAFETY L=%lu R=%lu T=%lu E=%d\r\n",
                   (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                   (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                   (unsigned long)(elapsed_ticks / 10U), last_error);
            break;
        }

        line_error = read_inf_delayed_with_threshold(TASK34_DA_INF_THRESHOLD);

        if (line_error != TASK2_LINE_LOST_VALUE)
        {
            /*
             * 任意恢复、确认状态重新见线都说明此前不是稳定线尾，立即返回终点窗口。
             * 恢复后的第一帧关闭D项，防止误差跳变引起电机瞬间猛转。
             */
            if ((state == TASK34_DA_STATE_LINE_RECOVERY) ||
                (state == TASK34_DA_STATE_END_CONFIRM))
            {
                printf("DA REGAIN L=%lu R=%lu T=%lu E=%d\r\n",
                       (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                       (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                       (unsigned long)(elapsed_ticks / 10U), line_error);
                state = TASK34_DA_STATE_END_WINDOW;
            }

            recovery_count = 0U;
            confirm_count = 0U;
            tracking_lost_count = 0U;
            /*
             * 记录运行中已经成功识别过灰度；真正开启窗口还要求下方valid_count
             * 连续达到2帧，防止偏离轨迹后的编码器累计数被误当作A点位置。
             */
            line_confirmed = 1U;

            if (valid_count < TASK34_DA_WINDOW_VALID_COUNT)
            {
                valid_count++;
            }

            if ((state == TASK34_DA_STATE_TRACKING) &&
                (prewindow_lost_active != 0U))
            {
                prewindow_lost_active = 0U;
                printf("DA PREREGAIN L=%lu R=%lu T=%lu E=%d\r\n",
                       (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                       (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                       (unsigned long)(elapsed_ticks / 10U), line_error);
            }

            derivative = (line_was_lost != 0U) ? 0 : (line_error - last_error);
            line_was_lost = 0U;
            last_error = line_error;
            if (line_error > 0)
            {
                search_direction = 1;
            }
            else if (line_error < 0)
            {
                search_direction = -1;
            }

            /*
             * 必须在当前连续2帧确实检测到灰度时才能开启终点窗口。编码器和时间
             * 只限定A点附近范围，不能用偏离轨迹后累计的里程替代当前见线状态。
             */
            if ((state == TASK34_DA_STATE_TRACKING) &&
                (valid_count >= TASK34_DA_WINDOW_VALID_COUNT) &&
                (min_delta >= min_count) &&
                ((min_time_ticks == 0U) ||
                 (elapsed_ticks >= min_time_ticks)))
            {
                state = TASK34_DA_STATE_END_WINDOW;
                base_pwm = TASK34_DA_APPROACH_PWM;
                printf("DA WINDOW L=%lu R=%lu T=%lu\r\n",
                       (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                       (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                       (unsigned long)(elapsed_ticks / 10U));
            }
        }
        else
        {
            line_was_lost = 1U;
            valid_count = 0U;

            if (state == TASK34_DA_STATE_END_WINDOW)
            {
                /*
                 * 第一次丢线只建立候选并立即停车，不直接宣布到达A点。恢复上限
                 * 当前为1，所以下一帧仍丢线便进入静止确认，不再继续向前探测；
                 * 若下一帧灰度恢复，则有效样本分支会返回终点窗口继续循迹。
                 */
                stop_motor();
                recovery_count = 0U;
                confirm_count = 0U;
                recovery_limit = ((last_error >= -TASK34_A_CENTER_ERROR_MAX) &&
                                  (last_error <= TASK34_A_CENTER_ERROR_MAX)) ?
                                 TASK34_A_CENTER_RECOVERY_LIMIT :
                                 TASK34_A_EDGE_RECOVERY_LIMIT;
                state = TASK34_DA_STATE_LINE_RECOVERY;
                printf("DA LOST L=%lu R=%lu T=%lu E=%d N=%u\r\n",
                       (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                       (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                       (unsigned long)(elapsed_ticks / 10U), last_error,
                       (unsigned int)recovery_limit);
                show_task1_status(qei_total[0], qei_total[1]);
                delay_ms(TASK_OTHER_ARC_LOOP_DELAY_MS);
                continue;
            }

            if (state == TASK34_DA_STATE_LINE_RECOVERY)
            {
                if (recovery_count < recovery_limit)
                {
                    recovery_count++;
                }

                if (recovery_count >= recovery_limit)
                {
                    stop_motor();
                    state = TASK34_DA_STATE_END_CONFIRM;
                    confirm_count = 1U;
                    printf("DA VERIFY L=%lu R=%lu T=%lu E=%d\r\n",
                           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                           (unsigned long)(elapsed_ticks / 10U), last_error);
                    show_task1_status(qei_total[0], qei_total[1]);
                    delay_ms(TASK_OTHER_ARC_LOOP_DELAY_MS);
                    continue;
                }

                /* 低速维持最后有效方向探测前方灰度，找到线后由有效样本分支恢复。 */
                base_pwm = TASK34_DA_APPROACH_PWM;
                line_error = last_error;
                derivative = 0;
            }
            else if (state == TASK34_DA_STATE_END_CONFIRM)
            {
                stop_motor();
                if (confirm_count < TASK34_A_LOST_CONFIRM_COUNT)
                {
                    confirm_count++;
                }

                show_task1_status(qei_total[0], qei_total[1]);
                if (confirm_count >= TASK34_A_LOST_CONFIRM_COUNT)
                {
                    arc_exit_reason = ARC_EXIT_LINE_LOST;
                    arc_exit_count = min_delta;
                    printf("DA END L=%lu R=%lu T=%lu E=%d\r\n",
                           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                           (unsigned long)(elapsed_ticks / 10U), last_error);
                    break;
                }

                delay_ms(TASK_OTHER_ARC_LOOP_DELAY_MS);
                continue;
            }
            else
            {
                /*
                 * TRACKING阶段丢线不允许结束，沿最后误差方向使用固定强修正主动
                 * 回线。只记录第一次窗口前丢线，并在恢复时输出DA PREREGAIN。
                 */
                if ((line_confirmed != 0U) &&
                    (prewindow_lost_reported == 0U) &&
                    (elapsed_ticks >= TASK34_DA_PRELOST_LOG_MIN_TIME_TICKS))
                {
                    prewindow_lost_reported = 1U;
                    printf("DA PRELOST L=%lu R=%lu T=%lu E=%d\r\n",
                           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                           (unsigned long)(elapsed_ticks / 10U), last_error);
                }
                if (tracking_lost_count < TASK34_DA_LOST_GRACE_COUNT + 1U)
                {
                    tracking_lost_count++;
                }

                /*
                 * 1~3帧丢线按滤波波动处理，保持普通PD输出；连续超过3帧才认为
                 * 真正偏离，并沿最后有效误差方向施加强回线修正。
                 */
                if (tracking_lost_count > TASK34_DA_LOST_GRACE_COUNT)
                {
                    prewindow_lost_active = 1U;
                    force_search_correction = 1U;
                }
                line_error = last_error;
                derivative = 0;
            }
        }

        if (force_search_correction != 0U)
        {
            correction = search_direction * TASK34_DA_LOST_SEARCH_CORRECTION;
        }
        else
        {
            correction = TASK34_DA_LINE_KP * line_error +
                         TASK34_DA_LINE_KD * derivative;
        }
        if (correction > TASK34_DA_MAX_CORRECTION)
        {
            correction = TASK34_DA_MAX_CORRECTION;
        }
        else if (correction < -TASK34_DA_MAX_CORRECTION)
        {
            correction = -TASK34_DA_MAX_CORRECTION;
        }

        left_pwm = limit_arc_pwm(base_pwm + correction);
        right_pwm = limit_arc_pwm(base_pwm - correction);
        if (min_delta >= next_log_count)
        {
            printf("DA SAMPLE L=%lu R=%lu MIN=%lu T=%lu E=%d ST=%u VALID=%u LOST=%u C=%d LP=%d RP=%d\r\n",
                   (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                   (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                   (unsigned long)min_delta,
                   (unsigned long)(elapsed_ticks / 10U), line_error,
                   (unsigned int)state, (unsigned int)valid_count,
                   (unsigned int)tracking_lost_count, correction,
                   left_pwm, right_pwm);
            next_log_count += TASK_DIAGNOSTIC_COUNT_STEP;
        }
        set_motor_speed(left_pwm, right_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK_OTHER_ARC_LOOP_DELAY_MS);
    }

    /* 等待车身和传感器稳定后再由调用者重建A点航向基准。 */
    stop_motor();
    delay_ms(TASK34_A_SETTLE_MS);
}

/*
 * 测试2、3中途丢线时的原地摆头找线。搜索中心取触发丢线瞬间的真实累计航向，
 * 左右各摆10°，总扫描跨度20°；转向过程中每5ms读取一次稳定灰度结果，连续3次
 * 识别到任意黑线即停车返回。该动作只恢复黑线，不修改循迹结束条件和编码器零点。
 */
static bool task23_search_line_by_gyro_sweep(void)
{
    int32_t center_heading_md = task4_get_heading_md();
    int32_t target_heading_md = task4_normalize_angle_md(
        center_heading_md - TASK23_SEARCH_HALF_ANGLE_MD);
    int32_t error_md;
    int32_t abs_error_md;
    int turn_pwm;
    uint8_t target_right_side = 0U;
    uint8_t line_hit_count = 0U;
    uint32_t start_tick = TimerA1_get_tick_100us();

    task4_gyro_sync_time();

    while ((TimerA1_get_tick_100us() - start_tick) <
           TASK23_SEARCH_TIMEOUT_TICKS)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        if (read_inf() != TASK2_LINE_LOST_VALUE)
        {
            line_hit_count++;
            if (line_hit_count >= TASK23_SEARCH_LINE_HIT_LIMIT)
            {
                stop_motor();
                return true;
            }
        }
        else
        {
            line_hit_count = 0U;
        }

        error_md = task4_normalize_angle_md(target_heading_md -
                                             task4_get_heading_md());
        abs_error_md = (error_md >= 0L) ? error_md : -error_md;

        if (abs_error_md <= TASK4_TURN_TOLERANCE_MD)
        {
            /* 到达一侧端点后立即切换到另一侧，持续覆盖20°搜索扇区。 */
            target_right_side ^= 1U;
            target_heading_md = task4_normalize_angle_md(
                center_heading_md +
                ((target_right_side != 0U) ? TASK23_SEARCH_HALF_ANGLE_MD :
                                             -TASK23_SEARCH_HALF_ANGLE_MD));
        }
        else
        {
            turn_pwm = (int)(abs_error_md / TASK4_TURN_KP_DIV);
            if (turn_pwm < TASK4_TURN_MIN_PWM)
            {
                turn_pwm = TASK4_TURN_MIN_PWM;
            }
            else if (turn_pwm > TASK4_TURN_MAX_PWM)
            {
                turn_pwm = TASK4_TURN_MAX_PWM;
            }

            if (error_md > 0L)
            {
                set_motor_speed(turn_pwm, -turn_pwm);
            }
            else
            {
                set_motor_speed(-turn_pwm, turn_pwm);
            }
        }

        delay_ms(TASK4_TURN_LOOP_DELAY_MS);
    }

    stop_motor();
    task4_gyro_fault = 1U;
    return false;
}

/*
 * 循迹循环包含12ms灰度采样和可配置附加延时；为防止整轮超过陀螺仪允许的30ms
 * 最大采样间隔，这里把附加延时拆成不超过5ms的小段，每段结束立即积分一次。
 * 返回false表示I2C读取失败，调用者必须停车退出。
 */
static bool task23_delay_with_gyro_update(uint32_t total_delay_ms)
{
    uint32_t delay_step_ms;

    while (total_delay_ms > 0U)
    {
        delay_step_ms = (total_delay_ms > 5U) ? 5U : total_delay_ms;
        delay_ms(delay_step_ms);
        total_delay_ms -= delay_step_ms;

        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }
    }

    return true;
}

/*
 * 测试2、3专用循迹：电机修正完全沿用原灰度比例逻辑，MPU6050只累计航向并在
 * 指定终点窗口前连续丢线达到recovery_lost_limit时执行±10°摆头恢复。进入终点
 * 窗口后继续按最后有效误差跨越短暂盲区，只有连续丢线帧数和end_lost_delay_ms
 * 持续时间同时满足才停车。两个阈值独立，避免细黑线位于两探头之间时提前结束。
 * min_time_ticks单位为100us，end_lost_delay_ms单位为毫秒。
 */
static bool task23_follow_arc_with_recovery(uint32_t min_count,
                                             uint32_t force_end_count,
                                             uint8_t recovery_lost_limit,
                                             uint8_t end_confirm_limit,
                                             uint8_t stop_after,
                                             uint32_t min_time_ticks,
                                             uint32_t loop_delay_ms,
                                             uint32_t end_lost_delay_ms,
                                             int initial_error,
                                             uint8_t task2_motor_control)
{
    uint32_t min_delta;
    uint32_t next_log_count = TASK_DIAGNOSTIC_COUNT_STEP;
    uint32_t start_tick = TimerA1_get_tick_100us();
    /*
     * 起步尚未稳定识别灰度时不能使用0误差直行。调用者按圆弧方向传入
     * 初始误差，使车辆先沿预期弧向缓慢找线；一旦见线即由实测误差接管。
     */
    int line_error = initial_error;
    int last_error = initial_error;
    int correction;
    int left_pwm;
    int right_pwm;
    int arc_base_pwm = (task2_motor_control != 0U) ?
                       TASK2_ARC_BASE_PWM : TASK34_CB_BASE_PWM;
    int arc_line_kp = (task2_motor_control != 0U) ?
                      TASK2_LINE_KP : TASK34_CB_LINE_KP;
    int desired_correction = 0;
    int filtered_correction = 0;
    uint8_t is_task34_arc = (initial_error < 0) ? 1U : 0U;
    uint8_t gray_lost_now = 0U;
    uint8_t lost_count = 0U;
    uint8_t end_confirm_count = 0U;
    uint8_t seen_count = 0U;
    uint8_t line_confirmed = 0U;
    uint8_t forced_count_exit = 0U;
    uint8_t end_lost_timing = 0U;
    uint32_t end_lost_start_tick = 0U;
    uint32_t end_lost_elapsed_ticks = 0U;

    qei_zero_soft();
    task4_gyro_sync_time();

    while (1)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        update_qei_total();
        min_delta = qei_min_total();

        /*
         * 任务3/4的正常终点只由灰度线尾决定。380000仅作为传感器或轨迹异常
         * 时的绝对安全边界，触发后终止本次任务，不再继续执行B→D。
         */
        if ((is_task34_arc != 0U) &&
            (min_delta >= TASK34_CB_SAFETY_COUNT))
        {
            stop_motor();
            printf("TRACK SAFETY L=%lu R=%lu MIN=%lu H=%ld\r\n",
                   (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                   (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                   (unsigned long)min_delta, (long)task4_get_heading_md());
            return false;
        }

        /*
         * 第三、第四题C→B实测正常终点的较小轮计数集中在242000~246000。
         * 若灰度在B点附近持续存在而没有形成线尾，到250000时按计数强制停车，
         * 防止继续沿线向前。force_end_count为0时关闭，保持任务2原有行为。
         */
        if ((force_end_count > 0U) &&
            (min_delta >= force_end_count) &&
            ((TimerA1_get_tick_100us() - start_tick) >= min_time_ticks))
        {
            forced_count_exit = 1U;
            break;
        }
        line_error = read_inf_delayed();
        /* 灰度采样本身耗时12ms，采样完成后立即积分，避免与后续延时累计超限。 */
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        gray_lost_now = (line_error == TASK2_LINE_LOST_VALUE) ? 1U : 0U;
        if (gray_lost_now != 0U)
        {
            /*
             * 进入终点窗口后从第一次全白开始计时，但确认期间继续使用最后有效
             * 灰度误差低速循迹，使细黑线落在两探头之间时车辆能够跨过盲区。
             * 只有连续丢线帧数和持续时间同时满足才停车；重新见线会清零全部确认量。
             */
            if ((line_confirmed != 0U) &&
                (min_delta >= min_count) &&
                ((TimerA1_get_tick_100us() - start_tick) >= min_time_ticks))
            {
                if (end_lost_timing == 0U)
                {
                    end_lost_start_tick = TimerA1_get_tick_100us();
                    end_lost_timing = 1U;
                }
                end_lost_elapsed_ticks =
                    TimerA1_get_tick_100us() - end_lost_start_tick;
                if (end_confirm_count < end_confirm_limit)
                {
                    end_confirm_count++;
                }

                if ((end_confirm_count >= end_confirm_limit) &&
                    (end_lost_elapsed_ticks >= (end_lost_delay_ms * 10U)))
                {
                    stop_motor();
                    break;
                }

                line_error = last_error;
            }
            else
            {
                if (lost_count < recovery_lost_limit)
                {
                    lost_count++;
                }

                if ((line_confirmed != 0U) &&
                    (lost_count >= recovery_lost_limit))
                {
                    stop_motor();
                    OLED_ShowString(0, 4, (u8 *)"Line Search");
                    if (!task23_search_line_by_gyro_sweep())
                    {
                        return false;
                    }

                    lost_count = 0U;
                    line_error = read_inf();
                    if (line_error != TASK2_LINE_LOST_VALUE)
                    {
                        last_error = line_error;
                    }
                    continue;
                }

                line_error = last_error;
            }
        }
        else
        {
            lost_count = 0U;
            end_confirm_count = 0U;
            end_lost_timing = 0U;
            end_lost_elapsed_ticks = 0U;
            if (seen_count < TASK2_LINE_SEEN_LIMIT)
            {
                seen_count++;
                if (seen_count >= TASK2_LINE_SEEN_LIMIT)
                {
                    line_confirmed = 1U;
                }
            }
            last_error = line_error;
        }

        if (task2_motor_control != 0U)
        {
            /*
             * 任务2原始灰度比例循迹：不添加固定圆弧前馈、修正斜率或额外
             * 航向控制，左右轮只由当前/最后有效灰度误差直接差速。
             */
            correction = line_error * arc_line_kp;
            left_pwm = limit_arc_pwm(arc_base_pwm + correction);
            right_pwm = limit_arc_pwm(arc_base_pwm - correction);
        }
        else if (is_task34_arc != 0U)
        {
            /*
             * 小幅左弧前馈只负责提供圆弧趋势，灰度修正范围大于前馈，因此有效
             * 灰度可以向任一方向纠偏。修正仍经过限幅与斜率限制以抑制快速摇摆。
             * 连续丢线达到阈值后会在上方停车找线，不再长期依靠前馈盲走。
             */
            desired_correction = (gray_lost_now != 0U) ?
                                 filtered_correction :
                                 (line_error * arc_line_kp);
            if (desired_correction > TASK34_CB_MAX_GRAY_CORRECTION)
            {
                desired_correction = TASK34_CB_MAX_GRAY_CORRECTION;
            }
            else if (desired_correction < -TASK34_CB_MAX_GRAY_CORRECTION)
            {
                desired_correction = -TASK34_CB_MAX_GRAY_CORRECTION;
            }

            if (desired_correction >
                (filtered_correction + TASK34_CB_CORRECTION_SLEW_STEP))
            {
                filtered_correction += TASK34_CB_CORRECTION_SLEW_STEP;
            }
            else if (desired_correction <
                     (filtered_correction - TASK34_CB_CORRECTION_SLEW_STEP))
            {
                filtered_correction -= TASK34_CB_CORRECTION_SLEW_STEP;
            }
            else
            {
                filtered_correction = desired_correction;
            }

            correction = filtered_correction;
            left_pwm = limit_arc_pwm(arc_base_pwm -
                                     TASK34_CB_LEFT_TURN_BIAS + correction);
            right_pwm = limit_arc_pwm(arc_base_pwm +
                                      TASK34_CB_LEFT_TURN_BIAS - correction);
        }
        else
        {
            correction = line_error * arc_line_kp;
            left_pwm = limit_arc_pwm(arc_base_pwm + correction);
            right_pwm = limit_arc_pwm(arc_base_pwm - correction);
        }
        /*
         * 圆弧每50000计数记录一次灰度与控制量，便于区分“错过终点”
         * 和“中途丢线”：SEEN表示已经稳定见线，LOST是连续丢线次数。
         */
        if (min_delta >= next_log_count)
        {
            printf("TRACK SAMPLE L=%lu R=%lu MIN=%lu T=%lu RAW=%d E=%d SEEN=%u LOST=%u END=%u H=%ld C=%d LP=%d RP=%d\r\n",
                   (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
                   (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
                   (unsigned long)min_delta,
                   (unsigned long)((TimerA1_get_tick_100us() -
                                   start_tick) / 10U),
                   (gray_lost_now != 0U) ?
                   TASK2_LINE_LOST_VALUE : line_error,
                   line_error, (unsigned int)line_confirmed,
                   (unsigned int)lost_count,
                   (unsigned int)end_confirm_count,
                   (long)task4_get_heading_md(), correction,
                   left_pwm, right_pwm);
            next_log_count += TASK_DIAGNOSTIC_COUNT_STEP;
        }
        set_motor_speed(left_pwm, right_pwm);
        show_task1_status(qei_total[0], qei_total[1]);
        if (!task23_delay_with_gyro_update(loop_delay_ms))
        {
            return false;
        }
    }

    if (stop_after != 0U)
    {
        stop_motor();
    }
    /*
     * 该函数在任务3、4中对应C→B弧线。退出时通过串口记录两轮累计数、运行时间、
     * 最后有效灰度误差和航向，用于区分B点偏差来自位置不一致还是转角不准确。
     * 任务2复用该函数时也会输出同格式日志，日志标签使用TRACK以避免误导。
     */
    printf("TRACK END EXIT=%s L=%lu R=%lu T=%lu E=%d H=%ld\r\n",
           (forced_count_exit != 0U) ? "COUNT" : "LINE",
           (unsigned long)qei_total[STRAIGHT_LEFT_QEI_INDEX],
           (unsigned long)qei_total[STRAIGHT_RIGHT_QEI_INDEX],
           (unsigned long)((TimerA1_get_tick_100us() - start_tick) / 10U),
           last_error, (long)task4_get_heading_md());
    return true;
}

static void task2_point_prompt(const char *label)
{
    stop_motor();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)label);
    point_signal_once();
    delay_ms(150);
}

/*
 * 测试四E→C、F→D专用主动找线。车辆转回局部0°后，如果灰度尚未见线，就在
 * 陀螺仪航向闭环下低速直行；首次见线后立即停车并原地连续复核，避免车辆高速
 * 越过较细的黑线。复核期间若再次全白则恢复直行。只有连续命中达到阈值才返回
 * true并允许声光提示；超过最大距离、超时或陀螺仪读取失败均停车返回false。
 */
static bool task4_seek_point_gray_forward(const char *point_label,
                                           int32_t target_heading_md)
{
    uint32_t start_tick = TimerA1_get_tick_100us();
    uint32_t min_delta = 0U;
    uint8_t gray_hit_count = 0U;
    int line_error = TASK2_LINE_LOST_VALUE;
    int32_t heading_error_md;
    int correction;

    qei_zero_soft();
    task4_gyro_sync_time();
    while ((min_delta < TASK4_POINT_GRAY_MAX_COUNT) &&
           ((TimerA1_get_tick_100us() - start_tick) <
            TASK4_POINT_GRAY_TIMEOUT_TICKS))
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            printf("TASK4 GRAY POINT=%s EXIT=GYRO\r\n", point_label);
            return false;
        }

        update_qei_total();
        min_delta = qei_min_total();
        line_error = read_inf();
        if (line_error != TASK2_LINE_LOST_VALUE)
        {
            /*
             * 一旦任一路灰度探头检测到黑线便先停车，再累计连续样本。
             * 这样确认期间不会继续向前冲，可减小细黑线造成的停车位置离散。
             */
            stop_motor();
            gray_hit_count++;
            if (gray_hit_count >= TASK4_POINT_GRAY_HIT_LIMIT)
            {
                printf("TASK4 GRAY POINT=%s EXIT=LINE MIN=%lu E=%d HIT=%u\r\n",
                       point_label, (unsigned long)min_delta, line_error,
                       (unsigned int)gray_hit_count);
                return true;
            }

            delay_ms(TASK1_LOOP_DELAY_MS);
            continue;
        }

        gray_hit_count = 0U;
        heading_error_md = task4_normalize_angle_md(target_heading_md -
                                                     task4_get_heading_md());
        correction = (int)(heading_error_md / TASK4_STRAIGHT_KP_DIV);
        if (correction > TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = TASK4_STRAIGHT_MAX_CORRECTION;
        }
        else if (correction < -TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = -TASK4_STRAIGHT_MAX_CORRECTION;
        }

        set_motor_speed(limit_pwm(TASK4_POINT_GRAY_SEEK_PWM + correction),
                        limit_pwm(TASK4_POINT_GRAY_SEEK_PWM - correction));
        show_task1_status(qei_total[0], qei_total[1]);
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    printf("TASK4 GRAY POINT=%s EXIT=LIMIT MIN=%lu T=%lu E=%d HIT=%u\r\n",
           point_label, (unsigned long)min_delta,
           (unsigned long)((TimerA1_get_tick_100us() - start_tick) / 10U),
           line_error, (unsigned int)gray_hit_count);
    return false;
}

static NOINLINE void task2_drive_first_loop(void)
{
    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"Run Task2 Gyro");
    delay_ms(300);

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> B");
    if (!task4_drive_straight_until_line("A-B", STRAIGHT_LINE_MIN_COUNT,
                                          STRAIGHT_LINE_MAX_COUNT,
                                          TASK1_BASE_PWM,
                                          STRAIGHT_LINE_HIT_LIMIT,
                                          TASK23_HEADING_AB_MD) ||
        !task4_gyro_turn_to(TASK23_HEADING_AB_MD))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point B");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"B -> C");
    /*
     * B→C不再使用5s最低时间，只由最低编码器距离、连续丢线帧数和600ms
     * 持续全白共同确认终点；最低距离前丢线仍执行±10°摆头找线。
     */
    if (!task23_follow_arc_with_recovery(TASK2_ARC_MIN_COUNT,
                                          0U,
                                          TASK2_BC_LINE_LOST_LIMIT,
                                          TASK2_BC_LINE_LOST_LIMIT, 1U,
                                          0U,
                                          TASK2_ARC_LOOP_DELAY_MS,
                                          TASK2_BC_LINE_LOST_DELAY_MS,
                                          TASK2_BC_ACQUIRE_ERROR, 1U))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point C");

    /*
     * C点先由陀螺仪回正到180°，再闭环转到补偿后的170°。不使用右轮单轮
     * 补偿；轮轴前移和后续C→D均保持170°，避免补偿只在原地动作中生效。
     */
    if (!task4_gyro_turn_to(TASK23_HEADING_CD_MD) ||
        !task4_gyro_turn_to(TASK2_C_EXIT_HEADING_MD) ||
        !task4_forward_by_count("C", TASK_SENSOR_TO_AXLE_ALIGN_COUNT,
                                TASK2_CD_BASE_PWM,
                                TASK2_C_EXIT_HEADING_MD))
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"C -> D");
    if (!task4_drive_straight_until_line("C-D", STRAIGHT_LINE_MIN_COUNT,
                                          TASK2_CD_LINE_MAX_COUNT,
                                          TASK2_CD_BASE_PWM,
                                          TASK2_CD_LINE_HIT_LIMIT,
                                          TASK2_C_EXIT_HEADING_MD) ||
        !task4_gyro_turn_to(TASK23_HEADING_CD_MD))
    {
        task4_show_gyro_error();
        return;
    }
    task2_point_prompt("Point D");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"D -> A");
    /*
     * 测试2的D→A完整复用测试3当前生效的纯灰度P控制函数和TASK3_REF_*参数。
     * 陀螺仪只被动记录航向，不参与左右轮PWM；不使用最低时间或摆头找线。
     */
    task34_follow_arc_reference(TASK3_REF_ARC_BASE_PWM,
                                TASK3_REF_ARC_MIN_COUNT,
                                TASK3_REF_LINE_KP,
                                TASK3_REF_LINE_LOST_LIMIT,
                                TASK3_REF_LINE_SEEN_LIMIT,
                                TASK3_REF_ARC_LOOP_DELAY_MS,
                                0U, 1U);
    task2_point_prompt("Point A");
    stop_motor();
}

static NOINLINE bool task34_drive_route_internal(uint8_t final_stop,
                                                  uint8_t show_start,
                                                  uint8_t use_task4_params,
                                                  uint8_t task4_later_loop)
{
    int32_t b_tangent_heading_md;
    int32_t stage_heading_md = (use_task4_params != 0U) ?
                               TASK4_ROUTE_STAGE_HEADING_MD :
                               TASK3_STAGE_HEADING_MD;
    uint32_t ae_stage_count = (use_task4_params == 0U) ?
                              TASK3_AE_STAGE_COUNT :
                              ((task4_later_loop != 0U) ?
                               TASK4_ROUTE_AE_STAGE_COUNT_LATER :
                               TASK4_ROUTE_AE_STAGE_COUNT_FIRST);
    uint32_t bf_stage_count = (use_task4_params == 0U) ?
                              TASK3_BF_STAGE_COUNT :
                              ((task4_later_loop != 0U) ?
                               TASK4_ROUTE_BF_STAGE_COUNT_LATER :
                               TASK4_ROUTE_BF_STAGE_COUNT_FIRST);
    uint32_t stage_settle_ms = (use_task4_params != 0U) ?
                               TASK4_ROUTE_STAGE_SETTLE_MS :
                               TASK3_STAGE_SETTLE_MS;
    int straight_pwm = (use_task4_params != 0U) ?
                       TASK4_ROUTE_STRAIGHT_PWM :
                       TASK3_ROUTE_STRAIGHT_PWM;
    uint32_t b_axle_align_count = (use_task4_params != 0U) ?
                                  TASK4_ROUTE_B_AXLE_ALIGN_COUNT :
                                  TASK3_B_AXLE_ALIGN_COUNT;
    int32_t local_zero_heading_md = (use_task4_params != 0U) ?
                                    TASK4_ROUTE_LOCAL_ZERO_HEADING_MD :
                                    TASK3_LOCAL_ZERO_HEADING_MD;
    int32_t b_return_heading_md = (use_task4_params != 0U) ?
                                  TASK4_ROUTE_B_RETURN_HEADING_MD :
                                  TASK3_B_RETURN_HEADING_MD;
    int32_t b_right_compensate_md = (use_task4_params != 0U) ?
                                    TASK4_ROUTE_B_RIGHT_COMPENSATE_MD :
                                    TASK3_B_RIGHT_WHEEL_COMPENSATE_MD;
    int b_right_wheel_pwm = (use_task4_params != 0U) ?
                            TASK4_ROUTE_B_RIGHT_WHEEL_PWM :
                            TASK3_B_RIGHT_WHEEL_PWM;
    int32_t b_right_tolerance_md = (use_task4_params != 0U) ?
                                   TASK4_ROUTE_B_RIGHT_TOLERANCE_MD :
                                   TASK3_B_RIGHT_WHEEL_TOLERANCE_MD;
    uint32_t b_right_timeout_ticks = (use_task4_params != 0U) ?
                                     TASK4_ROUTE_B_RIGHT_TIMEOUT_TICKS :
                                     TASK3_B_RIGHT_WHEEL_TIMEOUT_TICKS;
    int arc_base_pwm = (use_task4_params != 0U) ?
                       TASK4_REF_ARC_BASE_PWM :
                       TASK3_REF_ARC_BASE_PWM;
    uint32_t arc_min_count = (use_task4_params != 0U) ?
                             TASK4_REF_ARC_MIN_COUNT :
                             TASK3_REF_ARC_MIN_COUNT;
    int arc_line_kp = (use_task4_params != 0U) ?
                      TASK4_REF_LINE_KP :
                      TASK3_REF_LINE_KP;
    uint8_t arc_lost_limit = (use_task4_params != 0U) ?
                             TASK4_REF_LINE_LOST_LIMIT :
                             TASK3_REF_LINE_LOST_LIMIT;
    uint8_t arc_seen_limit = (use_task4_params != 0U) ?
                             TASK4_REF_LINE_SEEN_LIMIT :
                             TASK3_REF_LINE_SEEN_LIMIT;
    uint32_t arc_loop_delay_ms = (use_task4_params != 0U) ?
                                 TASK4_REF_ARC_LOOP_DELAY_MS :
                                 TASK3_REF_ARC_LOOP_DELAY_MS;

    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return false;
    }

    if (show_start != 0U)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"Run Task3 Gyro");
        delay_ms(300);
    }

    /*
     * A→E：E位于CD边上且距C 15.4cm。小车按陀螺仪保持43.399°直线，
     * 由编码器在轮轴中心到达E时停车，不再利用C点灰度横线结束斜线。
     */
    if (!task4_gyro_turn_to(stage_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> E");
    if (!task4_forward_by_count("A-E", ae_stage_count,
                                 straight_pwm,
                                 stage_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }
    OLED_ShowString(0, 2, (u8 *)"Point E");
    delay_ms(stage_settle_ms);

    /*
     * E距C正好154mm。轮轴停在E后原地转到0°，前置灰度中心随车身转到
     * C点轨迹入口。若此时尚未压到黑线，测试四沿局部0°继续低速直行，
     * 直到灰度连续见线才确认C点并声光提示；测试三保持原有路线行为。
     */
    if (!task4_gyro_turn_to(local_zero_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }
    if ((use_task4_params != 0U) &&
        !task4_seek_point_gray_forward("C", local_zero_heading_md))
    {
        return false;
    }
    task2_point_prompt("Point C");

    /*
     * C→B使用参考工程测试三的纯灰度P控制，陀螺仪只记录航向。
     * 测试四额外启用全白延时确认，跨过细黑线落在相邻探头之间造成的短暂盲区；
     * 测试三传入0，保持原有按连续丢线帧数结束的行为。
     */
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"C -> B Gray");
    task34_follow_arc_reference(arc_base_pwm, arc_min_count, arc_line_kp,
                                arc_lost_limit, arc_seen_limit,
                                arc_loop_delay_ms,
                                (use_task4_params != 0U) ?
                                TASK4_CB_LINE_LOST_DELAY_MS : 0U,
                                1U);
    /*
     * B点提示只会在C→B灰度循迹满足“稳定见线、最低距离、连续全白帧数”
     * 以及测试四专用600ms全白确认后执行，因此提示条件来自灰度线尾。
     */
    task2_point_prompt("Point B");

    /*
     * 与测试2到达C点后的处理相同：声光提示后先回正到180°，再只驱动
     * 右轮完成-1°补偿。随后保持179°航向前进43000计数，使轮轴到达B。
     */
    if (!task4_gyro_turn_to(b_return_heading_md) ||
        !compensate_with_right_wheel(b_right_compensate_md,
                                     b_right_wheel_pwm,
                                     b_right_tolerance_md,
                                     b_right_timeout_ticks))
    {
        task4_show_gyro_error();
        return false;
    }
    b_tangent_heading_md = b_return_heading_md + b_right_compensate_md;
    if (!task4_forward_by_count("B", b_axle_align_count,
                                 straight_pwm,
                                 b_tangent_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }

    /*
     * 在B点把当前切线方向重新定义为局部0°，避免累计航向误差影响B→F。
     * F位于CD边上且距D 15.4cm；B→F相对B点切线左转43.399°。
     */
    task4_yaw_integral_numerator = 0LL;
    task4_gyro_sync_time();
    if (!task4_gyro_turn_to(-stage_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"B -> F");
    if (!task4_forward_by_count("B-F", bf_stage_count,
                                 straight_pwm,
                                 -stage_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }
    OLED_ShowString(0, 2, (u8 *)"Point F");
    delay_ms(stage_settle_ms);

    /*
     * F距D正好154mm。轮轴停在F后转回局部0°，前置灰度中心应落到D点。
     * 若此时尚未压到黑线，测试四沿局部0°继续低速直行，直到灰度连续见线
     * 才确认D点并声光提示；测试三仍按原有路线直接进入D→A循迹。
     */
    if (!task4_gyro_turn_to(local_zero_heading_md))
    {
        task4_show_gyro_error();
        return false;
    }
    if ((use_task4_params != 0U) &&
        !task4_seek_point_gray_forward("D", local_zero_heading_md))
    {
        return false;
    }
    task2_point_prompt("Point D");

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"D -> A Gray");
    /*
     * D→A继续使用原有连续丢线帧数结束，不附加C→B专用延时，
     * 避免修改测试四C→B时连带改变A点停车位置。
     */
    task34_follow_arc_reference(arc_base_pwm, arc_min_count, arc_line_kp,
                                arc_lost_limit, arc_seen_limit,
                                arc_loop_delay_ms, 0U, 1U);

    /*
     * A点提示直接由D→A灰度循迹的线尾确认触发。提示完成后测试四再执行
     * 航向回正，回正结果只影响下一圈出发姿态，不参与本次到点提示判定。
     */
    task2_point_prompt("Point A");
    if ((use_task4_params != 0U) &&
        !task4_gyro_turn_to(TASK4_ROUTE_A_RETURN_HEADING_MD))
    {
        task4_show_gyro_error();
        return false;
    }

    if (final_stop != 0U)
    {
        stop_motor();
    }
    return true;
}

static NOINLINE void task3_drive_second_loop(void)
{
    (void)task34_drive_route_internal(1U, 1U, 0U, 0U);
}

/*
 * 第四问点位附近的短距离前进也保持进入时的航向，防止固定时间开环动作在连续四圈
 * 中逐步积累偏差。该动作只按时间结束，不参与红外点位判断。
 */
static bool task4_forward_by_time(uint32_t forward_ms)
{
    int32_t target_heading_md = task4_get_heading_md();
    int32_t heading_error_md;
    int correction;
    uint32_t start_tick = TimerA1_get_tick_100us();
    uint32_t duration_ticks = forward_ms * 10U;

    task4_gyro_sync_time();
    while ((TimerA1_get_tick_100us() - start_tick) < duration_ticks)
    {
        if (!task4_gyro_update())
        {
            stop_motor();
            return false;
        }

        heading_error_md = task4_normalize_angle_md(target_heading_md -
                                                     task4_get_heading_md());
        correction = (int)(heading_error_md / TASK4_STRAIGHT_KP_DIV);
        if (correction > TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = TASK4_STRAIGHT_MAX_CORRECTION;
        }
        else if (correction < -TASK4_STRAIGHT_MAX_CORRECTION)
        {
            correction = -TASK4_STRAIGHT_MAX_CORRECTION;
        }

        set_motor_speed(limit_pwm(TASK1_BASE_PWM + correction),
                        limit_pwm(TASK1_BASE_PWM - correction));
        delay_ms(TASK1_LOOP_DELAY_MS);
    }

    stop_motor();
    return true;
}

/* 显示第四问陀螺仪故障并保持停车，提示用户检查 3.3V、共地及 PA0/PA1 接线。 */
static void task4_show_gyro_error(void)
{
    stop_motor();
    buzzer_off();
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MPU6050 ERROR");
    OLED_ShowString(0, 2, (u8 *)"SDA:PA0");
    OLED_ShowString(0, 4, (u8 *)"SCL:PA1");
    printf("task4 MPU6050 error, check 3V3/GND/PA0/PA1\r\n");
    point_signal_once();
    delay_ms(500U);
}

/*
 * 第四问复用当前测试3的路线顺序与实车角度：A→C、C→B、B→D、D→A连续运行
 * 四圈。A→C和B→D采用陀螺仪直线闭环，C→B灰度循迹时同步积分航向。D点按
 * 测试3当前设置原地右转30°后，D→A只使用灰度传感器并至少运行4.5s；到A后
 * 以灰度路线出口方向重新建立0°基准，再执行左转补偿，避免使用未积分的旧航向。
 */
static NOINLINE void task4_drive_four_loops(void)
{
    uint8_t loop;

    for (loop = 0U; loop < 4U; loop++)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, (u8 *)"Task4 Loop:");
        OLED_ShowNum(96, 0, (uint32_t)loop + 1U, 1, 16);
        delay_ms(100U);

        /*
         * 测试4与测试3使用相同路线结构，但传入测试4独立参数。
         * 两套参数当前数值一致，后续可以分别调整。
         */
        if (!task34_drive_route_internal(1U, 0U, 1U,
                                          (loop > 0U) ? 1U : 0U))
        {
            return;
        }

        /*
         * 第1、2、3圈回到A并完成回正、声光提示后，沿A点回正航向短距离
         * 前进，为下一圈准备起始位置。第4圈结束后保持停车。
         */
        if ((loop < 3U) &&
            !task4_forward_by_count("A-NEXT",
                                     TASK4_ROUTE_A_FORWARD_COUNT,
                                     TASK4_ROUTE_STRAIGHT_PWM,
                                     TASK4_ROUTE_A_RETURN_HEADING_MD))
        {
            task4_show_gyro_error();
            return;
        }
    }

    stop_motor();
}

static NOINLINE void task1_drive_a_to_b(void);

static NOINLINE void run_selected_task(char key)
{
    OLED_Clear();

    switch (key)
    {
    case 'A':
        OLED_ShowString(0, 0, (u8 *)"Run Task1");
        OLED_ShowString(0, 2, (u8 *)"A -> B");
        delay_ms(300);
        task1_drive_a_to_b();
        break;
    case 'B':
        task2_drive_first_loop();
        break;
    case 'C':
        task3_drive_second_loop();
        break;
    case 'D':
        task4_drive_four_loops();
        break;
    case '#':
        sensor_collect_mode();
        break;
    case '*':
        motor_direction_test_mode();
        break;
    default:
        stop_motor();
        break;
    }
}
static NOINLINE void task1_drive_a_to_b(void)
{
    /*
     * 测试一完整复用测试二A→B控制：启动时校准MPU6050并把当前车头定义为0°，
     * 行驶期间由陀螺仪航向误差修正左右轮PWM；超过最低距离后，灰度传感器
     * 连续命中B点横线即立即停车。编码器最大计数只承担传感器异常安全边界。
     */
    if (!task4_gyro_prepare())
    {
        task4_show_gyro_error();
        return;
    }

    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"A -> B Gray");
    if (!task4_drive_straight_until_line("A-B", STRAIGHT_LINE_MIN_COUNT,
                                          STRAIGHT_LINE_MAX_COUNT,
                                          TASK1_BASE_PWM,
                                          STRAIGHT_LINE_HIT_LIMIT,
                                          TASK23_HEADING_AB_MD) ||
        !task4_gyro_turn_to(TASK23_HEADING_AB_MD))
    {
        task4_show_gyro_error();
        return;
    }

    /*
     * 声光提示只在与测试二相同的A→B控制正常返回后执行，避免陀螺仪故障时
     * run_selected_task仍无条件鸣叫，造成已经到达B点的错误提示。
     */
    task2_point_prompt("Point B");
}

int main(void)
{
    char task_key;

    SYSCFG_DL_init();
    uart0_init(115200);
    timerg6_pwm_rgb_init();
    TimerA1_init();
    keyboard_init();

    OLED_Init();
    OLED_Clear();

    stop_motor();
    buzzer_off();
    set_RGB(0x00, 0x00, 0x20);
    printf("task menu ready\r\n");

    while (1)
    {
        show_task_menu();
        task_key = wait_task_key();
        printf("run task %c\r\n", task_key);
        run_selected_task(task_key);
        stop_motor();
        buzzer_off();
        delay_ms(300);
    }
}
