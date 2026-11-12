#include "IMU.h"

/*
 * IMU 姿态解算模块。
 *
 * 职责：
 * - 调用底层 LSM6DSV 驱动完成芯片初始化和六轴原始数据读取。
 * - 对陀螺仪静止零偏做在线估计，降低长时间积分漂移。
 * - 使用加速度计观测到的重力方向修正陀螺积分，输出 yaw/pitch/roll 姿态角。
 *
 * 当前硬件：
 * - SPI1: PB16(SCLK)、PB15(MOSI/PICO)、PB14(MISO/POCI)。
 * - IMU GPIO: PC2(INT1)、PC3(INT2)、PC4(CS)，当前驱动采用主循环轮询，未依赖中断脚。
 *
 * 时间基准：
 * - nowtime 由 TIMA1 每 1ms 递增 1。
 * - 四元数积分使用 halfT = dt / 2，其中 dt 单位为秒，因此 halfT = tick_delta / 2000。
 */

#define IMU_LSM6DSV_EXPECTED_WHOAMI        (0x70U)
#define IMU_GYRO_VARIANCE_WINDOW           (100)
#define IMU_GYRO_VARIANCE_THRESHOLD_DPS2   (0.02f)
#define IMU_GYRO_OFFSET_WARMUP_SAMPLES     (100)
#define IMU_ACCEL_NORM_MIN                 (1.0e-6f)
#define IMU_QUAT_NORM_MIN                  (1.0e-6f)

/*
 * Mahony 六轴姿态修正参数。
 *
 * Kp：
 * - 初始化阶段使用较大的比例增益，便于姿态快速贴合重力方向。
 * - 静止零偏稳定后降为 0.5，减少正常运动时加速度扰动对姿态的拉扯。
 *
 * Ki：
 * - 积分项用于吸收慢速陀螺零偏。
 * - 数值较小，避免车辆加减速时把线加速度误认为重力误差并长期积累。
 */
#define IMU_AHRS_KI                         (0.001f)
#define IMU_AHRS_KP_STARTUP                 (10.0f)
#define IMU_AHRS_KP_RUNNING                 (0.5f)

extern volatile uint32_t nowtime;

extern int setup_imu(int use_ln, int accel_en, int gyro_en);
extern int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);

static bool imu_initialized = false;
static uint32_t last_update_tick = 0U;

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

static float exInt = 0.0f;
static float eyInt = 0.0f;
static float ezInt = 0.0f;
static float imu_kp = IMU_AHRS_KP_STARTUP;

static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static float TTangles_gyro[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 25.0f};

static double gyro_window[3][IMU_GYRO_VARIANCE_WINDOW];
static double gyro_sum[3] = {0.0, 0.0, 0.0};
static double gyro_square_sum[3] = {0.0, 0.0, 0.0};
static int gyro_window_filled = 0;
static int gyro_window_index = 0;
static int gyro_offset_warmup_count = 0;

/*
 * 安全倒平方根。
 *
 * 输入：
 * - value：待归一化向量的平方和。
 *
 * 输出：
 * - 返回 1/sqrt(value)，当 value 太小或非法时返回 0。
 *
 * 实现原因：
 * - 姿态解算会对加速度向量和四元数归一化；若 SPI 读数异常导致全零，直接求倒数会产生 NaN。
 */
static float imu_inv_sqrt(float value)
{
    if (value <= IMU_ACCEL_NORM_MIN) {
        return 0.0f;
    }

    return 1.0f / sqrtf(value);
}

/*
 * 浮点限幅辅助函数。
 *
 * 输入：
 * - value：需要限制的浮点值。
 * - min_value/max_value：允许输出的最小值和最大值。
 *
 * 输出：
 * - 当 value 超出边界时返回边界值，否则返回 value 本身。
 *
 * 实现原因：
 * - pitch 角由 asinf() 计算，理论输入范围为 [-1, 1]。
 * - 四元数每轮都会归一化，但单精度浮点仍可能出现 1.0000001 这类微小越界；
 *   若不做限幅，asinf() 会返回 NaN 并污染后续姿态输出。
 */
static float imu_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/*
 * 重置姿态解算和陀螺零偏估计状态。
 *
 * 调用时机：
 * - IMU_init() 成功后调用一次。
 * - 上电初始四元数设为单位四元数，表示尚未发生旋转。
 *
 * 输出：
 * - 清空积分误差、滑动窗口和零偏缓存；last_update_tick 对齐当前 nowtime。
 */
static void imu_reset_runtime_state(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
    imu_kp = IMU_AHRS_KP_STARTUP;
    last_update_tick = nowtime;

    gyro_offset[0] = 0.0f;
    gyro_offset[1] = 0.0f;
    gyro_offset[2] = 0.0f;
    gyro_sum[0] = 0.0;
    gyro_sum[1] = 0.0;
    gyro_sum[2] = 0.0;
    gyro_square_sum[0] = 0.0;
    gyro_square_sum[1] = 0.0;
    gyro_square_sum[2] = 0.0;
    gyro_window_filled = 0;
    gyro_window_index = 0;
    gyro_offset_warmup_count = 0;
    memset(gyro_window, 0, sizeof(gyro_window));
    memset(TTangles_gyro, 0, sizeof(TTangles_gyro));
    TTangles_gyro[6] = 25.0f;
}

/*
 * 计算本次四元数积分使用的半周期。
 *
 * 输入：
 * - 使用全局 nowtime，单位为 1ms/tick。
 *
 * 输出：
 * - 返回 dt/2，单位秒。
 *
 * 硬件约束：
 * - nowtime 是无符号递增计数，使用无符号减法可自然处理 32 位回绕。
 * - 若两次调用在同一个 1ms tick 内发生，返回 0，本轮只更新输出缓存不积分。
 */
static float imu_get_half_period_seconds(void)
{
    uint32_t tick = nowtime;
    uint32_t delta = tick - last_update_tick;

    last_update_tick = tick;
    return (float)delta / 2000.0f;
}

/*
 * 陀螺仪滑动方差估计。
 *
 * 输入：
 * - data_dps：当前三轴陀螺仪角速度，单位 dps。
 * - length：滑动窗口长度，当前固定为 100 个样本。
 *
 * 输出：
 * - variance_dps2：三轴方差，单位 dps^2。
 * - average_dps：三轴平均值，单位 dps。
 *
 * 实现原因：
 * - 当车辆静止时，陀螺仪输出方差很小，此时窗口平均值可作为零偏。
 * - 采用 sum/square_sum 递推，避免每次重新遍历 100 个样本，降低主循环负担。
 */
static void imu_calculate_gyro_variance(const float data_dps[3], int length,
                                        float variance_dps2[3], float average_dps[3])
{
    int axis;
    double window_len;

    if ((data_dps == NULL) || (variance_dps2 == NULL) || (average_dps == NULL) ||
        (length <= 0) || (length > IMU_GYRO_VARIANCE_WINDOW)) {
        return;
    }

    if (gyro_window_filled == 0) {
        for (axis = 0; axis < 3; axis++) {
            gyro_window[axis][gyro_window_index] = data_dps[axis];
            gyro_sum[axis] += data_dps[axis];
            gyro_square_sum[axis] += (double)data_dps[axis] * (double)data_dps[axis];
            variance_dps2[axis] = 100.0f;
            average_dps[axis] = 0.0f;
        }
    } else {
        for (axis = 0; axis < 3; axis++) {
            gyro_sum[axis] -= gyro_window[axis][gyro_window_index];
            gyro_square_sum[axis] -= gyro_window[axis][gyro_window_index] * gyro_window[axis][gyro_window_index];

            gyro_window[axis][gyro_window_index] = data_dps[axis];
            gyro_sum[axis] += gyro_window[axis][gyro_window_index];
            gyro_square_sum[axis] += gyro_window[axis][gyro_window_index] * gyro_window[axis][gyro_window_index];
        }
    }

    gyro_window_index++;
    if (gyro_window_index >= length) {
        gyro_window_index = 0;
        gyro_window_filled = 1;
        imu_kp = IMU_AHRS_KP_RUNNING;
    }

    if (gyro_window_filled == 0) {
        return;
    }

    window_len = (double)length;
    for (axis = 0; axis < 3; axis++) {
        average_dps[axis] = (float)(gyro_sum[axis] / window_len);
        variance_dps2[axis] =
            (float)((gyro_square_sum[axis] - (gyro_sum[axis] * gyro_sum[axis] / window_len)) / window_len);
    }
}

/*
 * 读取六轴数据并应用静止零偏校正。
 *
 * 输入：
 * - values：长度至少为 7 的数组。
 *
 * 输出：
 * - values[0..2]：加速度 X/Y/Z，单位 mg。
 * - values[3..5]：扣除静止零偏后的陀螺仪 X/Y/Z，单位 dps。
 * - values[6]：温度，单位摄氏度。
 *
 * 说明：
 * - 底层函数名仍为 bsp_IcmGetRawData 是旧接口兼容名，实际已经读取 LSM6DSV。
 */
static bool imu_read_values(float *values)
{
    float acc_gyro_temp[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 25.0f};
    float variance_gyro[3] = {100.0f, 100.0f, 100.0f};
    float average_gyro[3] = {0.0f, 0.0f, 0.0f};

    if (values == NULL) {
        return false;
    }

    if (bsp_IcmGetRawData(acc_gyro_temp, &acc_gyro_temp[3], &acc_gyro_temp[6]) != 0) {
        return false;
    }

    TTangles_gyro[0] = acc_gyro_temp[0];
    TTangles_gyro[1] = acc_gyro_temp[1];
    TTangles_gyro[2] = acc_gyro_temp[2];
    TTangles_gyro[3] = acc_gyro_temp[3];
    TTangles_gyro[4] = acc_gyro_temp[4];
    TTangles_gyro[5] = acc_gyro_temp[5];
    TTangles_gyro[6] = acc_gyro_temp[6];

    imu_calculate_gyro_variance(&TTangles_gyro[3], IMU_GYRO_VARIANCE_WINDOW,
                                variance_gyro, average_gyro);

    /*
     * 静止零偏更新条件：
     * - 三轴方差均低于阈值，说明车体基本静止。
     * - 至少经历 100 次样本预热，避免刚上电滤波器稳定阶段误写零偏。
     * - 更新零偏后清空积分误差，避免旧误差继续推四元数。
     */
    if ((variance_gyro[0] < IMU_GYRO_VARIANCE_THRESHOLD_DPS2) &&
        (variance_gyro[1] < IMU_GYRO_VARIANCE_THRESHOLD_DPS2) &&
        (variance_gyro[2] < IMU_GYRO_VARIANCE_THRESHOLD_DPS2) &&
        (gyro_offset_warmup_count >= IMU_GYRO_OFFSET_WARMUP_SAMPLES - 1)) {
        gyro_offset[0] = average_gyro[0];
        gyro_offset[1] = average_gyro[1];
        gyro_offset[2] = average_gyro[2];
        exInt = 0.0f;
        eyInt = 0.0f;
        ezInt = 0.0f;
        gyro_offset_warmup_count = 0;
    } else if (gyro_offset_warmup_count < IMU_GYRO_OFFSET_WARMUP_SAMPLES) {
        gyro_offset_warmup_count++;
    }

    values[0] = acc_gyro_temp[0];
    values[1] = acc_gyro_temp[1];
    values[2] = acc_gyro_temp[2];
    values[3] = acc_gyro_temp[3] - gyro_offset[0];
    values[4] = acc_gyro_temp[4] - gyro_offset[1];
    values[5] = acc_gyro_temp[5] - gyro_offset[2];
    values[6] = acc_gyro_temp[6];

    return true;
}

/*
 * 兼容旧代码的原始数据读取接口。
 *
 * 输入输出：
 * - values 格式同 imu_read_values()。
 *
 * 说明：
 * - 旧工程里该函数不是头文件公开接口，但可能被用户调试代码直接引用，因此保留同名符号。
 */
void IMU_getValues(float *values)
{
    (void)imu_read_values(values);
}

/*
 * 六轴 AHRS 四元数更新。
 *
 * 输入：
 * - gx/gy/gz：三轴角速度，单位 rad/s。
 * - ax/ay/az：三轴加速度，单位 g，主要用于观测重力方向。
 *
 * 输出：
 * - 更新模块内 q0/q1/q2/q3 四元数。
 *
 * 实现原因：
 * - LSM6DSV 当前按例程读取加速度和陀螺仪原始值；没有磁力计输入。
 * - 因此这里使用六轴姿态解算，pitch/roll 由重力修正，yaw 由陀螺积分得到。
 */
static void imu_ahrs_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float halfT;
    float tempq0;
    float tempq1;
    float tempq2;
    float tempq3;
    float q0q0;
    float q0q1;
    float q0q2;
    float q1q1;
    float q2q2;
    float q3q3;

    halfT = imu_get_half_period_seconds();
    if (halfT <= 0.0f) {
        return;
    }

    norm = imu_inv_sqrt((ax * ax) + (ay * ay) + (az * az));
    if (norm > 0.0f) {
        ax *= norm;
        ay *= norm;
        az *= norm;

        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        /*
         * 根据当前四元数推算机体坐标系下的重力方向。
         * 加速度计在低动态时测到的重力方向与该估计方向的叉积，就是姿态误差。
         */
        vx = 2.0f * ((q1 * q3) - q0q2);
        vy = 2.0f * (q0q1 + (q2 * q3));
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        ex = (ay * vz) - (az * vy);
        ey = (az * vx) - (ax * vz);
        ez = (ax * vy) - (ay * vx);

        if ((ex != 0.0f) || (ey != 0.0f) || (ez != 0.0f)) {
            exInt += ex * IMU_AHRS_KI * halfT;
            eyInt += ey * IMU_AHRS_KI * halfT;
            ezInt += ez * IMU_AHRS_KI * halfT;

            gx += (imu_kp * ex) + exInt;
            gy += (imu_kp * ey) + eyInt;
            gz += (imu_kp * ez) + ezInt;
        }
    }

    /*
     * 四元数微分方程积分。
     * halfT 已经是 dt/2，因此公式中不再额外除以 2。
     */
    tempq0 = q0 + ((-q1 * gx) - (q2 * gy) - (q3 * gz)) * halfT;
    tempq1 = q1 + ((q0 * gx) + (q2 * gz) - (q3 * gy)) * halfT;
    tempq2 = q2 + ((q0 * gy) - (q1 * gz) + (q3 * gx)) * halfT;
    tempq3 = q3 + ((q0 * gz) + (q1 * gy) - (q2 * gx)) * halfT;

    norm = imu_inv_sqrt((tempq0 * tempq0) + (tempq1 * tempq1) +
                        (tempq2 * tempq2) + (tempq3 * tempq3));
    if (norm <= IMU_QUAT_NORM_MIN) {
        return;
    }

    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

/*
 * 更新并读取当前四元数。
 *
 * 输入：
 * - q：长度至少为 4 的数组。
 *
 * 输出：
 * - 成功时 q[0..3] 为当前四元数并返回 true。
 * - 若 IMU 尚未初始化、参数为空或底层读数失败，则返回 false。
 *
 * 说明：
 * - 加速度从 mg 转成 g，陀螺仪从 dps 转成 rad/s 后进入 AHRS。
 * - 初始化失败时不访问寄存器，避免旧兼容接口在硬件未就绪时触发无意义 SPI 事务。
 */
static bool imu_update_and_get_quaternion(float *q)
{
    float values[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 25.0f};

    if (q == NULL) {
        return false;
    }

    if (!imu_initialized) {
        return false;
    }

    if (!imu_read_values(values)) {
        return false;
    }

    imu_ahrs_update(values[3] * M_PI / 180.0f,
                    values[4] * M_PI / 180.0f,
                    values[5] * M_PI / 180.0f,
                    values[0] / 1000.0f,
                    values[1] / 1000.0f,
                    values[2] / 1000.0f);

    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
    return true;
}

/*
 * 兼容旧代码的四元数读取接口。
 *
 * 输入输出：
 * - q：长度至少为 4 的数组，接收当前四元数。
 */
void IMU_getQ(float *q)
{
    (void)imu_update_and_get_quaternion(q);
}

/*
 * 初始化 IMU。
 *
 * 调用约束：
 * - 调用前必须已经完成 SYSCFG_DL_init()，使 SPI1 和 CS GPIO 处于正确模式。
 * - 建议上电后 delay_ms(100) 再调用，主程序当前已经遵守该顺序。
 *
 * 输出：
 * - 成功后 IMU_isReady() 返回 true。
 * - 失败时打印 WHO_AM_I，便于定位 SPI 模式、CS 或芯片型号问题。
 */
void IMU_init(void)
{
    uint8_t whoami = 0U;

    imu_initialized = false;

    if (setup_imu(1, 1, 1) == 0) {
        imu_reset_runtime_state();
        imu_initialized = true;
        return;
    }

    printf("LSM6DSV IMU init failed\r\n");
    if (IMU_probe_whoami(&whoami) == 0) {
        printf("WHO_AM_I=0x%02x, expect 0x%02x; check PC4 CS and SPI1 PB14-16\r\n",
               (unsigned)whoami, (unsigned)IMU_LSM6DSV_EXPECTED_WHOAMI);
    } else {
        printf("WHO_AM_I read failed; check CS idle high and SPI1 wiring\r\n");
    }
}

/*
 * 查询 IMU 是否初始化成功。
 *
 * 输出：
 * - true：setup_imu() 已成功完成，后续可读取姿态。
 * - false：初始化失败，读取接口会保持静默保护。
 */
bool IMU_isReady(void)
{
    return imu_initialized;
}

/*
 * 获取当前 yaw/pitch/roll 姿态角。
 *
 * 输入：
 * - angles：长度至少为 3 的 float 数组。
 *
 * 输出：
 * - angles[0]：Yaw 航向角，单位度。
 * - angles[1]：Pitch 俯仰角，单位度。
 * - angles[2]：Roll 横滚角，单位度。
 *
 * 注意：
 * - 当前为六轴解算，没有磁力计绝对航向约束，因此 yaw 会随陀螺零偏缓慢漂移。
 * - pitch/roll 由重力方向闭环修正，静止和低动态场景会更稳定。
 */
void IMU_getYawPitchRoll(float *angles)
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float pitch_sin;

    if (angles == NULL) {
        return;
    }

    if (!imu_initialized) {
        angles[0] = 0.0f;
        angles[1] = 0.0f;
        angles[2] = 0.0f;
        return;
    }

    (void)imu_update_and_get_quaternion(q);

    angles[0] = -atan2f((2.0f * q[1] * q[2]) + (2.0f * q[0] * q[3]),
                        (-2.0f * q[2] * q[2]) - (2.0f * q[3] * q[3]) + 1.0f) *
                180.0f / M_PI;
    pitch_sin = imu_clampf((-2.0f * q[1] * q[3]) + (2.0f * q[0] * q[2]), -1.0f, 1.0f);
    angles[1] = -asinf(pitch_sin) * 180.0f / M_PI;
    angles[2] = atan2f((2.0f * q[2] * q[3]) + (2.0f * q[0] * q[1]),
                       (-2.0f * q[1] * q[1]) - (2.0f * q[2] * q[2]) + 1.0f) *
                180.0f / M_PI;
}

/*
 * 获取最近一次底层原始数据缓存。
 *
 * 输入：
 * - zsjganda：长度至少为 7 的数组。
 *
 * 输出：
 * - [0..2] 加速度 mg，[3..5] 原始陀螺 dps，[6] 温度摄氏度。
 *
 * 用途：
 * - 保留旧调试接口，便于串口打印或上层算法观察滤波前数据。
 */
void IMU_TT_getgyro(float *zsjganda)
{
    int i;

    if (zsjganda == NULL) {
        return;
    }

    for (i = 0; i < 7; i++) {
        zsjganda[i] = TTangles_gyro[i];
    }
}

/*
 * 兼容旧 MPU6050 工程留下的接口。
 *
 * 说明：
 * - 当前 LSM6DSV 零偏由 imu_read_values() 内部在线估计。
 * - 保留空实现，避免旧应用层或调试脚本链接失败。
 */
void MPU6050_InitAng_Offset(void)
{
}
