#include "MOTOR/motor_qei.h"

/*
 * QEI 初始化与读数，流程参考 SDK timg_qei_mode 例程：
 * SYSCFG_DL_QEI_*_init() 已完成引脚/模式配置，此处启动计数器。
 */

static GPTIMER_Regs *motor_qei_inst(motor_id_t id)
{
    switch (id) {
    case MOTOR_M1:
        return QEI_M1_INST;
    case MOTOR_M2:
        return QEI_1_INST;
    default:
        return NULL;
    }
}

void motor_qei_init(void)
{
    /*
     * 必须先 SYSCFG_DL_init()。Load=65535 由 SysConfig 设置，计数在 PHA/PHB 边沿更新。
     */
    DL_TimerG_startCounter(QEI_M1_INST);
    DL_TimerG_startCounter(QEI_1_INST);
}

uint16_t motor_qei_get_count(motor_id_t id)
{
    GPTIMER_Regs *inst = motor_qei_inst(id);

    if (inst == NULL) {
        return 0U;
    }

    return (uint16_t)DL_TimerG_getTimerCount(inst);
}

bool motor_qei_get_direction_up(motor_id_t id)
{
    GPTIMER_Regs *inst = motor_qei_inst(id);

    if (inst == NULL) {
        return true;
    }

    return (DL_TimerG_getQEIDirection(inst) == DL_TIMER_QEI_DIR_UP);
}
