#ifndef __ENGINEER_CTRL_H
#define __ENGINEER_CTRL_H
#include <rtthread.h>

#define POS_DELTA_SPEED 0.1f
#define POS_DELTA_PIROID 0.002f

#define FOREARMANGLE_TOLERANCE 0.05f



/**
 * @brief   数据处理线程初始化
 * @param   None
 * @return  rt_err_t 是否正常初始化
 * @author  mylj
 */
rt_err_t RoboremoteCtrl_Init(void);

#endif /*__ENGINEER_CTRL_H*/
