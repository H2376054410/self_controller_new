#ifndef __FUNC_GPIO_CTRL_H__
#define __FUNC_GPIO_CTRL_H__

#include "rtdef.h"
#include <board.h>

/*气缸与吸盘*/
#define AIRPUMP_PIN GET_PIN(B, 3)
#define SOLENEOID_PIN GET_PIN(D, 2)
/*********************************指示灯*********************************/

/**
 * @brief 电源指示灯
 */
void PowerPL_Init(void);

/*******************************气泵控制封装*****************************/

typedef enum
{
    AirPump_Open = 0,    // 打开气泵
    AirPump_Close,       // 关闭气泵
    SolenoidValve_Open,  // 打开电磁阀
    SolenoidValve_Close, // 关闭电磁阀
} Sucker_e;              // 吸盘状态枚举体

/**
 * @brief 吸盘状态设定
 * @param SuckerState
 */
void SuckerState_Set(Sucker_e SuckerState);

/**
 * @brief 电磁阀延迟关闭
 */
void Sucker_DelayClose(rt_tick_t time);

#endif /*__FUNC_GPIO_CTRL_H__*/
