#ifndef __FUNC_GPIO_CTRL_H__
#define __FUNC_GPIO_CTRL_H__

#include "rtdef.h"
#include <board.h>

/*气缸与吸盘*/
#define AIRPUMP_PIN GET_PIN(B, 3)
#define SOLENEOID_PIN GET_PIN(D, 2)
/*********************************指示灯*********************************/
extern rt_base_t RIGHTFA ;
extern rt_base_t STORAGEBENG ;
extern rt_base_t LEFTFA ;
extern rt_base_t MAINBENG ;
extern rt_base_t ZHUFA ;
/**
 * @brief 电源指示灯
 */
void PowerPL_Init(void);

/*******************************气泵控制封装*****************************/
typedef struct
{
	uint8_t main_beng_state;
	uint8_t left_state;
	uint8_t right_state;
} the_state;    

typedef enum
{
    main_beng_open = 0,    // 打开气泵
    main_beng_close,       // 关闭气泵
    left_open,
	  left_close,
	  right_open,
	  right_close,
} Sucker_e;                // 吸盘状态枚举体

/**
 * @brief 吸盘状态设定
 * @param SuckerState
 */
void SuckerState_Set(Sucker_e SuckerState);

/**
 * @brief 电磁阀延迟关闭
 */
void Sucker_DelayClose(rt_tick_t time);
void buzzer_set(uint8_t state);
#endif /*__FUNC_GPIO_CTRL_H__*/
