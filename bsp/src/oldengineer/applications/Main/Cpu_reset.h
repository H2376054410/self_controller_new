#ifndef __CPU_RESET__
#define __CPU_RESET__

#include "rtdef.h"
#include <board.h>

/**
 * @brief 单片机复位
 */
void CPU_Reset(void);

/**
 * @brief 单片机延时复位
 * @param time
 */
void CPU_DelayReset(rt_tick_t time);

#endif /*__CPU_RESET__*/
