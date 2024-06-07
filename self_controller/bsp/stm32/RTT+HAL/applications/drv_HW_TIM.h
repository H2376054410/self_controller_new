#ifndef __DRV_HW_TIM_H__
#define __DRV_HW_TIM_H__

#include "stm32f1xx_hal.h"

extern void HW_TimFrequency_Change(int frequency);
extern void HW_Tim_Start(void);
extern void HW_TIM_Init(void);

#endif /* __DRV_HW_TIM_H__ */
