#ifndef __FUNC_RXHANDLE_H__
#define __FUNC_RXHANDLE_H__

#include "stm32f1xx_hal.h"

extern uint32_t Read_RxData(uint8_t uartx, uint8_t data_type);

extern void RxHandle_Init(void);

#endif /* __FUNC_RXHANDLE_H__ */
