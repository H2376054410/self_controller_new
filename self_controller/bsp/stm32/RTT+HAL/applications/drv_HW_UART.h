#ifndef __DRV_HW_UART_H__
#define __DRV_HW_UART_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

extern void HW_UartDma_Start(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern UART_HandleTypeDef *Get_UART_Dev(int uartx);
extern void HW_UARTandDMA_Init(int IsTx);

#endif /* __DRV_HW_UART_H__ */
