#include "func_TxHandle.h"
#include "drv_HW_TIM.h"
#include "drv_HW_UART.h"
#include "func_RxHandle.h"

typedef struct
{
    uint32_t random_0;
    uint32_t random_1;
    uint32_t tick_0;
    uint32_t tick_1;
    uint32_t sum;
} TxData_Frame_t;

// 串口句柄
UART_HandleTypeDef *huart;

// 发送数据结构体
TxData_Frame_t data;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 更新数据
    data.random_0 = Read_RxData(0, 0);
    data.random_1 = Read_RxData(1, 0);
    data.tick_0 = Read_RxData(0, 1);
    data.tick_1 = Read_RxData(1, 1);
    data.sum = data.random_0 + data.random_1 + data.tick_0 + data.tick_1;
    // 启动发送
    HW_UartDma_Start(huart, (uint8_t *)&data, sizeof(data));
}

/**
 * @brief 发送处理初始化
 * @author dty
 */
void TxHandle_Init(void)
{
    // 初始化发送串口及DMA
    HW_UARTandDMA_Init(1);
    // 获得串口句柄
    huart = Get_UART_Dev(2);
    // 初始化定时器
    HW_TIM_Init();
    // 更改频率
    HW_TimFrequency_Change(2000);
    // 启动定时器
    HW_Tim_Start();
}
