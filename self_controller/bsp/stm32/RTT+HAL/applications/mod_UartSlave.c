#include "mod_UartSlave.h"
#include "func_RxHandle.h"
#include "func_TxHandle.h"

/**
 * @brief 初始化并启动串口从机
 * @author dty
 */
void UartSlave_Init_and_Start(void)
{
    RxHandle_Init();
    TxHandle_Init();
}
