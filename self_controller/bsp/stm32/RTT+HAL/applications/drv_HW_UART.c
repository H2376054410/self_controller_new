#include "drv_HW_UART.h"
#include "drv_HAL_Main.h"

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */

    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */

    /* USER CODE END USART3_IRQn 1 */
}

// /**
//  * @brief This function handles UART4 global interrupt.
//  */
// void UART4_IRQHandler(void)
// {
//     /* USER CODE BEGIN UART4_IRQn 0 */

//     /* USER CODE END UART4_IRQn 0 */
//     HAL_UART_IRQHandler(&huart4);
//     /* USER CODE BEGIN UART4_IRQn 1 */

//     /* USER CODE END UART4_IRQn 1 */
// }

/**
 * @brief This function handles DMA2 channel4 and channel5 global interrupts.
 */
void DMA1_Channel3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Channel4_5_IRQn 0 */

    /* USER CODE END DMA2_Channel4_5_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA2_Channel4_5_IRQn 1 */

    /* USER CODE END DMA2_Channel4_5_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel4 and channel5 global interrupts.
 */
void DMA1_Channel5_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Channel4_5_IRQn 0 */

    /* USER CODE END DMA2_Channel4_5_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
    /* USER CODE BEGIN DMA2_Channel4_5_IRQn 1 */

    /* USER CODE END DMA2_Channel4_5_IRQn 1 */
}

// /**
//  * @brief This function handles DMA2 channel4 and channel5 global interrupts.
//  */
// void DMA2_Channel4_5_IRQHandler(void)
// {
//     /* USER CODE BEGIN DMA2_Channel4_5_IRQn 0 */

//     /* USER CODE END DMA2_Channel4_5_IRQn 0 */
//     HAL_DMA_IRQHandler(&hdma_uart4_tx);
//     /* USER CODE BEGIN DMA2_Channel4_5_IRQn 1 */

//     /* USER CODE END DMA2_Channel4_5_IRQn 1 */
// }

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 1500000;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 1500000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 1500000;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

static void MX_TXDMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Channel4_5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
}
static void MX_RXDMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief 打开指定串口的DMA
 * @author dty
 * @param huart 串口句柄
 * @param pData 缓冲区地址
 * @param Size 缓冲区大小
 */
void HW_UartDma_Start(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (huart == &huart1 || huart == &huart3)
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
    else if (huart == &huart4)
        HAL_UART_Transmit_DMA(huart, pData, Size);
}

/**
 * @brief 返回对应串口的句柄
 * @author dty
 * @param uartx 0：串口1（接收）；
 *              1：串口3（接收）；
 *              2：串口4（发送）；
 * @return UART_HandleTypeDef* 对应串口的句柄
 */
UART_HandleTypeDef *Get_UART_Dev(int uartx)
{
    switch (uartx)
    {
    case 0:
        return &huart1;
    case 1:
        return &huart3;
    case 2:
        return &huart4;
    }
    return NULL;
}

/**
 * @brief 初始化串口（由cubemx生成），
 *        串口1,3为接收串口，初始化接收DMA
 *        串口4为发送串口，初始化发送DMA
 * @author dty
 * @param IsTx 0为接收，1为发送
 */
void HW_UARTandDMA_Init(int IsTx)
{
    if (IsTx)
    {
        MX_TXDMA_Init();
        MX_UART4_Init();
    }
    else
    {
        MX_RXDMA_Init();
        MX_USART1_UART_Init();
        MX_USART3_UART_Init();
    }
}
