/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-06     thread-liu   first version
 */

#include "board.h"

#if defined(BSP_USING_FDCAN1) || defined(BSP_USING_FDCAN2)

#include "drv_fdcan.h"
#include "stm32h7xx_hal_fdcan.h"

#define LOG_TAG             "drv.fdcan"
#include <drv_log.h>

#ifdef BSP_USING_FDCAN1
static struct stm32_fdcan drv_fdcan1 =
    {
        .name = "fdcan1",
        .fdcan.Instance = FDCAN1,
};
#endif

#ifdef BSP_USING_FDCAN2
static struct stm32_fdcan drv_fdcan2 =
    {
        .name = "fdcan2",
        .fdcan.Instance = FDCAN2,
};
#endif

static rt_err_t rt_fdcan_init(rt_device_t dev)
{
    struct stm32_fdcan *drv_fdcan;

    RT_ASSERT(dev);
    drv_fdcan = (struct stm32_fdcan *)dev;
    RT_ASSERT(drv_fdcan);
    drv_fdcan->fdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    drv_fdcan->fdcan.Init.Mode = FDCAN_MODE_NORMAL;
    drv_fdcan->fdcan.Init.AutoRetransmission = DISABLE;
    drv_fdcan->fdcan.Init.TransmitPause = DISABLE;
    drv_fdcan->fdcan.Init.ProtocolException = DISABLE;
    drv_fdcan->fdcan.Init.NominalPrescaler = 15;
    drv_fdcan->fdcan.Init.NominalSyncJumpWidth = 1;
    drv_fdcan->fdcan.Init.NominalTimeSeg1 = 2;
    drv_fdcan->fdcan.Init.NominalTimeSeg2 = 2;
    drv_fdcan->fdcan.Init.DataPrescaler = 1;
    drv_fdcan->fdcan.Init.DataSyncJumpWidth = 1;
    drv_fdcan->fdcan.Init.DataTimeSeg1 = 1;
    drv_fdcan->fdcan.Init.DataTimeSeg2 = 1;
    // drv_fdcan->fdcan.Init.MessageRAMOffset = 0;
    if (drv_fdcan->fdcan.Instance == FDCAN1)
        drv_fdcan->fdcan.Init.MessageRAMOffset = 0x0000;
    else if (drv_fdcan->fdcan.Instance == FDCAN2)
        drv_fdcan->fdcan.Init.MessageRAMOffset = drv_fdcan1.fdcan.msgRam.EndAddress - SRAMCAN_BASE;

    drv_fdcan->fdcan.Init.StdFiltersNbr = 0;
    drv_fdcan->fdcan.Init.ExtFiltersNbr = 0;
    if (drv_fdcan->fdcan.Instance==FDCAN1)
    {
        drv_fdcan->fdcan.Init.RxFifo0ElmtsNbr = 32;
        drv_fdcan->fdcan.Init.RxFifo1ElmtsNbr = 0;
    }
    else if (drv_fdcan->fdcan.Instance == FDCAN2)
    {
        drv_fdcan->fdcan.Init.RxFifo0ElmtsNbr = 32;
        drv_fdcan->fdcan.Init.RxFifo1ElmtsNbr = 0;
    }
    drv_fdcan->fdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    drv_fdcan->fdcan.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    drv_fdcan->fdcan.Init.RxBuffersNbr = 0;
    drv_fdcan->fdcan.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    drv_fdcan->fdcan.Init.TxEventsNbr = 0;
    drv_fdcan->fdcan.Init.TxBuffersNbr = 0;
    drv_fdcan->fdcan.Init.TxFifoQueueElmtsNbr = 16;
    drv_fdcan->fdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    drv_fdcan->fdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&drv_fdcan->fdcan) != HAL_OK)
    {
        return -RT_ERROR;
    }
    HAL_FDCAN_Start(&drv_fdcan->fdcan);

    return RT_EOK;
}

static rt_err_t rt_fdcan_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct stm32_fdcan *drv_fdcan;

    RT_ASSERT(dev);
    drv_fdcan = (struct stm32_fdcan *)dev;
    RT_ASSERT(drv_fdcan);
    if (oflag & RT_DEVICE_FLAG_INT_RX)
    {
        if(drv_fdcan->fdcan.Instance==FDCAN1)
        {
            HAL_FDCAN_ActivateNotification(&drv_fdcan->fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        }
        else if (drv_fdcan->fdcan.Instance == FDCAN2)
        {
            HAL_FDCAN_ActivateNotification(&drv_fdcan->fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        }
    }
    return RT_EOK;
}

rt_size_t rt_fdcan_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct stm32_fdcan *drv_fdcan;
    struct rt_can_msg* can_msg = (struct rt_can_msg*)buffer;
    RT_ASSERT(can_msg);
    RT_ASSERT(dev);
    drv_fdcan = (struct stm32_fdcan *)dev;
    RT_ASSERT(drv_fdcan);

    HAL_StatusTypeDef staus = HAL_OK;

    FDCAN_RxHeaderTypeDef g_Can1RxHeader;
    if (drv_fdcan->fdcan.Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(&drv_fdcan->fdcan, FDCAN_RX_FIFO0, &g_Can1RxHeader, can_msg->data);
    }
    else if (drv_fdcan->fdcan.Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(&drv_fdcan->fdcan, FDCAN_RX_FIFO0, &g_Can1RxHeader, can_msg->data);
    }
    if (staus != HAL_OK)
        return 0;
    can_msg->id = g_Can1RxHeader.Identifier;
    can_msg->len = g_Can1RxHeader.DataLength;
    return can_msg->len;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN1)
    {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
        {
            if(drv_fdcan1.dev.rx_indicate!=RT_NULL)
                drv_fdcan1.dev.rx_indicate(&drv_fdcan1.dev, 8);
                HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        }
    }
    else if (hfdcan->Instance == FDCAN2)
        {
            if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
            {
                if (drv_fdcan2.dev.rx_indicate != RT_NULL)
                    drv_fdcan2.dev.rx_indicate(&drv_fdcan2.dev, 8);
                HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            }
        }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if (hfdcan->Instance == FDCAN1)
    {
        if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
        {
            if (drv_fdcan1.dev.rx_indicate != RT_NULL)
                drv_fdcan1.dev.rx_indicate(&drv_fdcan1.dev, 8);
            HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
        {
            if (drv_fdcan2.dev.rx_indicate != RT_NULL)
                drv_fdcan2.dev.rx_indicate(&drv_fdcan2.dev, 8);
            HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        }
    }
}

rt_size_t rt_fdcan_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct stm32_fdcan *drv_fdcan;

    RT_ASSERT(dev);
    drv_fdcan = (struct stm32_fdcan *)dev;
    RT_ASSERT(drv_fdcan);
    RT_ASSERT(size == sizeof(struct rt_can_msg));
    struct rt_can_msg* can_msg = (struct rt_can_msg *)buffer;
    FDCAN_TxHeaderTypeDef TxHeader = {0};

    /* 配置发送参数 */
    TxHeader.Identifier = can_msg->id;                      /* 设置接收帧消息的ID */
    TxHeader.IdType = FDCAN_STANDARD_ID;              /* 标准ID */
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;          /* 数据帧 */
    TxHeader.DataLength = can_msg->len >= 8 ? can_msg->len << 16 : FDCAN_DLC_BYTES_8; /* 发送数据长度 */
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  /* 设置错误状态指示 */
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           /* 开启可变波特率 */
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /* FDCAN格式 */
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /* 用于发送事件FIFO控制, 不存储 */
    TxHeader.MessageMarker = 0;                       /* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF */

    /* 添加数据到TX FIFO */
    if(HAL_FDCAN_AddMessageToTxFifoQ(&drv_fdcan->fdcan, &TxHeader, can_msg->data)==HAL_OK)
        return can_msg->len;
    else
        return 0;
}

void FDCAN1_IT0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_FDCAN_IRQHandler(&drv_fdcan1.fdcan);

    /* leave interrupt */
    rt_interrupt_leave();
}

void FDCAN1_IT1_IRQHandler(void)
{
   /* enter interrupt */
   rt_interrupt_enter();

   HAL_FDCAN_IRQHandler(&drv_fdcan1.fdcan);

   /* leave interrupt */
   rt_interrupt_leave();
}

void FDCAN2_IT0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_FDCAN_IRQHandler(&drv_fdcan2.fdcan);

    /* leave interrupt */
    rt_interrupt_leave();
}

void FDCAN2_IT1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_FDCAN_IRQHandler(&drv_fdcan2.fdcan);

    /* leave interrupt */
    rt_interrupt_leave();
}

int fdcan_init(void)
{

#ifdef BSP_USING_FDCAN1
    /* register FDCAN1 device */
    drv_fdcan1.dev.type = RT_Device_Class_CAN;
    drv_fdcan1.dev.init = rt_fdcan_init;
    drv_fdcan1.dev.open = rt_fdcan_open;
    drv_fdcan1.dev.close = RT_NULL;
    drv_fdcan1.dev.read = rt_fdcan_read;
    drv_fdcan1.dev.write = rt_fdcan_write;
    drv_fdcan1.dev.control = RT_NULL;
    drv_fdcan1.dev.user_data = RT_NULL;
    rt_device_register(&(drv_fdcan1.dev),
                       drv_fdcan1.name,0);
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_FDCAN2
    /* register FDCAN2 device */
    drv_fdcan2.dev.type = RT_Device_Class_CAN;
    drv_fdcan2.dev.init = rt_fdcan_init;
    drv_fdcan2.dev.open = rt_fdcan_open;
    drv_fdcan2.dev.close = RT_NULL;
    drv_fdcan2.dev.read = rt_fdcan_read;
    drv_fdcan2.dev.write = rt_fdcan_write;
    drv_fdcan2.dev.control = RT_NULL;
    drv_fdcan2.dev.user_data = RT_NULL;
    rt_device_register(&(drv_fdcan2.dev),
                       drv_fdcan2.name,0);
#endif /* BSP_USING_CAN2 */

    return RT_EOK;
}
INIT_DEVICE_EXPORT(fdcan_init);


#endif
