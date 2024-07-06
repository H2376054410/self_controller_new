/**
 * @file drv_canthread.c
 * @brief can线程
 * @author mylj
 * @version 1.0
 * @date 2022-12-29
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_canthread.h"
#include "mod_Can_data.h"
#include "drv_thread.h"
#include "app_monitor.h"
static struct rt_semaphore can1_rx_sem; // 用于接收消息的信号量
static rt_device_t can1_dev;            // CAN 设备句柄

static struct rt_semaphore can2_rx_sem; // 用于接收消息的信号量
static rt_device_t can2_dev;            // CAN 设备句柄

/**
 * @brief  can1接收回调
 * @retval RT_EOK
 */
static rt_err_t Can1_Rx_Call(rt_device_t dev, rt_size_t size)
{
    // CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&can1_rx_sem);
    return RT_EOK;
}

/**
 * @brief  can1读取线程
 */
static void Can1_Rx_Thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};
    CREAT_ID(id);
    ADDTOMONITOR_ID("Can1_Rx_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        // 阻塞等待接收信号量
        rt_sem_take(&can1_rx_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
        // 从 CAN 读取一帧数据
        rt_device_read(can1_dev, 0, &rxmsg, sizeof(rxmsg));
        // Can1Receive(&rxmsg);
    }
}

/**
 * @brief  can2接收回调
 * @retval RT_EOK
 */
static rt_err_t Can2_Rx_Call(rt_device_t dev, rt_size_t size)
{
    // CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&can2_rx_sem);
    return RT_EOK;
}

/**
 * @brief  can2读取线程
 */
static void Can2_Rx_Thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};

    CREAT_ID(id);
    ADDTOMONITOR_ID("Can2_Rx_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        // 阻塞等待接收信号量
        rt_sem_take(&can2_rx_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
        // 从 CAN 读取一帧数据
        rt_device_read(can2_dev, 0, &rxmsg, sizeof(rxmsg));
        // Can2Receive(&rxmsg);
    }
}

/**
 * @brief  can1初始化，can1数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
static rt_err_t Can1_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t Can1_thread;

    /* 初始化 CAN 发送接收信号量 */
    res = rt_sem_init(&can1_rx_sem, "can1_rx_sem", 0, RT_IPC_FLAG_FIFO);
    /*配置can驱动*/
    can1_dev = rt_device_find("can1");
    res = rt_device_open(can1_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    res = rt_device_control(can1_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    res = rt_device_control(can1_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    /* 设置接收回调函数*/
    res = rt_device_set_rx_indicate(can1_dev, Can1_Rx_Call);
    /* can接收线程初始化*/
    Can1_thread = rt_thread_create("can1_rx", Can1_Rx_Thread, RT_NULL,
                                   THREAD_STACK_Can1Rx,
                                   THREAD_PRIO_Can1Rx,
                                   THREAD_TICK_Can1Rx);
    if (Can1_thread != RT_NULL)
    {
        res = rt_thread_startup(Can1_thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    return RT_EOK;
}

/**
 * @brief  can2初始化，can2数据处理线程和中断设定
 * @param  None
 * @retval rt_err_t
 */
static rt_err_t Can2_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t Can2_thread;

    /* 初始化 CAN 发送接收信号量 */
    res = rt_sem_init(&can2_rx_sem, "can2_rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 配置can驱动*/
    can2_dev = rt_device_find("can2");
    res = rt_device_open(can2_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    res = rt_device_control(can2_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    /* 设置接收回调函数*/
    res = rt_device_set_rx_indicate(can2_dev, Can2_Rx_Call);
    /* can接收线程初始化*/
    Can2_thread = rt_thread_create("can2_rx", Can2_Rx_Thread, RT_NULL,
                                   THREAD_STACK_Can2Rx,
                                   THREAD_PRIO_Can2Rx,
                                   THREAD_TICK_Can2Rx);
    if (Can2_thread != RT_NULL)
    {
        res = rt_thread_startup(Can2_thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;
    return RT_EOK;
}

/**
 * @brief Can线程初始化
 */
void Can_Init(void)
{
    Can1_Init();
    Can2_Init();
}
