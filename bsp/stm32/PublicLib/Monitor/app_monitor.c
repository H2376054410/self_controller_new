#include "app_monitor.h"
#include "mod_Monitor.h"
#include "string.h"
#if (defined(BSP_USING_RGB_LIGHT) || defined(BSP_USING_BUZZER)) && (defined(MONITOR_NEED_ALARM))
#include "func_Alarm.h"
#endif
/* ------------------------------------XXX----------------------------------------------------------
CREAT_ID(id);
ADDTOMONITOR_ID("Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
SWDG_START(id);
SWDG_FEED(id);
------------------------------------XXX----------------------------------------------------------*/

static rt_thread_t Monitor_Thread = RT_NULL; // 监视器线程
static rt_timer_t Monitor_Timer = RT_NULL;   // 监视器定时器
static rt_sem_t Monitor_Sem = RT_NULL;       // 监视器信号量

swdg_info_t *appSwdg_info; // 名字和ID
errSwdgInfo_t errSwdg;

__weak rt_err_t monitorErrHandle(rt_bool_t status)
{
    rt_hw_cpu_reset();
    return RT_EOK;
}

static void appMonitorInit(void)
{
    appSwdg_info = getSwdgInfo();
    rt_memset(&errSwdg, 0, sizeof(errSwdg));
    modMonitorInit();
}

static void Monitor_Timer_Timeout_Handler(void *parameter)
{
    if (Monitor_Sem)
    {
        rt_sem_release(Monitor_Sem);
    }
}

rt_inline void AddErrSwdg(swdg_info_t *swdg)
{
    errSwdg.allNum = getSwdgNum();
    if (errSwdg.errSwdghistory[swdg->swdg_Id])
    { // 记录错误次数
        errSwdg.errSwdghistory[swdg->swdg_Id]->errCounts = (rt_tick_get() - errSwdg.errSwdghistory[swdg->swdg_Id]->swdg_dev.time_deadline) /
                                                           errSwdg.errSwdghistory[swdg->swdg_Id]->swdg_dev.time_threshold;
    }
    else if (errSwdg.errNum < sizeof(errSwdg.errSwdghistory) / sizeof(errSwdg.errSwdghistory[0]) - 1)
    {
        errSwdg.errSwdghistory[swdg->swdg_Id] = swdg;
        errSwdg.errSwdghistory[swdg->swdg_Id]->errCounts++;
        errSwdg.errNum++;
    }
}

/**
 * @brief    监视器线程 遍历整个监视链表
 * @author
 */
static void Monitor_Thread_Entry(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("Monitor_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        SWDG_FEED(id);
#ifdef BSP_USING_MONITOR

        uint16_t index = 0;
        while (SWDG_INITED_FLAG == appSwdg_info[index].swdg_dev.flag_inited) // 需要判断该模块是否真的已经被初始化
        {
            if ((appSwdg_info[index].if_start))
            {
                // 当未被标记为异常 超时未喂狗则将执行处理函数并加入异常链表当中
                if (appSwdg_info[index].swdg_dev.time_deadline < rt_tick_get())
                {
                    AddErrSwdg(&appSwdg_info[index]);
                    // 当未被标记为异常
                    if (appSwdg_info[index].isError == 0)
                    {
                        appSwdg_info[index].isError = RT_TRUE;
#if (defined(BSP_USING_RGB_LIGHT) || defined(BSP_USING_BUZZER)) && (defined(MONITOR_NEED_ALARM))
                        Mlist_Insert(&appSwdg_info[index]); // 挂载异常监视器
#endif
                    }

                    // 只要异常就执行异常处理函数
                    if (appSwdg_info[index].swdg_dev.handle != RT_NULL)
                        (*appSwdg_info[index].swdg_dev.handle)(appSwdg_info[index].isError); // 处理函数
                }
            }
            index++;
        }

#endif // BSP_USING_MONITOR

// 喂硬件看门狗
#ifdef BSP_USING_WDG
        Hwdt_Feed();
#endif
        // 延时
        rt_sem_take(Monitor_Sem, RT_WAITING_FOREVER);
    }
}

/**
 * @brief    该函数初始化监视器
 * @return   RT_ERROR：初始化失败
 */
rt_err_t Monitor_Init(rt_uint32_t stack_size, rt_uint8_t priority, rt_uint32_t tick)
{

/*报警线程初始化*/
#if (defined(MONITOR_USING_BUZZER) || defined(MONITOR_USING_RGB_LIGHT)) && (defined(MONITOR_NEED_ALARM))
    if (Alarm_Init() != RT_EOK)
        return RT_ERROR;
#endif

    appMonitorInit();
    // 监视器信号量创建
    Monitor_Sem = rt_sem_create("monitor sem", 0, RT_IPC_FLAG_FIFO);
    if (!Monitor_Sem)
        return RT_ERROR;
    rt_sem_trytake(Monitor_Sem);     //无等待信号量的获取

    // 监视器定时器创建
    Monitor_Timer = rt_timer_create(
        "monitor timer", Monitor_Timer_Timeout_Handler,
        RT_NULL, MONITOR_PERIOD, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (Monitor_Timer)
    {
        if (rt_timer_start(Monitor_Timer) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;
    // 监视器线程创建
    Monitor_Thread = rt_thread_create(
        "monitor", Monitor_Thread_Entry, RT_NULL,
        stack_size, priority, tick);

    // 查看是否创建成功
    if (Monitor_Thread != RT_NULL)
    {
        if (rt_thread_startup(Monitor_Thread) != RT_EOK)
            return RT_ERROR;
    }
    else
        return RT_ERROR;

        /*硬件看门狗初始化*/
#ifdef BSP_USING_WDG
    if (Hwdt_Init() != RT_EOK)
        return RT_ERROR;

    /* 初始化看门狗以后进行一次喂狗*/
    Hwdt_Feed();
#endif

    return RT_EOK;
}
