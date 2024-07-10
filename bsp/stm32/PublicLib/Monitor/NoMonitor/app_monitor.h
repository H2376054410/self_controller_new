#ifndef __APP_MONITOR_H__
#define __APP_MONITOR_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mod_Monitor.h"
#include "func_Alarm.h"
#ifdef BSP_USING_WDT
#include "drv_HardWdt.h"
#endif

typedef enum
{
    ALARM_NULL = -1,
    ALARM_WHITE = 0,
    ALARM_RED,
    ALARM_BLUE,
    ALARM_GREEN,
    ALARM_YELLOW,
    ALARM_PURPLE,
    ALARM_BROWN,

} Alarm_color_e;

/* ------------------------------------XXX----------------------------------------------------------
CREAD_ID(id);
ADDTOMONITOR_ID("Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
SWDG_START(id);
------------------------------------XXX----------------------------------------------------------*/

/**
 * @brief    该函数初始化监视器
 * @return   RT_ERROR：初始化失败
 */
rt_err_t Monitor_Init(rt_uint32_t stack_size, rt_uint8_t priority, rt_uint32_t tick);

/*监视器初始化，建议用这个而不是初始化函数*/
#if (defined BSP_USING_MONITOR) || (defined BSP_USING_WDG)
#define MONITOR_INIT(x, y, z) Monitor_Init(x, y, z)
#endif

#define MONITOR_DEHANDLER (NULL)

#if defined BSP_USING_MONITOR
#define ADDTOMONITOR(name, time_threshold, handle, color, if_alarm) Swdg_Add_to_Monitor(name, time_threshold, handle, color, if_alarm)
#define ADDTOMONITOR_ID(name, time_threshold, handle, color, if_alarm, applay_id) applay_id = ADDTOMONITOR(name, time_threshold, handle, color, if_alarm)
#define SWDG_FEED_NAME(name) Swdg_Feed_UseName(name)
#define SWDG_START(ID) Swdg_Start(ID) /* 启动一个指定监视器 */
#define SWDG_FEED(ID) Swdg_Feed(ID)   /*普通喂狗*/
#define CREAD_ID(name) int16_t name = -1

#else
#define MONITOR_INIT(x, y, z) RT_EOK
#define ADDTOMONITOR(name, time_threshold, handle, color, if_alarm) (void)RT_EOK
#define ADDTOMONITOR_ID(name, time_threshold, handle, color, if_alarm, applay_id) (void)RT_EOK
#define SWDG_START(ID) (void)RT_EOK
#define SWDG_FEED(ID) (void)RT_EOK
#define CREAD_ID(name) (void)RT_EOK
#endif



#endif
