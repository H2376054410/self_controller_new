#ifndef __FUNC_ALARM_H__
#define __FUNC_ALARM_H__
#include <rtthread.h>

#ifdef BSP_USING_RGB_LIGHT
#include "drv_rgblight.h"
#define Set_RGB set_RGB
#endif

/*设定报警周期，单位ms*/
#define ALARM_PERIOD 100

typedef struct
{
    rt_uint8_t pinState;
    rt_uint8_t islock;
} keyStruct_t;

#define ALARM_RTT_PIN ((*RTT_SWITCH_PIN_PORT - 'A') * 16 + RTT_SWITCH_PIN_NUM)

/*------------------------------------根据板子上是否有蜂鸣器来选择----------------------------------------------------------*/

#ifdef BSP_USING_BUZZER
#include "drv_buzzer.h"

/*程序初始化失败和复位的提醒；串口会一直重复打印;蜂鸣器会一直滴答滴答响*/
#define WARN_PROGRAM_EXCEPTION               \
    do                                       \
    {                                        \
        set_buzzer(2000, 1);                 \
        rt_thread_mdelay(50);                \
        set_buzzer(0, 1);                    \
        rt_thread_mdelay(50);                \
    } while (0)

#define ALARM_SET()            \
    do                         \
    {                          \
        set_buzzer(2000, 1);   \
        rt_thread_mdelay(300); \
        set_buzzer(0, 1);      \
        rt_thread_mdelay(400); \
    } while (0)

#define ALARM_RESET()          \
    do                         \
    {                          \
        set_buzzer(900, 1);    \
        rt_thread_mdelay(120); \
        set_buzzer(0, 1);      \
        rt_thread_mdelay(800); \
    } while (0)

#define USER_ALARM_INTERVAL()          \
    do                         \
    {                          \
        set_buzzer(1200, 1);    \
        rt_thread_mdelay(300); \
        set_buzzer(0, 1);      \
        rt_thread_mdelay(800); \
    } while (0)
#else // BSP_USING_BUZZER
#define WARN_PROGRAM_EXCEPTION
#define ALARM_SET
#define ALARM_RESET
#endif
/*----------------------------------------------------------------------------------------------*/
 
// 报警初始化
rt_err_t Alarm_Init(void);
rt_uint8_t getAlarmState(void);
uint8_t attachUserAlarmFun(void (*fun)(void));
#endif
