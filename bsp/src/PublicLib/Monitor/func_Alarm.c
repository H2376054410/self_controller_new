#include "func_Alarm.h"
#include "drv_Monitor.h"
#include "drv_gpio.h"
#include "app_monitor.h"

typedef void (*pAlarm_fun)(void);

static alarm_dev_t *fun_alarm_hp;

// 根据按键可以关闭蜂鸣器
static keyStruct_t key;
uint8_t isAlarm = 1; /*可debug关闭蜂鸣器*/

// 自定义报警蜂鸣器等
static pAlarm_fun userAlarmFun[5];
static uint8_t funNum = 0;

uint8_t attachUserAlarmFun(void (*fun)(void))
{
    if (funNum < sizeof(userAlarmFun) / sizeof(userAlarmFun[0]))
    {
        userAlarmFun[funNum] = fun;
        funNum++;
        return 1;
    }
    else
        return 0;
}

// 得到按键状态
static void refreshKeyState(void)
{
    // 每次按下切换状态
    if (!rt_pin_read(ALARM_RTT_PIN))
    {
        if (!key.islock)
        {
            key.pinState = !key.pinState;
            key.islock = 1;
            if (key.pinState)
                InitSwdgHandle(NULL);
            else
                InitSwdgHandle(MONITOR_DEHANDLER);
        }
    }
    else
        key.islock = 0;
}
// 报警线程
static void Alarm_Thread(void *parameter)
{
    key.pinState = 1;
    while (1)
    {
        rt_uint8_t nowIsAlarm = 0;
        refreshKeyState();
        alarm_dev_t *alarm_dev_tem = fun_alarm_hp->next;

        while (alarm_dev_tem != fun_alarm_hp) // 索引
        {
            if (alarm_dev_tem->swdg->isError == 0)
                Mlist_Remove(alarm_dev_tem->swdg->swdg_Id);

            // 是否报警
            if (alarm_dev_tem->swdg->swdg_dev.if_alarm == RT_TRUE && alarm_dev_tem->swdg->if_start)
            {
                /*由于当前不能同时显示蜂鸣器和RGB*/
#if defined(BSP_USING_RGB_LIGHT) && !defined(BSP_USING_BUZZER)
                // RGB灯报警，多个异常看门狗，则轮流显示对应的颜色
                switch (alarm_dev_tem->swdg->swdg_dev.color)
                {
                case ALARM_WHITE:
                    Set_RGB(CORE_WHITE);
                    break;
                case ALARM_RED:
                    Set_RGB(CORE_RED);
                    break;
                case ALARM_BLUE:
                    Set_RGB(CORE_BLUE);
                    break;
                case ALARM_GREEN:
                    Set_RGB(CORE_GREEN);
                    break;
                case ALARM_YELLOW:
                    Set_RGB(CORE_YELLOW);
                    break;
                case ALARM_PURPLE:
                    Set_RGB(CORE_PURPLE);
                    break;
                default:
                    break;
                }
#endif
#ifdef BSP_USING_BUZZER
                if (key.pinState && isAlarm)
                {
                    // ID值多少就响几次
                    for (int i = 0; i < alarm_dev_tem->swdg->swdg_Id; i++)
                    {
                        set_buzzer(2000, 1);
                        rt_thread_mdelay(150);
                        set_buzzer(0, 1);
                        rt_thread_mdelay(400);
                        refreshKeyState();
                    }
                    ALARM_RESET(); // 不同音调，区分多个ID
                }
#endif
            if (!nowIsAlarm)
                nowIsAlarm = 1;
            } // if
            alarm_dev_tem = alarm_dev_tem->next;
        }

        // 表示监视器报警完成一轮
        if (nowIsAlarm && funNum && isAlarm)
            USER_ALARM_INTERVAL();

        // 用户自定义报警函数
        for (int i = 0; i < funNum; i++)
        {
            if (userAlarmFun[i])
            {
                userAlarmFun[i]();
                refreshKeyState();
            }
            else
                break;
        }
        rt_thread_mdelay(ALARM_PERIOD);
    } // while(1)
}

rt_uint8_t getAlarmState(void)
{
    return key.pinState;
}

/**
 * @brief    报警初始化
 * @param    无
 * @return   初始化成功or失败
 * @author   Lvfp
 */
rt_err_t Alarm_Init(void)
{
    rt_thread_t alarm_device = RT_NULL; // 报警线程句柄

    for (int i = 0; i < sizeof(userAlarmFun) / sizeof(userAlarmFun[0]); i++)
        userAlarmFun[i] = NULL;

#ifdef RTT_SWITCH_PIN_NUM
    rt_pin_mode(RTT_SWITCH_PIN_NUM, PIN_MODE_INPUT_PULLUP); /*拨码开关控制 */
#endif

    fun_alarm_hp = getAlarm_hp();

    // 初始化报警双向链表和创建所有监视器对象
    Mlist_Init(fun_alarm_hp);

    // 报警线程创建
    alarm_device = rt_thread_create(
        "alarm",
        Alarm_Thread,
        RT_NULL,
        1024,
        26,
        2);
    // 查看是否创建成功
    if (rt_thread_startup(alarm_device) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
