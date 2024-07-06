#include "app_KeymouseData_Ctrl.h"
#include "drv_remote.h"
#include "drv_Key_Set.h"
#include "func_Key_Record.h"
#include "drv_thread.h"
#include "app_monitor.h"
/**
 * @brief  处理遥控器和键盘数据的线程
 * @author mylj
 */
static void KeymouseData_entry(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("KeymouseData_entry", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        /* 等待接收到遥控器数据 */
        rt_sem_take(&RoboControl_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
        // 处理遥控器键盘数据
        RC_Key_Process();
    }
}

/**
 * @brief：KeymouseData_Thread_Init初始化
 * @author：mylj
 */
void KeymouseData_Thread_Init(void)
{
    // 将移动用的按键的去抖动设置为1次，提高移动过程响应速度
    Key_SetPressConfirm(FOREWORD_KEY, 1);
    Key_SetPressConfirm(BACK_KEY, 1);
    Key_SetPressConfirm(LEFT_KEY, 1);
    Key_SetPressConfirm(RIGHT_KEY, 1);
    Key_SetPressConfirm(FOREWORD_KEY, 1);

    /* 遥控器数据处理函数创建 */
    rt_thread_t thread = rt_thread_create("KeymouseData",
                                          KeymouseData_entry, RT_NULL,
                                          THREAD_STACK_KEYMOUSEDATA,
                                          THREAD_PRIO_KEYMOUSEDATA,
                                          THREAD_TICK_KEYMOUSEDATA);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
}
