/**
 * @file func_Gpio_Ctrl.c
 * @brief 工程机器人气泵和指示灯的控制
 * @author mylj
 * @version 1.0
 * @date 2023-03-16
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "func_Gpio_Ctrl.h"
#include "drv_rgblight.h"
#include <rtthread.h>

/*********************************指示灯*********************************/

/**
 * @brief 电源指示灯
 */
void PowerPL_Init(void)
{
    set_RGB(1, 0, 0);
}

/*******************************吸盘控制封装*****************************/

/**
 * @brief 吸盘状态设定
 * @param SuckerState
 */
void SuckerState_Set(Sucker_e SuckerState)
{
    switch (SuckerState)
    {
    case AirPump_Open:
        rt_pin_write(AIRPUMP_PIN, PIN_LOW);
        break;
    case AirPump_Close:
        rt_pin_write(AIRPUMP_PIN, PIN_HIGH);
        break;
    case SolenoidValve_Open:
        rt_pin_write(SOLENEOID_PIN, PIN_LOW);
        break;
    case SolenoidValve_Close:
        rt_pin_write(SOLENEOID_PIN, PIN_HIGH);
        break;

    default:
        break;
    }
}
/**
 * @brief GPIO初始化
 */
static int GPIO_Init(void)
{
    rt_pin_mode(AIRPUMP_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SOLENEOID_PIN, PIN_MODE_OUTPUT); // /*电源指示灯*/
    SuckerState_Set(AirPump_Close);
    SuckerState_Set(SolenoidValve_Close);

    return 0;
}

/**
 * @brief 电磁阀延迟关闭
 */
void Sucker_DelayClose(rt_tick_t time)
{
    static rt_tick_t AirCloseTick;
    static rt_uint8_t AirCloseTick_flag;

    switch (AirCloseTick_flag)
    {
    case 0:
        AirCloseTick = rt_tick_get();
        AirCloseTick_flag = 1;
        break;
    case 1:
        if ((rt_tick_get() - AirCloseTick) > time)
        {
            // 避免电磁阀常开
            AirCloseTick_flag = 0;
            SuckerState_Set(SolenoidValve_Close);
        }
        break;
    default:
        AirCloseTick = 0;
        AirCloseTick_flag = 0;
        break;
    }
}

INIT_BOARD_EXPORT(GPIO_Init);
