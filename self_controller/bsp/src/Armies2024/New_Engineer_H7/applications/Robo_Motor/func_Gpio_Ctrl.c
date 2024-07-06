/**
 * @file func_Gpio_Ctrl.c
 * @brief 工程机器人气泵和指示灯的控制
 * @author mylj
 * @version 1.0
 * @date 2023-03-16
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "func_Gpio_Ctrl.h"
#include "drv_RemoteCtrl_data.h"
//#include "drv_rgblight.h"
#include <rtthread.h>
rt_base_t MAINBENG =GET_PIN(B,1);//-->left
rt_base_t STORAGEBEN_RIGHT =GET_PIN(B,2);//--right
rt_base_t STORAGEBEN_LEFT =GET_PIN(E,7);//--right
rt_base_t THEVOICE =GET_PIN(B,15);
the_state state_sucker = 
{.main_beng_state=0,
	.left_state=0,
	.right_state=0
};
/*********************************指示灯*********************************/

void set_RGB(rt_uint8_t r, rt_uint8_t g, rt_uint8_t b)
{
    
}

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
     case main_beng_open:
         rt_pin_write(MAINBENG, PIN_LOW);
		     Robo_Control(AirPump_State, Write, (void *)AirPumpState_Open); // 气泵打开状态
		     state_sucker.main_beng_state=1;
         break;
     case main_beng_close:
         rt_pin_write(MAINBENG, PIN_HIGH);
		     Robo_Control(AirPump_State, Write, (void *)AirPumpState_Close); // 气泵关闭状态
		     state_sucker.main_beng_state=0;
         break;
     case left_open:
         rt_pin_write(STORAGEBEN_LEFT, PIN_LOW);
		 		 Robo_Control(Left_AirPump_State, Write, (void *)AirPumpState_Open); // 气泵打开状态
		     state_sucker.left_state=1;
         break;
     case left_close:
         rt_pin_write(STORAGEBEN_LEFT, PIN_HIGH);
		     Robo_Control(Left_AirPump_State, Write, (void *)AirPumpState_Close); // 气泵打开状态
		     state_sucker.left_state=0;
		     break;
		 case right_open:
         rt_pin_write(STORAGEBEN_RIGHT, PIN_LOW);
		     Robo_Control(Right_AirPump_State, Write, (void *)AirPumpState_Open); // 气泵打开状态
		     state_sucker.right_state=1;
		     break;
		 case right_close:
         rt_pin_write(STORAGEBEN_RIGHT, PIN_HIGH);
		     Robo_Control(Right_AirPump_State, Write, (void *)AirPumpState_Close); // 气泵打开状态
		     state_sucker.right_state=0;
		     break;
     default:
         break;
     }
}
void buzzer_set(uint8_t state)
{
 	  rt_pin_write(THEVOICE, state);//zhubeng
}
//气泵拉低打开
//气阀拉高打开
//蜂鸣器
/**
 * @brief GPIO初始化
 */
static int GPIO_Init(void)
{
   rt_pin_mode(STORAGEBEN_RIGHT, PIN_MODE_OUTPUT);
   rt_pin_mode(STORAGEBEN_LEFT, PIN_MODE_OUTPUT);
	 rt_pin_mode(MAINBENG, PIN_MODE_OUTPUT);

//	  rt_pin_mode(THEVOICE, PIN_MODE_OUTPUT);
    rt_pin_write(STORAGEBEN_RIGHT,1);
	  rt_pin_write(STORAGEBEN_LEFT, 1);
	  rt_pin_write(MAINBENG, 1);
    Robo_Control(AirPump_State, Write, (void *)AirPumpState_Close);
    Robo_Control(Right_AirPump_State, Write, (void *)AirPumpState_Close); // 气泵打开状态
    Robo_Control(Left_AirPump_State, Write, (void *)AirPumpState_Close); // 气泵打开状态
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
        }
        break;
    default:
        AirCloseTick = 0;
        AirCloseTick_flag = 0;
        break;
    }
}

INIT_BOARD_EXPORT(GPIO_Init);
