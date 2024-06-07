#include <rtthread.h>

#include "drv_buzzer.h"
#include "drv_remote.h"
#include "drv_thread.h"
#include "drv_Arm_Solve.h"
#include "drv_canthread.h"

#include "func_Gpio_Ctrl.h"
#include "func_encoderCail.h"
#include "func_MusicPlayer.h"
#include "func_ArmMotor_Ctrl.h"
#include "func_ImageMotor_Ctrl.h"
#include "func_ChassisMotor_Ctrl.h"

#include "mod_Can_data.h"
#include "mod_RefSystem.h"
#include "mod_RoboStateCtrl.h"

#include "app_ui.h"
#include "app_monitor.h"
#include "app_RoboCtrl.h"
#include "app_RoboRemote_Ctrl.h"
#include "app_KeymouseData_Ctrl.h"

int main(void)
{
    /*电源指示灯*/
    PowerPL_Init();

    Monitor_Init(2048, 1, 2);
    /*提示音初始化*/
    // 附带开机音效
    SoundDisplay_Init();

    DJI_Init();
    UI_Init();

	
    // /*上电延时*/
    rt_thread_mdelay(100);

    /*序列号初始化*/
    EngineerRobo_Datanum_Find();
    MotorCan_Datanum_Find();

    /*CAN初始化*/
    SPICAN_Init();
    rt_thread_mdelay(50);
    /*电机初始化*/
    ArmMotor_Init();
    ImageMotor_Init();
    Wheels_Motors_Init();
    Send_Slave2_Init();

    /*上电延时*/
    rt_thread_mdelay(50);

    /*遥控相关初始化*/
    // 串口初始化
    remote_uart_init();
    // 遥控器键鼠接收线程
    KeymouseData_Thread_Init();
    // 遥控器键鼠数据处理线程
    RoboremoteCtrl_Init();

    // /*控制线程初始化*/
    ArmCtrl_Thread_Init();

    // /*重载编码器标定数据*/
    // Load_EncoderCaliData();
    return RT_EOK;
}
