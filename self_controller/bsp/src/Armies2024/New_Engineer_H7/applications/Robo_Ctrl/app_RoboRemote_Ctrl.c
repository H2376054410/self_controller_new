/**
 * @file app_RoboRemote_Ctrl.c
 * @brief 工程机器人模拟量数据处理文件
 * @author mylj
 * @version 1.0
 * @date 2023-03-16
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */

#include "app_RoboRemote_Ctrl.h"
#include "drv_utils.h"
#include "drv_Vector.h"
#include "drv_thread.h"
#include "drv_remote.h"
#include "drv_encoder.h"
#include "drv_Arm_Solve.h"
#include "drv_ExactSmooth.h"
#include "drv_SetPlanning.h"
#include "drv_Chassis_Solve.h"
#include "drv_gyroscope_data.h"
#include "drv_RemoteCtrl_data.h"
#include "func_Gpio_Ctrl.h"
#include "func_ChassisMotor_Ctrl.h"
#include "mod_RoboStateCtrl.h"
#include "app_monitor.h"
#include "app_RoboCtrl.h"

DMA_HandleTypeDef hdma_spi1_tx;

DMA_HandleTypeDef hdma_spi1_rx;

DMA_HandleTypeDef hdma_uart4_rx;

DMA_HandleTypeDef hdma_uart4_tx;

DMA_HandleTypeDef hdma_uart7_rx;

DMA_HandleTypeDef hdma_uart7_tx;

DMA_HandleTypeDef hdma_usart1_rx;

DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_usart3_rx;

DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_usart6_rx;

DMA_HandleTypeDef hdma_usart6_rx; // 用于应对CubeMx

static struct rt_semaphore RoboremoteCtrl_sem;
/************************************机械臂*************************************/
CustCtrler_Data_s CustCtrlerRC_Data;    // 自定义控制器的值
CustCtrler_Data_s CustCtrlerRC_DataOld; // 上一次自定义控制器的值
ArmPos_Remote_s ArmStateNow_Filter = {
    .Pos.x = 0,
    .Pos.y = 0,
    .Pos.z = 0,
    .Angle.arm_yaw = 0,
    .Angle.arm_yaw = 0,
    .Angle.arm_yaw = 0,
};

VectorXYZ_Str ArmPos_Set = {
    .x = 0,
    .y = 0,
    .z = 0,
};

/*大机械臂滤波后的值*/
static BoomMotor_s Angle_rad;
ArmPos_Remote_s ArmState_Add = {
    .Pos.x = 0,
    .Pos.y = 0,
    .Pos.z = 0,
    .Angle.arm_yaw = 0,
    .Angle.arm_yaw = 0,
    .Angle.arm_yaw = 0,
}; // 最终机械臂部分增量值

/****************************底盘速度设定值****************************/
ChassisMotion_t ChassisSpeState_Set = {.vel.x = 0, .vel.y = 0, .angvel = 0};
ChassisMotor_t ChassisWheelSpe_Set = {0, 0, 0, 0};
/*********************推矿***********************/
/*在坐标遥控和一键推矿模式下，所锁存的BoomYaw和小臂矢量值*/
float BoomYawLatch = 0;
float ForearmYawLatch = 0;
VectorXYZ_Str ForearmVectorLatch1 = {.x = 0, .y = 0, .z = 0};
VectorXYZ_Str ForearmVectorLatch2 = {.x = 0, .y = 0, .z = 0};

float ChassisYaw_RadNow; // 底盘当前yaw角度，相对大地坐标系
// static float ChassisYaw_SpeedNow;                           // 底盘当前转速
float Image_YawRad_SetAdd;  // 云台yaw增量设定值
float Arm_Chassis_Errorrad; // 机械臂和底盘偏差弧度值
VectorXYZ_Str ImagePos_xyz; // 图传位置的xyz值坐标
/**********************************遥控器控制代码部分***********************************/

/**
 * @brief  机械臂控制定时器回调函数
 */
static void RoboremoteCtrl_Handler(void *parameter)
{
    // 每隔1ms 释放一次发送信号量
    rt_sem_release(&RoboremoteCtrl_sem);
}
/*仅仅是为了调试用！！！*/
float SpeedGain;
 ImageMotor_s ImageMotor_Set;      // 图传云台设定值
 ImageMotor_s ImageMotor_Set_ADD;      // 图传云台设定值
ImageMotor_s ImageMotor_Now;              // 图传云台当前值
ArmPos_Remote_s ArmPosState_Add_temp;     // 机械臂部分增量值的中间量
ChassisMotion_t ChassisSpeState_Set_temp; // 底盘部分设定值的中间量

/**
 * @brief   机器人遥控线程
 * @param   parameter
 * @return  None
 */
static void RoboRCtrl_Thread(void *parameter)
{


    CREAT_ID(id);
    ADDTOMONITOR_ID("RoboRCtrl_Thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);

    while (1)
    {
        // 阻塞等待接收信号量
        rt_sem_take(&RoboremoteCtrl_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
        // 初始化
        ArmPosRemote_Struct_Init(&ArmState_Add);
        ArmPosRemote_Struct_Init(&ArmPosState_Add_temp);
        ChassisSpeRemote_Struct_Init(&ChassisSpeState_Set_temp);

        // 从数据服务器读取数据
        ArmPosStateNowRead(&ArmStateNow_Filter);
        // BoomMotor_RadFilter_Read(&Angle_rad); // 大机械臂Yaw为跨圈值
        ArmPosSet_Read(&ArmPos_Set);
        ImageMotor_NowAngle_Read(&ImageMotor_Now);
        // 判断控制模式
        RoboCtrl_State_Judge();
        // 解算遥控器数据,写入平滑滤波器
        SmootFilterhData_Set();
        // 获取速度增益
        SpeedGain = Robo_SpeedGain_Get();
        // 从平滑滤波器内读取数据
        SmootFilterhData_Get(&ArmPosState_Add_temp,
                             &ChassisSpeState_Set_temp.vel,
                             &ImageMotor_Set_ADD,
                             &Image_YawRad_SetAdd,
                             &CustCtrlerRC_Data);
        // 计算增益下的速度
        Robo_GainSpeedCalculate(&ArmPosState_Add_temp,
                                &ChassisSpeState_Set_temp,
                                &ImageMotor_Set_ADD,
                                Image_YawRad_SetAdd,
                                SpeedGain,
                                &ArmState_Add,
                                &ImageMotor_Set,
                                &ChassisSpeState_Set_temp,
                                &Image_YawRad_SetAdd);
        /*底盘yaw转速设定*/
        ChassisSpeState_Set_temp.angvel = Image_YawRad_SetAdd;

        /*底盘部分处理*/
        // 底盘控制方向是否反向
        if ((uint32_t)(Robo_Control(ChassisReverse_State, Read, NULL)) == ChassisReverse)
        {
            // 底盘方向旋转180度
            Vector2D_Rotate(&ChassisSpeState_Set_temp.vel,
                            PI,
                            &ChassisSpeState_Set_temp.vel);
        }
        //  换算速度
        UnitCverRemote2standard(&ChassisSpeState_Set_temp, &ChassisSpeState_Set_temp);
        // 加速度限幅
        Chassis_AccLimit_Process(&ChassisSpeState_Set_temp);
        // 底盘速度限幅
        MotPack_Only_Chass(&ChassisSpeState_Set_temp,
                           &ChassisSpeState_Set);
        // 运动解算
        MecanOmni_Resolve(&ChassisSpeState_Set,
                          &ChassisWheelSpe_Set.speed[0]);
        // 将遥控数据写入数据服务器
        ImageMotor_AngleHope_Write(&ImageMotor_Set);
        ArmPosStateSet_ADD(&ArmState_Add,&jiesuan_again,&Now_Zitai);
        ChassisSpeStateSet_ABS(&ChassisWheelSpe_Set);
    }
}
static char RoboRCtrl_Thread_stack[THREAD_STACK_REMOTECTRL];

/**
 * @brief   数据处理线程初始化
 * @param   None
 * @return  rt_err_t 是否正常初始化
 * @author  mylj
 */
rt_err_t RoboremoteCtrl_Init(void)
{
    static struct rt_thread RoboRCtrl;

    // 初始化信号量
    if (rt_sem_init(&RoboremoteCtrl_sem, "RoboremoteCtrl_sem", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
        return RT_ERROR;

    // 速度增益初始化
    RoboState_NormalGain();
    /*初始化机器人遥控线程*/
    rt_thread_init(&RoboRCtrl,
                   "RoboRCtrl",
                   RoboRCtrl_Thread,
                   RT_NULL,
                   &RoboRCtrl_Thread_stack[0],
                   sizeof(RoboRCtrl_Thread_stack),
                   THREAD_PRIO_REMOTECTRL,
                   THREAD_TICK_REMOTECTRL);
    rt_thread_startup(&RoboRCtrl);

    // 创建线程定时器
    rt_timer_t timer = rt_timer_create("Roboremote_timer",
                                       RoboremoteCtrl_Handler,
                                       RT_NULL,
                                       ROBOREMOTE_TIMER_PIRIOD,
                                       RT_TIMER_FLAG_PERIODIC |
                                           RT_TIMER_FLAG_SOFT_TIMER);

    // 启动定时器
    if (rt_timer_start(timer) != RT_EOK)
        return RT_ERROR;

    return RT_EOK;
}
