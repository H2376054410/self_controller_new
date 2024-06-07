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

DMA_HandleTypeDef hdma_usart6_rx; // 用于应对CubeMx

static struct rt_semaphore RoboremoteCtrl_sem;
/************************************机械臂*************************************/
CustCtrler_Data_s CustCtrlerRC_Data;    // 自定义控制器的值
CustCtrler_Data_s CustCtrlerRC_DataOld; // 上一次自定义控制器的值
ArmPos_Remote_s ArmStateNow_Filter = {
    .Pos.x = 0,
    .Pos.y = 0,
    .Pos.z = 0,
    .Angle.ForearmYaw = 0,
    .Angle.ForearmPitch = 0,
    .Angle.ForearmRoll = 0,
    .ImagePitch = 0,
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
    .Angle.ForearmPitch = 0,
    .Angle.ForearmYaw = 0,
    .Angle.ForearmRoll = 0,
    .ImagePitch = 0,
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

/*****************小陀螺数据*****************/
float ChassisYaw_RadNow; // 底盘当前yaw角度，相对大地坐标系
// static float ChassisYaw_SpeedNow;                           // 底盘当前转速
float GimbalYawRad_SetAdd; // 云台yaw增量设定值
// static float GimbalYawRad_Set;                              // 云台yaw设定值
static float BoomYaw_EncoderCali_Now;                       // 大机械臂yaw编码器校正后的数据
Gyroscope_data_s GyroData;                                  // 陀螺仪数据
float Arm_Chassis_Errorrad;                                 // 机械臂和底盘偏差弧度值
CrossCircleData_s GyroData_Circle = {.Circle_Len = 2 * PI}; // 陀螺仪跨圈数据处理

float test;
VectorXYZ_Str ImagePos_xyz; // 图传位置的xyz值坐标
/**
 * @brief  机械臂控制定时器回调函数
 */
static void RoboremoteCtrl_Handler(void *parameter)
{
    // 每隔1ms 释放一次发送信号量
    rt_sem_release(&RoboremoteCtrl_sem);
}

/**
 * @brief   机器人遥控线程
 * @param   parameter
 * @return  None
 */
static void RoboRCtrl_Thread(void *parameter)
{
    float SpeedGain;
    //    float GimbalYaw_Settemp;              // 云台yaw设定值（写入数据服务器的）
    //    static rt_uint8_t gyro_lock_flag = 1; // 陀螺数据锁
    //    static float gyroyaw_angle = 0;       // 陀螺yaw角度

    rt_uint8_t ImageFollow_Mode;              // 图传跟随模式
    static ImageMotor_s ImageGimbal_Set;      // 图传云台设定值
    ImageMotor_s ImageGimbal_Now;             // 图传云台当前值
    ArmPos_Remote_s ArmPosState_Add_temp;     // 机械臂部分增量值的中间量
    ChassisMotion_t ChassisSpeState_Set_temp; // 底盘部分设定值的中间量

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
        BoomMotor_RadFilter_Read(&Angle_rad); // 大机械臂Yaw为跨圈值
        ArmPosSet_Read(&ArmPos_Set);
        ImageGimbal_Angle_Read(&ImageGimbal_Now);

        GyroYaw_AngleNow_Read(&GyroData.origin.ang);
        GyroYaw_SpeedNow_Read(&GyroData.origin.spe);

        BoomYaw_CoderCaliRad_Read(&BoomYaw_EncoderCali_Now);
        //        GimbalYawRad_Set_Read(&GimbalYaw_Settemp);
        // 判断控制模式
        RoboCtrl_State_Judge();
        // 解算遥控器数据,写入平滑滤波器
        SmootFilterhData_Set();
        // 获取速度增益
        SpeedGain = Robo_SpeedGain_Get();
        // 从平滑滤波器内读取数据
        SmootFilterhData_Get(&ArmPosState_Add_temp,
                             &ChassisSpeState_Set_temp.vel,
                             &GimbalYawRad_SetAdd,
                             &CustCtrlerRC_Data);
        // 计算增益下的速度
        Robo_GainSpeedCalculate(&ArmPosState_Add_temp,
                                &ChassisSpeState_Set_temp,
                                GimbalYawRad_SetAdd,
                                SpeedGain,
                                &ArmState_Add,
                                &ChassisSpeState_Set_temp,
                                &GimbalYawRad_SetAdd);
        /*底盘yaw转速设定*/
        ChassisSpeState_Set_temp.angvel = GimbalYawRad_SetAdd;
        // /*云台yaw角度设定及限幅*/
        // utils_truncate_number(&GimbalYawRad_SetAdd,
        //                       -0.004f, 0.004f);
        // GimbalYawRad_Set = GimbalYawRad_SetAdd + GimbalYaw_Settemp;
        /*陀螺仪数据处理*/
        Gyro_angle2rad(&GyroData);
        CrossCircle_ProcessingSpecial(&GyroData_Circle, GyroData.rad.ang);
        // if ((uint32_t)(Robo_Control(GyroMode_State, Read, NULL)) != GyroDisable_activeMode ||
        //     (uint32_t)(Robo_Control(GyroMode_State, Read, NULL)) != GyroDisable_passiveMode)
        // {
        //     ChassisYaw_RadNow = CrossCircle_Data_Read(&GyroData_Circle); // 底盘yaw角度值
        //     ChassisYaw_SpeedNow = GyroData.rad.spe;
        //     gyro_lock_flag = 1; // 开锁
        // }
        // else
        // {
        //     if (gyro_lock_flag)
        //     {
        //         // 不再修改陀螺仪yaw角度
        //         GyroYaw_radNow_Read(&gyroyaw_angle);
        //         gyro_lock_flag = 0;
        //     }
        //     ChassisYaw_RadNow = gyroyaw_angle;
        // }

        // /*机械臂图传视角*/
        // XYZ_Forearm2Robo(&ArmState_Add.Pos,
        //                  ImageGimbal_Now.ImagePitch,
        //                  ImageGimbal_Now.ImageYaw - Angle_rad.BoomYaw,
        //                  &ArmState_Add.Pos);
        /*矢量推矿*/
        /* 仅在一键兑矿和坐标点移动时需要锁存BoomYaw和小臂矢量值*/
        if (((uint32_t)(Robo_Control(Push_State, Read, NULL)) == PushReady_Mode))
        {
            RoboPos_Latch(&ArmStateNow_Filter,
                          Angle_rad.BoomYaw,
                          &ForearmVectorLatch1,
                          &ForearmVectorLatch2,
                          &BoomYawLatch,
                          &ForearmYawLatch);
            Robo_Control(Push_State, Write, (void *)PushStart_Mode); // 开始推矿
        }
        else if (((uint32_t)(Robo_Control(Push_State, Read, NULL)) == PushStart_Mode))
        {
            ForearmVector2Arm_DeltaGet(&ArmPos_Set,
                                       Angle_rad.BoomYaw,
                                       BoomYawLatch,
                                       ForearmYawLatch,
                                       &ForearmVectorLatch1,
                                       &ForearmVectorLatch2,
                                       POS_DELTA_SPEED,
                                       POS_DELTA_PIROID,
                                       &ArmState_Add);
        }
        // /*陀螺处理*/
        // if ((uint32_t)(Robo_Control(GyroMode_State, Read, NULL)) == GyroMode) // 正常陀螺模式
        // {
        //     // 小陀螺转速
        //     ChassisSpeState_Set_temp.angvel = 200.0f; // 小陀螺转速
        // }
        // else if ((uint32_t)(Robo_Control(GyroMode_State, Read, NULL)) == ChassisFollowing) // 底盘跟随模式
        // {
        //     // 跟随模式
        //     MotModule_FollowPid(-PI / 2.0f,
        //                         -BoomYaw_EncoderCali_Now,
        //                         RT_TRUE,
        //                         &ChassisSpeState_Set_temp.angvel);
        //     ChassisSpeState_Set_temp.angvel += GimbalYawRad_SetAdd * 3000; // 转速补偿
        // }
        // else
        // {
        // }

        /*图传电机跟随*/
        // TODO:待测试
        ImageFollow_Mode = (uint32_t)Robo_Control(ImageFollow_State, Read, NULL);
        switch (ImageFollow_Mode)
        {
        case Following_Mode:
            // 换云台系
            ImagePos_Resolve(&Angle_rad, &ImagePos_xyz);
            ArmImagePos2ImageAngle(&ArmStateNow_Filter.Pos,
                                   Angle_rad.BoomYaw,
                                   ArmStateNow_Filter.Angle.ForearmYaw,
                                   ArmStateNow_Filter.Angle.ForearmPitch,
                                   &ImagePos_xyz,
                                   &ImageGimbal_Set);
            ImageMotor_AngleHope_Write(&ImageGimbal_Set);
            break;
        case Notfollowing_Mode:
            ImageGimbal_Set.ImageYaw = PI / 2.0f;
            ImageMotor_YawAngleHope_Write(ImageGimbal_Set.ImageYaw);
            break;
        default:
            break;
        }

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
        // // 旋转正方向到云台坐标系
        // Vector2D_Rotate(&ChassisSpeState_Set.vel,
        //                 BoomYaw_EncoderCali_Now + PI / 2.0f,
        //                 &ChassisSpeState_Set.vel);
        // 运动解算
        MecanOmni_Resolve(&ChassisSpeState_Set,
                          &ChassisWheelSpe_Set.speed[0]);
        // 将遥控数据写入数据服务器
        ArmPosStateSet_ADD(&ArmState_Add);
        ChassisSpeStateSet_ABS(&ChassisWheelSpe_Set);
        // GyroYaw_radNow_Write(ChassisYaw_RadNow);        // 陀螺仪弧度值
        // GimbalYawRad_Set_Write(GimbalYawRad_Set);       // 云台设定角度
        // ChassisYawSpeed_Now_Write(ChassisYaw_SpeedNow); // 底盘当前转速
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
