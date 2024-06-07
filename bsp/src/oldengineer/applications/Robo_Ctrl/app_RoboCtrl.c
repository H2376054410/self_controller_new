/**
 * @file app_RoboCtrl.c
 * @brief 机械人控制线程
 * @author mylj
 * @version 1.0
 * @date 2022-12-30
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "app_RoboCtrl.h"
#include "drv_utils.h"
#include "drv_thread.h"
#include "drv_motor_Locked.h"
#include "drv_SetPlanning3D.h"
#include "func_Dataserver.h"
#include "func_encoderCail.h"
#include "func_MusicPlayer.h"
#include "func_ArmMotor_Ctrl.h"
#include "func_ImageMotor_Ctrl.h"
#include "func_ChassisMotor_Ctrl.h"
#include "mod_Can_data.h"
#include "mod_RoboStateCtrl.h"
#include "app_monitor.h"

static struct rt_semaphore ArmCtrl_sem; // 用于控制的信号量

// 图传云台
ImageState_Data_s ImageState_Data = {
    .AngleNowFilter.ImageYaw = 0,
    .AngleNowFilter.ImagePitch = 0,
    .AngleHope.ImageYaw = 0,
    .AngleHope.ImagePitch = 0,
    .SpeedNowFilter.ImageYaw = 0,
    .SpeedNowFilter.ImagePitch = 0,
};

BoomState_Data_s BoomState_Data = {
    .AngleNowFilter.BoomYaw = 0,
    .AngleNowFilter.BoomPitch1 = 0,
    .AngleNowFilter.BoomPitch2 = 0,

    .AngleHope.BoomYaw = PI / 2,
    .AngleHope.BoomPitch1 = 2.268f,
    .AngleHope.BoomPitch2 = -0.226f,

    .SpeedNowFilter.BoomYaw = 0,
    .SpeedNowFilter.BoomPitch1 = 0,
    .SpeedNowFilter.BoomPitch2 = 0,
}; // 大机械臂数据

ForearmState_Data_s ForearmState_Data =
    {
        .AngleHope.ForearmYaw = PI / 2.0f,
        .AngleHope.ForearmPitch = 0.0f,
        .AngleHope.ForearmRoll = 0.0f,

        .SpeedNowFilter.ForearmYaw = 0,
        .SpeedNowFilter.ForearmPitch = 0,
        .SpeedNowFilter.ForearmRoll = 0,
}; // 小机械臂数据

ArmPosState_s ArmPosState_Data = {
    .ArmPos_SetOld.x = 0,
    .ArmPos_SetOld.y = 0,
    .ArmPos_SetOld.z = 0,

    .ArmPos_SetPlanOld.x = 0,
    .ArmPos_SetPlanOld.y = 0,
    .ArmPos_SetPlanOld.z = 0,
}; // 机械臂位置速度姿态

SetPlanning3D_Str SetPlanning3D_Struct; // 三维设定值规划

Chassis_t ChassisState_Data = {
    .filter_now.speed[0] = 0,
    .filter_now.speed[1] = 0,
    .filter_now.speed[2] = 0,
    .filter_now.speed[3] = 0,
};

Arm_Limit_s Arm_Limit_Data;            // 机械臂超限信息
static Image_Limit_s Image_Limit_Data; // 图传超限信息
static DataValid_s flag_DataValid;     // 通信有效性标志位
static int SlowStart_finishflag;       // 机械臂缓启动完成标志位

// 编码器
float BoomYawEncoder_radnow;
float BoomYawEncoderCali_radnow;
float BoomYawEncoderLock_radnow; // 校正之后，锁定后编码器的数据
// static EncoderCali_State_e EncoderCali_State;      // 编码器标定情况
float ImageYawEncoder_radnow;         // 图传编码器yaw的数值
float ImageYawEncoder_radlast = 0;    // 上一次图传编码器yaw的数值
float ImageYawEncoderLock_radnow;     // 图传编码器yaw的锁定值
float ImageGimbalYawInit_rad;         // 图传编码器yaw的初始位置
float ImageGimbalYaw_rad_now;         // 当前图传云台数值
float ImageMotorYawInit_rad;          // 图传电机yaw的初始位置
rt_uint8_t ImageGimbalAlionState = 0; // 图传云台对位状态

/*云台系角度及陀螺仪角度*/
// static float GyroYaw_RadNow;       // 当前陀螺仪yaw弧度值
//  float GimbalYaw_RadNow;            // 当前云台yaw弧度值
// static float GimbalYawRad_Set;     // 当前云台yaw设定值
// static float GimbalYawRad_Setlast; // 上一次云台yaw设定值
//  statiF float GimbalYaw_SpeedNow; // 云台转速

/**
 * @brief  机械臂控制定时器回调函数
 */
static void ArmCtrl_1ms_Handler(void *parameter)
{
    // 每隔1ms 释放一次发送信号量
    rt_sem_release(&ArmCtrl_sem);
}

// static VectorXYZ_Str ArmPosState_Set_temp; // 机械臂末端坐标设定值

/**
 * @brief 机械臂控制线程
 * @param parameter
 */
static void ArmCtrl_Thread(void *parameter)
{
    int Arm_Limitflag;      // 机械臂限位标志位
    int ArmSetPos_Zeroflag; // 机械臂位置零标志位
                            //    float ChassisYawSpeed_Now; // 底盘当前转速
                            //    float GimbalYaw_RadSetADD; // 云台yaw角度设定值的增量

    ChassisMotor_t ChassisStateData_now_temp2; // 当前值的中间量(速度修正的输出)
    ChassisMotor_t ChassisStateData_set_temp;  // 当前值的中间量(单位换算的输出)

    CREAT_ID(id);
    ADDTOMONITOR_ID("ArmCtrl_Thread", 2000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        // 阻塞等待接收信号量
        rt_sem_take(&ArmCtrl_sem, RT_WAITING_FOREVER);

        SWDG_FEED(id);

        flag_DataValid.Arm_IfValid = ArmMotor_DataValid_If();
        flag_DataValid.Chassis_IfValid = ChassisMotor_DataValid_If();
        flag_DataValid.Image_IfValid = ImageMotor_DataValid_If();
        flag_DataValid.BoomYawEncoder_IfValid = BoomYawEncoder_DataValid_If();
        flag_DataValid.ImageEncoder_IfValid = ImageEncoder_DataValid_If();

        /*数据读取*/
        ArmMotor_NowAngle_Read(&BoomState_Data.AngleNow,
                               &ForearmState_Data.AngleNow);
        ArmMotor_NowSpeed_Read(&BoomState_Data.SpeedNow,
                               &ForearmState_Data.SpeedNow);
        ArmMotor_SetPos_Read(&ArmPosState_Data.ArmPos_Set,
                             &ForearmState_Data.AngleHope);

        Chassis_NowSpeed_Read(&ChassisState_Data.now);
        Chassis_SetSpeed_Read(&ChassisState_Data.set);

        ImageMotor_NowAngle_Read(&ImageState_Data.AngleNow);
        ImageMotor_NowSpeed_Read(&ImageState_Data.SpeedNow);
        ImageMotor_AngleHope_Read(&ImageState_Data.AngleSet_Gimbal);

        BoomYaw_CoderRad_Read(&BoomYawEncoder_radnow);
        ImageYaw_CoderAngle_Read(&ImageYawEncoder_radnow);

        // EncoderCali_State_Read((uint8_t *)&EncoderCali_State); // 编码器标定状态
        // GimbalYawRad_Set_Read(&GimbalYawRad_Set);              // 读取云台角度设定值
        // GyroYaw_radNow_Read(&GyroYaw_RadNow);                  // 读取陀螺仪yaw的弧度值
        // ChassisYawSpeed_Now_Read(&ChassisYawSpeed_Now);        // 底盘当前转速
        // 编码器数据校准处理
        // if (EncoderCali_State == Cali_OK && flag_DataValid.BoomYawEncoder_IfValid)
        // {
        //     EncoderCorrectData_Get(RAD2DEG_f(BoomYawEncoder_radnow),
        //                            &BoomYawEncoderCali_radnow);
        // }
        // else
        // {
        //     if (flag_DataValid.Arm_IfValid &&
        //         flag_DataValid.BoomYawEncoder_IfValid)
        //     {
        //         // 臂从x轴起，逆时针旋转
        //         EncoderCailData_Record(BoomState_Data.AngleNow.BoomYaw,
        //                                RAD2DEG_f(BoomYawEncoder_radnow));
        //     }
        // }

        // if (flag_DataValid.Arm_IfValid &&
        //     flag_DataValid.BoomYawEncoder_IfValid &&
        //     (EncoderCali_State == Cali_OK))
        if (flag_DataValid.Arm_IfValid)
        {
            /*堵转判断及对位*/
            ArmRollAlign();
            /*将机械臂电机速度转换成弧度/s*/
            ForearmEncoderSpeTorpm(&ForearmState_Data.SpeedNow,
                                   &ForearmState_Data.SpeedNow);
            BoomEncoderSpeTorpm(&BoomState_Data.SpeedNow,
                                &BoomState_Data.SpeedNow);

            ForearmSpeedrpmTorad(&ForearmState_Data.SpeedNow,
                                 &ForearmState_Data.SpeedNow);
            BoomSpeedrpmTorad(&BoomState_Data.SpeedNow,
                              &BoomState_Data.SpeedNow);
            /*机械臂编码器值转成角度，附加传动比的转换*/
            // 仅对4310及10020电机数据处理
            ArmEncoder2Angle(&BoomState_Data.AngleNow,
                             &ForearmState_Data.AngleNow,
                             &BoomState_Data.AngleNow,
                             &ForearmState_Data.AngleNow);
            /*将机械臂电机角度转换成弧度制*/
            Arm_angle2rad(&BoomState_Data.AngleNow,
                          &ForearmState_Data.AngleNow,
                          &BoomState_Data.AngleNow,
                          &ForearmState_Data.AngleNow);
            /*零点校准*/
            Arm_ZeroAdjustment(&BoomState_Data.AngleNow,
                               &ForearmState_Data.AngleNow,
                               BoomYawEncoder_radnow,
                               &BoomState_Data.AngleNow,
                               &ForearmState_Data.AngleNow);
            /*对机械臂电机角度值和速度值进行滤波*/
            BoomMotDataFilter(&BoomState_Data);
            ForearmMotDataFilter(&ForearmState_Data);

            /*解算大机械臂末端当前坐标值*/
            BoomAngle2BoomPos(&BoomState_Data.AngleNowFilter,
                              &ArmPosState_Data.BoomPos_Now);

            /*解算小机械臂末端当前坐标值*/
            ArmAngle2ArmPos(&ForearmState_Data.AngleNowFilter,
                            BoomState_Data.AngleNowFilter.BoomYaw,
                            &ArmPosState_Data.BoomPos_Now,
                            &ArmPosState_Data.ArmPos_Now);

            /******************************限幅判断***************************/
            /*解算大机械臂末端期望坐标值*/
            ArmSetPos_Zeroflag = Forearm2BoomPos(&ArmPosState_Data.ArmPos_Set,
                                                 &ForearmState_Data.AngleHope,
                                                 &ArmPosState_Data.BoomPos_Set);
            if (!ArmSetPos_Zeroflag)
            {
                // 零向量提示音
                SoundDisplay_AddSound(IsZero_EQ);
                // 设定值设为上次设定值
                Vector3D_Transmit(&ArmPosState_Data.ArmPos_SetOld,
                                  &ArmPosState_Data.ArmPos_Set);
            }
            // 解算规划后电机角度
            BoomPos2BoomMotAngle(&ArmPosState_Data.BoomPos_Set,
                                 BoomState_Data.AngleNowFilter.BoomYaw,
                                 &BoomState_Data.AngleHope);
            /*机械限幅*/
            // 确保整个机械臂六个电机的的期望角度可以真实达到
            Arm_Limitflag = ArmMachinelimit_If(&BoomState_Data.AngleHope,
                                               &ForearmState_Data.AngleHope,
                                               &Arm_Limit_Data);
            if (Arm_Limitflag ||
                (fabsf(ArmPosState_Data.BoomPos_Set.y) +
                 fabsf(ArmPosState_Data.BoomPos_Set.x)) < 0.09f)
            {
                // 设定值规划超限，并且到达最大值
                // 则设置设定值为上一次设定值
                // 并按照上一次设定值规划的值进行解算控制
                Vector3D_Transmit(&ArmPosState_Data.ArmPos_SetOld,
                                  &ArmPosState_Data.ArmPos_Set);
                Vector3D_Transmit(&ArmPosState_Data.BoomPos_SetOld,
                                  &ArmPosState_Data.BoomPos_Set);

                // 设定值超限报警
                ForearmState_Data.AngleHope.ForearmYaw = ForearmState_Data.AngleHopeOld.ForearmYaw;
                ForearmState_Data.AngleHope.ForearmPitch = ForearmState_Data.AngleHopeOld.ForearmPitch;
                ForearmState_Data.AngleHope.ForearmRoll = ForearmState_Data.AngleHopeOld.ForearmRoll;

                //  蜂鸣器报警（仅用于测试）
                SoundDisplay_AddSound(IsZero_EQ); // 缓启动完成提示音
            }
            /*力矩补偿*/
            if (SlowStart_finishflag != 2)
            {
                SlowStart_finishflag = ArmComp_Slow(2000, // 缓启动时间
                                                    &BoomState_Data.AngleNowFilter,
                                                    &ForearmState_Data.AngleNowFilter,
                                                    &BoomState_Data.Compensation,
                                                    &ForearmState_Data.Compensation); // 正常补偿
            }
            else
                ArmComp(&BoomState_Data.AngleNowFilter,
                        &ForearmState_Data.AngleNowFilter,
                        &BoomState_Data.Compensation,
                        &ForearmState_Data.Compensation); // 正常补偿
            /*电机控制*/
            switch (SlowStart_finishflag)
            {
            case 1:
                // 补偿缓启动完成再开始闭环控制
                SoundDisplay_AddSound(Slowstartfinish_EQ); // 缓启动完成提示音
                SlowStart_finishflag = 2;
                break;
            case 2:
                /***********************************进行设定值规划*********************************/
                // Forearm三自由度的设定值规划
                ForearmSetPlanning(&ForearmState_Data,
                                   &ForearmState_Data.AngleSetPlan,
                                   &ForearmState_Data.SpeedFeedforward);

                // Boom三自由度的设定值规划
                Vector3D_Transmit(&ArmPosState_Data.BoomPos_Now,
                                  &SetPlanning3D_Struct.Input.Now.Pos);
                Vector3D_Transmit(&ArmPosState_Data.BoomPos_Set,
                                  &SetPlanning3D_Struct.Input.Set.Pos);

                SetPlanning3D_Cal(&SetPlanning3D_Struct);

                // 因为三维设定值规划的存在
                // 确保在几何解算过程中的一定有解
                // 无需进行几何限幅

                /*对Boom三自由度的设定值规划的输出解算*/
                // 解算规划后电机角度
                BoomPos2BoomMotAngle(&SetPlanning3D_Struct.Output.Pos,
                                     BoomState_Data.AngleNow_CrossCircle.BoomYaw,
                                     &BoomState_Data.AngleSetPlan);
                // CrossCircle_Normal(BoomState_Data.AngleSetPlan.BoomYaw, 2 * PI, 0,
                //                    &BoomState_Data.AngleSetPlan.BoomYaw); // 跨圈处理
                // 计算ArmPos设定值规划值坐标
                ArmAngle2ArmPos(&ForearmState_Data.AngleSetPlan,
                                BoomState_Data.AngleSetPlan.BoomYaw,
                                &SetPlanning3D_Struct.Output.Pos,
                                &ArmPosState_Data.ArmPos_SetPlan);

                // Vector3D_Rotate_z(&ArmPosState_Data.ArmPos_SetPlan,
                //                   -GimbalYaw_RadSetADD,
                //                   &ArmPosState_Data.ArmPos_SetPlan);
                /*机械臂角度限幅*/
                Arm_angleLimit(&BoomState_Data.AngleSetPlan,
                               &ForearmState_Data.AngleSetPlan);

                // 解算规划后电机速度
                BoomSpe2BoomMotSpe(&SetPlanning3D_Struct.Output.Pos,
                                   &BoomState_Data.AngleSetPlan,
                                   &SetPlanning3D_Struct.Output.Spe,
                                   &BoomState_Data.SpeedFeedforward);
                // 补偿缓启动完成再开始闭环控制
                BoomMotor_Ctrl(BoomYaw, &BoomState_Data);
                BoomMotor_Ctrl(BoomPitch1, &BoomState_Data);
                BoomMotor_Ctrl(BoomPitch2, &BoomState_Data);

                ForearmMotor_Ctrl(ForearmYaw, &ForearmState_Data);
                ForearmMotor_Ctrl(ForearmPitch, &ForearmState_Data);
                ForearmMotor_Ctrl(ForearmRoll, &ForearmState_Data);
                break;
            default:
                break;
            }
            /*电机电流设定值计算*/
            ArmMotorinput_Calculate(&BoomState_Data.Compensation,
                                    &ForearmState_Data.Compensation,
                                    &BoomState_Data.MotorCtrl_Out,
                                    &ForearmState_Data.MotorCtrl_Out);
        }
        else
        {
            Send_Slave2_Init();
        }
        /*********************************************图传部分*****************************************/
        if (flag_DataValid.Image_IfValid)
        {
            /*堵转判断及对位*/
            ImageGimbalAlign();

            /*将机械臂电机速度转换成弧度/s*/
            ImageEncoderSpeTorpm(&ImageState_Data.SpeedNow, 
                                 &ImageState_Data.SpeedNow);

            ImageSpeedrpmTorad(&ImageState_Data.SpeedNow,
                               &ImageState_Data.SpeedNow);

            /*将机械臂电机角度转换成弧度制*/
            Image_angle2rad(&ImageState_Data.AngleNow,
                            &ImageState_Data.AngleNow);
            /*零点校准*/
            Image_ZeroAdjustment(&ImageState_Data.AngleNow,
                                 &ImageState_Data.AngleNow);
            /*对机械臂电机角度值和速度值进行滤波*/
            ImageMotDataFilter(&ImageState_Data);
            /*****************************图传部分****************************/
            /*编码器数据处理*/
            if (ImageYawEncoder_radnow > 2.0f)
                ImageYawEncoder_radnow = ImageYawEncoder_radnow - 2 * PI;
            // if (fabsf(ImageYawEncoder_radnow - ImageYawEncoder_radlast) > 0.05f && ImageYawEncoder_radlast != 0&&
            // (!(fabsf(ImageYawEncoder_radnow - ImageYawEncoder_radlast)>PI+0.1f)))
            // {
            //     ImageYawEncoder_radnow = ImageYawEncoder_radlast;
            // }
            // ImageYawEncoder_radlast = ImageYawEncoder_radnow;

            ImageYawEncoderLock_radnow = ImageYawEncoder_radnow - IMAGEYAW_ZEROPOIN;
            ImageYawEncoder_radlast = ImageYawEncoder_radnow;
            ImageGimbalYawInit_rad = ImageState_Data.AngleNowFilter.ImagePitch -
                                     ImageState_Data.AngleNowFilter.ImageYaw + ImageYawEncoderLock_radnow;
            ImageGimbalYaw_rad_now = ImageYawEncoderLock_radnow;
            // if ((ImageGimbalAlionState != 1) && Get_ImageMotorAlign())
            // {
            //     // 图传电机对位完成，图传云台对位未完成
            //     /*编码器数据处理*/
            //     ImageYawEncoderLock_radnow = ImageYawEncoder_radnow - IMAGEYAW_ZEROPOIN;
            //     ImageMotorYawInit_rad = ImageState_Data.AngleNowFilter.ImageYaw -
            //                             ImageState_Data.AngleNowFilter.ImagePitch;
            //     ImageGimbalYawInit_rad = ImageYawEncoderLock_radnow - ImageMotorYawInit_rad;
            //     ImageGimbalAlionState ++;
            // }
            // else if (ImageGimbalAlionState==100)
            // {
            //     ImageGimbalYaw_rad_now = ImageState_Data.AngleNowFilter.ImageYaw -
            //                              ImageState_Data.AngleNowFilter.ImagePitch + ImageGimbalYawInit_rad;
            // };
            // 图传超限
            if (ImageMachinelimit_If(&ImageState_Data.AngleSet_Gimbal,
                                     &Image_Limit_Data))
            {
                if (ImageState_Data.AngleSet_Gimbal.ImagePitch > IMAGEPITCH_ANGLEMAX)
                    ImageState_Data.AngleSet_Gimbal.ImagePitch = IMAGEPITCH_ANGLEMAX;
                else if (ImageState_Data.AngleSet_Gimbal.ImagePitch < IMAGEPITCH_ANGLEMIN)
                    ImageState_Data.AngleSet_Gimbal.ImagePitch = IMAGEPITCH_ANGLEMIN;

                ImageState_Data.AngleSet_Gimbal.ImageYaw = ImageState_Data.AngleHope_Old.ImageYaw;
            };
            // // 获取图传云台当前角度
            // ImageMotorNow2GimbalNow(&ImageState_Data.AngleNowFilter,
            //                         ImageYawEncoderLock_radnow,
            //                         &ImageState_Data.AngleNow_Gimbal);

            // 获取图传云台当前角度
            ImageMotorNow2GimbalNow(&ImageState_Data.AngleNowFilter,
                                    ImageGimbalYaw_rad_now,
                                    &ImageState_Data.AngleNow_Gimbal);
            // 解算电机设定角度
            ImageGimbalSet2MotorSet(&ImageState_Data.AngleSet_Gimbal,
                                    ImageGimbalYawInit_rad,
                                    &ImageState_Data.AngleHope);
            //  图传云台设定值规划
            ImageSetPlanning(&ImageState_Data,
                             &ImageState_Data.AngleSetPlan,
                             &ImageState_Data.SpeedFeedforward);
            // 闭环控制
            ImageMotor_Ctrl(ImageYaw, &ImageState_Data);
            ImageMotor_Ctrl(ImagePitch, &ImageState_Data);

            ImageMotorinput_Calculate(ROBOCTRL_TIMER_PIRIOD * 500,
                                      DISTURBANCE_AMPLITUDE,
                                      &ImageState_Data.MotorCtrl_Out);
        }
        else
        {
            Send_Slave1_Init();
        }

        /*****************************底盘部分****************************/
        if (flag_DataValid.Chassis_IfValid)
        {
            // 限制设定速度
            Wheels_Speed_Limit(RATED_SPEED, &ChassisState_Data.set,
                               &ChassisState_Data.set);
            // 修正速度
            Wheel_NowSpeed_Revise(&ChassisState_Data.now,
                                  &ChassisStateData_now_temp2);
            // 速度滤波
            WheelsMot_SpeedFilter(&ChassisStateData_now_temp2,
                                  &ChassisState_Data.filter_now);
            // 设定值换算单位
            WheelsUnitCverStandard2Rpm(&ChassisState_Data.set,
                                       &ChassisStateData_set_temp);
            // PID计算
            WheelsMotPID_Cal(&ChassisState_Data.filter_now,
                             &ChassisStateData_set_temp,
                             &ChassisState_Data.out);
        }

        // BoomState_Data.MotorCtrl_Out.BoomYaw = 0;
        // BoomState_Data.MotorCtrl_Out.BoomPitch1 = 0;
        // BoomState_Data.MotorCtrl_Out.BoomPitch2 = 0;

        // ForearmState_Data.MotorCtrl_Out.ForearmYaw = 0;
        // ForearmState_Data.MotorCtrl_Out.ForearmPitch = 0;
        // ForearmState_Data.MotorCtrl_Out.ForearmRoll = 0;

        // ImageState_Data.MotorCtrl_Out.ImageYaw = 0;
        // ImageState_Data.MotorCtrl_Out.ImagePitch = 0;

        // ChassisState_Data.out.speed[0] = 0;
        // ChassisState_Data.out.speed[1] = 0;
        // ChassisState_Data.out.speed[2] = 0;
        // ChassisState_Data.out.speed[3] = 0;

        Send_Slave1_Handle(&ChassisState_Data.out);
        Send_Slave2_Handle(&BoomState_Data.MotorCtrl_Out,
                           &ForearmState_Data.MotorCtrl_Out,
                           &ImageState_Data.MotorCtrl_Out);
        // Send_Slave_Debug();

        /*记录上一次设定值*/
        // 设定值需要修改到键鼠
        // 故需要存入数据服务器
        // 键鼠遥控时也需要先从数据服务器读取更新
        Vector3D_Transmit(&ArmPosState_Data.ArmPos_Set,
                          &ArmPosState_Data.ArmPos_SetOld);
        Vector3D_Transmit(&ArmPosState_Data.ArmPos_SetPlan,
                          &ArmPosState_Data.ArmPos_SetPlanOld);
        Vector3D_Transmit(&ArmPosState_Data.BoomPos_Set,
                          &ArmPosState_Data.BoomPos_SetOld);

        ForearmState_Data.AngleHopeOld.ForearmYaw = ForearmState_Data.AngleHope.ForearmYaw;
        ForearmState_Data.AngleHopeOld.ForearmPitch = ForearmState_Data.AngleHope.ForearmPitch;
        ForearmState_Data.AngleHopeOld.ForearmRoll = ForearmState_Data.AngleHope.ForearmRoll;

        ImageState_Data.AngleHope_Old.ImageYaw = ImageState_Data.AngleSet_Gimbal.ImageYaw;
        ImageState_Data.AngleHope_Old.ImagePitch = ImageState_Data.AngleSet_Gimbal.ImagePitch;

        //        GimbalYawRad_Setlast = GimbalYawRad_Set; // 记录云台角度设定值

        /*写入数据服务器*/
        ArmMotor_NowPos_Write(&ArmPosState_Data.ArmPos_Now);
        ArmMotor_SetPos_Write(&ArmPosState_Data.ArmPos_Set);
        ArmMotor_SetAngle_Write(&ForearmState_Data.AngleHope);
        ImageGimbalPitchSet_Write(ImageState_Data.AngleSet_Gimbal.ImagePitch);

        ArmMotor_RadFilter_Write(&BoomState_Data.AngleNowFilter,
                                 &ForearmState_Data.AngleNowFilter);
        ImageGimbal_Angle_Write(&ImageState_Data.AngleNow_Gimbal);
        Arm_IfLimit_Data_Write((uint8_t *)&Arm_Limit_Data);
        BoomYaw_CoderCaliRad_Write(BoomYawEncoderLock_radnow); // 大机械臂BoomYaw编码器校正值
    }
}

static char ArmCtrl_Thread_stack[THREAD_STACK_ArmCtrl];
/**
 * @brief 机械臂控制线程初始化
 * @return rt_err_t
 */
rt_err_t ArmCtrl_Thread_Init(void)
{
    rt_err_t res = RT_EOK;
    static struct rt_thread ArmCtrl;

    ArmPosState_Data.ArmPos_Set.x = 0;
    ArmPosState_Data.ArmPos_Set.y = 0.3054f;
    ArmPosState_Data.ArmPos_Set.z = 0.11085f;

    ForearmState_Data.AngleHope.ForearmYaw = 1.5707f;
    ForearmState_Data.AngleHope.ForearmPitch = -1.57f;
    ForearmState_Data.AngleHope.ForearmRoll = 0;

    ArmMotor_SetPos_Write(&ArmPosState_Data.ArmPos_Set);
    ArmMotor_SetAngle_Write(&ForearmState_Data.AngleHope);

    Vector3D_Transmit(&ArmPosState_Data.ArmPos_Set,
                      &ArmPosState_Data.ArmPos_SetOld);

    Vector3D_Transmit(&ArmPosState_Data.ArmPos_Set,
                      &ArmPosState_Data.ArmPos_SetPlan);

    /*三维设定值规划初始化*/
    PlanSettings3D_Init(&SetPlanning3D_Struct, POS_ERRORMAX,
                        POS_TOLERANCE, SPE_TOLERANCE,
                        SPEEDMAX_R, SPEEDMAX_T,
                        ACCLMAX_R, ACCLMAX_T,
                        SETPLANNING_PERIOD);
    /*初始化信号量*/
    res = rt_sem_init(&ArmCtrl_sem,
                      "ArmCtrl_sem", 0,
                      RT_IPC_FLAG_FIFO);

    /*创建机械臂控制线程*/
    rt_thread_init(&ArmCtrl,
                   "ArmCtrl",
                   ArmCtrl_Thread,
                   RT_NULL,
                   &ArmCtrl_Thread_stack[0],
                   sizeof(ArmCtrl_Thread_stack),
                   THREAD_PRIO_ArmCtrl, THREAD_TICK_ArmCtrl);
    rt_thread_startup(&ArmCtrl);

    /*创建线程定时器*/
    rt_timer_t timer = rt_timer_create("ArmCtrl_1ms_timer",
                                       ArmCtrl_1ms_Handler, // 回调函数
                                       RT_NULL,
                                       ROBOCTRL_TIMER_PIRIOD, // 定时时间（ms）
                                       RT_TIMER_FLAG_PERIODIC);
    res = rt_timer_start(timer); // 启动定时器
    if (res != RT_EOK)
        return res; // 开启失败

    return RT_EOK;
}
