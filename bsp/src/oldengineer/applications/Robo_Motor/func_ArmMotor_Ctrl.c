/**
 * @file func_ArmMotor_Ctrl.c
 * @brief 工程机器人电机控制
 * @brief 主要是电机和pid初始化，以及电机对位
 * @author mylj
 * @version 1.0
 * @date 2023-03-04
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "func_ArmMotor_Ctrl.h"
#include "pid.h"
#include "drv_utils.h"
#include "drv_encoder.h"
#include "drv_motor_Locked.h"

/*电机相关结构体*/
Motor_t BoomYawMotor_Str, BoomPitch1Motor_Str, BoomPitch2Motor_Str,
    ForearmYawMotor_Str, ForearmPitchMotor_Str, ForearmRollMotor_Str;
MotorAlign_t ForearmRollMotorAlign_Str;

static SetPlanning_Str ForearmYaw_MotorPlan,
    ForearmPitch_MotorPlan,
    ForearmRoll_MotorPlan; // 小机械臂电机

Encoder_s BoomYawEncoder_Str; // 大yaw编码器的值

/**
 * @brief 生成一波正弦波，用于削减电机的阻力
 * @param Period            周期 单位：ms
 * @param Amplitude         幅度
 */
static float SinWave_Get(float Period, float Amplitude)
{

    return Amplitude * arm_sin_f32(2 * PI / Period * rt_tick_get());
}

#define ARM_MOTOROUTCLOSE_ENABLE 0
/**
 * @brief 机械臂部分的电机及pid控制初始化
 */
void ArmMotor_Init(void)
{
    motor_init(&BoomYawMotor_Str, 0,      // 电机结构体和电机ID
               BoomYaw_Ratio,             // 电机减速比
               ANGLE_CTRL_ABS,            // 电机角度控制模式——输出轴、设定值不可跨圈
               BoomYaw_EncoderLen,        // 编码器长度
               (2 * PI), 0, 0);           // 最大值、最小值，正向
    motor_init(&BoomPitch1Motor_Str, 0,   // 电机结构体和电机ID
               BoomPitch1_Ratio,          // 电机减速比
               ANGLE_CTRL_ABS,            // 电机角度控制模式——输出轴、设定值不可跨圈
               BoomPitch1_EncoderLen,     // 编码器长度
               180, -180, 1);             // 最大值、最小值，反向
    motor_init(&BoomPitch2Motor_Str, 0,   // 电机结构体和电机ID
               BoomPitch2_Ratio,          // 电机减速比
               ANGLE_CTRL_ABS,            // 电机角度控制模式——输出轴、设定值可跨圈
               BoomPitch2_EncoderLen,     // 编码器长度
               180, -180, 0);             // 最大值、最小值，反向
    motor_init(&ForearmYawMotor_Str, 0,   // 电机结构体和电机ID
               ForearmYaw_Ratio,          // 电机减速比
               ANGLE_CTRL_ABS,            // 电机角度控制模式——输出轴、设定值可跨圈
               ForearmYaw_EncoderLen,     // 编码器长度
               180, -180, 0);             // 最大值、最小值，反向
    motor_init(&ForearmPitchMotor_Str, 0, // 电机结构体和电机ID
               ForearmPitch_Ratio,        // 电机减速比
               ANGLE_CTRL_ABS,            // 电机角度控制模式——输出轴、设定值可跨圈
               ForearmPitch_EncoderLen,   // 编码器长度
               180, -180, 0);             // 最大值、最小值，不反向
    motor_init(&ForearmRollMotor_Str, 0,  // 电机结构体和电机ID
               ForearmRoll_Ratio,         // 电机减速比
               ANGLE_CTRL_FULL,           // 电机角度控制模式——输出轴、设定值不可跨圈
               ForearmRoll_EncoderLen,    // 编码器长度
               360, 0, 0);                // 最大值、最小值，不反向

    EncoderData_Init(&BoomYawEncoder_Str,
                     3, DEFAULT_VALUE,
                     AngleMode_ABS,
                     reverse);

#if ARM_MOTOROUTCLOSE_ENABLE
    pid_init(&BoomYawMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&BoomYawMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&BoomPitch1Motor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&BoomPitch1Motor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&BoomPitch2Motor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&BoomPitch2Motor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&ForearmYawMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&ForearmYawMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&ForearmPitchMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&ForearmPitchMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&ForearmRollMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&ForearmRollMotor_Str.spe, 0, 0, 0, 0, 0, -0);
#else
    pid_init(&BoomYawMotor_Str.ang, 20, 0, 100, 0, 200, -200);
    pid_init(&BoomYawMotor_Str.spe, 300, 0, 500, 0, 1000, -1000);

    // pid_init(&BoomPitch1Motor_Str.ang, 100, 0, 0, 0, 200, -200);
    // pid_init(&BoomPitch1Motor_Str.spe, 100, 0, 200, 0, 1000, -1000);

    // pid_init(&BoomPitch2Motor_Str.ang, 100, 0, 0, 0, 200, -200);
    // pid_init(&BoomPitch2Motor_Str.spe, 100, 0, 200, 0, 1000, -1000);

    pid_init(&BoomPitch1Motor_Str.ang, 130, 0, 0, 0, 200, -200);
    pid_init(&BoomPitch1Motor_Str.spe, 40, 0, 200, 0, 1000, -1000);

    pid_init(&BoomPitch2Motor_Str.ang, 130, 0, 0, 0, 200, -200);
    pid_init(&BoomPitch2Motor_Str.spe, 40, 0, 200, 0, 1000, -1000);

    pid_init(&ForearmYawMotor_Str.ang, 10, 0, 0, 0, 200, -200);
    pid_init(&ForearmYawMotor_Str.spe, 250, 0, 0, 100, 200, -200);

    pid_init(&ForearmPitchMotor_Str.ang, 10, 0, 0.1, 0, 200, -200);
    pid_init(&ForearmPitchMotor_Str.spe, 200, 0, 0, 100, 1000, -1000);

    pid_init(&ForearmRollMotor_Str.ang, 100, 0, 10, 0, 200, -200);
    pid_init(&ForearmRollMotor_Str.spe, 500, 2, 8000, 1000, 5000, -5000);

#endif
    // pid_ErrorDeadzone_Set(&BoomYawMotor_Str.ang, 1, 0.015f); // 死区
    MotorAlign_Init(&ForearmRollMotorAlign_Str, // 对位结构体指针
                    Start_SpecificAngle_Align,  // 特定角度对位
                    1000, 20,                   // 最大的设定电流值、最小速度、
                    FowardStall, 5,             // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, 140.0f);    // 电机的控制模式、电机初始角度

    SetPlanSettings_Init(&ForearmYaw_MotorPlan.Settings,
                         FOREARMYAW_SPEEDMAX,
                         ForearmYAW_POSERRORMAX,
                         ForearmYAW_ACCLMAX,
                         SETPPLANPERIOD);
    SetPlanSettings_Init(&ForearmPitch_MotorPlan.Settings,
                         FOREARMPITCH_SPEEDMAX,
                         FOREARMPITCH_POSERRORMAX,
                         FOREARMPITCH_ACCLMAX,
                         SETPPLANPERIOD);
    SetPlanSettings_Init(&ForearmRoll_MotorPlan.Settings,
                         FOREARMROLL_SPEEDMAX,
                         FOREARMROLL_POSERRORMAX,
                         FOREARMROLL_ACCLMAX,
                         SETPPLANPERIOD);
}

/*******************************电机控制***************************/
float debug______shjfjshf;
float anglesetplan;
CrossCircleData_s BoomYaw_AngleSetPlan_cross = {.Circle_Len = 2 * PI}; // 设定值规划输出跨圈处理
/**
 * @brief 大机械臂电机闭环控制
 * @param ArmMotor
 * @param BoomStateData
 */
void BoomMotor_Ctrl(ArmMotor_e ArmMotor,
                    BoomState_Data_s *BoomStateData)
{
    switch (ArmMotor)
    {
    case BoomYaw:
        // CrossCircle_ProcessingSpecial(&BoomYaw_AngleSetPlan_cross, BoomStateData->AngleSetPlan.BoomYaw);

        // Motor_Write_SetAngle_ABS(&BoomYawMotor_Str, CrossCircle_Data_Read(&BoomYaw_AngleSetPlan_cross)); // 设定值
        // anglesetplan = CrossCircle_Data_Read(&BoomYaw_AngleSetPlan_cross);

        // debug______shjfjshf = CrossCircle_Data_Read(&BoomYaw_AngleSetPlan_cross) -
        //                       BoomStateData->AngleNow_CrossCircle.BoomYaw;

        Motor_Write_SetAngle_ABS(&BoomYawMotor_Str, BoomStateData->AngleSetPlan.BoomYaw); // 设定值
        Motor_AnglePIDCalculate(&BoomYawMotor_Str, BoomStateData->AngleNowFilter.BoomYaw);
        Motor_Write_SetSpeed_ABS(&BoomYawMotor_Str,
                                 BoomYawMotor_Str.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomYaw);
        Motor_SpeedPIDCalculate(&BoomYawMotor_Str,
                                BoomStateData->SpeedNowFilter.BoomYaw);
        break;
    case BoomPitch1:

        Motor_Write_SetAngle_ABS(&BoomPitch1Motor_Str, BoomStateData->AngleSetPlan.BoomPitch1); // 设定值
        Motor_AnglePIDCalculate(&BoomPitch1Motor_Str,
                                BoomStateData->AngleNowFilter.BoomPitch1);
        // Motor_Write_SetSpeed_ABS(&BoomPitch1Motor_Str,
        //                          BoomPitch1Motor_Str.ang.out);
        Motor_Write_SetSpeed_ABS(&BoomPitch1Motor_Str,
                                 BoomPitch1Motor_Str.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomPitch1);
        Motor_SpeedPIDCalculate(&BoomPitch1Motor_Str,
                                BoomStateData->SpeedNowFilter.BoomPitch1);
        break;
    case BoomPitch2:
        Motor_Write_SetAngle_ABS(&BoomPitch2Motor_Str, BoomStateData->AngleSetPlan.BoomPitch2); // 设定值
        Motor_AnglePIDCalculate(&BoomPitch2Motor_Str, BoomStateData->AngleNowFilter.BoomPitch2);
        // Motor_Write_SetSpeed_ABS(&BoomPitch2Motor_Str,
        //                          BoomPitch2Motor_Str.ang.out);
        Motor_Write_SetSpeed_ABS(&BoomPitch2Motor_Str,
                                 BoomPitch2Motor_Str.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomPitch2);
        Motor_SpeedPIDCalculate(&BoomPitch2Motor_Str,
                                BoomStateData->SpeedNowFilter.BoomPitch2);
        break;
    default:
        break;
    }
}

/**
 * @brief 小机械臂电机闭环控制
 * @param ArmMotor
 * @param ForearmStateData
 */
void ForearmMotor_Ctrl(ArmMotor_e ArmMotor,
                       ForearmState_Data_s *ForearmStateData)
{
    switch (ArmMotor)
    {
    case ForearmYaw:
        Motor_Write_SetAngle_ABS(&ForearmYawMotor_Str,
                                 ForearmStateData->AngleSetPlan.ForearmYaw); // 设定值
        Motor_AnglePIDCalculate(&ForearmYawMotor_Str, ForearmStateData->AngleNowFilter.ForearmYaw);

        Motor_Write_SetSpeed_ABS(&ForearmYawMotor_Str,
                                 ForearmYawMotor_Str.ang.out +
                                     ForearmStateData->SpeedFeedforward.ForearmYaw);
        Motor_SpeedPIDCalculate(&ForearmYawMotor_Str,
                                ForearmStateData->SpeedNowFilter.ForearmYaw);
        break;
    case ForearmPitch:
        Motor_Write_SetAngle_ABS(&ForearmPitchMotor_Str,
                                 ForearmStateData->AngleSetPlan.ForearmPitch); // 设定值
        Motor_AnglePIDCalculate(&ForearmPitchMotor_Str, ForearmStateData->AngleNowFilter.ForearmPitch);

        Motor_Write_SetSpeed_ABS(&ForearmPitchMotor_Str,
                                 ForearmPitchMotor_Str.ang.out +
                                     ForearmStateData->SpeedFeedforward.ForearmPitch);
        Motor_SpeedPIDCalculate(&ForearmPitchMotor_Str,
                                ForearmStateData->SpeedNowFilter.ForearmPitch);
        break;
    case ForearmRoll:
        if (ForearmRollMotorAlign_Str.AlignState == Align_OK) // 对位完成的状态
        {
            if (ForearmRollMotorAlign_Str.ControlMode == 3) // 角度闭环模式
            {

                Motor_Write_SetAngle_ABS(&ForearmRollMotor_Str, ForearmStateData->AngleSetPlan.ForearmRoll);

                Motor_AnglePIDCalculate(&ForearmRollMotor_Str, ForearmStateData->AngleNowFilter.ForearmRoll);
                Motor_Write_SetSpeed_ABS(&ForearmRollMotor_Str,
                                         ForearmRollMotor_Str.ang.out +
                                             ForearmStateData->SpeedFeedforward.ForearmRoll);
                Motor_SpeedPIDCalculate(&ForearmRollMotor_Str,
                                        ForearmStateData->SpeedNowFilter.ForearmRoll);
            }
        }
        else if (ForearmRollMotorAlign_Str.ControlMode == 2) // 转速闭环模式
        {
            Motor_SpeedPIDCalculate(&ForearmRollMotor_Str,
                                    ForearmStateData->SpeedNowFilter.ForearmRoll);
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Forearm部分电机的设定值规划
 * @brief 改变电机AngleSetPlan和SpeedFeedforward的数据
 * @param ForearmStateData   电机位姿信息（角度，角速度，期望角度）
 * @param AngleSetPlan
 * @param SpeedFeedforward
 */
void ForearmSetPlanning(ForearmState_Data_s *ForearmStateData,
                        ForearmMotor_s *AngleSetPlan,
                        ForearmMotor_s *SpeedFeedforward)
{
    /*ForearmYaw设定值规划*/
    ForearmYaw_MotorPlan.Input.Now.pos = ForearmStateData->AngleNowFilter.ForearmYaw;
    ForearmYaw_MotorPlan.Input.Now.spe = ForearmStateData->SpeedNowFilter.ForearmYaw;
    ForearmYaw_MotorPlan.Input.Set.pos = ForearmStateData->AngleHope.ForearmYaw;
    SetPlanning_Cal(&ForearmYaw_MotorPlan);
    AngleSetPlan->ForearmYaw = ForearmYaw_MotorPlan.Output.pos;
    SpeedFeedforward->ForearmYaw = ForearmYaw_MotorPlan.Output.spe;

    /*ForearmPitch设定值规划*/
    ForearmPitch_MotorPlan.Input.Now.pos = ForearmStateData->AngleNowFilter.ForearmPitch;
    ForearmPitch_MotorPlan.Input.Now.spe = ForearmStateData->SpeedNowFilter.ForearmPitch;
    ForearmPitch_MotorPlan.Input.Set.pos = ForearmStateData->AngleHope.ForearmPitch;
    SetPlanning_Cal(&ForearmPitch_MotorPlan);
    AngleSetPlan->ForearmPitch = ForearmPitch_MotorPlan.Output.pos;
    SpeedFeedforward->ForearmPitch = ForearmPitch_MotorPlan.Output.spe;

    /*ForearmRoll设定值规划*/
    ForearmRoll_MotorPlan.Input.Now.pos = ForearmStateData->AngleNowFilter.ForearmRoll;
    ForearmRoll_MotorPlan.Input.Now.spe = ForearmStateData->SpeedNowFilter.ForearmRoll;
    ForearmRoll_MotorPlan.Input.Set.pos = ForearmStateData->AngleHope.ForearmRoll;
    SetPlanning_Cal(&ForearmRoll_MotorPlan);
    AngleSetPlan->ForearmRoll = ForearmRoll_MotorPlan.Output.pos;
    SpeedFeedforward->ForearmRoll = ForearmRoll_MotorPlan.Output.spe;
}

/**
 * @brief 将Boom读取到的编码器的速度转换成RPM速度
 * @param Spe_encoder
 * @param Speed_rpm
 */
void BoomEncoderSpeTorpm(BoomMotor_s *Spe_encoder,
                         BoomMotor_s *Speed_rpm)
{
    Speed_rpm->BoomYaw =
        Spe_encoder->BoomYaw / BoomYaw_RPMRatio;
    Speed_rpm->BoomPitch1 =
        Spe_encoder->BoomPitch1 / BoomPitch1_RPMRatio;
    Speed_rpm->BoomPitch2 =
        Spe_encoder->BoomPitch2 / BoomPitch2_RPMRatio;
}

/**
 * @brief 将Forearm读取到的编码器的速度转换成RPM速度
 * @param Spe_encoder
 * @param Speed_rpm
 */
void ForearmEncoderSpeTorpm(ForearmMotor_s *Spe_encoder,
                            ForearmMotor_s *Speed_rpm)
{
    Speed_rpm->ForearmPitch =
        Spe_encoder->ForearmPitch / ForearmPitch_RPMRatio;
    Speed_rpm->ForearmYaw =
        Spe_encoder->ForearmYaw / ForearmYaw_RPMRatio;
    Speed_rpm->ForearmRoll =
        Spe_encoder->ForearmRoll / ForearmRoll_RPMRatio;
}

/**
 * @brief 对Boom电机角度值及速度值进行滤波
 * @param BoomStateData
 */
void BoomMotDataFilter(BoomState_Data_s *BoomStateData)
{

    /*BoomYaw*/
    BoomStateData->AngleNowFilter.BoomYaw =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomYaw,
                      BoomStateData->AngleNow.BoomYaw, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomYaw =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomYaw,
                      BoomStateData->SpeedNow.BoomYaw, 0.8f);

    /*BoomPitch1*/
    BoomStateData->AngleNowFilter.BoomPitch1 =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomPitch1,
                      BoomStateData->AngleNow.BoomPitch1, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomPitch1 =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomPitch1,
                      BoomStateData->SpeedNow.BoomPitch1, 0.8f);

    /*BoomPitch2*/
    BoomStateData->AngleNowFilter.BoomPitch2 =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomPitch2,
                      BoomStateData->AngleNow.BoomPitch2, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomPitch2 =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomPitch2,
                      BoomStateData->SpeedNow.BoomPitch2, 0.8f);
}

/**
 * @brief 对Forearm电机角度值及速度值进行滤波
 * @param ForearmStateData
 */
void ForearmMotDataFilter(ForearmState_Data_s *ForearmStateData)
{

    /*ForearmYaw*/
    ForearmStateData->AngleNowFilter.ForearmYaw =
        UTILS_LP_FAST(ForearmStateData->AngleNowFilter.ForearmYaw,
                      ForearmStateData->AngleNow.ForearmYaw, 0.8f); // 滞后滤波
    ForearmStateData->SpeedNowFilter.ForearmYaw =
        UTILS_LP_FAST(ForearmStateData->SpeedNowFilter.ForearmYaw,
                      ForearmStateData->SpeedNow.ForearmYaw, 0.8f);

    /*ForearmPitch*/
    ForearmStateData->AngleNowFilter.ForearmPitch =
        UTILS_LP_FAST(ForearmStateData->AngleNowFilter.ForearmPitch,
                      ForearmStateData->AngleNow.ForearmPitch, 0.5f); // 滞后滤波
    ForearmStateData->SpeedNowFilter.ForearmPitch =
        UTILS_LP_FAST(ForearmStateData->SpeedNowFilter.ForearmPitch,
                      ForearmStateData->SpeedNow.ForearmPitch, 0.5f);

    /*ForearmRoll*/
    ForearmStateData->AngleNowFilter.ForearmRoll =
        UTILS_LP_FAST(ForearmStateData->AngleNowFilter.ForearmRoll,
                      ForearmStateData->AngleNow.ForearmRoll, 0.3f); // 滞后滤波
    ForearmStateData->SpeedNowFilter.ForearmRoll =
        UTILS_LP_FAST(ForearmStateData->SpeedNowFilter.ForearmRoll,
                      ForearmStateData->SpeedNow.ForearmRoll, 0.3f);
}

/**
 * @brief Boom电机转速单位由RPM换算成RAD
 * @param Speed_rpm
 * @param Speed_rad
 */
void BoomSpeedrpmTorad(BoomMotor_s *Speed_rpm,
                       BoomMotor_s *Speed_rad)
{
    Speed_rad->BoomYaw = PI * Speed_rpm->BoomYaw / 30.0f;
    Speed_rad->BoomPitch1 = PI * Speed_rpm->BoomPitch1 / 30.0f;
    Speed_rad->BoomPitch2 = PI * Speed_rpm->BoomPitch2 / 30.0f;
}

/**
 * @brief Boom电机转速单位由RAD换算成RPM
 * @param Speed_rad
 * @param Speed_rpm
 */
void BoomSpeedradTorpm(BoomMotor_s *Speed_rad,
                       BoomMotor_s *Speed_rpm)
{
    Speed_rpm->BoomYaw = Speed_rad->BoomYaw * 30.0f / PI;
    Speed_rpm->BoomPitch1 = Speed_rad->BoomPitch1 * 30.0f / PI;
    Speed_rpm->BoomPitch2 = Speed_rad->BoomPitch2 * 30.0f / PI;
}

/**
 * @brief Forearm电机转速单位由RPM换算成RAD
 * @param Speed_rpm
 * @param Speed_rad
 */
void ForearmSpeedrpmTorad(ForearmMotor_s *Speed_rpm,
                          ForearmMotor_s *Speed_rad)
{
    Speed_rad->ForearmYaw = PI * Speed_rpm->ForearmYaw / 30.0f;
    Speed_rad->ForearmPitch = PI * Speed_rpm->ForearmPitch / 30.0f;
    Speed_rad->ForearmRoll = PI * Speed_rpm->ForearmRoll / 30.0f;
}

/**
 * @brief Forearm电机转速单位由RAD换算成RPM
 * @param Speed_rad
 * @param Speed_rpm
 */
void ForearmSpeedradTorpm(ForearmMotor_s *Speed_rad,
                          ForearmMotor_s *Speed_rpm)
{
    Speed_rpm->ForearmYaw = Speed_rad->ForearmYaw * 30.0f / PI;
    Speed_rpm->ForearmPitch = Speed_rad->ForearmPitch * 30.0f / PI;
    Speed_rpm->ForearmRoll = Speed_rad->ForearmRoll * 30.0f / PI;
}

/**
 * @brief 大机械臂Yaw轴电机控制
 * @param Motor_Structure
 * @param BoomStateData  电机位姿信息（角度，角速度，期望角度）
 */
static void BoomYaw_AngleModeChange(Motor_t *Motor_Structure,
                                    float BoomYaw,
                                    float *Delta_out)
{
    if (Motor_Structure->dji.Angle_CtrlMode == ANGLE_CTRL_ABS)
    {
        if (Motor_Structure->dji.Data_Valid) // 收到报文，数据有效
        {
            Motor_Structure->dji.Angle_CtrlMode = ANGLE_CTRL_FULL;
            *Delta_out = Motor_Read_NowAngle(Motor_Structure) - BoomYaw;
        }
    }
}

static float BoomYawDelta_fromModeChange;

/**
 * @brief 机械臂编码器值转成角度值
 * @brief 包括传动比
 * @param BoomAngle_in
 * @param ForearmAngle_in
 * @param BoomAngle_out
 * @param ForearmAngle_out
 */
void ArmEncoder2Angle(BoomMotor_s *BoomAngle_in,
                      ForearmMotor_s *ForearmAngle_in,
                      BoomMotor_s *BoomAngle_out,
                      ForearmMotor_s *ForearmAngle_out)
{

    if (BoomYawMotor_Str.dji.Angle_CtrlMode == ANGLE_CTRL_ABS)
    {
        float BoomAngleOut_temp;

        BoomAngleOut_temp = BoomAngle_in->BoomYaw * 360.0f /
                            BoomYaw_EncoderLen / BoomYawBelt_Ratio;
        BoomYaw_AngleModeChange(&BoomYawMotor_Str,
                                BoomAngleOut_temp,
                                &BoomYawDelta_fromModeChange);
        BoomAngle_out->BoomYaw = BoomAngleOut_temp;
    }
    else
    {
        BoomAngle_out->BoomYaw = BoomAngle_in->BoomYaw - BoomYawDelta_fromModeChange;
    }

    BoomAngle_out->BoomYaw = BoomAngle_in->BoomYaw;
    // BoomAngle_out->BoomYaw = BoomAngle_in->BoomYaw * 360.0f /
    //                          BoomYaw_EncoderLen;
    BoomAngle_out->BoomPitch1 = BoomAngle_in->BoomPitch1 * 360.0f /
                                BoomPitch1_EncoderLen;
    BoomAngle_out->BoomPitch2 = BoomAngle_in->BoomPitch2 * 360.0f /
                                BoomPitch2_EncoderLen;

    ForearmAngle_out->ForearmYaw = ForearmAngle_in->ForearmYaw * 360.0f /
                                   ForearmYaw_EncoderLen;
    ForearmAngle_out->ForearmPitch = ForearmAngle_in->ForearmPitch * 360.0f /
                                     ForearmPitch_EncoderLen;
}

/**
 * @brief 机械臂零点校准
 * @param BoomAngle_in
 * @param ForearmAngle_in
 * @param BoomYawEncoder_in
 * @param Boomrad_out
 * @param Forearmrad_out
 */
void Arm_ZeroAdjustment(BoomMotor_s *BoomAngle_in,
                        ForearmMotor_s *ForearmAngle_in,
                        float BoomYawEncoder_in,
                        BoomMotor_s *Boomrad_out,
                        ForearmMotor_s *Forearmrad_out)
{
    // static rt_uint8_t Lock_flag; // 锁定标志位
    // static float BoomYaw_InitialLatch;
    // if (Lock_flag == 0 && BoomYawEncoder_Str.Data_Valid == valid)
    // {
    //     BoomYaw_InitialLatch = BoomYawEncoder_in - BOOMYAW_ZEROPOINT;
    //     Lock_flag++;
    // }
    // Boomrad_out->BoomYaw = BoomAngle_in->BoomYaw + BoomYaw_InitialLatch;

    Boomrad_out->BoomYaw = BoomAngle_in->BoomYaw - BOOMYAW_ZEROPOINT;

    Boomrad_out->BoomPitch1 = BoomAngle_in->BoomPitch1 - BOOMPITCH1_ZEROPOINT;
    Boomrad_out->BoomPitch2 = BoomAngle_in->BoomPitch2 - BOOMPITCH2_ZEROPOINT;

    Forearmrad_out->ForearmYaw = ForearmAngle_in->ForearmYaw - FOREARMYAW_ZEROPOINT;
    Forearmrad_out->ForearmPitch = ForearmAngle_in->ForearmPitch - FOREARMPITCH_ZEROPOINT;
}

/**
 * @brief 电机位置信息由角度转向弧度
 * @param BoomAngle_in
 * @param ForearmAngle_in
 * @param Boomrad_out
 * @param Forearmrad_out
 */
void Arm_angle2rad(BoomMotor_s *BoomAngle_in,
                   ForearmMotor_s *ForearmAngle_in,
                   BoomMotor_s *Boomrad_out,
                   ForearmMotor_s *Forearmrad_out)
{

    Boomrad_out->BoomYaw = DEG2RAD_f(BoomAngle_in->BoomYaw);
    Boomrad_out->BoomPitch1 = DEG2RAD_f(BoomAngle_in->BoomPitch1);
    Boomrad_out->BoomPitch2 = DEG2RAD_f(BoomAngle_in->BoomPitch2);

    Forearmrad_out->ForearmYaw = DEG2RAD_f(ForearmAngle_in->ForearmYaw);
    Forearmrad_out->ForearmPitch = DEG2RAD_f(ForearmAngle_in->ForearmPitch);
    Forearmrad_out->ForearmRoll = DEG2RAD_f(ForearmAngle_in->ForearmRoll);
}

/**
 * @brief 机械臂角度限幅
 * @param ForearmAngle
 */
void Forearm_angleLimit(ForearmMotor_s *ForearmAngle)
{
    utils_truncate_number(&ForearmAngle->ForearmYaw,
                          ForearmYaw_AngleMin, ForearmYaw_AngleMax);
    utils_truncate_number(&ForearmAngle->ForearmPitch,
                          ForearmPitch_AngleMin, ForearmPitch_AngleMax);
    utils_truncate_number(&ForearmAngle->ForearmRoll,
                          ForearmRoll_AngleMin, ForearmRoll_AngleMax);
}

/**
 * @brief 机械臂角度限幅
 * @param BoomAngle
 * @param ForearmAngle
 */
void Arm_angleLimit(BoomMotor_s *BoomAngle,
                    ForearmMotor_s *ForearmAngle)
{

    utils_truncate_number(&BoomAngle->BoomYaw,
                          BoomYaw_AngleMin, BoomYaw_AngleMax);
    utils_truncate_number(&BoomAngle->BoomPitch1,
                          BoomPitch1_AngleMin, BoomPitch1_AngleMax);
    utils_truncate_number(&BoomAngle->BoomPitch2,
                          BoomPitch2_AngleMin, BoomPitch2_AngleMax);

    utils_truncate_number(&ForearmAngle->ForearmYaw,
                          ForearmYaw_AngleMin, ForearmYaw_AngleMax);
    utils_truncate_number(&ForearmAngle->ForearmPitch,
                          ForearmPitch_AngleMin, ForearmPitch_AngleMax);
    utils_truncate_number(&ForearmAngle->ForearmRoll,
                          ForearmRoll_AngleMin, ForearmRoll_AngleMax);
}

/**
 * @brief 机械臂电机数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ArmMotor_DataValid_If(void)
{
    /*等待电机第一次通信完毕*/
    rt_uint8_t CheckSum = 0;

    CheckSum = 0;

    CheckSum += BoomYawMotor_Str.dji.Data_Valid;
    CheckSum += BoomPitch1Motor_Str.dji.Data_Valid;
    CheckSum += BoomPitch2Motor_Str.dji.Data_Valid;
    CheckSum += ForearmYawMotor_Str.dji.Data_Valid;
    CheckSum += ForearmPitchMotor_Str.dji.Data_Valid;
    CheckSum += ForearmRollMotor_Str.dji.Data_Valid;

    if (CheckSum == 6)
        return 1;
    else
    {
        return 0;
    }
}

/**
 * @brief  大机械臂yaw编码器数据有效性
 * @return 有效则返回1 ，无效则返回0
 */
int BoomYawEncoder_DataValid_If(void)
{
    /*等待电机第一次通信完毕*/
    rt_uint8_t CheckSum = 0;

    CheckSum = 0;

    CheckSum += BoomYawEncoder_Str.Data_Valid;

    if (CheckSum == 1)
        return 1;
    else
    {
        return 0;
    }
}

/**
 * @brief 电机输入电流扭矩计算
 * @param BoomComp_in       扭矩补偿输入
 * @param ForearmComp_in       扭矩补偿输入
 * @param Boom_out          Boom电机的电流设定输出
 * @param Forearm_out       Forearm电机的电流设定输出
 */
void ArmMotorinput_Calculate(BoomMotor_s *BoomComp_in,
                             ForearmMotor_s *ForearmComp_in,
                             BoomMotor_s *Boom_out,
                             ForearmMotor_s *Forearm_out)
{
    Boom_out->BoomYaw = BoomYawMotor_Str.spe.out;
    Boom_out->BoomPitch1 = BoomPitch1Motor_Str.spe.out + BoomComp_in->BoomPitch1;
    Boom_out->BoomPitch2 = BoomPitch2Motor_Str.spe.out + BoomComp_in->BoomPitch2;

    Forearm_out->ForearmYaw = ForearmYawMotor_Str.spe.out;
    Forearm_out->ForearmPitch = ForearmPitchMotor_Str.spe.out + ForearmComp_in->ForearmPitch;
    Forearm_out->ForearmRoll = ForearmRollMotor_Str.spe.out + SinWave_Get(FOREARMMOTOR_CTRLPERIOD,
                                                                          ARMDISTURBANCE_AMPLITUDE);
}

/**
 * @brief 获取电机结构体
 * @author mylj
 * @param  ArmMotor         指定需要获取的电机
 * @return void*            返回指向滤波器的指针
 */
void *Get_ArmMotor(ArmMotor_e ArmMotor)
{
    switch (ArmMotor)
    {
    case BoomYaw:
        return (void *)&BoomYawMotor_Str;
    case BoomPitch1:
        return (void *)&BoomPitch1Motor_Str;
    case BoomPitch2:
        return (void *)&BoomPitch2Motor_Str;
    case ForearmYaw:
        return (void *)&ForearmYawMotor_Str;
    case ForearmPitch:
        return (void *)&ForearmPitchMotor_Str;
    case ForearmRoll:
        return (void *)&ForearmRollMotor_Str;
    default:
        return NULL;
    }
}

/**
 * @brief 获取电机校准结构体
 * @author mylj
 * @param  ArmMotorAlign        指定需要获取的电机
 * @return void*                返回指向滤波器的指针
 */
void *Get_ArmMotorAlign(ArmMotor_e ArmMotorAlign)
{
    switch (ArmMotorAlign)
    {
    case ForearmRoll:
        return (void *)&ForearmRollMotorAlign_Str;
    default:
        return NULL;
    }
}

/**
 * @brief 机械臂roll轴对位
 */
void ArmRollAlign(void)
{
    Motor_IfLocked(&ForearmRollMotor_Str,
                   &ForearmRollMotorAlign_Str);

    Align(&ForearmRollMotor_Str,
          &ForearmRollMotorAlign_Str);
}

/**
 * @brief  获取大机械Yaw编码器结构体
 * @author mylj
 * @param  ImageEncoder         指定需要获取的电机
 * @return void*              返回指向电机结构体的指针
 */
void *Get_ArmEncoder(ArmMotor_e ArmEncoder)
{
    switch (ArmEncoder)
    {
    case BoomYaw:
        return (void *)&BoomYawEncoder_Str;
    default:
        return NULL;
    }
}
