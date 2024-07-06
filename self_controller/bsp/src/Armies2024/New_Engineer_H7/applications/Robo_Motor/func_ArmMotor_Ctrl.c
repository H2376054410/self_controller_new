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
#include "drv_RemoteCtrl_data.h"
/*电机相关结构体*/
Motor_t BoomYawMotor_Str, BoomPitch1Motor_Str, BoomPitch2Motor_Str,
    ForearmYawMotor_Str, ForearmPitchMotor_Str, ForearmRollMotor_Str, BoomMoveMotor_Str;
MotorAlign_t ForearmPitchMotorAlign_Str,ForearmRollMotorAlign_Str;
uint8_t rollalign_flag = 0;
uint8_t pitchalign_flag = 0;
static SetPlanning_Str ForearmYaw_MotorPlan,
    ForearmPitch_MotorPlan,
    ForearmRoll_MotorPlan; // 小机械臂电机
static SetPlanning_Str Gold_ArmYaw_MotorPlan,
    Gold_ArmRoll_MotorPlan,
    Gold_ArmPitch_MotorPlan; // 小机械臂电机
static SetPlanning_Str BoomYaw_MotorPlan,
    BoomPitch1_MotorPlan, BoomPitch2_MotorPlan,
    BoomMove_MotorPlan;       // 小机械臂电机
Encoder_s BoomYawEncoder_Str; // 大yaw编码器的值
int Remote_Tanslate=1;//决定是否使用遥控器数据
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
    motor_init(&BoomYawMotor_Str, 0,    // 电机结构体和电机ID,控制th0角度的电机
               BoomYaw_Ratio,           // 电机减速比
               ANGLE_CTRL_ABS,          // 电机角度控制模式——输出轴、设定值不可跨圈
               BoomYaw_EncoderLen,      // 编码器长度
               360, 0, 0);         // 最大值、最小值，正向
    motor_init(&BoomPitch1Motor_Str, 0, // 控制th1角度的电机
               BoomPitch1_Ratio,
               ANGLE_CTRL_ABS,
               BoomPitch1_EncoderLen,
               180, -180, 0);
    motor_init(&BoomPitch2Motor_Str, 0, // 控制th2角度电机
               BoomPitch2_Ratio,
               ANGLE_CTRL_ABS,
               BoomPitch2_EncoderLen,
               180, -180, 0);
    motor_init(&ForearmYawMotor_Str, 0, // 控制th3角度电机
               ForearmYaw_Ratio,
               ANGLE_CTRL_ABS,
               ForearmYaw_EncoderLen,
               180, -180, 0);
    motor_init_DM(&ForearmPitchMotor_Str, 0, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  ForearmPitch_EncoderLen,
                  180, -180, 0);
    motor_init_DM(&ForearmRollMotor_Str, 0, // 控制roll角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  ForearmRoll_EncoderLen,
                  180, -180, 0);
    motor_init(&BoomMoveMotor_Str, 0, // 控制底端机械臂移动的
               BoomMove_Ratio,
               ANGLE_CTRL_FULL,
               BoomMove_Ecoderlen,
               360, 0, 0);

#if ARM_MOTOROUTCLOSE_ENABLE
    pid_init(&BoomYawMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&BoomYawMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&BoomPitch1Motor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&BoomPitch1Motor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&BoomPitch2Motor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&BoomPitch2Motor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&BoomMoveMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&BoomMoveMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&ForearmYawMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&ForearmYawMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&ForearmPitchMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&ForearmPitchMotor_Str.spe, 0, 0, 0, 0, 0, -0);

    pid_init(&ForearmRollMotor_Str.ang, 0, 0, 0, 0, 0, -0);
    pid_init(&ForearmRollMotor_Str.spe, 0, 0, 0, 0, 0, -0);
#else
    pid_init(&BoomYawMotor_Str.ang, 30, 0, 3, 0, 15, -15); 
    pid_init(&BoomYawMotor_Str.spe, 200, 0, 3, 0, 600, -600);

    // pid_init(&BoomPitch1Motor_Str.ang, 100, 0, 0, 0, 200, -200);
    // pid_init(&BoomPitch1Motor_Str.spe, 100, 0, 200, 0, 1000, -1000);

    // pid_init(&BoomPitch2Motor_Str.ang, 100, 0, 0, 0, 200, -200);
    // pid_init(&BoomPitch2Motor_Str.spe, 100, 0, 200, 0, 1000, -1000);

    pid_init(&BoomPitch1Motor_Str.ang, 60, 0, 1.5, 40, 100, -100);
    pid_init(&BoomPitch1Motor_Str.spe, 250, 0, 0, 0, 950, -950);

    pid_init(&BoomPitch2Motor_Str.ang, 20, 0.1, 0, 5, 20, -20);
    pid_init(&BoomPitch2Motor_Str.spe, 150, 0, 0, 0, 500, -500);

    pid_init(&BoomMoveMotor_Str.ang, 15, 0, 0, 0, 45, -45);
    pid_init(&BoomMoveMotor_Str.spe, 80, 0, 0, 0, 6000, -6000);

    pid_init(&ForearmYawMotor_Str.ang, 10, 0, 0, 0, 10, -10);
    pid_init(&ForearmYawMotor_Str.spe, 120, 0, 0, 0, 600, -600);

    pid_init(&ForearmPitchMotor_Str.ang,1.5, 0.02, 0, 1.5, 10, -10);
    pid_init(&ForearmPitchMotor_Str.spe, 37, 0, 10, 0, 360, -360);
 
    pid_init(&ForearmRollMotor_Str.ang, 1.5, 0.02, 0, 1.5, 10, -10);
    pid_init(&ForearmRollMotor_Str.spe, 37, 0, 10,0, 360, -360);

#endif
    // pid_ErrorDeadzone_Set(&BoomYawMotor_Str.ang, 1, 0.015f); // 死区.2023
    MotorAlign_Init(&ForearmPitchMotorAlign_Str, // 对位结构体指针  
                    Start_SpecificAngle_Align,  // 特定角度对位
                    120, 200,                   // 最大的设定电流值、最小速度、
                    FowardStall, 4,             // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, 120.0f);    // 电机的控制模式、电机初始角度

    MotorAlign_Init(&ForearmRollMotorAlign_Str, // 对位结构体指针  
                    Start_SpecificAngle_Align,  // 特定角度对位
                    120, 200,                   // 最大的设定电流值、最小速度、
                    ReversStall, 4,             // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, -120.0f);    // 电机的控制模式、电机初始角度
    SetPlanSettings_Init(&Gold_ArmYaw_MotorPlan.Settings,
                         FOREARMYAW_SPEEDMAX,
                         ForearmYAW_POSERRORMAX,
                         ForearmYAW_ACCLMAX,
                         SETPPLANPERIOD);
    SetPlanSettings_Init(&Gold_ArmPitch_MotorPlan.Settings,
                         FOREARMPITCH_SPEEDMAX,
                         FOREARMPITCH_POSERRORMAX,
                         FOREARMPITCH_ACCLMAX,
                         SETPPLANPERIOD);
    SetPlanSettings_Init(&Gold_ArmRoll_MotorPlan.Settings,
                         FOREARMROLL_SPEEDMAX,
                         FOREARMROLL_POSERRORMAX,
                         FOREARMROLL_ACCLMAX,
                         SETPPLANPERIOD);
	  SetPlanSettings_Init(&BoomYaw_MotorPlan.Settings,
                         0.05,
                         0.05,
                         0.2,
                         SETPPLANPERIOD);
		SetPlanSettings_Init(&BoomPitch1_MotorPlan.Settings,
                         0.05,
                         0.05,
                         0.2,
                         SETPPLANPERIOD);
		SetPlanSettings_Init(&BoomPitch2_MotorPlan.Settings,
                         0.05,
                         0.05,
                         0.2,
                         SETPPLANPERIOD);
}

/*******************************电机控制***************************/
float debug______shjfjshf;
float anglesetplan;
CrossCircleData_s BoomYaw_AngleSetPlan_cross = {.Circle_Len = 2 * PI}; // 设定值规划输出跨圈处理
int boommove_clear=0;
int16_t Align_flag=0;
int16_t Align_flag_2=0;
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
    case BoomMove:
			if(Move_flag==1)
			{
			if(boommove_clear==0)
			{
        BoomMoveMotor_Str.dji.angle=0;
        BoomMoveMotor_Str.dji.loop=0;
        BoomMoveMotor_Str.dji.old_angle=0;
        BoomMoveMotor_Str.dji.extra_angle=0;
				boommove_clear++;
			}
		}
        Motor_Write_SetAngle_ABS(&BoomMoveMotor_Str, BoomStateData->AngleSetPlan.BoomMove); // 设定值
        Motor_AnglePIDCalculate(&BoomMoveMotor_Str, BoomStateData->AngleNowFilter.BoomMove);
        Motor_Write_SetSpeed_ABS(&BoomMoveMotor_Str,
                                 BoomMoveMotor_Str.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomMove);
        Motor_SpeedPIDCalculate(&BoomMoveMotor_Str,
                                BoomStateData->SpeedNowFilter.BoomMove);
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
        if (ForearmPitchMotorAlign_Str.AlignState == Align_OK) // 对位完成的状态
        {
            if (ForearmPitchMotorAlign_Str.ControlMode == 3) // 角度闭环模式
            {
							if(Align_flag>20)
							{ 
								if ((uint32_t)(Robo_Control(AirPump_State, Read, NULL)) == AirPumpState_Open)
								{
									    pid_init(&ForearmPitchMotor_Str.ang,3, 0.02, 0, 1.5, 20, -20);
                      pid_init(&ForearmPitchMotor_Str.spe, 65, 0, 10, 0, 1500, -1500);
								}else 
								{
									    pid_init(&ForearmPitchMotor_Str.ang,2, 0.05, 0, 2, 10, -10);
                      pid_init(&ForearmPitchMotor_Str.spe, 15, 0, 0, 0, 500, -500);
								}

								      Motor_Write_SetAngle_ABS(&ForearmPitchMotor_Str,
                                         ForearmStateData->AngleSetPlan.ForearmPitch); // 设定值
                      Motor_AnglePIDCalculate(&ForearmPitchMotor_Str, ForearmStateData->AngleNowFilter.ForearmPitch);

                      Motor_Write_SetSpeed_ABS(&ForearmPitchMotor_Str,
                                         ForearmPitchMotor_Str.ang.out +
                                             ForearmStateData->SpeedFeedforward.ForearmPitch);
                      Motor_SpeedPIDCalculate(&ForearmPitchMotor_Str,
                                        ForearmStateData->SpeedNowFilter.ForearmPitch);
							}
							else
							{
								Align_flag++;
							}
            }
        }
        else if (ForearmPitchMotorAlign_Str.ControlMode == 2) // 转速闭环模式
        {
            ForearmPitchMotor_Str.spe.set=ForearmStateData->SpeedHope.ForearmPitch;
            Motor_SpeedPIDCalculate(&ForearmPitchMotor_Str,
                                    ForearmStateData->SpeedNowFilter.ForearmPitch);
        }
        break;
    case ForearmRoll:
        if (ForearmPitchMotorAlign_Str.AlignState == Align_OK) // 对位完成的状态
        {
            if (ForearmPitchMotorAlign_Str.ControlMode == 3) // 角度闭环模式
            {
							if(Align_flag>20)
							{

								if ((uint32_t)(Robo_Control(AirPump_State, Read, NULL)) == AirPumpState_Open)
								{
									
                      pid_init(&ForearmRollMotor_Str.ang, 3, 0.05, 0, 2, 20, -20);
                      pid_init(&ForearmRollMotor_Str.spe, 65, 0, 10,0, 1500, -1500);
								}else 
								{
                      pid_init(&ForearmRollMotor_Str.ang, 2, 0.02, 0, 1.5, 10, -10);
                      pid_init(&ForearmRollMotor_Str.spe, 15, 0, 0,0, 360, -360);										
								}
                Motor_Write_SetAngle_ABS(&ForearmRollMotor_Str, ForearmStateData->AngleSetPlan.ForearmRoll);

                Motor_AnglePIDCalculate(&ForearmRollMotor_Str, ForearmStateData->AngleNowFilter.ForearmRoll);
                Motor_Write_SetSpeed_ABS(&ForearmRollMotor_Str,
                                         ForearmRollMotor_Str.ang.out +
                                             ForearmStateData->SpeedFeedforward.ForearmRoll);
                Motor_SpeedPIDCalculate(&ForearmRollMotor_Str,
                                ForearmStateData->SpeedNowFilter.ForearmRoll);
							}
							
            }
        }
        else if (ForearmPitchMotorAlign_Str.ControlMode == 2) // 转速闭环模式
        {
            ForearmRollMotor_Str.spe.set=ForearmStateData->SpeedHope.ForearmRoll;
            Motor_SpeedPIDCalculate(&ForearmRollMotor_Str,
                                    ForearmStateData->SpeedNowFilter.ForearmRoll);
        }
        break;
    default:
        break;
    }
}


/**
 * @brief 矿臂姿态电机的设定值规划
 * @brief 改变电机AngleSetPlan和SpeedFeedforward的数据
 * @param ForearmStateData   电机位姿信息（角度，角速度，期望角度）
 * @param AngleSetPlan
 * @param SpeedFeedforward
 */
void GoldArmSetPlanning(Gold_Data_s *GoldArmStateData,
                        Arm_Angle_Gold *AngleSetPlan,
                        Arm_Angle_Gold *SpeedFeedforward)
{
    /*矿臂yaw设定值规划*/
    Gold_ArmYaw_MotorPlan.Input.Now.pos = GoldArmStateData->AngleNowFilter.arm_yaw;
    Gold_ArmYaw_MotorPlan.Input.Now.spe = GoldArmStateData->SpeedNowFilter.arm_yaw;
    Gold_ArmYaw_MotorPlan.Input.Set.pos = GoldArmStateData->AngleHope.arm_yaw;
    SetPlanning_OldCal(&Gold_ArmYaw_MotorPlan);
    AngleSetPlan->arm_yaw = Gold_ArmYaw_MotorPlan.Output.pos;
    SpeedFeedforward->arm_yaw = Gold_ArmYaw_MotorPlan.Output.spe;

    /*矿臂pitch设定值规划*/
    Gold_ArmPitch_MotorPlan.Input.Now.pos = GoldArmStateData->AngleNowFilter.arm_pitch;
    Gold_ArmPitch_MotorPlan.Input.Now.spe = GoldArmStateData->SpeedNowFilter.arm_pitch;
    Gold_ArmPitch_MotorPlan.Input.Set.pos = GoldArmStateData->AngleHope.arm_pitch;
    SetPlanning_OldCal(&Gold_ArmPitch_MotorPlan);
    AngleSetPlan->arm_pitch = Gold_ArmPitch_MotorPlan.Output.pos;
    SpeedFeedforward->arm_pitch = Gold_ArmPitch_MotorPlan.Output.spe;

    /*矿臂roll设定值规划*/
    Gold_ArmRoll_MotorPlan.Input.Now.pos = GoldArmStateData->AngleNowFilter.arm_roll;
    Gold_ArmRoll_MotorPlan.Input.Now.spe = GoldArmStateData->SpeedNowFilter.arm_roll;
    Gold_ArmRoll_MotorPlan.Input.Set.pos = GoldArmStateData->AngleHope.arm_roll;
    SetPlanning_OldCal(&Gold_ArmRoll_MotorPlan);
    AngleSetPlan->arm_roll = Gold_ArmRoll_MotorPlan.Output.pos;
    SpeedFeedforward->arm_roll = Gold_ArmRoll_MotorPlan.Output.spe;
}


/**
 * @brief BOOM姿态电机的设定值规划
 * @brief 改变电机AngleSetPlan和SpeedFeedforward的数据
 * @param ForearmStateData   电机位姿信息（角度，角速度，期望角度）
 * @param AngleSetPlan
 * @param SpeedFeedforward
 */
void BoomArmSetPlanning(BoomState_Data_s *BoomArmStateData,
                        BoomMotor_s *AngleSetPlan,
                        BoomMotor_s *SpeedFeedforward)
{
    /*矿臂yaw设定值规划*/
    BoomYaw_MotorPlan.Input.Now.pos = BoomArmStateData->AngleNowFilter.BoomYaw;
    BoomYaw_MotorPlan.Input.Now.spe = BoomArmStateData->SpeedNowFilter.BoomYaw;
    BoomYaw_MotorPlan.Input.Set.pos = BoomArmStateData->AngleHope.BoomYaw;
    SetPlanning_OldCal(&BoomYaw_MotorPlan);
    AngleSetPlan->BoomYaw = BoomYaw_MotorPlan.Output.pos;
    SpeedFeedforward->BoomYaw = BoomYaw_MotorPlan.Output.spe;

    /*矿臂yaw设定值规划*/
    BoomPitch1_MotorPlan.Input.Now.pos = BoomArmStateData->AngleNowFilter.BoomPitch1;
    BoomPitch1_MotorPlan.Input.Now.spe = BoomArmStateData->SpeedNowFilter.BoomPitch1;
    BoomPitch1_MotorPlan.Input.Set.pos = BoomArmStateData->AngleHope.BoomPitch1;
    SetPlanning_OldCal(&BoomPitch1_MotorPlan);
    AngleSetPlan->BoomPitch1 = BoomPitch1_MotorPlan.Output.pos;
    SpeedFeedforward->BoomPitch1 = BoomPitch1_MotorPlan.Output.spe;


    /*矿臂yaw设定值规划*/
//    BoomPitch2_MotorPlan.Input.Now.pos = BoomArmStateData->AngleNowFilter.BoomPitch2;
//    BoomPitch2_MotorPlan.Input.Now.spe = BoomArmStateData->SpeedNowFilter.BoomPitch2;
//    BoomPitch2_MotorPlan.Input.Set.pos = BoomArmStateData->AngleHope.BoomPitch2;
//    SetPlanning_OldCal(&BoomPitch2_MotorPlan);
//    AngleSetPlan->BoomPitch2 = BoomPitch2_MotorPlan.Output.pos;
//    SpeedFeedforward->BoomPitch2 = BoomPitch2_MotorPlan.Output.spe;

}
/**
 * @brief 将Boom读取到的编码器的速度转换成rad
 * @param Spe_encoder
 * @param Speed_rpm
 */
void BoomEncoderSpeTorpm(BoomMotor_s *Spe_encoder,
                         BoomMotor_s *Speed_rad)
{
    Speed_rad->BoomYaw =
        (Spe_encoder->BoomYaw / 10.0f) * PI / 30.0f;
    Speed_rad->BoomPitch1 =
        (Spe_encoder->BoomPitch1 / 10.0f) * PI / 30.0f;
    Speed_rad->BoomPitch2 =
        (Spe_encoder->BoomPitch2 / 20.0f) * PI / 30.0f;
    Speed_rad->BoomMove =
        Spe_encoder->BoomMove /BoomMove_Ecoderlen/ BooomMove_RPMRatio ;
}

/**
 * @brief 将Forearm读取到的编码器的速度转换成RAD
 * @param Spe_encoder
 * @param Speed_rpm
 */
void ForearmEncoderSpeTorpm(ForearmMotor_s *Spe_encoder,
                            ForearmMotor_s *Speed_rad)
{
    Speed_rad->ForearmPitch =
        Spe_encoder->ForearmPitch / ForearmPitch_EncoderLen*45;
    Speed_rad->ForearmYaw =
        (Spe_encoder->ForearmYaw/10.0f ) * PI /30.0f ;
    Speed_rad->ForearmRoll =
        Spe_encoder->ForearmRoll /ForearmPitch_EncoderLen*45 ;
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

    /*boommove*/
    BoomStateData->AngleNowFilter.BoomMove =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomMove,
                      BoomStateData->AngleNow.BoomMove, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomMove =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomMove,
                      BoomStateData->SpeedNow.BoomMove, 0.8f);
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
 * @brief 把大机械臂编码器值转成角度值
 * @brief 包括传动比
 * @param BoomAngle_in
 * @param BoomAngle_out
 */
void BoomEncoder_angle(BoomMotor_s *BoomAngle_in,
                       BoomMotor_s *BoomAngle_out,
											float *bp2_last)
{
    BoomAngle_out->BoomYaw = BoomAngle_in->BoomYaw * 360.0f / BoomYaw_EncoderLen;
    BoomAngle_out->BoomPitch1 = BoomAngle_in->BoomPitch1 * 360.0f /
                                BoomPitch1_EncoderLen;
		BoomAngle_out->BoomPitch2 =(BoomAngle_in->BoomPitch2) * 360.0f /
                                BoomPitch2_EncoderLen/1.6f+35.0f;
	 if(Move_flag==0&&BoomAngle_out->BoomPitch2<-25.0f)
		BoomAngle_out->BoomPitch2 = BoomAngle_out->BoomPitch2+180.0f;
	 else{
		if(*bp2_last>0.0f&&BoomAngle_out->BoomPitch2<-25.0f)
			BoomAngle_out->BoomPitch2 = BoomAngle_out->BoomPitch2+180.0f;
		else if(*bp2_last<0.0f&&BoomAngle_out->BoomPitch2>25.0f)
			BoomAngle_out->BoomPitch2 = BoomAngle_out->BoomPitch2-180.0f;
			}
		*bp2_last =BoomAngle_out->BoomPitch2;
    BoomAngle_out->BoomMove = BoomAngle_in->BoomMove;
}

/**
 * @brief 把小机械臂编码器值转成角度值
 * @brief 包括传动比
 * @param ForearmAngle_in
 * @param ForearmAngle_out
 */
void ForearmEncoder_angle(ForearmMotor_s *ForearmAngle_in,
                          ForearmMotor_s *ForearmAngle_out)
{
    ForearmAngle_out->ForearmPitch = ForearmAngle_in->ForearmPitch ;
    ForearmAngle_out->ForearmRoll = ForearmAngle_in->ForearmRoll ;
    ForearmAngle_out->ForearmYaw = ForearmAngle_in->ForearmYaw * 360.0f /
                                   ForearmYaw_EncoderLen;
}
/**
 * @brief 把机械臂数据转化为弧度制
 */
void Arm_MotorAngle_Rad(BoomMotor_s *BoomAngle_in,
                        BoomMotor_s *BoomAngle_out,
                        ForearmMotor_s *ForearmAngle_in,
                        ForearmMotor_s *ForearmAngle_out)
{
    BoomAngle_out->BoomPitch1 = BoomAngle_in->BoomPitch1*PI/180.0f;
    BoomAngle_out->BoomPitch2 = BoomAngle_in->BoomPitch2*PI/180.0f;
    BoomAngle_out->BoomYaw = BoomAngle_in->BoomYaw*PI/180.0f;
    BoomAngle_out->BoomMove = BoomAngle_in->BoomMove*PI/180.0f;
        
    ForearmAngle_out->ForearmPitch = ForearmAngle_in->ForearmPitch*PI/180.0f;
    ForearmAngle_out->ForearmRoll = ForearmAngle_in->ForearmRoll*PI/180.0f;
    ForearmAngle_out->ForearmYaw = ForearmAngle_in->ForearmYaw*PI/180.0f;
}

/**
 * @brief 实际角度转化为算法角度
 * @param BoomAngle_in
 * @param ForearmAngle_in
 * @param Boomrad_out
 * @param Forearmrad_out
 */
void Arm_angle2rad(BoomMotor_s *BoomAngle_in,
                   ForearmMotor_s *ForearmAngle_in,
                   Boom_Mid_Data_s *Boomrad_out,
                   Forearm_Mid_Data_s *Forearmrad_out)
{
	// 实际角度转化为算法角度
    // Boomrad_out->AngleNow.Data.BoomYaw = DEG2RAD_f(BoomAngle_in->BoomYaw-90.0f);
    // Boomrad_out->AngleNow.Data.BoomPitch1 = DEG2RAD_f(125.0f-BoomAngle_in->BoomPitch1);
    // Boomrad_out->AngleNow.Data.BoomPitch2 = DEG2RAD_f(BoomAngle_in->BoomPitch2);
    // Forearmrad_out->AngleNow.Data.ForearmYaw = DEG2RAD_f(ForearmAngle_in->ForearmYaw-85.0f);
    // Forearmrad_out->AngleNow.Data.ForearmPitch = (DEG2RAD_f(ForearmAngle_in->ForearmRoll-ForearmAngle_in->ForearmPitch))/2.0f;
    // Forearmrad_out->AngleNow.Data.ForearmRoll = (DEG2RAD_f(ForearmAngle_in->ForearmRoll+ForearmAngle_in->ForearmPitch))/4.0f;
    Boomrad_out->AngleNow.Data.BoomYaw = (BoomAngle_in->BoomYaw-90.0f/180.0f*PI);
Boomrad_out->AngleNow.Data.BoomPitch1 = (125.0f/180.0f*PI-BoomAngle_in->BoomPitch1);
Boomrad_out->AngleNow.Data.BoomPitch2 = (BoomAngle_in->BoomPitch2-35.0f/180.0f*PI);
Forearmrad_out->AngleNow.Data.ForearmYaw = (ForearmAngle_in->ForearmYaw-85.0f/180.0f*PI);
Forearmrad_out->AngleNow.Data.ForearmPitch = ((ForearmAngle_in->ForearmRoll-ForearmAngle_in->ForearmPitch))/2.0f;
Forearmrad_out->AngleNow.Data.ForearmRoll = ((ForearmAngle_in->ForearmRoll+ForearmAngle_in->ForearmPitch))/4.0f;
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
    CheckSum += BoomMoveMotor_Str.dji.Data_Valid;
	if (CheckSum == 7)
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

float move_size = 0;
/**
 * @brief 电机输入电流扭矩计算
 * @param BoomComp_in       扭矩补偿输入
 * @param ForearmComp_in       扭矩补偿输入
 * @param Boom_out          Boom电机的电流设定输出
 * @param Forearm_out       Forearm电机的电流设定输出
 */
float changshu=-4.0f;
float changshu2=9.5f;
int Move_flag = 0;
float comp_in_pitch=-25.0f;
float comp_in_roll=25.0f;
void ArmMotorinput_Calculate(BoomMotor_s *BoomComp_in,
                             ForearmMotor_s *ForearmComp_in,
                             BoomMotor_s *Boom_out,
                             ForearmMotor_s *Forearm_out)
{
    Boom_out->BoomYaw = BoomYawMotor_Str.spe.out;
    Boom_out->BoomPitch1 = BoomPitch1Motor_Str.spe.out + BoomComp_in->BoomPitch1*changshu2;
    Boom_out->BoomPitch2 = BoomPitch2Motor_Str.spe.out + BoomComp_in->BoomPitch2*changshu;
	  if(Move_flag==1)
		{
//					  Boom_out->BoomMove = 0;
        Boom_out->BoomMove = BoomMoveMotor_Str.spe.out ;

		}
		else if(Move_flag==0)
		{
		Boom_out->BoomMove = 4500;
//		  Boom_out->BoomMove = 0;
		}
		else
		{
		Boom_out->BoomMove = -move_size;
		}
    Forearm_out->ForearmYaw = ForearmYawMotor_Str.spe.out;
		if ((uint32_t)(Robo_Control(AirPump_State, Read, NULL)) == AirPumpState_Open)
		{
    Forearm_out->ForearmPitch =ForearmPitchMotor_Str.spe.out+comp_in_pitch*ForearmComp_in->ForearmPitch ;//+ ForearmComp_in->ForearmPitch;
    Forearm_out->ForearmRoll = ForearmRollMotor_Str.spe.out +comp_in_roll*ForearmComp_in->ForearmPitch;//+ SinWave_Get(FOREARMMOTOR_CTRLPERIOD,ARMDISTURBANCE_AMPLITUDE);
		}
		else
		{
		Forearm_out->ForearmPitch =ForearmPitchMotor_Str.spe.out ;//+ ForearmComp_in->ForearmPitch;
    Forearm_out->ForearmRoll = ForearmRollMotor_Str.spe.out ;
		}

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
    case BoomMove:
        return (void *)&BoomMoveMotor_Str;
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
    case ForearmPitch:
        return (void *)&ForearmPitchMotorAlign_Str;
    default:
        return NULL;
    }
}


/**
 * @brief 机械臂Pitch轴和roll对位
 */
void ArmPitchAlign(void)
{
    Motor_IfLocked_Time(&ForearmPitchMotor_Str,
                   &ForearmPitchMotorAlign_Str);

    Align(&ForearmPitchMotor_Str,
          &ForearmPitchMotorAlign_Str);

    Motor_IfLocked_Time(&ForearmRollMotor_Str,
                   &ForearmRollMotorAlign_Str);

    Align(&ForearmRollMotor_Str,
          &ForearmRollMotorAlign_Str);
	if(ForearmPitchMotorAlign_Str.LockedFlag == Motor_Err)
	{
			pitchalign_flag=1;
	}
		if(ForearmRollMotorAlign_Str.LockedFlag == Motor_Err)
	{
			rollalign_flag=1;
	}
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
