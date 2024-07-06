/**
 * @file func_ImageMotor_Ctrl.c
 * @brief 工程机器人图传云台电机控制
 * @brief 主要是电机和pid初始化，以及电机对位，电机数据换算
 * @author mylj
 * @version 1.0
 * @date 2023-07-17
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "func_ImageMotor_Ctrl.h"
#include "drv_utils.h"
#include "drv_Image_Solve.h"
#include "drv_motor_Locked.h"

// 为了方便调试，故没加static
/*电机相关结构体*/
Motor_t ImageYawMotor_Str,
    ImagePitchMotor_Str,ImageHighMotor_Str;
MotorAlign_t ImageYawMotorAlign_Str,
    ImagePitchMotorAlign_Str,ImageHighMotorAlign_Str;
SetPlanning_Str ImageYaw_MotorPlan,
    ImagePitch_MotorPlan,ImageHigh_MotorPlan;      // 图传云台电机
Encoder_s ImageYawEncoder_Str; // 图传yaw编码器的值
Encoder_s ImagePitchEncoder_Str,ImageHighEncoder_Str;
#define IMAGE_MOTOROUTCLOSE_ENABLE 0
/**
 * @brief 图传部分的电机、编码器及pid控制初始化
 */
void ImageMotor_Init(void)
{
    motor_init(&ImageYawMotor_Str, 0,   // 电机结构体和电机ID
               1,          // 电机减速比
               ANGLE_CTRL_FULL,         // 电机角度控制模式——输出轴、设定值不可跨圈
               IMAGEYAW_ENCODERLEN,     // 编码器长度
               360, 0, 0);              // 最大值、最小值，不反向
    motor_init(&ImagePitchMotor_Str, 0, // 电机结构体和电机ID
               IMAGEPITCH_RPMRATIO,        // 电机减速比
               ANGLE_CTRL_FULL,         // 电机角度控制模式——输出轴、设定值不可跨圈
               IMAGEPITCH_ENCODERLEN,   // 编码器长度
               360, 0, 0);              // 最大值、最小值，不反向
    motor_init(&ImageHighMotor_Str, 0, // 电机结构体和电机ID
               IMAGEHIGH_RPMRATIO,        // 电机减速比
               ANGLE_CTRL_FULL,         // 电机角度控制模式——输出轴、设定值不可跨圈
               IMAGEHIGH_ENCODERLEN,   // 编码器长度
               360, 0, 0);              // 最大值、最小值，不反向
    // EncoderData_Init(&ImageHighEncoder_Str,
    //                  1, DEFAULT_VALUE,
    //                  AngleMode_FULL,
    //                  reverse);
#if IMAGE_MOTOROUTCLOSE_ENABLE
    pid_init(&ImageYawMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&ImageYawMotor_Str.spe, 0, 0, 0, 0, 0, 0);

    pid_init(&ImagePitchMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&ImagePitchMotor_Str.spe, 0, 0, 0, 0, 0, 0);

    pid_init(&ImageHighMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&ImageHighMotor_Str.spe, 0, 0, 0, 0, 0, 0);
#else
    pid_init(&ImageYawMotor_Str.ang, 15, 0, 0, 0, 20, -20);
    pid_init(&ImageYawMotor_Str.spe, 150, 0, 0, 0, 2000, -2000);

    pid_init(&ImagePitchMotor_Str.ang, 60, 0, 0, 0, 20, -20);
    pid_init(&ImagePitchMotor_Str.spe, 300, 0, 0, 0, 3000, -3000);

    pid_init(&ImageHighMotor_Str.ang, 12.5, 0.05, 0, 3, 80, -80);
    pid_init(&ImageHighMotor_Str.spe, 60, 0, 0, 0, 5000, -5000);
#endif
    MotorAlign_Init(&ImageHighMotorAlign_Str,   // 对位结构体指针
                    Start_SpecificAngle_Align, // 特定角度对位
                    1150, 10,                  // 最大的设定电流值、最小速度、
                    FowardStall, 20,            // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, 0.0f);   // 电机的控制模式、电机初始角度
    MotorAlign_Init(&ImagePitchMotorAlign_Str, // 对位结构体指针
                    Start_SpecificAngle_Align, // 特定角度对位
                    2500, 5,                  // 最大的设定电流值、最小速度、
                    ReversStall,20,            // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, 0.0f);   // 电机的控制模式、电机初始角度
    MotorAlign_Init(&ImageYawMotorAlign_Str, // 对位结构体指针
                    Start_SpecificAngle_Align, // 特定角度对位
                    1750, 5,                  // 最大的设定电流值、最小速度、
                    ReversStall,12,            // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, 0.0f);   // 电机的控制模式、电机初始角度
    SetPlanSettings_Init(&ImageYaw_MotorPlan.Settings,
                         IMAGEYAW_SPEEDMAX,
                         IMAGEYAW_POSERRORMAX,
                         IMAGEYAW_ACCLMAX,
                         IMAGE_SETPPLANPERIOD);
    SetPlanSettings_Init(&ImagePitch_MotorPlan.Settings,
                         IMAGEPITCH_SPEEDMAX,
                         IMAGEPITCH_POSERRORMAX,
                         IMAGEPITCH_ACCLMAX,
                         IMAGE_SETPPLANPERIOD);
    SetPlanSettings_Init(&ImageHigh_MotorPlan.Settings,
                         IMAGEPITCH_SPEEDMAX,
                         IMAGEPITCH_POSERRORMAX,
                         IMAGEPITCH_ACCLMAX,
                         IMAGE_SETPPLANPERIOD);                         
}

/*********************电机控制********************/

/**
 * @brief 图传电机闭环控制
 * @param ImageMotor
 * @param ImageStateData
 */
void ImageMotor_Ctrl(ImageMotor_e ImageMotor,
                     ImageState_Data_s *ImageStateData)
{
    switch (ImageMotor)
    {
    case ImageYaw:
			  if (ImageYawMotorAlign_Str.AlignState == Align_OK) // 对位完成的状态
       {
           if (ImageYawMotorAlign_Str.ControlMode == 3) // 编码器闭环
           {
                Motor_Write_SetAngle_ABS(&ImageYawMotor_Str, ImageStateData->AngleSetPlan.ImageYaw); // 设定值
                Motor_AnglePIDCalculate(&ImageYawMotor_Str, ImageStateData->AngleNowFilter.ImageYaw);
                Motor_Write_SetSpeed_ABS(&ImageYawMotor_Str,
                                         ImageYawMotor_Str.ang.out);
                Motor_SpeedPIDCalculate(&ImageYawMotor_Str,
                                        ImageStateData->SpeedNowFilter.ImageYaw);
           }
       }
       else if (ImageYawMotorAlign_Str.ControlMode == 2) // 转速闭环模式
       {
								Motor_SpeedPIDCalculate(&ImageYawMotor_Str,
                                        ImageStateData->SpeedNowFilter.ImageYaw);
       }						 
        break;
    case ImagePitch:
       if (ImagePitchMotorAlign_Str.AlignState == Align_OK) // 对位完成的状态
       {
           if (ImagePitchMotorAlign_Str.ControlMode == 3) // 编码器闭环
           {
                Motor_Write_SetAngle_ABS(&ImagePitchMotor_Str, ImageStateData->AngleSetPlan.ImagePitch); // 设定值
                Motor_AnglePIDCalculate(&ImagePitchMotor_Str, ImageStateData->AngleNowFilter.ImagePitch);
                Motor_Write_SetSpeed_ABS(&ImagePitchMotor_Str,
                                         ImagePitchMotor_Str.ang.out );
                Motor_SpeedPIDCalculate(&ImagePitchMotor_Str,
                                        ImageStateData->SpeedNowFilter.ImagePitch);
           }
       }
       else if (ImagePitchMotorAlign_Str.ControlMode == 2) // 转速闭环模式
       {
           Motor_SpeedPIDCalculate(&ImagePitchMotor_Str,
                                   ImageStateData->SpeedNowFilter.ImagePitch);
       }
        break;
    case ImageHigh:
       if (ImageHighMotorAlign_Str.AlignState == Align_OK) // 对位完成的状态
       {
           if (ImageHighMotorAlign_Str.ControlMode == 3) // 角度闭环模式
           {
						 
                Motor_Write_SetAngle_ABS(&ImageHighMotor_Str, ImageStateData->AngleSetPlan.ImageHigh); // 设定值
                Motor_AnglePIDCalculate(&ImageHighMotor_Str, ImageStateData->AngleNowFilter.ImageHigh);
								if(ImageHighMotor_Str.ang.out>0)
								{
									    pid_init(&ImageHighMotor_Str.spe, 20, 0, 10, 0, 5000, -5000);
								}
//								else if(ImageHighMotor_Str.ang.out<-45)
//								{
//									    pid_init(&ImageHighMotor_Str.spe, 120, 0, 0, 0, 5000, -5000);
//								}
								else
								{
									pid_init(&ImageHighMotor_Str.spe, 50, 0, 20, 0, 5000, -5000);
								}
                Motor_Write_SetSpeed_ABS(&ImageHighMotor_Str,
                                         ImageHighMotor_Str.ang.out +
                                             ImageStateData->SpeedFeedforward.ImageHigh);
                Motor_SpeedPIDCalculate(&ImageHighMotor_Str,
                                        ImageStateData->SpeedNowFilter.ImageHigh);
           }
       }
       else if (ImageHighMotorAlign_Str.ControlMode == 2) // 转速闭环模式
       {
           Motor_SpeedPIDCalculate(&ImageHighMotor_Str,
                                   ImageStateData->SpeedNowFilter.ImageHigh);
       }
        break;
    default:
        break;
    
    }
}

/**
 * @brief 图传部分电机的设定值规划
 * @brief 改变电机AngleSetPlan和SpeedFeedforward的数据
 * @param ImageStateData   电机位姿信息（角度，角速度，期望角度）
 * @param AngleSetPlan
 * @param SpeedFeedforward
 */
void ImageSetPlanning(ImageState_Data_s *ImageStateData,
                      ImageMotor_s *AngleSetPlan,
                      ImageMotor_s *SpeedFeedforward)
{
    /*ImageYaw设定值规划*/
    ImageYaw_MotorPlan.Input.Now.pos = ImageStateData->AngleNowFilter.ImageYaw;
    ImageYaw_MotorPlan.Input.Now.spe = ImageStateData->SpeedNowFilter.ImageYaw;
    ImageYaw_MotorPlan.Input.Set.pos = ImageStateData->AngleHope.ImageYaw;
    SetPlanning_Cal(&ImageYaw_MotorPlan);
    AngleSetPlan->ImageYaw = ImageYaw_MotorPlan.Output.pos;
    SpeedFeedforward->ImageYaw = ImageYaw_MotorPlan.Output.spe;

    /*ImagePitch设定值规划*/
    ImagePitch_MotorPlan.Input.Now.pos = ImageStateData->AngleNowFilter.ImagePitch;
    ImagePitch_MotorPlan.Input.Now.spe = ImageStateData->SpeedNowFilter.ImagePitch;
    ImagePitch_MotorPlan.Input.Set.pos = ImageStateData->AngleHope.ImagePitch;
    SetPlanning_Cal(&ImagePitch_MotorPlan);
    AngleSetPlan->ImagePitch = ImagePitch_MotorPlan.Output.pos;
    SpeedFeedforward->ImagePitch = ImagePitch_MotorPlan.Output.spe;

    /*Imagehigh设定值规划*/
    ImageHigh_MotorPlan.Input.Now.pos = ImageStateData->AngleNowFilter.ImageHigh;
    ImageHigh_MotorPlan.Input.Now.spe = ImageStateData->SpeedNowFilter.ImageHigh;
    ImageHigh_MotorPlan.Input.Set.pos = ImageStateData->AngleHope.ImageHigh;
    SetPlanning_Cal(&ImageHigh_MotorPlan);
    AngleSetPlan->ImageHigh = ImageHigh_MotorPlan.Output.pos;
    SpeedFeedforward->ImageHigh = ImageHigh_MotorPlan.Output.spe;
}

/**
 * @brief 将Image读取到的编码器的速度转换成rad
 * @param Spe_encoder
 * @param Speed_rpm
 */
void ImageEncoderSpeTorpm(ImageMotor_s *Spe_encoder,
                          ImageMotor_s *Speed_rad)
{
    Speed_rad->ImagePitch = PI * (Spe_encoder->ImagePitch / IMAGEPITCH_RPMRATIO) / 30.0f;
    Speed_rad->ImageHigh=PI*(Spe_encoder->ImagePitch / IMAGEHIGH_RPMRATIO) / 30.0f;
    Speed_rad->ImageYaw=PI*(Spe_encoder->ImageYaw / IMAGEYAW_RPMRATIO) / 30.0f;	
}

/**
 * @brief 对Image电机角度值及速度值进行滤波
 * @param ImageStateData
 */
void ImageMotDataFilter(ImageState_Data_s *ImageStateData)
{
    /*ImageYaw*/
    ImageStateData->AngleNowFilter.ImageYaw =
        UTILS_LP_FAST(ImageStateData->AngleNowFilter.ImageYaw,
                      ImageStateData->AngleNow.ImageYaw, 0.8f); // 滞后滤波
    ImageStateData->SpeedNowFilter.ImageYaw =
        UTILS_LP_FAST(ImageStateData->SpeedNowFilter.ImageYaw,
                      ImageStateData->SpeedNow.ImageYaw, 0.8f);

    /*ImagePitch*/
    ImageStateData->AngleNowFilter.ImagePitch =
        UTILS_LP_FAST(ImageStateData->AngleNowFilter.ImagePitch,
                      ImageStateData->AngleNow.ImagePitch, 0.8f); // 滞后滤波
    ImageStateData->SpeedNowFilter.ImagePitch =
        UTILS_LP_FAST(ImageStateData->SpeedNowFilter.ImagePitch,
                      ImageStateData->SpeedNow.ImagePitch, 0.8f);

        /*ImageHigh*/
    ImageStateData->AngleNowFilter.ImageHigh =
        UTILS_LP_FAST(ImageStateData->AngleNowFilter.ImageHigh,
                      ImageStateData->AngleNow.ImageHigh, 0.8f); // 滞后滤波
    ImageStateData->SpeedNowFilter.ImageHigh =
        UTILS_LP_FAST(ImageStateData->SpeedNowFilter.ImageHigh,
                      ImageStateData->SpeedNow.ImageHigh, 0.8f);              
}



/**
 * @brief Image电机转速单位由RAD换算成RPM
 * @param Speed_rad
 * @param Speed_rpm
 */
void ImageSpeedradTorpm(ImageMotor_s *Speed_rad,
                        ImageMotor_s *Speed_rpm)
{
    Speed_rpm->ImageYaw = Speed_rad->ImageYaw * 30.0f / PI;
    Speed_rpm->ImagePitch = Speed_rad->ImagePitch * 30.0f / PI;
}

/**
 * @brief 图传云台零点校准
 * @brief high不需要零点校准
 * @param ImageAngle_in
 * @param ImageAngle_out
 */
void Image_ZeroAdjustment(ImageMotor_s *ImageAngle_in,
                          ImageMotor_s *ImageAngle_out)
{
    ImageAngle_out->ImageYaw = ImageAngle_in->ImageYaw - IMAGEYAW_ZEROPOINT;
    ImageAngle_out->ImagePitch = ImageAngle_in->ImagePitch - IMAGEPITCH_ZEROPOINT;
}

/**
 * @brief 图传电机从编码器值转换到实际角度
 * @param ImageAngle_in
 * @param Imagerad_out
 */
 void ImageEncoder_angle(ImageMotor_s *ImageAngle_in,
                     ImageMotor_s *ImageAngle_out)
{
    ImageAngle_out->ImageHigh  = ImageAngle_in->ImageHigh;
    ImageAngle_out->ImagePitch =ImageAngle_in->ImagePitch;
    ImageAngle_out->ImageYaw =ImageAngle_in->ImageYaw;
}
/**
 * @brief 图传yaw跨圈处理
 * @param ImageAngle_in
 * @param Imagerad_out
 */
uint8_t  flag_cross_one=0;
void Image_Yaw_Cross_Circle(ImageMotor_s *ImageAngle_Now,
                            ImageMotor_s *ImageAngle_Last)
{
    if(flag_cross_one==0)
    {
        ImageAngle_Last->ImageYaw=ImageAngle_Now->ImageYaw;
        flag_cross_one++;
    }
    else{
	if(ImageAngle_Now->ImageYaw-ImageAngle_Last->ImageYaw>300.0f)
	{
        if(ImageAngle_Last->ImageYaw>330.0f)
		    ImageAngle_Now->ImageYaw = 360.0f+ImageAngle_Now->ImageYaw;
        else if(ImageAngle_Last->ImageYaw<5.0f)
            ImageAngle_Now->ImageYaw = ImageAngle_Now->ImageYaw-360.0f;
	}
    }
    ImageAngle_Last->ImageYaw=ImageAngle_Now->ImageYaw;
}
/**
 * @brief 电机位置信息由角度转向弧度
 * @param ImageAngle_in
 * @param Imagerad_out
 */
void Image_angle2rad(ImageMotor_s *ImageAngle_in,
                     ImageMotor_s *Imagerad_out)
{
    Imagerad_out->ImageYaw = DEG2RAD_f(ImageAngle_in->ImageYaw);
    Imagerad_out->ImagePitch = DEG2RAD_f(ImageAngle_in->ImagePitch);
    Imagerad_out->ImageHigh = DEG2RAD_f(ImageAngle_in->ImageHigh);
	
}

/**
 * @brief 图传云台角度限幅
 * @param ImageAngle
 */
void Image_angleLimit(ImageMotor_s *ImageAngle)
{
    utils_truncate_number(&ImageAngle->ImageYaw,
                          IMAGEYAW_ANGLEMIN, IMAGEYAW_ANGLEMAX);
    utils_truncate_number(&ImageAngle->ImagePitch,
                          IMAGEPITCH_ANGLEMIN, IMAGEPITCH_ANGLEMAX);
}

/**
 * @brief 图传云台电机数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ImageMotor_DataValid_If(void)
{
    /*等待电机第一次通信完毕*/
    rt_uint8_t CheckSum = 0;

    CheckSum = 0;

    CheckSum += ImageYawMotor_Str.dji.Data_Valid;
    CheckSum += ImagePitchMotor_Str.dji.Data_Valid;
    CheckSum += ImageHighMotor_Str.dji.Data_Valid;
    if (CheckSum == 3) // 非主要，可以减小
        return 1;
    else
    {
        return 0;
    }
}

/**
 * @brief 图传云台yaw编码器数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ImageEncoder_DataValid_If(void)
{
    /*等待电机第一次通信完毕*/
    rt_uint8_t CheckSum = 0;

    CheckSum = 0;

    CheckSum += ImageYawEncoder_Str.Data_Valid;

    if (CheckSum == 1) // 非主要，可以减小
        return 1;
    else
    {
        return 0;
    }
}

/**
 * @brief 生成一波正弦波，用于削减电机的阻力
 * @param Period            周期 单位：ms
 * @param Amplitude         幅度
 */
static float SinWave_Get(float Period, float Amplitude)
{

    return Amplitude * arm_sin_f32(2 * PI / Period * rt_tick_get());
}

/**
 * @brief 图传电机电机输入电流扭矩计算
 * @param Period          周期 单位：ms
 * @param Amplitude       幅度
 * @param Image_out       Image电机的电流设定输出
 */
void ImageMotorinput_Calculate(float Period,
                               float Amplitude,
                               ImageMotor_s *Image_out)
{
    Image_out->ImageYaw = ImageYawMotor_Str.spe.out ;//+ SinWave_Get(Period, Amplitude);
    Image_out->ImagePitch = ImagePitchMotor_Str.spe.out;// + SinWave_Get(Period, Amplitude);
		Image_out->ImageHigh = ImageHighMotor_Str.spe.out;
}

/**
 * @brief 图传云台堵转对位,只有pitch轴和high轴需要
 */
void ImageGimbalAlign(void)
{
    Motor_IfLocked(&ImagePitchMotor_Str,
                   &ImagePitchMotorAlign_Str);
    Align(&ImagePitchMotor_Str,
          &ImagePitchMotorAlign_Str);
    Motor_IfLocked(&ImageHighMotor_Str,
                   &ImageHighMotorAlign_Str);
    Align(&ImageHighMotor_Str,
          &ImageHighMotorAlign_Str);
	  Motor_IfLocked(&ImageYawMotor_Str,
                   &ImageYawMotorAlign_Str);
    Align(&ImageYawMotor_Str,
          &ImageYawMotorAlign_Str);
}

/**
 * @brief  获取图传云台电机结构体
 * @author mylj
 * @param  ImageMotor         指定需要获取的电机
 * @return void*              返回指向电机结构体的指针
 */
void *Get_ImageMotor(ImageMotor_e ImageMotor)
{
    switch (ImageMotor)
    {
    case ImageYaw:
        return (void *)&ImageYawMotor_Str;
    case ImagePitch:
        return (void *)&ImagePitchMotor_Str;
    case ImageHigh:
        return (void *)&ImageHighMotor_Str;
    default:
        return NULL;
    }
}

/**
 * @brief  获取图传云台编码器结构体
 * @author mylj
 * @param  ImageEncoder         指定需要获取的电机
 * @return void*              返回指向电机结构体的指针
 */
void *Get_ImageEncoder(ImageMotor_e ImageEncoder)
{
    switch (ImageEncoder)
    {
    case ImageYaw:
        return (void *)&ImageYawEncoder_Str;
    case ImagePitch:
        return (void *)&ImageYawEncoder_Str;
    case ImageHigh:
        return (void *)&ImageYawEncoder_Str;
    default:
        return NULL;
    }
}

/**
 * @brief 得到图传电机对位情况
 * @return int 1：两个电机对位完成   0：两个电机对位未完成
 */
int Get_ImageMotorAlign(void)
{
    if ((ImageYawMotorAlign_Str.AlignState == Align_OK) &&
        (ImagePitchMotorAlign_Str.AlignState == Align_OK))
    {
        return 1;
    }
    else
        return 0;
}
