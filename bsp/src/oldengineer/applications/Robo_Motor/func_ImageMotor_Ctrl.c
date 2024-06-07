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
    ImagePitchMotor_Str;
MotorAlign_t ImageYawMotorAlign_Str,
    ImagePitchMotorAlign_Str;
SetPlanning_Str ImageYaw_MotorPlan,
    ImagePitch_MotorPlan;      // 图传云台电机
Encoder_s ImageYawEncoder_Str; // 图传yaw编码器的值

#define IMAGE_MOTOROUTCLOSE_ENABLE 0
/**
 * @brief 图传部分的电机、编码器及pid控制初始化
 */
void ImageMotor_Init(void)
{
    motor_init(&ImageYawMotor_Str, 0,   // 电机结构体和电机ID
               IMAGEYAW_RATIO,          // 电机减速比
               ANGLE_CTRL_FULL,         // 电机角度控制模式——输出轴、设定值不可跨圈
               IMAGEYAW_ENCODERLEN,     // 编码器长度
               360, 0, 0);              // 最大值、最小值，不反向
    motor_init(&ImagePitchMotor_Str, 0, // 电机结构体和电机ID
               IMAGEPITCH_RATIO,        // 电机减速比
               ANGLE_CTRL_FULL,         // 电机角度控制模式——输出轴、设定值不可跨圈
               IMAGEPITCH_ENCODERLEN,   // 编码器长度
               360, 0, 1);              // 最大值、最小值，不反向

    EncoderData_Init(&ImageYawEncoder_Str,
                     1, DEFAULT_VALUE,
                     AngleMode_ABS,
                     reverse);

#if IMAGE_MOTOROUTCLOSE_ENABLE
    pid_init(&ImageYawMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&ImageYawMotor_Str.spe, 0, 0, 0, 0, 0, 0);

    pid_init(&ImagePitchMotor_Str.ang, 0, 0, 0, 0, 0, 0);
    pid_init(&ImagePitchMotor_Str.spe, 0, 0, 0, 0, 0, 0);
#else
    pid_init(&ImageYawMotor_Str.ang, 250, 0, 10, 0, 200, -200);
    pid_init(&ImageYawMotor_Str.spe, 500, 2, 8000, 1000, 3000, -3000);

    pid_init(&ImagePitchMotor_Str.ang, 250, 0, 10, 0, 200, -200);
    pid_init(&ImagePitchMotor_Str.spe, 500, 2, 8000, 1000, 3000, -3000);
#endif
    MotorAlign_Init(&ImageYawMotorAlign_Str,   // 对位结构体指针
                    Start_SpecificAngle_Align, // 特定角度对位
                    1000, 20,                  // 最大的设定电流值、最小速度、
                    ReversStall, 8,            // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, -53.0f);   // 电机的控制模式、电机初始角度
    MotorAlign_Init(&ImagePitchMotorAlign_Str, // 对位结构体指针
                    Start_SpecificAngle_Align, // 特定角度对位
                    1000, 20,                  // 最大的设定电流值、最小速度、
                    ReversStall,8,            // 校准对位模式、对位时的速度
                    AngleCtrl_Mode, -53.0f);   // 电机的控制模式、电机初始角度

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
            if (ImageYawMotorAlign_Str.ControlMode == 3) // 角度闭环模式
            {
                Motor_Write_SetAngle_ABS(&ImageYawMotor_Str, ImageStateData->AngleSetPlan.ImageYaw); // 设定值
                Motor_AnglePIDCalculate(&ImageYawMotor_Str, ImageStateData->AngleNowFilter.ImageYaw);
                Motor_Write_SetSpeed_ABS(&ImageYawMotor_Str,
                                         ImageYawMotor_Str.ang.out +
                                             ImageStateData->SpeedFeedforward.ImageYaw);
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
            if (ImagePitchMotorAlign_Str.ControlMode == 3) // 角度闭环模式
            {
                Motor_Write_SetAngle_ABS(&ImagePitchMotor_Str, ImageStateData->AngleSetPlan.ImagePitch); // 设定值
                Motor_AnglePIDCalculate(&ImagePitchMotor_Str, ImageStateData->AngleNowFilter.ImagePitch);
                Motor_Write_SetSpeed_ABS(&ImagePitchMotor_Str,
                                         ImagePitchMotor_Str.ang.out +
                                             ImageStateData->SpeedFeedforward.ImagePitch);
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
}

/**
 * @brief 将Image读取到的编码器的速度转换成RPM速度
 * @param Spe_encoder
 * @param Speed_rpm
 */
void ImageEncoderSpeTorpm(ImageMotor_s *Spe_encoder,
                          ImageMotor_s *Speed_rpm)
{
    Speed_rpm->ImageYaw =
        Spe_encoder->ImageYaw / IMAGEYAW_RPMRATIO;
    Speed_rpm->ImagePitch =
        Spe_encoder->ImagePitch / IMAGEPITCH_RPMRATIO;
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
}

/**
 * @brief Image电机转速单位由RPM换算成RAD
 * @param Speed_rpm
 * @param Speed_rad
 */
void ImageSpeedrpmTorad(ImageMotor_s *Speed_rpm,
                        ImageMotor_s *Speed_rad)
{
    Speed_rad->ImageYaw = PI * Speed_rpm->ImageYaw / 30.0f;
    Speed_rad->ImagePitch = PI * Speed_rpm->ImagePitch / 30.0f;
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
 * @brief 电机位置信息由角度转向弧度
 * @param ImageAngle_in
 * @param Imagerad_out
 */
void Image_angle2rad(ImageMotor_s *ImageAngle_in,
                     ImageMotor_s *Imagerad_out)
{
    Imagerad_out->ImageYaw = DEG2RAD_f(ImageAngle_in->ImageYaw);
    Imagerad_out->ImagePitch = DEG2RAD_f(ImageAngle_in->ImagePitch);
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

    if (CheckSum == 2) // 非主要，可以减小
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
    Image_out->ImageYaw = ImageYawMotor_Str.spe.out + SinWave_Get(Period, Amplitude);
    Image_out->ImagePitch = ImagePitchMotor_Str.spe.out + SinWave_Get(Period, Amplitude);
}

/**
 * @brief 图传云台堵转对位
 */
void ImageGimbalAlign(void)
{
    Motor_IfLocked(&ImageYawMotor_Str,
                   &ImageYawMotorAlign_Str);
    Motor_IfLocked(&ImagePitchMotor_Str,
                   &ImagePitchMotorAlign_Str);

    Align(&ImageYawMotor_Str,
          &ImageYawMotorAlign_Str);
    Align(&ImagePitchMotor_Str,
          &ImagePitchMotorAlign_Str);
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
