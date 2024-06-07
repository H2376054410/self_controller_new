#ifndef __DRV_IMAGEMOTOR_CTRL_H__
#define __DRV_IMAGEMOTOR_CTRL_H__

#include <rtthread.h>
#include "drv_Vector.h"

#define IMAGEYAW_ZEROPOIN  -0.55f
typedef struct
{
    float ImageYaw;   // 图传电机1
    float ImagePitch; // 图传电机2
} ImageMotor_s;

typedef enum
{
    ImageYaw = 0, // 图传电机1
    ImagePitch,   // 图传电机2
} ImageMotor_e;

typedef struct
{
    ImageMotor_s AngleNow;
    ImageMotor_s AngleNow_Gimbal; // 解算之前，锥齿轮云台的当前角度
    ImageMotor_s AngleSet_Gimbal; // 解算之前，锥齿轮云台的设定角度
    ImageMotor_s AngleHope;
    ImageMotor_s AngleHope_Old;
    ImageMotor_s AngleSetPlan;
    ImageMotor_s AngleNowFilter;
    ImageMotor_s SpeedNow;
    ImageMotor_s SpeedNowFilter;
    ImageMotor_s SpeedFeedforward;
    ImageMotor_s MotorCtrl_Out; // 设定值规划后的速度前馈
} ImageState_Data_s;

typedef __packed struct
{
    rt_uint8_t ImageYaw_Iflimit : 1;
    rt_uint8_t ImagePitch_Iflimit : 1;
} Image_Limit_s;

/*控制角度限度*/
#define IMAGEYAW_ANGLEMAX 2.5f
#define IMAGEYAW_ANGLEMIN 0.5f
#define IMAGEPITCH_ANGLEMAX 0.5f
#define IMAGEPITCH_ANGLEMIN -0.97f

/**
 * @brief 机械限幅判断函数
 * @brief 根据图传电机角度判断是否超限
 * @param ImageAngle
 * @param Image_Limit_data
 * @return int 1 则表示超限 0 则表示没有超限
 */
int ImageMachinelimit_If(ImageMotor_s *ImageAngle,
                         Image_Limit_s *Image_Limit_data);

/**
 * @brief 图传云台设定值换算为电机角度设定值
 * @param ImageGinbal_Set
 * @param Yaw_Init          yaw的初始角度
 * @param ImageMotor_Set
 */
void ImageGimbalSet2MotorSet(ImageMotor_s *ImageGimbal_Set,
                             float Yaw_Init,
                             ImageMotor_s *ImageMotor_Set);

/**
 * @brief 根据图传电机的角度，换算当前云台角度
 * @param ImageMotor_Now
 * @param Yaw_Now
 * @param ImageGimbal_Now
 */
void ImageMotorNow2GimbalNow(ImageMotor_s *ImageMotor_Now,
                             float Yaw_Now,
                             ImageMotor_s *ImageGimbal_Now);

/**
 * @brief 通过机械臂末端和图传的相对位置，解算图传角度
 * @param Arm_in
 * @param BoomYaw_Now
 * @param Image_in
 * @param Image_out
 */
void ArmImagePos2ImageAngle(VectorXYZ_Str *Arm_in,
                            float BoomYaw_Now,
                            float ForearmYaw_now,
                            float ForearmPitch_now,
                            VectorXYZ_Str *Image_in,
                            ImageMotor_s *Image_out);

#endif
