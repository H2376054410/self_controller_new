/**
 * @file drv_Image_Solve.c
 * @author mylj
 * @version 1.0
 * @date 2023-07-17
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_Image_Solve.h"
#include "drv_utils.h"
#include "drv_Vector.h"

/**
 * @brief 限幅判断函数（只用于判断，不修改值）
 * @param in
 * @param max
 * @param min
 * @return int  1：超限 0：没有超限
 */
static int LimitIf(float in, float max, float min)
{
    if (in > max || in < min)
        return 1;
    else
        return 0;
}

/**
 * @brief  机械限幅判断函数
 * @brief  根据图传电机角度判断是否超限
 * @param  ImageAngle
 * @param  Image_Limit_data
 * @return int 1 则表示超限 0 则表示没有超限
 */
int ImageMachinelimit_If(ImageMotor_s *ImageAngle,
                         Image_Limit_s *Image_Limit_data)
{
    rt_uint8_t flag = 0;
    // 判断各电机位置是否到机械限幅
    Image_Limit_data->ImageYaw_Iflimit = LimitIf(ImageAngle->ImageYaw,
                                                 IMAGEYAW_ANGLEMAX, IMAGEYAW_ANGLEMIN);
    Image_Limit_data->ImagePitch_Iflimit = LimitIf(ImageAngle->ImagePitch,
                                                   IMAGEPITCH_ANGLEMAX, IMAGEPITCH_ANGLEMIN);
    flag = Image_Limit_data->ImageYaw_Iflimit |
           Image_Limit_data->ImagePitch_Iflimit;
    if (flag > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 图传云台设定值换算为电机角度设定值
 * @param ImageGinbal_Set
 * @param Yaw_Init          yaw的初始角度
 * @param ImageMotor_Set
 */
void ImageGimbalSet2MotorSet(ImageMotor_s *ImageGimbal_Set,
                             float Yaw_Init,
                             ImageMotor_s *ImageMotor_Set)
{
    ImageMotor_Set->ImageYaw = 0.5f * (ImageGimbal_Set->ImageYaw +
                                       2 * ImageGimbal_Set->ImagePitch -
                                       Yaw_Init);

    ImageMotor_Set->ImagePitch = 0.5f * (2 * ImageGimbal_Set->ImagePitch -
                                         ImageGimbal_Set->ImageYaw +
                                         Yaw_Init);
}

/**
 * @brief 根据图传电机的角度，换算当前云台角度
 * @param ImageMotor_Now
 * @param Yaw_Now
 * @param ImageGimbal_Now
 */
void ImageMotorNow2GimbalNow(ImageMotor_s *ImageMotor_Now,
                             float Yaw_Now,
                             ImageMotor_s *ImageGimbal_Now)
{
    ImageGimbal_Now->ImageYaw = Yaw_Now;
    ImageGimbal_Now->ImagePitch = 0.5f * (ImageMotor_Now->ImageYaw + ImageMotor_Now->ImagePitch);
}

/**
 * @brief 通过机械臂末端和图传的相对位置，解算图传角度
 * @param Arm_in
 * @param BoomYaw_Now
 * @param ForearmYaw_now
 * @param ForearmPitch_now
 * @param Image_in
 * @param Image_out
 */
void ArmImagePos2ImageAngle(VectorXYZ_Str *Arm_in,
                            float BoomYaw_Now,
                            float ForearmYaw_now,
                            float ForearmPitch_now,
                            VectorXYZ_Str *Image_in,
                            ImageMotor_s *Image_out)
{
    VectorXYZ_Str Ore_Pos_xyz;
    VectorXYZ_Str Oretemp_Pos_xyz; // 矿石的坐标点(笛卡尔坐标系)
    VectorPYM_Str Oretemp_Pos_pym; // 矿石的坐标点（球坐标）
    VectorXYZ_Str Delta_Pos_xyz;
    VectorPYM_Str Delta_Pos_pym;

    Oretemp_Pos_pym.mod = 0.1f;
    Oretemp_Pos_pym.pitch = ForearmPitch_now;
    Oretemp_Pos_pym.yaw = ForearmYaw_now + BoomYaw_Now - PI / 2.0f;

    Vector3D_ToXYZ(&Oretemp_Pos_pym, &Oretemp_Pos_xyz);
    Vector3D_Add(&Oretemp_Pos_xyz, Arm_in, &Ore_Pos_xyz);

    Vector3D_Subb(&Ore_Pos_xyz, Image_in, &Delta_Pos_xyz);

    Vector3D_ToPYM(&Delta_Pos_xyz, &Delta_Pos_pym);

    Image_out->ImageYaw = Delta_Pos_pym.yaw - BoomYaw_Now + PI / 2.0f-0.4f;
    Image_out->ImagePitch = Delta_Pos_pym.pitch-0.1f;
}
