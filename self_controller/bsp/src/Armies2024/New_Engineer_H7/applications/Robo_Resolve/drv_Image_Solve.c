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
    Image_Limit_data->ImageHigh_Iflimit = LimitIf(ImageAngle->ImageHigh,
                                                   IMAGEHIGH_ANGLEMAX, IMAGEHIGH_ANGLEMIN);                                               
    flag = Image_Limit_data->ImageYaw_Iflimit |
           Image_Limit_data->ImagePitch_Iflimit|Image_Limit_data->ImageHigh_Iflimit;
    if (flag > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/*
 * @brief  图传相机超限后处理
 * @param  ImageAngle
*/
void Imageoutlimit(ImageMotor_s *ImageAngle,ImageMotor_s *ImageAngle_old)
{
    if (ImageAngle->ImagePitch > IMAGEPITCH_ANGLEMAX||ImageAngle->ImagePitch < IMAGEPITCH_ANGLEMIN)
        ImageAngle->ImagePitch = ImageAngle_old->ImagePitch;
    else
        ImageAngle_old->ImagePitch=ImageAngle->ImagePitch;

    if(ImageAngle->ImageYaw > IMAGEYAW_ANGLEMAX||ImageAngle->ImageYaw < IMAGEYAW_ANGLEMIN)
        ImageAngle->ImageYaw=ImageAngle_old->ImageYaw;
    else
        ImageAngle_old->ImageYaw=ImageAngle->ImageYaw;

    if(ImageAngle->ImageHigh > IMAGEHIGH_ANGLEMAX)
        ImageAngle->ImageHigh=IMAGEHIGH_ANGLEMAX;
    else if(ImageAngle->ImageHigh < IMAGEHIGH_ANGLEMIN)
        ImageAngle->ImageHigh=IMAGEHIGH_ANGLEMIN; 
    else 
        ImageAngle_old->ImageHigh=ImageAngle->ImageHigh;

}

/*
 * @brief  图传相机超限后处理
 * @param  ImageAngle
*/
void ImageNolimit(ImageMotor_s *ImageAngle,ImageMotor_s *ImageAngle_old)
{
    ImageAngle_old->ImagePitch=ImageAngle->ImagePitch;
    ImageAngle_old->ImageHigh=ImageAngle->ImageHigh;
    ImageAngle_old->ImageYaw=ImageAngle->ImageYaw;
}

// /**
//  * @brief 根据图传电机的角度，换算当前云台角度
//  * @param ImageMotor_Now
//  * @param Yaw_Now
//  * @param ImageGimbal_Now
//  */
// void ImageMotorNow2GimbalNow(ImageMotor_s *ImageMotor_Now,float z,
                             
//                              ImageMotor_s *ImageGimbal_Now)
// {   ImageGimbal_Now->ImageHigh=z; 
//     ImageGimbal_Now->ImageYaw = ForearmStateData->yaw;
//     ImageGimbal_Now->ImagePitch =  ForearmStateData->ImagePitch;
// }

// /**
//  * @brief 通过机械臂末端和图传的相对位置，解算图传角度
//  * @param Arm_in
//  * @param BoomYaw_Now
//  * @param ForearmYaw_now
//  * @param ForearmPitch_now
//  * @param Image_in
//  * @param Image_out
//  */
// void ArmImagePos2ImageAngle(VectorXYZ_Str *Arm_in,
//                             float BoomYaw_Now,
//                             float ForearmYaw_now,
//                             float ForearmPitch_now,
//                             VectorXYZ_Str *Image_in,
//                             ImageMotor_s *Image_out)
// {
//     VectorXYZ_Str Ore_Pos_xyz;
//     VectorXYZ_Str Oretemp_Pos_xyz; // 矿石的坐标点(笛卡尔坐标系)
//     VectorPYM_Str Oretemp_Pos_pym; // 矿石的坐标点（球坐标）
//     VectorXYZ_Str Delta_Pos_xyz;
//     VectorPYM_Str Delta_Pos_pym;

//     Oretemp_Pos_pym.mod = 0.1f;
//     Oretemp_Pos_pym.pitch = ForearmPitch_now;
//     Oretemp_Pos_pym.yaw = ForearmYaw_now + BoomYaw_Now - PI / 2.0f;

//     Vector3D_ToXYZ(&Oretemp_Pos_pym, &Oretemp_Pos_xyz);
//     Vector3D_Add(&Oretemp_Pos_xyz, Arm_in, &Ore_Pos_xyz);

//     Vector3D_Subb(&Ore_Pos_xyz, Image_in, &Delta_Pos_xyz);

//     Vector3D_ToPYM(&Delta_Pos_xyz, &Delta_Pos_pym);

//     Image_out->ImageYaw = Delta_Pos_pym.yaw - BoomYaw_Now + PI / 2.0f-0.4f;
//     Image_out->ImagePitch = Delta_Pos_pym.pitch-0.1f;
// }
