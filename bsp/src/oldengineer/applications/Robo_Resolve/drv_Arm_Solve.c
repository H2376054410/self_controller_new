/**
 * @file drv_Arm_Solve.c
 * @brief 机械臂姿态解算及扭矩补偿
 * @brief 根据小机械臂的姿态解算大机械臂的姿态
 * @brief 根据当前姿态解算补偿扭矩
 * @brief 特定位置、特定功能
 * @author mylj
 * @version 2.0
 * @date 2023-03-03
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */

#include "drv_Arm_Solve.h"
#include "drv_utils.h"
#include "drv_Vector.h"
#include "string.h"
#include "arm_math.h"

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

/*******************************静态力矩补偿解算*******************************/
/**
 * @brief 大机械臂pitch1轴静态扭矩补偿
 * @brief 输入为大机械臂pitch1当前的角度值
 * @brief 输出为大机械臂pitch1轴静态不动时的补偿扭矩对应的电流设定值
 */
static void BoomPitch1_StaticM2(float rad, float *out)
{
    float Angle_Temp;          // pitch1角度的余角的绝对值  弧度制
    float BoomPitch1_StaticM2; // 电机的补偿扭矩

    Angle_Temp = (0.5f * PI - rad);
    BoomPitch1_StaticM2 = (float)((0.5f * BoomMaster_EquiG1 + BoomSlave2_3_EquiG2 + Forearm_EquiG3) * arm_sin_f32(Angle_Temp) * BoomMaster_P1_L);
    BoomPitch1_StaticM2 = (float)(1.4f * BoomPitch1_M2_Ratio * BoomPitch1_StaticM2); // 2.1

    *out = -BoomPitch1_StaticM2;
}

/**
 * @brief 大机械臂pitch2轴静态扭矩补偿
 * @brief 输入为大机械臂pitch1,pitch2当前的角度值
 * @brief 输出为大机械臂pitch2轴静态不动时的补偿扭矩对应的电流设定值
 */
static void BoomPitch2_StaticM3(float rad, float *out)
{
    float BoomPitch2_StaticM3; // 电机的补偿扭矩

    BoomPitch2_StaticM3 = (0.5f * BoomSlave3_EquiG6 * BoomSLave3_P2_L + Forearm_EquiG3 * (BoomSLave3_P2_L + Boom_Forearm_EquiL2)) * arm_cos_f32(rad);
    BoomPitch2_StaticM3 = (1.35f * BoomPitch2_M3_Ratio * BoomPitch2_StaticM3);
    *out = BoomPitch2_StaticM3;
}

/**
 * @brief 小机械臂pitch轴静态扭矩补偿
 * @param rad
 * @param out
 */
static void ForearmPitch_StaticM5(float rad, float *out)
{
    float ForearmPitch_StaticM5;

    ForearmPitch_StaticM5 = ForearmPitch_EquiL3 * ForearmPitch_EquiG5 * arm_cos_f32(rad);
    ForearmPitch_StaticM5 = (ForearmPitch_M5_Ratio * ForearmPitch_StaticM5);
    *out = ForearmPitch_StaticM5;
}

static float time_now = 0; // 记录当前缓启动的时间
/**
 * @brief  补偿缓启动
 * @param  dt                控制周期 单位：ms
 * @param  period            缓启动的时长 单位：ms
 * @param  out               输出范围：0~1
 * @return int              1:表示缓启动已经完成
 */
static int Comp_SlowStart(int dt, int period, float *out)
{
    if (time_now == period)
    {
        *out = 1;
        return 1;
    }
    else
    {
        time_now += dt;
        *out = (float)time_now / (float)period;
        return 0;
    }
}

/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 * @param   period            缓启动的时长 单位：ms
 * @return  int
 */
int ArmComp_Slow(int period,
                 BoomMotor_s *boom_anglenow,
                 ForearmMotor_s *forearm_anglenow,
                 BoomMotor_s *boom_compout,
                 ForearmMotor_s *forearm_compout)
{
    float ratio;
    float SlowStart_flag;

    BoomMotor_s boom_comptemp;
    ForearmMotor_s forearm_comptemp;

    boom_comptemp.BoomPitch1 = 0;
    boom_comptemp.BoomPitch2 = 0;
    forearm_comptemp.ForearmPitch = 0;

    BoomPitch1_StaticM2(boom_anglenow->BoomPitch1, &boom_comptemp.BoomPitch1);
    BoomPitch2_StaticM3(boom_anglenow->BoomPitch2, &boom_comptemp.BoomPitch2);
    ForearmPitch_StaticM5(forearm_anglenow->ForearmPitch, &forearm_comptemp.ForearmPitch);

    SlowStart_flag = Comp_SlowStart(2, period, &ratio);

    boom_compout->BoomPitch1 = ratio * boom_comptemp.BoomPitch1;
    boom_compout->BoomPitch2 = ratio * boom_comptemp.BoomPitch2;
    forearm_compout->ForearmPitch = ratio * forearm_comptemp.ForearmPitch;

    return SlowStart_flag;
}

/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 */
void ArmComp(BoomMotor_s *boom_anglenow,
             ForearmMotor_s *forearm_anglenow,
             BoomMotor_s *boom_compout,
             ForearmMotor_s *forearm_compout)
{
    BoomPitch1_StaticM2(boom_anglenow->BoomPitch1, &boom_compout->BoomPitch1);
    BoomPitch2_StaticM3(boom_anglenow->BoomPitch2, &boom_compout->BoomPitch2);
    ForearmPitch_StaticM5(forearm_anglenow->ForearmPitch, &forearm_compout->ForearmPitch);
}

/*********************************机械臂相关解算**********************************/

/**
 * @brief 机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量(不考虑BoomYaw旋转时)
 * @param Angle
 * @param out
 */
static void ForearmVector_temp1(ForearmMotor_s *Angle,
                                VectorXYZ_Str *out)
{
    VectorPYM_Str PYM1_Str; // Yaw轴矢量计算
    VectorPYM_Str PYM2_Str; // Pitch轴矢量计算
    VectorXYZ_Str XYZ1_Str; // Yaw轴矢量值
    VectorXYZ_Str XYZ2_Str; // Pitch轴矢量值

    PYM1_Str.yaw = Angle->ForearmYaw;
    PYM1_Str.pitch = 0;
    PYM1_Str.mod = ForearmMaster_Y_L;

    Vector3D_ToXYZ(&PYM1_Str, &XYZ1_Str);
    XYZ1_Str.z = -Forearm_YtoP_L; // z轴单独算

    PYM2_Str.yaw = Angle->ForearmYaw;
    PYM2_Str.pitch = Angle->ForearmPitch;
    PYM2_Str.mod = ForearmMaster_P_L;
    Vector3D_ToXYZ(&PYM2_Str, &XYZ2_Str);

    Vector3D_Add(&XYZ1_Str, &XYZ2_Str, out);
}

/**
 * @brief 机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量(不考虑BoomYaw旋转时,不考虑机械变动)
 * @param Angle
 * @param out
 */
static void ForearmVector_temp2(ForearmMotor_s *Angle,
                                VectorXYZ_Str *out)
{
    VectorPYM_Str PYM2_Str; // Pitch轴矢量计算

    PYM2_Str.yaw = Angle->ForearmYaw;
    PYM2_Str.pitch = Angle->ForearmPitch;
    PYM2_Str.mod = ForearmMaster_P_L;
    Vector3D_ToXYZ(&PYM2_Str, out);
}

/**
 * @brief 机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量（在XY平面上）(不考虑BoomYaw旋转时)
 * @param Angle
 * @param out
 */
static void ForearmVector2D_XY_temp(float ForearmPitch,
                                    VectorXY_Str *out)
{
    VectorRT_Str RT1_Str; // Yaw轴矢量计算
    VectorRT_Str RT2_Str; // Pitch轴矢量计算
    VectorXY_Str XY1_Str; // Yaw轴矢量值
    VectorXY_Str XY2_Str; // Pitch轴矢量值

    RT1_Str.t = 0;
    RT1_Str.r = ForearmMaster_Y_L;

    Vector2D_ToXY(&RT1_Str, &XY1_Str);

    RT2_Str.t = ForearmPitch;
    RT2_Str.r = ForearmMaster_P_L;
    Vector2D_ToXY(&RT2_Str, &XY2_Str);
    XY2_Str.y = 0;

    Vector2D_Add(&XY1_Str, &XY2_Str, out);
}

/**
 * @brief 小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量的模长（二维）
 * @param ForearmPitch
 * @param mod
 */
static void ForearmPos2ForearmVector2D_mod(float ForearmPitch, float *mod)
{
    VectorXY_Str XY_Str;
    ForearmVector2D_XY_temp(ForearmPitch, &XY_Str);
    Vector2D_Mod(&XY_Str, mod);
}

/**
 * @brief 用机械臂的角度解算小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量
 * @param Angle1
 * @param BoomYaw
 * @param out
 */
void ArmAngle2ForearmVector1(ForearmMotor_s *Angle1,
                             float BoomYaw,
                             VectorXYZ_Str *out)
{
    VectorXYZ_Str XYZ_Str;

    ForearmVector_temp1(Angle1, &XYZ_Str);
    Vector3D_Rotate_z(&XYZ_Str,
                      -PI / 2.0f + BoomYaw, out);
}

/**
 * @brief 用机械臂的角度解算小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量（忽略机械z向变动）
 * @param Angle1
 * @param BoomYaw
 * @param out
 */
void ArmAngle2ForearmVector2(ForearmMotor_s *Angle1,
                             float BoomYaw,
                             VectorXYZ_Str *out)
{
    VectorXYZ_Str XYZ_Str;

    ForearmVector_temp2(Angle1, &XYZ_Str);
    Vector3D_Rotate_z(&XYZ_Str,
                      -PI / 2.0f + BoomYaw, out);
}

/**
 * @brief 用机械臂的角度和大臂位姿，解算机械臂小臂末端的位置坐标
 * @param Angle1
 * @param BoomYaw
 * @param BoomPosin
 * @param out
 */
void ArmAngle2ArmPos(ForearmMotor_s *Angle1,
                     float BoomYaw,
                     VectorXYZ_Str *BoomPosin,
                     VectorXYZ_Str *ArmPos_out)
{
    VectorXYZ_Str XYZ_Str;

    ArmAngle2ForearmVector1(Angle1, BoomYaw, &XYZ_Str);
    Vector3D_Add(BoomPosin, &XYZ_Str, ArmPos_out);
}

/**
 * @brief 小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量
 * @param ForearmPos
 * @param Angle
 * @param out
 * @return  1 正常 0：非正常
 */
int ForearmPos2ForearmVector(VectorXYZ_Str *ForearmPos,
                             ForearmMotor_s *Angle,
                             VectorXYZ_Str *out)
{
    float mod1; // 求小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量的模长(二维)
    float mod2; // 整个机械臂的模长(二维)

    float BoomYaw;
    float angle1;              // BoomYaw相对整个臂方向的角度（二维中，非BoomYaw）
    VectorPYM_Str ArmPos_temp; // 大臂的球坐标的矢量方向

    ForearmPos2ForearmVector2D_mod(Angle->ForearmPitch, &mod1);
    Vector3D_ToPYM(ForearmPos, &ArmPos_temp);
    if (Float_ZeroIf(ArmPos_temp.mod, 0.000001f))
    {
        // 防止输入设定坐标为0
        return 0;
    };

    arm_sqrt_f32(ForearmPos->x * ForearmPos->x + ForearmPos->y * ForearmPos->y, &mod2);

    if (Angle->ForearmYaw < PI / 2.0f)
    {
        angle1 = asinf_pro((mod1 / mod2) * arm_sin_f32(PI / 2.0f + Angle->ForearmYaw));
        BoomYaw = angle1 + ArmPos_temp.yaw;
    }
    else if (Angle->ForearmYaw > PI / 2.0f)
    {
        angle1 = -asinf_pro((mod1 / mod2) * arm_sin_f32(PI / 2.0f + Angle->ForearmYaw));
        BoomYaw = ArmPos_temp.yaw - angle1;
    }
    else
    {
        // 99.9999%不进此判断，嘿嘿
        angle1 = 0;
        BoomYaw = ArmPos_temp.yaw;
    }

    ArmAngle2ForearmVector1(Angle, BoomYaw, out);

    return 1;
}

/**
 * @brief 根据Forearm末端位置解算小机械臂Yaw轴的顶部
 * @param ForearmPos
 * @param Angle
 * @param BoomPos
 * @return  1 正常 0：非正常
 */
int Forearm2BoomPos(VectorXYZ_Str *ForearmPos,
                    ForearmMotor_s *Angle,
                    VectorXYZ_Str *BoomPos)
{
    int flag;
    VectorXYZ_Str forearm_temp; // 小臂矢量

    flag = ForearmPos2ForearmVector(ForearmPos, Angle, &forearm_temp);
    Vector3D_Subb(ForearmPos, &forearm_temp, BoomPos);
    return flag;
}

/**
 * @brief 小机械臂Yaw轴的顶部相对于大机械臂始端坐标矢量解算
 * @param Angle
 * @param VectorOut
 */
void BoomAngle2BoomPos(BoomMotor_s *Angle, VectorXYZ_Str *VectorOut)
{
    VectorPYM_Str PYM1_Str; // Yaw轴矢量计算
    VectorPYM_Str PYM2_Str; // Pitch轴矢量计算
    VectorPYM_Str PYM3_Str; //
    VectorXYZ_Str XYZ1_Str; // Yaw轴矢量值
    VectorXYZ_Str XYZ2_Str; // Pitch轴矢量值
    VectorXYZ_Str XYZ3_Str; //

    VectorXYZ_Str XYZTemp_Str; //

    PYM1_Str.yaw = Angle->BoomYaw;
    PYM1_Str.pitch = Angle->BoomPitch1;
    PYM1_Str.mod = BoomMaster_P1_L;

    PYM2_Str.yaw = Angle->BoomYaw;
    PYM2_Str.pitch = Angle->BoomPitch2;
    PYM2_Str.mod = BoomSLave3_P2_L;

    PYM3_Str.yaw = Angle->BoomYaw;
    PYM3_Str.pitch = 0;
    PYM3_Str.mod = BoomToFoerarm_y;

    Vector3D_ToXYZ(&PYM1_Str, &XYZ1_Str);
    Vector3D_ToXYZ(&PYM2_Str, &XYZ2_Str);
    Vector3D_ToXYZ(&PYM3_Str, &XYZ3_Str);

    XYZ3_Str.z = BoomToFoerarm_z;

    Vector3D_Add(&XYZ1_Str, &XYZ2_Str, &XYZTemp_Str);
    Vector3D_Add(&XYZTemp_Str, &XYZ3_Str, VectorOut);
}

/**
 * @brief 根据电机角度和转速，解算小机械臂Yaw轴的顶部三维速度矢量
 * @param Angle
 * @param Speed
 * @param BoomPos
 * @param BoomSpe
 */
void BoomMot2BoomSpe(BoomMotor_s *Angle,
                     BoomMotor_s *Speed,
                     VectorXYZ_Str *BoomPos,
                     VectorXYZ_Str *BoomSpe)
{
    float r;
    VectorXY_Str BoomPos_XOY;
    VectorXY_Str Yaw_Spe;
    VectorXY_Str Pitch1_Spe;
    VectorXY_Str Pitch2_Spe;
    VectorXY_Str PitchSpe_XOZ; // pitch面内的Boom末端二维速度矢量
    VectorXY_Str PitchSpe_XOY; // pitch面内的Boom末端二维速度矢量在XOY面的投影
    VectorRT_Str PitchSpe_RT;  // pitch面内的Boom末端二维速度矢量在XOY面的投影极坐标表示
    VectorXY_Str BoomSpe_XOY;  // Boom末端三维速度矢量在XOY面的投影

    BoomPos_XOY.x = BoomPos->x;
    BoomPos_XOY.y = BoomPos->y;

    // BoomYaw
    Vector2D_Mod(&BoomPos_XOY, &r);

    Yaw_Spe.x = 0;
    Yaw_Spe.y = r * Speed->BoomYaw;

    Vector2D_Rotate(&Yaw_Spe, Angle->BoomYaw, &Yaw_Spe);

    // BoomPitch1
    Pitch1_Spe.x = 0;
    Pitch1_Spe.y = BoomMaster_P1_L * Speed->BoomPitch1;

    Vector2D_Rotate(&Pitch1_Spe, Angle->BoomPitch1, &Pitch1_Spe);

    // BoomPitch2
    Pitch2_Spe.x = 0;
    Pitch2_Spe.y = BoomSLave3_P2_L * Speed->BoomPitch2;

    Vector2D_Rotate(&Pitch2_Spe, Angle->BoomPitch2, &Pitch2_Spe);

    // 在X(Y)OZ面，合成两个pitch的速度矢量
    Vector2D_Add(&Pitch1_Spe, &Pitch2_Spe, &PitchSpe_XOZ);

    PitchSpe_RT.r = PitchSpe_XOZ.x;
    PitchSpe_RT.t = Angle->BoomYaw;
    if (Float_ZeroIf(PitchSpe_RT.r, 0.001f) == 0)
    {
        Vector2D_ToXY(&PitchSpe_RT, &PitchSpe_XOY);
    }
    else
    {
        Vector2D_Init(&PitchSpe_XOY);
    }

    Vector2D_Add(&PitchSpe_XOY, &Yaw_Spe, &BoomSpe_XOY);

    BoomSpe->x = BoomSpe_XOY.x;
    BoomSpe->y = BoomSpe_XOY.y;
    BoomSpe->z = PitchSpe_XOZ.y;
}

/**
 * @brief 根据小机械臂Yaw轴的顶部解算当前Boom电机角度值
 * @param BoomPos
 * @param LastYaw
 * @param Angle_Out
 */
void BoomPos2BoomMotAngle(VectorXYZ_Str *BoomPos,
                          float LastYaw,
                          BoomMotor_s *Angle_Out)
{
    float Angle1; //
    float Angle2; //
    float r_Square;

    VectorXY_Str BoomPos_XOY;
    VectorXY_Str BoomEndPos_XOZ;    // 大机械臂末端在X(Y)OZ平面的坐标
    VectorRT_Str BoomEndPos_XOZ_RT; // 大机械臂末端在X(Y)OZ平面的极坐标
    VectorXY_Str BoomPosTemp;       // 大机械臂末端在X(Y)OZ平面的坐标绕Yaw转-Yaw后的坐标

    BoomPos2BoomYawangle(BoomPos, LastYaw, &Angle_Out->BoomYaw); // 计算BoomYaw的角度

    // if (BoomPos_XOY.y >= 0)
    // {
    BoomPos_XOY.x = BoomPos->x;
    BoomPos_XOY.y = BoomPos->y;
    // }
    // else
    // {
    //     // 使yaw会旋转180度
    //     BoomPos_XOY.x = -BoomPos->x;
    //     BoomPos_XOY.y = -BoomPos->y;
    // }

    Vector2D_Rotate(&BoomPos_XOY, -Angle_Out->BoomYaw, &BoomPosTemp);

    BoomEndPos_XOZ.x = BoomPosTemp.x - BoomToFoerarm_y;
    BoomEndPos_XOZ.y = BoomPos->z - BoomToFoerarm_z;

    Vector2D_ToRT(&BoomEndPos_XOZ, &BoomEndPos_XOZ_RT);

    r_Square = SQUARE(BoomEndPos_XOZ_RT.r);

    Angle1 = acosf_pro((BoomMaster_P1_L_Square + r_Square - BoomSLave3_P2_L_Square) /
                       (2 * BoomMaster_P1_L * BoomEndPos_XOZ_RT.r));

    Angle2 = acosf_pro((BoomMaster_P1_L_Square - r_Square + BoomSLave3_P2_L_Square) /
                       (2 * BoomMaster_P1_L * BoomSLave3_P2_L));

    Angle_Out->BoomPitch1 = BoomEndPos_XOZ_RT.t + Angle1;
    Angle_Out->BoomPitch2 = Angle2 + Angle_Out->BoomPitch1 - PI;
}

/**
 * @brief 根据小机械臂Yaw轴的顶部解算当前Boom电机Yaw角度值
 * @param BoomPos
 * @param LastYaw
 * @param Angle_Out
 */
void BoomPos2BoomYawangle(VectorXYZ_Str *BoomPos,
                          float LastYaw,
                          float *Angle_Out)
{
    float BoomPos_XOY_mod;
    VectorXY_Str BoomPos_XOY;
    VectorRT_Str BoomPos_XOY_RT;

    BoomPos_XOY.x = BoomPos->x;
    BoomPos_XOY.y = BoomPos->y;
    // 规划输出位置在XOY面上的投影
    Vector2D_Mod(&BoomPos_XOY, &BoomPos_XOY_mod);
    // if (BoomPos_XOY.y >= 0)
    {
        if (Float_ZeroIf(BoomPos_XOY_mod, 0.03f) == 0)
        {
            // 投影非0
            Vector2D_ToRT(&BoomPos_XOY, &BoomPos_XOY_RT);
            *Angle_Out = BoomPos_XOY_RT.t;
        }
        else
        {
            *Angle_Out = LastYaw;
        }
    }
    // else
    // {
    //     // 设定值规划位置输出过零点，按照上面解算，则会出现多解，且yaw会旋转180度
    //     if (Float_ZeroIf(BoomPos_XOY_mod, 0.03f) == 0)
    //     {
    //         BoomPos_XOY.x = -BoomPos_XOY.x;
    //         BoomPos_XOY.y = -BoomPos_XOY.y;
    //         // 投影非0
    //         Vector2D_ToRT(&BoomPos_XOY, &BoomPos_XOY_RT);
    //         *Angle_Out = BoomPos_XOY_RT.t;
    //     }
    //     else
    //     {
    //         *Angle_Out = LastYaw;
    //     }
    // }
}
/**
 * @brief 由Boom末端速度求解Boom电机的速度值
 */
void BoomSpe2BoomMotSpe(VectorXYZ_Str *BoomPos,
                        BoomMotor_s *BoomAngle,
                        VectorXYZ_Str *BoomSpe,
                        BoomMotor_s *Spe_Out)
{
    float BoomPos_XOY_mod;
    VectorXY_Str BoomPos_XOY;
    VectorXY_Str BoomSpe_XOY;      // 大机械臂末端速度在XOY平面的速度矢量
    VectorXY_Str BoomSpe_XOY_Temp; // 大机械臂末端速度在XOY平面的速度绕Yaw轴转-Yaw后的速度矢量

    VectorXY_Str BoomSpe_XOZ; //  大机械臂末端速度矢量在XOZ面的速度矢量

    VectorXY_Str Pitch1Spe_unit;   // pitch1的单位速度矢量
    VectorXY_Str Pitch2Spe_unit;   // pitch1的单位速度矢量
    VectorXY_Str PitchSpeOut_Temp; // pitch面的输出速度矢量

    // 位置在XOY面的投影
    BoomPos_XOY.x = BoomPos->x;
    BoomPos_XOY.y = BoomPos->y;
    // 速度在XOY面的投影
    BoomSpe_XOY.x = BoomSpe->x;
    BoomSpe_XOY.y = BoomSpe->y;

    Vector2D_Rotate(&BoomSpe_XOY, -BoomAngle->BoomYaw, &BoomSpe_XOY_Temp);

    // BoomYaw
    Vector2D_Mod(&BoomPos_XOY, &BoomPos_XOY_mod);
    if (Float_ZeroIf(BoomPos_XOY_mod, 0.025f))
    {

        Spe_Out->BoomYaw = 0;
    }
    else
    {
        Spe_Out->BoomYaw = BoomSpe_XOY_Temp.y / BoomPos_XOY_mod;
    }
    // BoomPitch1&&BoomPitch2
    BoomSpe_XOZ.x = BoomSpe_XOY_Temp.x;
    BoomSpe_XOZ.y = BoomSpe->z;

    // BoomPitch1
    Pitch1Spe_unit.x = 1;
    Pitch1Spe_unit.y = 0;
    // 求解单位方向向量
    Vector2D_Rotate(&Pitch1Spe_unit, BoomAngle->BoomPitch1, &Pitch1Spe_unit);

    // BoomPitch2
    Pitch2Spe_unit.x = 1;
    Pitch2Spe_unit.y = 0;
    // 求解单位方向向量
    Vector2D_Rotate(&Pitch2Spe_unit, BoomAngle->BoomPitch2, &Pitch2Spe_unit);

    if (Float_ZeroIf(BoomAngle->BoomPitch1 - BoomAngle->BoomPitch2, 0.01f))
    {
        // 现实由于机械干涉，是达不到的，仅是做了而已，同时也不需要
        Vector2D_Rotate(&Pitch1Spe_unit, PI / 2, &Pitch2Spe_unit); // 捏造一个垂直基底
        Vector2D_Resolve(&BoomSpe_XOZ, &Pitch1Spe_unit,
                         &Pitch2Spe_unit, &PitchSpeOut_Temp);

        Spe_Out->BoomPitch1 = PitchSpeOut_Temp.x / (2 * BoomMaster_P1_L);
        Spe_Out->BoomPitch2 = PitchSpeOut_Temp.x / (2 * BoomSLave3_P2_L);
    }
    else
    {
        Vector2D_Resolve(&BoomSpe_XOZ, &Pitch1Spe_unit,
                         &Pitch2Spe_unit, &PitchSpeOut_Temp);

        Spe_Out->BoomPitch1 = PitchSpeOut_Temp.x / BoomMaster_P1_L;
        Spe_Out->BoomPitch2 = PitchSpeOut_Temp.y / BoomSLave3_P2_L;
    }
}

/**
 * @brief 机械限幅判断函数
 * @brief 根据大机械期望角度判断是否超限
 * @brief 1 则表示超限 0 则表示没有超限
 */
static int ArmPosMachinelimit_If(BoomMotor_s *BoomAngle)
{
    int flag = 0;
    float DeltaAngle = 0; // 大机械臂中，大臂和小臂的夹角

    flag = 0;
    // 判断各电机位置是否到机械限幅
    flag = LimitIf(BoomAngle->BoomYaw,
                   BoomYaw_LimitMax, BoomYaw_LimitMin);
    flag += LimitIf(BoomAngle->BoomPitch1,
                    BoomPitch1_LimitMax, BoomPitch1_LimitMin);
    flag += LimitIf(BoomAngle->BoomPitch2,
                    BoomPitch2_LimitMax, BoomPitch2_LimitMin);

    DeltaAngle = PI - BoomAngle->BoomPitch1 + BoomAngle->BoomPitch2;

    flag += LimitIf(DeltaAngle,
                    DeltaAngle_LimitMax, DeltaAngle_Limit1Min); // 对最小值不做限制

    if ((BoomPitch1_Critical1 >= BoomAngle->BoomPitch1) &&
        (BoomPitch1_Critical2 < BoomAngle->BoomPitch1))
    {
        flag += LimitIf(DeltaAngle,
                        DeltaAngle_LimitMax, DeltaAngle_Limit1Min); // 对最小值不做限制
    }
    else if (BoomPitch1_Critical2 > BoomAngle->BoomPitch1)
    {
        flag += LimitIf(DeltaAngle,
                        DeltaAngle_LimitMax, DeltaAngle_Limit2Min); // 对最小值不做限制
    }

    if (flag > 0)
    {
        flag = 0;
        return 1;
    }
    else
    {
        flag = 0;
        return 0;
    }
}

/**
 * @brief BoomPos设定值逼近算法
 * @param BoomPos_Set_in
 * @param BoomPos_SetOld
 * @param BoomYaw
 * @param BoomPos_Set_out
 */
int BoomPos_Limit(VectorXYZ_Str *BoomPos_Set_in,
                  VectorXYZ_Str *BoomPos_SetOld,
                  float BoomYaw,
                  VectorXYZ_Str *BoomPos_Set_out)
{
    float BoomSet_Limited_Mod;           // 逼近之后的模长
    float BoomSet_Add_mod;               // 设定值增量模长
    float BoomSet_Origin2Set_mod;        // 设定值增量模长
    BoomMotor_s BoomAngSet_temp;         // 大臂设定角度,用于逼近后解算判断
    VectorXYZ_Str BoomSet_Add;           // 设定值增量矢量
    VectorXYZ_Str BoomSet_Origin;        // 设定值坐标原点
    VectorXYZ_Str BoomSet_Origin2Set;    // 设定值坐标原点到设定点矢量
    VectorXYZ_Str BoomSet_temp1;         // 回退两个步长后的设定值
    VectorXYZ_Str BoomSetLimit_now;      // 本次逼近后的值
    VectorXYZ_Str BoomSetLimit_nowfromO; // 本次逼近后的值
    VectorXYZ_Str BoomSetLimit_last;     // 上一次逼近的值

    // 以初始状态点为目标逼近
    BoomSet_Origin.x = APPROACHING_ORIGIN_X;
    BoomSet_Origin.y = APPROACHING_ORIGIN_Y;
    BoomSet_Origin.z = APPROACHING_ORIGIN_Z;

    Vector3D_Subb(BoomPos_Set_in,
                  &BoomSet_Origin,
                  &BoomSet_Origin2Set); // 解算初始点到目标点矢量

    Vector3D_Subb(BoomPos_Set_in,
                  BoomPos_SetOld,
                  &BoomSet_Add); // 求增量

    Vector3D_Mod(&BoomSet_Add, &BoomSet_Add_mod);               // 若超限，add必然非0
    Vector3D_Mod(&BoomSet_Origin2Set, &BoomSet_Origin2Set_mod); // 根据机械结构，此值非零

    BoomSet_Limited_Mod = BoomSet_Origin2Set_mod - 2 * BoomSet_Add_mod; // 计算回退2个步长后的值

    if (BoomSet_Limited_Mod <= 0)
    {
        // 无需逼近
        // 直接输出
        Vector3D_Transmit(BoomPos_SetOld, BoomPos_Set_out); // 报警
        // 报警
        return 1;
    }
    Vector3D_SetSize(&BoomSet_Origin2Set, BoomSet_Limited_Mod, &BoomSet_temp1); // 计算回退两个步长后的设定值

    Vector3D_Dichotomy(&BoomSet_temp1, &BoomSet_Origin2Set, &BoomSetLimit_now); // 初始逼近

    Vector3D_Transmit(&BoomSet_temp1, &BoomSetLimit_last);
    for (int i = 0; i < IFLIMIT_NUM_MAX; i++)
    {
        // 解算判断
        Vector3D_Add(&BoomSetLimit_now,
                     &BoomSet_Origin,
                     &BoomSetLimit_nowfromO);
        BoomPos2BoomMotAngle(&BoomSetLimit_nowfromO,
                             BoomYaw,
                             &BoomAngSet_temp);
        if (ArmPosMachinelimit_If(&BoomAngSet_temp))
        {
            // 超限
            break;
        }
        else
        {
            // 不超限
            Vector3D_Transmit(&BoomSetLimit_now, &BoomSetLimit_last);
            Vector3D_Dichotomy(&BoomSetLimit_last, &BoomSet_Origin2Set, &BoomSetLimit_now); // 逼近
        }
    }
    Vector3D_Add(&BoomSetLimit_last,
                 &BoomSet_Origin,
                 BoomPos_Set_out);
    return 0;
}

/**
 * @brief 机械限幅判断函数
 * @brief 根据机械臂角度判断是否超限
 * @param BoomAngle
 * @param ForearmAngle
 * @param Arm_Limit_data
 * @return int 1 则表示超限 0 则表示没有超限
 */
int ArmMachinelimit_If(BoomMotor_s *BoomAngle,
                       ForearmMotor_s *ForearmAngle,
                       Arm_Limit_s *Arm_Limit_data)
{
    float DeltaAngle = 0; // 大机械臂中，大臂和小臂的夹角

    // 判断各电机位置是否到机械限幅
    Arm_Limit_data->BoomYaw_Iflimit = LimitIf(BoomAngle->BoomYaw,
                                              BoomYaw_LimitMax, BoomYaw_LimitMin);
    Arm_Limit_data->BoomPitch1_Iflimit = LimitIf(BoomAngle->BoomPitch1,
                                                 BoomPitch1_LimitMax, BoomPitch1_LimitMin);
    Arm_Limit_data->BoomPitch2_Iflimit = LimitIf(BoomAngle->BoomPitch2,
                                                 BoomPitch2_LimitMax, BoomPitch2_LimitMin);

    Arm_Limit_data->ForearmYaw_Iflimit = LimitIf(ForearmAngle->ForearmYaw,
                                                 FOREARMYAW_LIMITMAX, FOREARMYAW_LIMITMIN);
    Arm_Limit_data->ForearmPitch_Iflimit = LimitIf(ForearmAngle->ForearmPitch,
                                                   FOREARMPITCH_LIMITMAX, FOREARMPITCH_LIMITMIN);
    Arm_Limit_data->ForearmRoll_Iflimit = LimitIf(ForearmAngle->ForearmRoll,
                                                  FOREARMROLL_LIMITMAX, FOREARMROLL_LIMITMIN);

    DeltaAngle = PI - BoomAngle->BoomPitch1 + BoomAngle->BoomPitch2;

    Arm_Limit_data->BoomPitch_Iflimit = LimitIf(DeltaAngle,
                                                DeltaAngle_LimitMax, DeltaAngle_Limit1Min); // 对最小值不做限制

    if ((BoomPitch1_Critical1 >= BoomAngle->BoomPitch1) &&
        (BoomPitch1_Critical2 < BoomAngle->BoomPitch1))
    {
        Arm_Limit_data->BoomPitch_Iflimit = LimitIf(DeltaAngle,
                                                    DeltaAngle_LimitMax, DeltaAngle_Limit1Min); // 对最小值不做限制
    }
    else if (BoomPitch1_Critical2 > BoomAngle->BoomPitch1)
    {
        Arm_Limit_data->BoomPitch_Iflimit = LimitIf(DeltaAngle,
                                                    DeltaAngle_LimitMax, DeltaAngle_Limit2Min); // 对最小值不做限制
    }

    rt_uint8_t flag;
    flag = Arm_Limit_data->BoomYaw_Iflimit |
           Arm_Limit_data->BoomPitch1_Iflimit |
           Arm_Limit_data->BoomPitch2_Iflimit |
           Arm_Limit_data->ForearmYaw_Iflimit |
           Arm_Limit_data->ForearmPitch_Iflimit |
           Arm_Limit_data->ForearmRoll_Iflimit |
           Arm_Limit_data->BoomPitch_Iflimit;
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
 * @brief 通过大机械臂角度解算图传位置坐标
 * @param Angle
 * @param Imagepos_out
 */
void ImagePos_Resolve(BoomMotor_s *Angle,
                      VectorXYZ_Str *Imagepos_out)
{
    VectorPYM_Str PYM1_Str; // Yaw轴矢量计算
    VectorPYM_Str PYM2_Str; // Pitch轴矢量计算
    VectorPYM_Str PYM3_Str; // 图传横杆矢量计算
    VectorPYM_Str PYM4_Str; // 图传竖直矢量计算

    VectorXYZ_Str XYZ1_Str; // Yaw轴矢量值
    VectorXYZ_Str XYZ2_Str; // Pitch轴矢量值
    VectorXYZ_Str XYZ3_Str; // 图传横杆矢量计算
    VectorXYZ_Str XYZ4_Str; // 图传竖直矢量计算

    VectorXYZ_Str XYZTemp_Str; // 用于矢量加减的中间量

    PYM1_Str.yaw = Angle->BoomYaw;
    PYM1_Str.pitch = Angle->BoomPitch1;
    PYM1_Str.mod = BoomMaster_P1_L;

    PYM2_Str.yaw = Angle->BoomYaw;
    PYM2_Str.pitch = Angle->BoomPitch2;
    PYM2_Str.mod = BOOMSLAVE2IMAGE_L;

    PYM3_Str.yaw = Angle->BoomYaw - PI / 2.0f;
    PYM3_Str.pitch = 0;
    PYM3_Str.mod = IMAGE_LEVEL_L;

    PYM4_Str.yaw = 0;
    PYM4_Str.pitch = -PI / 2.0f;
    PYM4_Str.mod = IMAGE_VERTIUCAL_L;

    Vector3D_ToXYZ(&PYM1_Str, &XYZ1_Str);
    Vector3D_ToXYZ(&PYM2_Str, &XYZ2_Str);
    Vector3D_ToXYZ(&PYM3_Str, &XYZ3_Str);
    Vector3D_ToXYZ(&PYM4_Str, &XYZ4_Str);

    Vector3D_Add(&XYZ1_Str, &XYZ2_Str, &XYZTemp_Str);
    Vector3D_Add(&XYZTemp_Str, &XYZ3_Str, &XYZTemp_Str);
    Vector3D_Add(&XYZTemp_Str, &XYZ4_Str, Imagepos_out);
}
