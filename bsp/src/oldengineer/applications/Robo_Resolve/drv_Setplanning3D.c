/**
 * @file drv_Setplanning3D.c
 * @brief 三维设定值规划
 * @author ych
 * @author mylj
 * @version 2.0
 * @date 2023-03-03
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_Setplanning3D.h"
#include "drv_utils.h"
#include "arm_math.h"

/**
 * @brief 三维设定值规划初始化
 * @param Settings          设定数据结构体
 * @param Pos_ErrorMax      输出位置设定值对应的最大Error
 * @param Pos_Tolerance     位置容差，用于判断期望位置矢量是否是0向量
 * @param Spe_Tolerance     速度容差，用于判断当前速度矢量是否是0向量
 * @param Speed_Max         调整过程最大速度
 * @param Accl_Max          最大加速度
 * @param Period            计算周期 单位s
 */
void PlanSettings3D_Init(SetPlanning3D_Str *Str,
                         float Pos_ErrorMax,
                         float Pos_Tolerance,
                         float Spe_Tolerance,
                         float SpeedMax_R,
                         float SpeedMax_T,
                         float AcclMax_R,
                         float AcclMax_T,
                         float Period)
{

    Str->Settings.Pos_ErrorMax = Pos_ErrorMax;
    Str->Settings.Pos_Tolerance = Pos_Tolerance;
    Str->Settings.Spe_Tolerance = Spe_Tolerance;

    Str->R_Str.Settings.Accl_Max = AcclMax_R;
    Str->R_Str.Settings.Speed_Max = SpeedMax_R;
    Str->R_Str.Settings.dt = Period;

    Str->T_Str.Settings.Accl_Max = AcclMax_T;
    Str->T_Str.Settings.Speed_Max = SpeedMax_T;
    Str->T_Str.Settings.dt = Period;

    Vector3D_Init(&Str->Input.Now.Pos);
    Vector3D_Init(&Str->Input.Now.Spe);

    Vector3D_Init(&Str->Input.Set.Pos);
    Vector3D_Init(&Str->Input.Set.Spe);

    Vector3D_Init(&Str->Output.Pos);
    Vector3D_Init(&Str->Output.Spe);

    SetPlanning_Init(&Str->R_Str, &Str->R_Str.Settings);
    SetPlanning_Init(&Str->T_Str, &Str->T_Str.Settings);
}

/**
 * @brief 三维路径规划设定值
 * @param Str
 * @param pos
 * @param spe
 */
void PlanSettings3D_Set(SetPlanning3D_Str *Str,
                        VectorXYZ_Str *pos,
                        VectorXYZ_Str *spe)
{
    Vector3D_Transmit(pos, &Str->Input.Set.Pos);
    Vector3D_Transmit(spe, &Str->Input.Set.Spe);
}

/**
 * @brief 三维设定值规划径向速度限幅
 * @param Str
 * @param SpeedMax_R
 */
void Plan3D_SpeedR_Max_Set(SetPlanning3D_Str *Str,
                           float SpeedMax_R)
{
    Str->R_Str.Settings.Speed_Max = SpeedMax_R;
}

/**
 * @brief 三维设定值规划径向加速度限幅
 * @param Str
 * @param SpeedMax_R
 */
void Plan3D_AcclR_Max_Set(SetPlanning3D_Str *Str,
                          float AcclMax_R)
{
    Str->R_Str.Settings.Accl_Max = AcclMax_R;
}

/**
 * @brief 三维设定值规划
 * @brief 传参为国际单位
 */
void SetPlanning3D_Cal(SetPlanning3D_Str *Str)
{
    // 目标位移方向和当前速度方向为设定值规划的根据，即是参考系，是根据这两个向量确定的分解方向和分解平面
    // 如果二者其一为零向量的话，则需特殊处理
    // 如果设定位置与当前位置只差为0看，那么就不需要进行设定值规划
    // 如果当前速度为0，则切向速度和位置直接设为0即可,无需进行切向设定值规划
    // 如果当前速度和目标位移方向相同,则切向速度和位置直接设为0即可,无需进行切向设定值规划

	
	  //更新当前的三维设定值规划数据
    // 计算上次输出位置与实际位置的差
    Vector3D_Subb(&Str->Output.Pos, &Str->Input.Now.Pos, &Str->Temp.DeltaPos1);
    // 计算差值的模长
    Vector3D_Mod(&Str->Temp.DeltaPos1, &Str->Temp.DeltaPos1_mod);

    if (Str->Temp.DeltaPos1_mod > Str->Settings.Pos_ErrorMax)
    {
        // 如果差值的模长大于阈值，则对上一次的输出位置进行修正
        Vector3D_SetSize(&Str->Temp.DeltaPos1, Str->Settings.Pos_ErrorMax, &Str->Temp.DeltaPos_Fixed);
        Vector3D_Add(&Str->Input.Now.Pos, &Str->Temp.DeltaPos_Fixed, &Str->Output.Pos);
    }

    // 计算目标位置与修正后上次输出值的差值
    Vector3D_Subb(&Str->Input.Set.Pos, &Str->Output.Pos, &Str->Temp.DeltaPos2);

    Vector3D_Mod(&Str->Temp.DeltaPos2, &Str->Temp.DeltaPos2_mod);
    Vector3D_Mod(&Str->Output.Spe, &Str->Temp.OutSpe_mod);

    Str->IfZeroflag.DeltaPos2 = Float_ZeroIf(Str->Temp.DeltaPos2_mod, Str->Settings.Pos_Tolerance);
    Str->IfZeroflag.OutSpe = Float_ZeroIf(Str->Temp.OutSpe_mod, Str->Settings.Spe_Tolerance);

    if (Str->IfZeroflag.DeltaPos2 == 0 && Str->IfZeroflag.OutSpe == 0)
    {
        // 设定位置变化和当前速度都是非0矢量
        // 沿目标位移的切向和径向，分解当前速度和目标位移
        Vector3D_DAngle(&Str->Temp.DeltaPos2, &Str->Output.Spe, &Str->Temp.Angle1);
        Str->Temp.SetPos.x = Str->Temp.DeltaPos2_mod;
        Str->Temp.NowSpe.x = Str->Temp.OutSpe_mod * arm_cos_f32(Str->Temp.Angle1); // 当目标位移方向和当前速度方向垂直时,此值为0
        Str->Temp.NowSpe.y = Str->Temp.OutSpe_mod * arm_sin_f32(Str->Temp.Angle1); // 当目标位移方向和当前速度方向平行时,此值为0 (无需处理)

        /* 运用矢量加减求解切向矢量*/
        //  计算上一次规划输出速度的径向分量三维矢量
        Vector3D_SetSize(&Str->Temp.DeltaPos2, Str->Temp.NowSpe.x, &Str->Temp.NowSpe_R_3D); // 径向方向速度向量
        // 计算上一次规划输出速度的切向分量三维矢量
        Vector3D_Subb(&Str->Output.Spe, &Str->Temp.NowSpe_R_3D, &Str->Temp.NowSpe_T_3D); // 切向方向速度向量

        // 径向设定值规划当前参数设定

        Str->R_Str.Output.spe = Str->Temp.NowSpe.x;
        Str->R_Str.Output.pos = 0;

        Str->R_Str.Input.Now.spe = Str->Temp.NowSpe.x;
        Str->R_Str.Input.Now.pos = 0;

        Str->R_Str.Input.Set.spe = 0;
        Str->R_Str.Input.Set.pos = Str->Temp.SetPos.x;

        // 切向设定值规划当前参数设定
        Str->T_Str.Output.spe = Str->Temp.NowSpe.y;
        Str->T_Str.Output.pos = 0;

        Str->T_Str.Input.Now.spe = Str->Temp.NowSpe.y;
        Str->T_Str.Input.Now.pos = 0;

        Str->T_Str.Input.Set.pos = 0;
        Str->T_Str.Input.Set.spe = 0;

        SetPlanning_Cal(&Str->R_Str);
        SetPlanning_Cal(&Str->T_Str);

        // 目标位移的方向就是径向方向，当输入径向期望位移不是0，则径向输出位移一定不是0，相应的径向输出速度也不是0
        // 故不需要进行特殊处理

        //  径向
        Vector3D_SetSize(&Str->Temp.DeltaPos2, Str->R_Str.Output.spe, &Str->Temp.OutSpe_R_3D);
        Vector3D_SetSize(&Str->Temp.DeltaPos2, Str->R_Str.Output.pos, &Str->Temp.OutPos_R_3D);

        // 判断切向速度是否为0，若是0，则不进行切向规划
        Vector3D_Mod(&Str->Temp.NowSpe_T_3D, &Str->Temp.NowSpe_T_3D_mod);
        Str->IfZeroflag.NowSpe_T_3D = Float_ZeroIf(Str->Temp.NowSpe_T_3D_mod, 0.000001f);
        // 切向
        if (Str->IfZeroflag.NowSpe_T_3D == 0)
        {
            // 切向速度非0
            Vector3D_SetSize(&Str->Temp.NowSpe_T_3D, Str->T_Str.Output.spe, &Str->Temp.OutSpe_T_3D);
            Vector3D_SetSize(&Str->Temp.NowSpe_T_3D, Str->T_Str.Output.pos, &Str->Temp.OutPos_T_3D);
        }
        else
        {
            Vector3D_Init(&Str->Temp.OutSpe_T_3D);
            Vector3D_Init(&Str->Temp.OutPos_T_3D);
        }
    }
    else if (Str->IfZeroflag.DeltaPos2 && Str->IfZeroflag.OutSpe)
    {
        // 当前速度和目标位移都为0
        Vector3D_Init(&Str->Temp.OutSpe_R_3D);
        Vector3D_Init(&Str->Temp.OutPos_R_3D);

        Vector3D_Init(&Str->Temp.OutSpe_T_3D);
        Vector3D_Init(&Str->Temp.OutPos_T_3D);
    }
    else if (Str->IfZeroflag.DeltaPos2)
    {
        // 目标位移为0，当前速度不为0
        // 则认为速度需要沿径向减速
        Str->Temp.Angle1 = 0;
        // 径向设定值规划当前参数设定
        Str->R_Str.Output.spe = Str->Temp.OutSpe_mod; // 不用一维设定值规划自己的修正
        Str->R_Str.Output.pos = 0;

        Str->R_Str.Input.Now.spe = Str->Temp.OutSpe_mod;
        Str->R_Str.Input.Now.pos = 0;

        Str->R_Str.Input.Set.pos = 0;
        Str->R_Str.Input.Set.spe = 0;

        SetPlanning_Cal(&Str->R_Str);

        Vector3D_SetSize(&Str->Output.Spe, Str->R_Str.Output.spe, &Str->Temp.OutSpe_R_3D);
        Vector3D_SetSize(&Str->Output.Spe, Str->R_Str.Output.pos, &Str->Temp.OutPos_R_3D);

        Vector3D_Init(&Str->Temp.OutSpe_T_3D);
        Vector3D_Init(&Str->Temp.OutPos_T_3D);
    }
    else
    {
        // 当前速度为0,目标位移不为0
        Str->Temp.Angle1 = 0;
        // 径向设定值规划当前参数设定

        Str->R_Str.Output.spe = 0;
        Str->R_Str.Output.pos = 0;

        Str->R_Str.Input.Now.spe = 0;
        Str->R_Str.Input.Now.pos = 0;

        Str->R_Str.Input.Set.pos = Str->Temp.DeltaPos2_mod;
        Str->R_Str.Input.Set.spe = 0;

        SetPlanning_Cal(&Str->R_Str);

        Vector3D_SetSize(&Str->Temp.DeltaPos2, Str->R_Str.Output.spe, &Str->Temp.OutSpe_R_3D);
        Vector3D_SetSize(&Str->Temp.DeltaPos2, Str->R_Str.Output.pos, &Str->Temp.OutPos_R_3D);

        Vector3D_Init(&Str->Temp.OutSpe_T_3D);
        Vector3D_Init(&Str->Temp.OutPos_T_3D);
    }

    Vector3D_Add(&Str->Temp.OutSpe_R_3D, &Str->Temp.OutSpe_T_3D, &Str->Output.Spe);

    Vector3D_Add(&Str->Temp.OutPos_R_3D, &Str->Temp.OutPos_T_3D, &Str->Temp.OutPos_XYZ_Delta);

    Vector3D_Add(&Str->Temp.OutPos_XYZ_Delta, &Str->Output.Pos, &Str->Output.Pos);
}
