/**
 * @file func_Setplanning3D.c
 * @brief 三维控制路径规划
 * @author mylj
 * @version 1.0
 * @date 2023-03-03
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "func_Setplanning3D.h"

/**
 * @brief 路径规划结构体初始化
 * @param Str
 */
void PathPlanning3D_Init(PathPlanning_3D *Str)
{
    // 点位初始化
    for (size_t i = 0; i < DotNum_Max; i++)
    {
        Vector3D_Init(&Str->dot[i].pos);
        Vector3D_Init(&Str->dot[i].spe);
    }
    Str->modtolerance_pos = 0;
    Str->Dotnum_now = 0;
    Str->plan3D_Dotnum = 0;
    Str->plan3D_schedule = 0;
    Str->plan3D_Openflag = Close_Mode;
    Str->plan3Dlastdot_finishflag = Start_Mode;
}

/**
 * @brief 三维路径规划设定函数
 * @param Str
 * @param dot
 * @param num 点的数量 用sizeof(&dot)/6
 * @param openflag
 */
void PathPlanning3D_Set(PathPlanning_3D *Str,
                        SetPlanning_Dot *dot,
                        rt_uint8_t num,
                        plan3D_flag_e openflag)
{
    for (size_t i = 0; i < num; i++)
    {
        Vector3D_Transmit(&dot[i].pos,
                          &Str->dot[i].pos);
        Vector3D_Transmit(&dot[i].spe,
                          &Str->dot[i].spe);
    }

    Str->plan3D_Dotnum = num;
    Str->plan3D_Openflag = openflag;
    Str->Dotnum_now = 0;

    if (num >= DotNum_Max)
    {
        // 所要规划点位超最大设置点数
        while (1)
            ;
    }
}

/**
 * @brief 开启三维路径规划
 * @param Str
 */
void PathPlanning3D_Open(PathPlanning_3D *Str)
{
    Str->plan3D_Openflag = Open_Mode;
}

/**
 * @brief 关闭三维路径规划
 * @param Str
 */
void PathPlanning3D_Close(PathPlanning_3D *Str)
{
    Str->plan3D_Openflag = Close_Mode;
}

/**
 * @brief
 * @param Str1
 * @param Str2
 */
void PathPlanning3D_Cal(SetPlanning3D_Str *Str1,
                        PathPlanning_3D *Str2)
{
    float mod;           // 当前位置与设定位置的模长
    VectorXYZ_Str Delta; // 设定值与当前值的矢量差

    if (Str2->plan3Dlastdot_finishflag == Finish_Mode ||
        Str2->Dotnum_now == 0)
    {
        Vector3D_Transmit(&Str2->dot[Str2->Dotnum_now].pos,
                          &Str1->Input.Set.Pos);
        Vector3D_Transmit(&Str2->dot[Str2->Dotnum_now].spe,
                          &Str1->Input.Set.Spe);
        Str2->plan3Dlastdot_finishflag = Start_Mode;
    }

    Vector3D_Subb(&Str1->Input.Set.Pos,
                  &Str1->Input.Now.Pos,
                  &Delta);
    Vector3D_Mod(&Delta, &mod);
    if (mod < Str2->modtolerance_pos)
    {
        Str2->Dotnum_now++;
        Str2->plan3Dlastdot_finishflag = Finish_Mode;
    }

    Str2->plan3D_schedule = (Str2->Dotnum_now + 1) /
                            (Str2->plan3D_Dotnum);
}
