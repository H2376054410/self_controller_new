#ifndef __DRV_SETPLANNING3D_H__
#define __DRV_SETPLANNING3D_H__

#include <rtdevice.h>
#include <rtthread.h>
#include "drv_Vector.h"
#include "drv_SetPlanning.h"

/*径向为x，切向为y*/

typedef struct
{
    VectorXYZ_Str Pos;
    VectorXYZ_Str Spe;
} MoveState_Str;

typedef struct
{
    MoveState_Str Now;
    MoveState_Str Set;
} Input_Str;

typedef struct
{
    float dt; // 计算周期               单位：s
    float Pos_ErrorMax;
    VectorXY_Str Accl_Max;
    VectorXY_Str Speed_Max;

    float Pos_Tolerance; // 用于判断期望位置矢量是否是0向量，位置容差 0.001m
    float Spe_Tolerance; // 用于判断当前速度矢量是否是0向量，速度容差 0.005m/s

} PlanSettings_Str;

typedef struct
{
    int OutSpe;      // 上一次输出速度值的模长是否为0标志位    1为0
    int DeltaPos2;   // 当前速度的模长是否为0标志位    1为0
    int NowSpe_T_3D; // 上一次规划输出速度的切向分量的模长是否为0标志位    1为0
} IfZeroflag_Str;

typedef struct
{
    float DeltaPos1_mod; // 上一次输出值与实际位置的差值的模长
    float DeltaPos2_mod; // 目标位置与修正后上次输出位置的差值的模长
    float OutSpe_mod;    // 上一次输出速度的模长
    float Angle1;        // 上次规划输出的速度与当前目标方向的夹角

    VectorXYZ_Str DeltaPos1;      // 上一次输出值与实际位置的差值
    VectorXYZ_Str DeltaPos2;      // 目标位置与修正后上次输出位置的差值
    VectorXYZ_Str DeltaPos_Fixed; // 修正后的相对于实际值的差矢量
    VectorXY_Str SetPos;          // 上一次规划输出速度和目标位置的径向和切向分量
    VectorXY_Str NowSpe;          // 上一次规划输出速度的径向和切向分量

    VectorXYZ_Str NowSpe_R_3D; // 上一次规划输出速度的径向分量三维矢量
    float NowSpe_T_3D_mod;     // 上一次规划输出速度的切向分量三维矢量的模长
    VectorXYZ_Str NowSpe_T_3D; // 上一次规划输出速度的切向分量三维矢量

    VectorXYZ_Str OutSpe_R_3D; // 径向设定值规划速度输出
    VectorXYZ_Str OutSpe_T_3D; // 切向设定值规划速度输出

    VectorXYZ_Str OutPos_R_3D; // 径向设定值规划位置输出
    VectorXYZ_Str OutPos_T_3D; // 切向设定值规划位置输出

    VectorXYZ_Str OutPos_XYZ_Delta; // 规划输出位置增量
} SetPlan3DData_Str;

typedef struct
{
    volatile rt_tick_t CalTick;
    Input_Str Input;
    MoveState_Str Output;
    PlanSettings_Str Settings;
    SetPlan3DData_Str Temp;
    IfZeroflag_Str IfZeroflag;
    SetPlanning_Str R_Str; // 径向
    SetPlanning_Str T_Str; // 切向
} SetPlanning3D_Str;

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
                         float Period);

/**
 * @brief 三维路径规划设定值
 * @param Str
 * @param pos
 * @param spe
 */
void PlanSettings3D_Set(SetPlanning3D_Str *Str,
                        VectorXYZ_Str *pos,
                        VectorXYZ_Str *spe);

/**
 * @brief 三维设定值规划径向速度限幅
 * @param Str
 * @param SpeedMax_R
 */
void Plan3D_SpeedR_Max_Set(SetPlanning3D_Str *Str,
                           float SpeedMax_R);

/**
 * @brief 三维设定值规划径向加速度限幅
 * @param Str
 * @param SpeedMax_R
 */
void Plan3D_AcclR_Max_Set(SetPlanning3D_Str *Str,
                          float AcclMax_R);

/**
 * @brief 三维设定值规划
 * @brief 传参为国际单位
 */
void SetPlanning3D_Cal(SetPlanning3D_Str *Str);

#endif /*#__DRV_SETPLANNING3D_H__*/
