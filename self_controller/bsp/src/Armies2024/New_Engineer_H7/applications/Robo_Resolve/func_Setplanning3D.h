#ifndef __FUNC_SETPLANNING3D_H__
#define __FUNC_SETPLANNING3D_H__

#include "drv_Setplanning3D.h"
#define DotNum_Max 10

typedef struct
{
    VectorXYZ_Str pos;
    VectorXYZ_Str spe;
} SetPlanning_Dot;

typedef enum
{
    Close_Mode = 0, // 设定值规划开始模式
    Open_Mode,      // 设定值规划停止模式
} plan3D_flag_e;    // 设定值规划开启标志位枚举体

typedef enum
{
    Start_Mode = 0,  // 设定值规划开始模式
    Finish_Mode,     // 设定值规划停止模式
} plan3Dlast_flag_e; // 上一个点设定值规划开启标志位枚举体

// 三维路径规划结构体
typedef struct
{
    rt_uint8_t Dotnum_now;    // 当前点的序列号
    rt_uint8_t plan3D_Dotnum; // 当前路径规划点的数目

    plan3D_flag_e plan3D_Openflag; // 是否停止当前路径规划

    float modtolerance_pos;                     // 模的位置容差 单位 ：m
    plan3Dlast_flag_e plan3Dlastdot_finishflag; // 上一个点的规划是否完成

    float plan3D_schedule; // 当前路径规划的完成度 max：100%

    SetPlanning_Dot dot[DotNum_Max]; // 设定值规划的点的状态（位置和速度）

} PathPlanning_3D;

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
                        plan3D_flag_e openflag);

/**
 * @brief 开启三维路径规划
 * @param Str
 */
void PathPlanning3D_Open(PathPlanning_3D *Str);

/**
 * @brief 关闭三维路径规划
 * @param Str
 */
void PathPlanning3D_Close(PathPlanning_3D *Str);

#endif
