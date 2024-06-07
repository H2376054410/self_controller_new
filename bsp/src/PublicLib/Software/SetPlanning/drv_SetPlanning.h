#ifndef __DRV_SETPLANNING_H__
#define __DRV_SETPLANNING_H__

#include <rtdevice.h>
#include <rtthread.h>

typedef struct
{
    float spe;
    float pos;
} MotionState_Str;

typedef struct
{
    float Speed_Max;     // 调整过程最大速度
    float POS_Error_Max; // 输出设定值对应的最大Error
    float Accl_Max;      // 最大加速度
    float dt;            // 计算周期 单位S
} SetPlanSettings_Str;

typedef struct
{
    MotionState_Str Set;
    MotionState_Str Now;
} SetPlanInput_Str;

typedef struct
{
    MotionState_Str OutputTemp;
    float Accl_Now;
    float DeltaPos_Temp; // 当前按照最大加速度减速到设定值需要的位移
    float DeltaSpe;
    float DeltaSpe_2;
    float DeltaPos;
} SetPlanData_Str;

typedef struct
{
    volatile rt_tick_t CalTick;
    SetPlanInput_Str Input;
    MotionState_Str Output;
    SetPlanSettings_Str Settings;
    SetPlanData_Str Temp;
} SetPlanning_Str;

extern void SetPlanning_OldCal(SetPlanning_Str *Str);

// 执行设定值规划计算
extern void SetPlanning_Cal(SetPlanning_Str *Str);

// 直接修改规划结果
extern void SetPlanning_SetOutput(SetPlanning_Str *Str, float PosSet);

// 更新设定值规划的数据
extern void Planning_updata(float Now_pos, float Set_spe, float Set_pos, float speed_max, SetPlanning_Str *Str);

// 初始化规划结构体（结构体, PID最大Error, 调整最大加速度, 计算周期）
extern void SetPlanning_Init(SetPlanning_Str *Str, SetPlanSettings_Str *Settings);

/**
 * @brief 设定值规划设定数据初始化
 * @param Settings          设定数据结构体
 * @param Speed_Max         调整过程最大速度
 * @param POS_Error_Max     输出设定值对应的最大Error
 * @param Accl_Max          最大加速度
 * @param Period            计算周期 单位s
 */
void SetPlanSettings_Init(SetPlanSettings_Str *Settings,
                          float Speed_Max,
                          float POS_Error_Max,
                          float Accl_Max,
                          float Period);
#endif
