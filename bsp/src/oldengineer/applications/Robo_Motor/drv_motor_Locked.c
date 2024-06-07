/**
 * @file drv_motor_Locked.c
 * @brief 电机堵转判断及对位
 * @author mylj
 * @version 1.0
 * @date 2022-12-29
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_motor_Locked.h"
#include <math.h>

/**
 * @brief 判断电机是否堵转
 * @param [Motor_t*]        Motor:        电机结构体指针
 * @param [MotorAlign_t*]   MotorAlign:   对位特用结构体指针
 */
void Motor_IfLocked(Motor_t *Motor, MotorAlign_t *Motor_IfLocked)
{
    if ((fabsf(Motor->spe.out) > Motor_IfLocked->MaxCurrent) && // 输出电流大于一定值
        (fabsf(Motor->dji.speed) < Motor_IfLocked->MinSpeed) && // 转速小于一定值
        (Motor_IfLocked->LockedFlag == Motor_Ok))               // 尚未判断为堵转
    {
        if (Motor_IfLocked->ifLockedCount < Motor_IfLocked->ifLockedTime)
            Motor_IfLocked->ifLockedCount++;
        else // 堵转了 ifLockedTime ms 判断为堵转
        {
            Motor_IfLocked->ifLockedCount = 0;
            Motor_IfLocked->LockedFlag = Motor_Err; // 堵转了
        }
    }
    else if (Motor_IfLocked->LockedFlag == Motor_Ok) // 尚未堵转
    {
        if (Motor_IfLocked->ifLockedCount == 0)
            Motor_IfLocked->ifLockedCount = 0;
        else
            Motor_IfLocked->ifLockedCount--;
    }
}

/**
 * @brief 电机堵转对位结构体初始化
 * @param [MotorAlign_t*]   MotorAlign      对位特用结构体指针
 * @param [AlignState_e]    AlignState      对位状态
                                            Start_SpecificAngle_Align 卡位对位
                                            Start_switch              限位开关对位
                                            Align_OK                  正常工作状态
 * @param [rt_uint16_t]     MaxCurrent      最大的设定电流值
 * @param [rt_uint16_t]     MinSpeed        最小速度
 * @param [rt_uint8_t]      AlignMode       校准对位模式
                                            0：当前位置为零位
                                            1：通过正向堵转进行零位校准
                                            2：通过反向堵转进行零位校准
 * @param [rt_uint8_t]      AlignSpeed      对位时的速度
 * @param [rt_uint8_t]      ControlMode     电机的控制模式
 * @param [float]           MotorAngleInit  初始化电机角度
 */
void MotorAlign_Init(MotorAlign_t *MotorAlign,
                     AlignState_e AlignState,
                     rt_uint16_t MaxCurrent,
                     rt_uint16_t MinSpeed,
                     rt_uint8_t AlignMode,
                     rt_uint8_t AlignSpeed,
                     rt_uint8_t ControlMode,
                     float MotorAngleInit)
{
    MotorAlign->ifLockedCount = 0;
    MotorAlign->ifLockedTime = 20;               // 20ms
    MotorAlign->LockedFlag = Motor_Ok;           // 默认电机为正常状态
    MotorAlign->MaxCurrent = MaxCurrent;         // 最大输出电流
    MotorAlign->MinSpeed = MinSpeed;             // 最小速度
    MotorAlign->AlignState = AlignState;         // 默认为不对位状态
    MotorAlign->AlignMode = AlignMode;           // 对位模式
    MotorAlign->AlignSpeed = AlignSpeed;         // 对位速度
    MotorAlign->ControlMode = ControlMode;       // 控制模式
    MotorAlign->MotorAngleInit = MotorAngleInit; // 电机初始角度
}

/**
 * @brief 对位程序
 * @param [Motor_t*]        Motor:        电机结构体指针
 * @param [MotorAlign_t*]   MotorAlign:   对位特用结构体指针
 */
void Align(Motor_t *Motor,
           MotorAlign_t *MotorAlign)
{
   // static uint8_t switch_cnt = 0, switch_pin_data = 0;
    switch (MotorAlign->AlignState)
    {
    case Align_OK:
        break;
    case Start_SpecificAngle_Align:
        if (MotorAlign->AlignMode == 0)
        {
            Motor->dji.extra_angle = MotorAlign->MotorAngleInit;
            Motor->dji.loop = 0;
            MotorAlign->AlignState = SpecificAngle_Align_OK;
        }
        else if (MotorAlign->AlignMode == 1)
        {
            /*正向堵转*/
            MotorAlign->ControlMode = 2; // 转速闭环
            Motor_Write_SetSpeed_ABS(Motor, (float)MotorAlign->AlignSpeed);
            if (MotorAlign->LockedFlag == Motor_Err)
            { // 堵转了
                MotorAlign->AlignState = Locked_SpecificAngle_Align;
            }
        }
        else if (MotorAlign->AlignMode == 2)
        {
            /*反向堵转*/
            MotorAlign->ControlMode = 2; // 转速闭环
            Motor_Write_SetSpeed_ABS(Motor, -(float)MotorAlign->AlignSpeed);
            if (MotorAlign->LockedFlag == Motor_Err)
            { // 堵转了
                MotorAlign->AlignState = Locked_SpecificAngle_Align;
            }
        }
        break;
    case Locked_SpecificAngle_Align:
        Motor->dji.extra_angle = MotorAlign->MotorAngleInit;
        Motor->dji.loop = 0;
        MotorAlign->LockedFlag = Motor_Ok;
        MotorAlign->AlignState = SpecificAngle_Align_OK;
        break;
    case SpecificAngle_Align_OK:
        MotorAlign->ControlMode = (rt_uint8_t)AngleCtrl_Mode;
        MotorAlign->AlignState = Align_OK;
        break;

    // case Start_switch:
    //     if (MotorAlign->AlignMode == 0)
    //     {
    //         Motor->dji.extra_angle = MotorAlign->MotorAngleInit;
    //         Motor->dji.loop = 0;
    //         MotorAlign->AlignState = Switch_trigger;
    //     }
    //     else if (MotorAlign->AlignMode == 1)
    //     {
    //         /*正向堵转*/
    //         MotorAlign->ControlMode = 2; // 转速闭环
    //         Motor_Write_SetSpeed_ABS(Motor, (float)MotorAlign->AlignSpeed);
    //         if (switch_pin_data == 0)
    //         {
    //             switch_cnt++;
    //         }
    //         if (switch_cnt >= 5)
    //         { // 堵转了
    //             MotorAlign->AlignState = Switch_trigger;
    //             switch_cnt = 0;
    //         }
    //     }
    //     else if (MotorAlign->AlignMode == 2)
    //     {                                // 反向堵转
    //         MotorAlign->ControlMode = 2; // 转速闭环
    //         Motor_Write_SetSpeed_ABS(Motor, -(float)MotorAlign->AlignSpeed);
    //         if (switch_pin_data == 0)
    //         {
    //             switch_cnt++;
    //         }
    //         if (switch_cnt >= 5)
    //         { // 堵转了
    //             MotorAlign->AlignState = Switch_trigger;
    //             switch_cnt = 0;
    //         }
    //     }
    //     break;
    // case Switch_trigger:
    //     Motor->dji.extra_angle = MotorAlign->MotorAngleInit;
    //     Motor->dji.loop = 0;
    //     MotorAlign->LockedFlag = Motor_Ok;
    //     MotorAlign->AlignState = Switch_OK;

    //     break;
    // case Switch_OK:
    //     MotorAlign->ControlMode = (rt_uint8_t)AngleCtrl_Mode;
    //     MotorAlign->AlignState = Align_OK;

    //    break;
    }
}
