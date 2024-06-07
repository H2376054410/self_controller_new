#ifndef __DRV_MOTOR_LOCKED_H__
#define __DRV_MOTOR_LOCKED_H__

#include "rtdef.h"
#include "drv_motor.h"

#define Motor_Err 1 // 电机发生故障（例如失联、堵转等）
#define Motor_Ok 0  // 电机正常运转

#define Motor_AngleRatio 100.0f

/*判断电机堵转及对位结构体*/
typedef struct
{
    /*堵转*/
    rt_uint8_t LockedFlag;     // 是否堵转
    rt_uint16_t MaxCurrent;    // 堵转保护识别的最大电流值
    rt_uint16_t ifLockedTime;  // 判断堵转的时间
    rt_uint16_t ifLockedCount; // 判断堵转的中间计数值
    rt_uint16_t MinSpeed;      // 堵转保护识别的最小转速
    /*对位*/
    rt_uint8_t AlignMode;   // 校准模式：
                            // 0：当前位置为零位
                            // 1：通过正向堵转进行零位校准
                            // 2：通过反向堵转进行零位校准
    rt_uint8_t AlignState;  // 校准状态
    rt_uint8_t AlignSpeed;  // 校准转速
    rt_uint8_t ControlMode; // 控制模式
                            // 1：不控制
                            // 2：转速闭环控制
                            // 3：角度闭环控制
    rt_uint8_t DataSource;  // 数据源
                            // 1：电机的角度值
                            // 2: 陀螺仪的角度值

    rt_uint8_t EncoderMode; // 编码器类型
    float MotorAngleInit;   // 初始化电机角度
} MotorAlign_t;

typedef enum
{
    MotNoCtrl_Mode = 1, // 1：不控制
    SpeedCtrl_Mode,     // 2：转速闭环控制
    AngleCtrl_Mode,     // 3：角度闭环控制
} MotCtrlMode_e;

typedef enum
{
    ZeroPoint = 0, // 0：当前位置为零位
    FowardStall,   // 1：通过正向堵转进行零位校准
    ReversStall,   // 2：通过反向堵转进行零位校准
} AlignMode_e;

/*对位状态枚举体*/
typedef enum
{
    Start_SpecificAngle_Align,  // 开始特定角度对位
    Locked_SpecificAngle_Align, // 特定角度堵转
    SpecificAngle_Align_OK,     // 特定角度对位完成

    Start_switch,   // 限位开关
    Switch_trigger, // 堵转
    Switch_OK,      // 对位完成

    Align_OK, // 对位完成的正常工作状态
} AlignState_e;

/*电机类型枚举体*/
typedef enum
{
    OutShaft_Mode,   // 输出轴编码器
    NoOutShaft_Mode, // 转子（非输出轴）编码器电机
} EncoderMode_e;

/**
 * @brief 判断电机是否堵转
 * @param [Motor_t*]        Motor:        电机结构体指针
 * @param [MotorAlign_t*]   MotorAlign:   对位特用结构体指针
 */
extern void Motor_IfLocked(Motor_t *Motor,
                           MotorAlign_t *Motor_IfLocked);

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
                                            1：不控制
                                            2：转速闭环控制
                                            3：角度闭环控制
 * @param [float]           MotorAngleInit  初始化电机角度
 */
void MotorAlign_Init(MotorAlign_t *MotorAlign,
                     AlignState_e AlignState,
                     rt_uint16_t MaxCurrent,
                     rt_uint16_t MinSpeed,
                     rt_uint8_t AlignMode,
                     rt_uint8_t AlignSpeed,
                     rt_uint8_t ControlMode,
                     float MotorAngleInit);

/**
 * @brief 对位程序
 * @param [Motor_t*]        Motor:        电机结构体指针
 * @param [MotorAlign_t*]   MotorAlign:   对位特用结构体指针
 */
void Align(Motor_t *Motor,
           MotorAlign_t *MotorAlign);

#endif /*__DRV_MOTOR_LOCKED_H*/
