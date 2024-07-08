#ifndef __CANINIT_H__
#define __CANINIT_H__

#include <rtthread.h>
#include "pid.h"
#define CAN_DEV_NAME "can1" 


#define BOOM_LEFTID 1
#define BOOM_RIGHTID 2
#define BOOM_YAWID 3
#define A4310_ENCODERLEN 36001
typedef  struct rt_can_msg  can_msg;

typedef enum
{
    // 若切换模式，则需要重新调参数
    ANGLE_CTRL_ABS,   // 按照编码器数值进行控制，设定值不能跨圈，减速比为1的云台电机推荐使用此设置，也可使用extra模式
    ANGLE_CTRL_EXTRA, // 按照减速电机的电机输出轴转动情况进行控制，设定值不能跨圈， 单位°
    ANGLE_CTRL_FULL   // 按照减速电机的电机输出轴转动情况进行控制，设定值可以跨圈， 单位°
} Angle_CtrlMode_E;

typedef struct DjiMotor
{
    rt_uint32_t motorID; /*电机反馈报文ID*/

    rt_int32_t angle;     /*编码器原始角度 0~8191*/
		rt_int32_t lastangle;
    rt_int32_t old_angle; /*上一次编码器数值*/
    rt_int32_t Encoder_LEN;
	  float Encoder_LEN_DM;
    rt_err_t oldangle_state; /* 记录当前历史数据是否有效，首次使用时历史数据无效 */
    rt_int32_t speed;
    rt_int32_t current;
    rt_uint8_t temperature;

		rt_uint8_t ErrorID;

    rt_uint8_t reverse_flag; // 电机数据是否需要反向, 需要反向为 1
    float extra_angle;       // 记录的圈数之外多出来的角度 单位 °
    rt_int32_t loop;         /*一共所转圈数*/
    float ratio;             /* 填写电机输出轴转动一圈时，编码器数据转过的圈数，以完整的M3508为例，应填写19.0f */

    Angle_CtrlMode_E Angle_CtrlMode; // 记录角度控制模式，如果有减速箱则为EXTRA模式，无减速箱则用ABS模式
    rt_int32_t Round_Len;            // 角度环计算时的整圈长度
    rt_int32_t Set_MAX;              // 角度环计算时的角度数值的最大值
    rt_int32_t Set_MIN;              // 角度环计算时的角度数值的最小值
    rt_int32_t Data_Valid;           /* 是否在初始化电机结构体后接收到新报文 */
    void (*Motor_Read_Msg)(rt_uint8_t rxmsg[], struct DjiMotor *motor);
    rt_int32_t FreshTick; // 记录最后一次数据刷新时刻，用于判断电机是否离线
} DjiMotor_t;


typedef struct __Motor_t
{
    DjiMotor_t dji;
    pid_t ang;
    pid_t spe;

} Motor_t;

extern Motor_t Boomleft_Motor,Boomright_Motor,Boomyaw_Motor;
extern  struct rt_semaphore rx_time;
void can_init(void);
void Send_Slave2_Init(void);
void motor_init_DM(Motor_t *motor, rt_uint32_t ID, float ratio, Angle_CtrlMode_E ModeSet, rt_int32_t Encoder_Len, rt_int32_t Set_Max, rt_int32_t Set_Min, int MotorReverse);
#endif
