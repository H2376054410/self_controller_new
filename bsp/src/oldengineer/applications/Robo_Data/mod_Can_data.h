#ifndef __FUNC_CAN_DATA_H__
#define __FUNC_CAN_DATA_H__

#include "drv_motor.h"
#include "drv_SPICAN.h"
#include "drv_HW_SPI.h"
#include "drv_Arm_Solve.h"
#include "drv_Image_Solve.h"
#include "drv_Chassis_Solve.h"

typedef enum
{
    can1 = 0,
    can2,
} CanNum_e;

typedef enum
{
    Slave1 = 0,
    Slave2,
    Slave3,
} SlaveNum_e;

// 无效数据
#define DATA_NUM_NULL -1

/**
 * @brief 非SPI时，can的接收
 * @param Msg   can报文
 */
extern void Can1Receive(struct rt_can_msg *Msg);

/**
 * @brief 非SPI时，can的接收
 * @param Msg   can报文
 */
extern void Can2Receive(struct rt_can_msg *Msg);

/**
 * @brief SPICAN初始化
 */
extern void SPICAN_Init(void);

/**
 * @brief  电机Can报文处理数据包序列号读取
 */
extern int MotorCan_Datanum_Find(void);

/*电机完整ID列举*/
// 接收
// CAN1  5~8
#define BoomYaw_MotorID 0x205    // 1
#define BoomPitch2_MotorID 0x206 // 3
#define BoomPitch1_MotorID 0x207 // 2
#define ImageYaw_MotorID 0x208   // 4
#define IMAGEYAW_EncoderID 0x204 // 5
#define CUSTCTRL_YPR_ID 0x20A    // 自定义控制器yaw、pitch、roll报文ID处理
#define CUSTCTRL_XYZ_ID 0x20B    // 自定义控制器xyz报文ID处理

#define ForearmYaw_MotorID 0x205   // 1
#define ForearmPitch_MotorID 0x206 // 2
#define ForearmRoll_MotorID 0x207  // 3
#define ImagePitch_MotorID 0x208   // 4

#define ChassisRF_MotorID 0x201
#define ChassisRB_MotorID 0x202
#define ChassisLB_MotorID 0x203
#define ChassisLF_MotorID 0x204

#define IMAGEYAW_MOTORID 0x201
#define IMAGEPITCH_MOTORID 0x202
#define CHASSISYAW_GYROSCOPEID 0x205

/*can报文标识符*/
// 发送
#define CHASSIS_CANID 0x200
#define IMAGE_CANID 0x200
#define ARM_CANID 0x1FF
#define DEBUG_CANID 0x7FF

/**
 * @brief 从机1的报文发送处理函数
 * @brief 底盘部分与图传部分电机报文发送
 * @param Chassis_in
 */
void Send_Slave1_Handle(ChassisMotor_t *Chassis_in);

/**
 * @brief 从机2的报文发送处理函数
 * @brief 机械臂部分电机报文发送
 * @param in
 */
void Send_Slave2_Handle(BoomMotor_s *Boom_in,
                        ForearmMotor_s *Forearm_in,
                        ImageMotor_s *Image_in);

/**
 * @brief 从机1的报文发送初始化函数
 * @brief 底盘部分电机报文发送
 * @param in
 */
void Send_Slave1_Init(void);

/**
 * @brief 从机2的报文发送初始化函数
 * @brief 机械臂部分电机报文发送
 * @param in
 */
void Send_Slave2_Init(void);

/**
 * @brief 给10020电机或4310电机设置数据
 * @brief 机械臂部分电机报文发送
 * @param in
 */
void Send_Slave_Debug(void);

/**
 * @brief 从机1的报文处理函数
 * @brief 底盘部分电机CAN报文的接收
 * @param Msg   can报文
 */
void Receive_Slave1_Handler(SPICAN_MsgOnTransfer_t *Msg);

/**
 * @brief 从机2的报文处理函数
 * @brief 机械臂末端陀螺仪  CAN报文的接收
 * @brief 底盘陀螺仪        CAN报文的接收
 * @param Msg   can报文
 */
void Receive_Slave2_Handler(SPICAN_MsgOnTransfer_t *Msg);

/**
 * @brief 从机3的报文处理函数
 * @brief 小机械臂与图传部分CAN报文的接收
 * @brief 大机械臂部分CAN报文的接收
 * @param Msg   can报文
 */
void Receive_Slave3_Handler(SPICAN_MsgOnTransfer_t *Msg);

#endif /*#__FUNC_CAN_DATA_H__*/
