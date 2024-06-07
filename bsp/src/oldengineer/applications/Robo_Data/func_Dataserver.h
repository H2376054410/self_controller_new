#ifndef __FUNC_DATASERVER_H__
#define __FUNC_DATASERVER_H__

#include "drv_Vector.h"
#include "drv_Arm_Solve.h"
#include "drv_Image_Solve.h"
#include "drv_Chassis_Solve.h"
typedef struct
{
    VectorXYZ_Str Pos;
    ForearmMotor_s Angle;
    float ImagePitch;
} ArmPos_Remote_s;

// 自定义控制器数据结构体
typedef struct
{
    rt_uint8_t Limit; // 超限标志位
    VectorXYZ_Str pos;
    EulerAngle_Str angle;
} CustCtrler_Data_s;

/**
 * @brief   从数据服务器读取机械臂电机的速度当前值
 * @param   out
 */
void ArmMotor_NowSpeed_Read(BoomMotor_s *Boom_out,
                            ForearmMotor_s *Forearm_out);

/**
 * @brief   从数据服务器读取机械臂电机的角度当前值
 * @param   out
 */
void ArmMotor_NowAngle_Read(BoomMotor_s *Boom_out,
                            ForearmMotor_s *Forearm_out);

/**
 * @brief   从数据服务器读取机械臂电机的姿态设定值
 * @param   out
 */
void ArmMotor_SetPos_Read(VectorXYZ_Str *Pos_out,
                          ForearmMotor_s *Forearm_out);

/**
 * @brief   机械臂电机的姿态当前值写入数据服务器
 * @param   out
 */
void ArmMotor_NowPos_Write(VectorXYZ_Str *Pos_out);

/**
 * @brief   机械臂电机的姿态设定值写入数据服务器
 * @param   out
 */
void ArmMotor_SetPos_Write(VectorXYZ_Str *Pos_out);

/**
 * @brief   机械臂电机的角度设定值写入数据服务器
 * @param   in
 */
void ArmMotor_SetAngle_Write(ForearmMotor_s *Forearm_in);

/**
 * @brief   机械臂电机的弧度滤波值写入数据服务器
 * @param   in
 */
void ArmMotor_RadFilter_Write(BoomMotor_s *Boom_in,
                              ForearmMotor_s *Forearm_in);

/**
 * @brief   从数据服务器读取机械臂电机的弧度滤波值
 * @param   out
 */
void ArmMotor_RadFilter_Read(BoomMotor_s *Boom_out,
                             ForearmMotor_s *Forearm_out);

/**
 * @brief   从数据服务器读取大机械臂电机的弧度滤波值
 * @param   out
 */
void BoomMotor_RadFilter_Read(BoomMotor_s *Boom_out);

/**
 * @brief   从数据服务器读取机械臂电机的姿态设定值
 * @param   out
 */
void ArmPosSet_Read(VectorXYZ_Str *Pos_out);

/**
 * @brief 机械臂姿态数据读取
 * @param out
 */
void ArmPosStateNowRead(ArmPos_Remote_s *out);

/**
 * @brief 小机械臂Yaw姿态绝对式设定
 * @param in
 */
void ForearmYawSet_ABS(float *in);

/**
 * @brief 机械臂姿态绝对式设定
 * @param in
 */
void ForearmStateSet_Read(ForearmMotor_s *Forearm_out);

/**
 * @brief 机械臂姿态增量式设定
 * @param in
 */
void ArmPosStateSet_ADD(ArmPos_Remote_s *in);

/**
 * @brief 向数据服务器写入图传云台pitch角度设定值
 * @param in
 */
void ImageGimbalPitchSet_Write(float in);

/**
 * @brief 机械臂姿态绝对式设定
 * @param in
 */
void ArmPosStateSet_ABS(ArmPos_Remote_s *in);

/**
 * @brief 机械臂姿态绝对式设定
 * @param in
 */
void BoomPosStateSet_ABS(VectorXYZ_Str *in);

/*****************************底盘相关*****************************/

/**
 * @brief 底盘运动速度绝对式设定
 * @param in
 */
void ChassisSpeStateSet_ABS(ChassisMotor_t *in);

/**
 * @brief   从数据服务器读取轮子的速度设定值
 * @param   out
 */
void Chassis_SetSpeed_Read(ChassisMotor_t *out);

/**
 * @brief   从数据服务器读取轮子的速度当前值
 * @param   out
 */
void Chassis_NowSpeed_Read(ChassisMotor_t *out);

/********************************速度增益**********************************/
/**
 * @brief 从数据服务器读取速度增益
 * @return speedgain 速度增益
 */
float Robo_SpeedGain_Get(void);

/**
 * @brief 向数据服务器设定速度增益
 * @param speedgain
 */
void Robo_SpeedGain_Set(float speedgain);

/**
 * @brief 向数据服务器写自定义控制器数据
 */
void CustCtrler_Data_Write(CustCtrler_Data_s *in);

/**
 * @brief 从数据服务器读取自定义控制器数据
 */
void CustCtrler_Data_Read(CustCtrler_Data_s *out);

/**
 * @brief 从数据服务器读取自定义控制器按键数据
 */
void CustCtrlerKey_Data_Read(uint8_t *out);

/**
 * @brief 向数据服务器写入机械臂超限数据
 */
void Arm_IfLimit_Data_Write(uint8_t *in);

/**
 * @brief 从数据服务器读取UI复位数据
 */
void UIReset_State_Data_Read(uint8_t *out);

/**
 * @brief 向数据服务器写入UI复位数据
 */
void UIReset_State_Data_Write(uint8_t in);

/**
 * @brief 从数据服务器读取机械臂超限数据
 */
void Arm_IfLimit_Data_Read(uint8_t *out);

/**
 * @brief   从数据服务器读取图传电机的速度当前值
 * @param   out
 */
void ImageMotor_NowSpeed_Read(ImageMotor_s *Image_out);

/**
 * @brief   从数据服务器读取图传电机的角度当前值
 * @param   out
 */
void ImageMotor_NowAngle_Read(ImageMotor_s *Image_out);

/**
 * @brief   从数据服务器读取图传电机的角度期望值
 * @param   out
 */
void ImageMotor_AngleHope_Read(ImageMotor_s *Image_out);

/**
 * @brief 向数据服务器写图传云台角度期望值
 */
void ImageMotor_AngleHope_Write(ImageMotor_s *in);

/**
 * @brief 向数据服务器写图传pitch角度设定值
 */
void ImageMotor_YawAngleHope_Write(float in);

/**
 * @brief 从数据服务器读取UI复位数据
 */
void Image_PosState_Read(uint8_t *out);

/**
 * @brief 向数据服务器写入UI复位数据
 */
void Image_PosState_Write(uint8_t in);

/**
 * @brief 从数据服务器读取机械臂大yaw编码器角度数据
 */
void BoomYaw_CoderRad_Read(float *out);

/**
 * @brief 向数据服务器写入机械臂大yaw编码器角度数据
 */
void BoomYaw_CoderAngle_Write(float in);

/**
 * @brief 从数据服务器读取图传yaw编码器角度数据
 */
void ImageYaw_CoderAngle_Read(float *out);

/**
 * @brief 向数据服务器写入图传yaw编码器角度数据
 */
void ImageYaw_CoderAngle_Write(float in);

/**
 * @brief 从数据服务器读取图传云台角度数据
 */
void ImageGimbal_Angle_Read(ImageMotor_s *out);
/**
 * @brief 向数据服务器写入图传云台角度数据
 */
void ImageGimbal_Angle_Write(ImageMotor_s *in);

/**
 * @brief 从数据服务器读取大机械臂yaw编码器速度滤波数据
 */
void BoomYaw_SpeedFilter_Read(float *out);

/**
 * @brief 向数据服务器写入大机械臂yaw编码器速度滤波数据
 */
void BoomYaw_SpeedFilter_Write(float in);

/**
 * @brief 从数据服务器读取陀螺仪yaw角度
 */
void GyroYaw_AngleNow_Read(float *out);

/**
 * @brief 从数据服务器读取陀螺仪yaw角速度
 */
void GyroYaw_SpeedNow_Read(float *out);

/**
 * @brief 从数据服务器读取陀螺仪yaw角度弧度值
 */
void GyroYaw_radNow_Read(float *out);

/**
 * @brief 从数据服务器读取陀螺仪yaw角速度值
 */
void GyroYaw_radNow_Write(float in);

/**
 * @brief 从数据服务器读取编码器标定状态位
 */
void EncoderCali_State_Read(uint8_t *out);

/**
 * @brief 从数据服务器写入编码器标定状态位
 */
void EncoderCali_State_Write(uint8_t in);

/**
 * @brief 从数据服务器读取当前云台角度
 */
void GimbalYawRad_Now_Read(float *out);

/**
 * @brief 向数据服务器写入当前云台角度
 */
void GimbalYawRad_Now_Write(float in);

/**
 * @brief 从数据服务器读取当前底盘角度
 */
void ChassisYawRad_Now_Read(float *out);

/**
 * @brief 向数据服务器写入当前底盘角度
 */
void ChassisYawRad_Now_Write(float in);

/**
 * @brief 从数据服务器读取当前底盘转速
 */
void ChassisYawSpeed_Now_Read(float *out);

/**
 * @brief 向数据服务器写入当前底盘转速
 */
void ChassisYawSpeed_Now_Write(float in);
/**
 * @brief 从数据服务器读取设定云台角度
 */
void GimbalYawRad_Set_Read(float *out);

/**
 * @brief 向数据服务器写入设定云台角度
 */
void GimbalYawRad_Set_Write(float in);

/**
 * @brief 向数据服务器增量式写入设定云台角度
 */
void GimbalYawRad_Set_ADD(float in);

/**
 * @brief 从数据服务器读取大机械臂编码器校正数据
 */
void BoomYaw_CoderCaliRad_Read(float *out);

/**
 * @brief 向数据服务器写入大机械臂编码器校正数据
 */
void BoomYaw_CoderCaliRad_Write(float in);

/**
 * @brief  工程机器人控制线程数据包序列号读取
 */
int EngineerRobo_Datanum_Find(void);

#endif /*__FUNC_DATASERVER_H__*/
