#ifndef __FUNC_ARMMOTOR_CTRL_H__
#define __FUNC_ARMMOTOR_CTRL_H__

#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_thread.h"
#include "drv_SetPlanning.h"
#include "drv_Arm_Solve.h"

/**
 * @brief 电机电流设定值的范围
 * @param M2006  -10000~10000
 * @param M3508  -16384~16384
 * @param GM6020 -30000~30000
 * @param A4310  -32768~32767
 * @param A10020 -32768~32767
 */

#define ARMMOTOR_CTRLPERIOD (ROBOMOTOR_TIMER_PIRIOD * 0.04f) // 单位：秒

/*正弦波*/
#define FOREARMMOTOR_CTRLPERIOD (1000 * ROBOCTRL_TIMER_PIRIOD * 0.001f) // 单位：秒
#define ARMDISTURBANCE_AMPLITUDE 1000

/*电机转动一圈对应多少编码器角度单位，例如GM6020采用8192线编码器，则应填写8192*/
#define M2006_ENCODERLEN 8192
#define M3508_ENCODERLEN 8192
#define GM6020_ENCODERLEN 8192
#define A4310_ENCODERLEN 36001
#define A10020_ENCODERLEN 36001
#define DM6006_ENCODERLEN 65535
/*减速比，用于角度解算*/
#define M2006_RATIO 36.0f
#define M3508_Ratio 19.2f
#define GM6020_Ratio 1.0f
#define A4310_Ratio 1.0f  // 36.0f
#define A10020_Ratio 1.0f // 12.0f
#define DM6006_Ratio 1.0f
#define M3508_Ratio_chilun 2.0f
/************************电机零点校准*************************/
#define BOOMYAW_ZEROPOINT -1.5707f // 2024机械臂还没有校准
#define BOOMPITCH1_ZEROPOINT 3.4028f
#define BOOMPITCH2_ZEROPOINT -1.4370f
#define FOREARMYAW_ZEROPOINT -0.73118f
#define FOREARMPITCH_ZEROPOINT 0.4561f
#define FOREARMROLL_ZEROPOINT 0.0f
#define BOOMMOVE_ZEROPOINT 0.08f
#define IMAGETRANS_ZEROPOINT 0.0f

/**********************编码器长度**************************/
/*机械臂电机编码器长度*/
#define BoomYaw_EncoderLen A4310_ENCODERLEN
#define BoomPitch1_EncoderLen A10020_ENCODERLEN
#define BoomPitch2_EncoderLen A4310_ENCODERLEN
#define ForearmYaw_EncoderLen A4310_ENCODERLEN
#define ForearmPitch_EncoderLen DM6006_ENCODERLEN
#define ForearmRoll_EncoderLen DM6006_ENCODERLEN
#define BoomMove_Ecoderlen M3508_ENCODERLEN

/**************************减速比*************************/
/*机械臂电机减速比*/
#define BoomYaw_Ratio A4310_Ratio
#define BoomPitch1_Ratio A10020_Ratio
#define BoomPitch2_Ratio A4310_Ratio
#define ForearmYaw_Ratio A4310_Ratio
#define ForearmPitch_Ratio DM6006_Ratio
#define ForearmRoll_Ratio DM6006_Ratio
#define BoomMove_Ratio 1.0f

/************************实际转速换算比******************************/
#define BoomYaw_RPMRatio 22.0f // 10*2.2f(减速比)//2024机械臂还没算
#define BoomPitch1_RPMRatio 10.0f
#define BoomPitch2_RPMRatio 10.0f
#define ForearmYaw_RPMRatio 10.0f
#define ForearmPitch_RPMRatio 6.0f
#define ForearmRoll_RPMRatio 6.0f // 1*36.0f减速比)
#define BooomMove_RPMRatio 19.2f

/************************设定值规划******************************/
#define SETPPLANPERIOD ARMMOTOR_CTRLPERIOD

#define FOREARMYAW_SPEEDMAX 0.1f
#define ForearmYAW_POSERRORMAX 0.5f
#define ForearmYAW_ACCLMAX 0.2f

#define FOREARMPITCH_SPEEDMAX 0.05f
#define FOREARMPITCH_POSERRORMAX 0.5f
#define FOREARMPITCH_ACCLMAX 0.2f

#define FOREARMROLL_SPEEDMAX 0.05f
#define FOREARMROLL_POSERRORMAX 0.5f
#define FOREARMROLL_ACCLMAX 0.2f

typedef enum
{
    BoomYaw,
    BoomPitch1,
    BoomPitch2,
    BoomMove,
    ForearmYaw,
    ForearmPitch,
    ForearmRoll,
} ArmMotor_e;
extern int Remote_Tanslate;//决定是否使用遥控器数据
extern int Move_flag;
/**
 * @brief 机械臂部分的电机及pid控制初始化
 */
void ArmMotor_Init(void);

/**
 * @brief 大机械臂电机闭环控制
 * @param ArmMotor
 * @param BoomStateData
 */
void BoomMotor_Ctrl(ArmMotor_e ArmMotor,
                    BoomState_Data_s *BoomStateData);

/**
 * @brief 小机械臂电机闭环控制
 * @param ArmMotor
 * @param ForearmStateData
 */
void ForearmMotor_Ctrl(ArmMotor_e ArmMotor,
                       ForearmState_Data_s *ForearmStateData);


/**
 * @brief 矿臂姿态电机的设定值规划
 * @brief 改变电机AngleSetPlan和SpeedFeedforward的数据
 * @param ForearmStateData   电机位姿信息（角度，角速度，期望角度）
 * @param AngleSetPlan
 * @param SpeedFeedforward
 */
void GoldArmSetPlanning(Gold_Data_s *GoldArmStateData,
                        Arm_Angle_Gold *AngleSetPlan,
                        Arm_Angle_Gold *SpeedFeedforward);
void BoomArmSetPlanning(BoomState_Data_s *BoomArmStateData,
                        BoomMotor_s *AngleSetPlan,
                        BoomMotor_s *SpeedFeedforward);
/**
 * @brief 将Boom读取到的编码器的速度转换成rad
 * @param Spe_encoder
 * @param Speed_rpm
 */
void BoomEncoderSpeTorpm(BoomMotor_s *Spe_encoder,
                         BoomMotor_s *Speed_rad);

/**
 * @brief 将Forearm读取到的编码器的速度转换成RPM速度
 * @param Spe_encoder
 * @param Speed_rpm
 */
void ForearmEncoderSpeTorpm(ForearmMotor_s *Spe_encoder,
                            ForearmMotor_s *Speed_rad);

/**
 * @brief 对Boom电机角度值及速度值进行滤波
 * @param BoomStateData
 */
void BoomMotDataFilter(BoomState_Data_s *BoomStateData);

/**
 * @brief 对Forearm电机角度值及速度值进行滤波
 * @param ForearmStateData
 */
void ForearmMotDataFilter(ForearmState_Data_s *ForearmStateData);

/**
 * @brief 机械臂编码器值转成角度值
 * @brief 包括传动比
 * @param BoomAngle_in
 * @param ForearmAngle_in
 * @param Boomrad_out
 * @param Forearmrad_out
 */
void BoomEncoder_angle(BoomMotor_s *BoomAngle_in,
                       BoomMotor_s *BoomAngle_out,		
												float *bp2_last);

/**
 * @brief 把小机械臂编码器值转成角度值
 * @brief 包括传动比
 * @param ForearmAngle_in
 * @param ForearmAngle_out
 */
void ForearmEncoder_angle(ForearmMotor_s *ForearmAngle_in,
                       ForearmMotor_s *ForearmAngle_out);

void Arm_MotorAngle_Rad(BoomMotor_s *BoomAngle_in,
                        BoomMotor_s *BoomAngle_out,
                        ForearmMotor_s *ForearmAngle_in,
                        ForearmMotor_s *ForearmAngle_out);

/**
 * @brief 电机位置信息由角度转向弧度
 * @param BoomAngle_in
 * @param ForearmAngle_in
 * @param Boomrad_out
 * @param Forearmrad_out
 */
void Arm_angle2rad(BoomMotor_s *BoomAngle_in,
                   ForearmMotor_s *ForearmAngle_in,
                   Boom_Mid_Data_s *Boomrad_out,
                   Forearm_Mid_Data_s *Forearmrad_out);



/**
 * @brief 机械臂电机数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ArmMotor_DataValid_If(void);

/**
 * @brief  大机械臂yaw编码器数据有效性
 * @return 有效则返回1 ，无效则返回0
 */
int BoomYawEncoder_DataValid_If(void);

/**
 * @brief 电机输入电流扭矩计算
 * @param BoomComp_in       扭矩补偿输入
 * @param ForearmComp_in       扭矩补偿输入
 * @param Boom_out          Boom电机的电流设定输出
 * @param Forearm_out       Forearm电机的电流设定输出
 */
void ArmMotorinput_Calculate(BoomMotor_s *BoomComp_in,
                             ForearmMotor_s *ForearmComp_in,
                             BoomMotor_s *Boom_out,
                             ForearmMotor_s *Forearm_out);

/**
 * @brief  获取电机结构体
 * @author mylj
 * @param  ArmMotor         指定需要获取的电机
 * @return void*            返回指向滤波器的指针
 */
void *Get_ArmMotor(ArmMotor_e ArmMotor);

/**
 * @brief 获取电机校准结构体
 * @author mylj
 * @param  ArmMotorAlign        指定需要获取的电机
 * @return void*                返回指向滤波器的指针
 */
void *Get_ArmMotorAlign(ArmMotor_e ArmMotorAlign);

/**
 * @brief 机械臂Pitch轴对位
 */
void ArmPitchAlign(void);

/**
 * @brief  获取大机械Yaw编码器结构体
 * @author mylj
 * @param  ImageEncoder         指定需要获取的电机
 * @return void*              返回指向电机结构体的指针
 */
void *Get_ArmEncoder(ArmMotor_e ArmEncoder);

#endif
