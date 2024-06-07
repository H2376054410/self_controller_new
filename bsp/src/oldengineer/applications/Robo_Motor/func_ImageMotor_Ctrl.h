#ifndef __FUNC_IMAGEMOTOR_CTRL_H__
#define __FUNC_IMAGEMOTOR_CTRL_H__

#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_thread.h"
#include "drv_Image_Solve.h"
#include "drv_SetPlanning.h"

#define IMAGEMOTOR_CTRLPERIOD (ROBOCTRL_TIMER_PIRIOD * 0.001f) // 单位：秒
#define DISTURBANCE_AMPLITUDE 150

/*电机转动一圈对应多少编码器角度单位，例如GM6020采用8192线编码器，则应填写8192*/
#define M2006_ENCODERLEN 8192
#define M3508_ENCODERLEN 8192
#define GM6020_ENCODERLEN 8192
#define A4310_ENCODERLEN 36001
#define A10020_ENCODERLEN 36001
/*减速比，用于角度解算*/
#define M2006_RATIO 36.0f
#define M3508_RATIO 19.2f
#define GM6020_RATIO 1.0f
#define A4310_RATIO 1.0f
#define A10020_RATIO 1.0f
/****************图传云台电机的基本信息*************/

/*电机编码器长度*/
#define IMAGEYAW_ENCODERLEN M2006_ENCODERLEN
#define IMAGEPITCH_ENCODERLEN M2006_ENCODERLEN

/*电机加速比*/
#define IMAGEYAW_RATIO M2006_RATIO
#define IMAGEPITCH_RATIO M2006_RATIO

/*电机零位校准*/
#define IMAGEYAW_ZEROPOINT 0.0f
#define IMAGEPITCH_ZEROPOINT 0.0f

/*设定值规划值*/
#define IMAGE_SETPPLANPERIOD IMAGEMOTOR_CTRLPERIOD

#define IMAGEYAW_SPEEDMAX 6
#define IMAGEYAW_POSERRORMAX 0.5f
#define IMAGEYAW_ACCLMAX 15

#define IMAGEPITCH_SPEEDMAX 6
#define IMAGEPITCH_POSERRORMAX 0.5f
#define IMAGEPITCH_ACCLMAX 15

/*实际转速换算比*/
#define IMAGEYAW_RPMRATIO 36.0f
#define IMAGEPITCH_RPMRATIO 36.0f

/************************电机零点校准*************************/
#define IMAGEYAW_ZEROPOINT 0.0f
#define IMAGEPITCH_ZEROPOINT 0.0f

/**
 * @brief 图传部分的电机、编码器及pid控制初始化
 */
void ImageMotor_Init(void);

/**
 * @brief 图传电机闭环控制
 * @param ImageMotor
 * @param ImageStateData
 */
void ImageMotor_Ctrl(ImageMotor_e ImageMotor,
                     ImageState_Data_s *ImageStateData);

/**
 * @brief 图传部分电机的设定值规划
 * @brief 改变电机AngleSetPlan和SpeedFeedforward的数据
 * @param ImageStateData   电机位姿信息（角度，角速度，期望角度）
 * @param AngleSetPlan
 * @param SpeedFeedforward
 */
void ImageSetPlanning(ImageState_Data_s *ImageStateData,
                      ImageMotor_s *AngleSetPlan,
                      ImageMotor_s *SpeedFeedforward);

/**
 * @brief 将Image读取到的编码器的速度转换成RPM速度
 * @param Spe_encoder
 * @param Speed_rpm
 */
void ImageEncoderSpeTorpm(ImageMotor_s *Spe_encoder,
                          ImageMotor_s *Speed_rpm);

/**
 * @brief 对Image电机角度值及速度值进行滤波
 * @param ImageStateData
 */
void ImageMotDataFilter(ImageState_Data_s *ImageStateData);

/**
 * @brief Image电机转速单位由RPM换算成RAD
 * @param Speed_rpm
 * @param Speed_rad
 */
void ImageSpeedrpmTorad(ImageMotor_s *Speed_rpm,
                        ImageMotor_s *Speed_rad);

/**
 * @brief Image电机转速单位由RAD换算成RPM
 * @param Speed_rad
 * @param Speed_rpm
 */
void ImageSpeedradTorpm(ImageMotor_s *Speed_rad,
                        ImageMotor_s *Speed_rpm);

/**
 * @brief 图传云台零点校准
 * @param ImageAngle_in
 * @param ImageAngle_out
 */
void Image_ZeroAdjustment(ImageMotor_s *ImageAngle_in,
                          ImageMotor_s *ImageAngle_out);

/**
 * @brief 电机位置信息由角度转向弧度
 * @param ImageAngle_in
 * @param Imagerad_out
 */
void Image_angle2rad(ImageMotor_s *ImageAngle_in,
                     ImageMotor_s *Imagerad_out);

/**
 * @brief 图传云台角度限幅
 * @param ImageAngle
 */
void Image_angleLimit(ImageMotor_s *ImageAngle);

/**
 * @brief 图传云台电机数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ImageMotor_DataValid_If(void);

/**
 * @brief 图传云台yaw编码器数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ImageEncoder_DataValid_If(void);

/**
 * @brief 图传电机电机输入电流扭矩计算
 * @param Period          周期 单位：ms
 * @param Amplitude       幅度
 * @param Image_out       Image电机的电流设定输出
 */
void ImageMotorinput_Calculate(float Period,
                               float Amplitude,
                               ImageMotor_s *Image_out);

/**
 * @brief 图传云台堵转对位
 */
void ImageGimbalAlign(void);

/**
 * @brief  获取图传云台电机结构体
 * @author mylj
 * @param  ImageMotor         指定需要获取的电机
 * @return void*              返回指向电机结构体的指针
 */
void *Get_ImageMotor(ImageMotor_e ImageMotor);

/**
 * @brief  获取图传云台编码器结构体
 * @author mylj
 * @param  ImageEncoder         指定需要获取的电机
 * @return void*              返回指向电机结构体的指针
 */
void *Get_ImageEncoder(ImageMotor_e ImageEncoder);

/**
 * @brief 得到图传电机对位情况
 * @return int 1：两个电机对位完成   0：两个电机对位未完成
 */
int Get_ImageMotorAlign(void);

#endif /*__FUNC_IMAGEMOTOR_CTRL_H__*/
