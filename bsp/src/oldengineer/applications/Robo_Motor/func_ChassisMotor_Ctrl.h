#ifndef __FUNC_CHASSISMOTOR_CTRL_H
#define __FUNC_CHASSISMOTOR_CTRL_H

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_Vector.h"
#include "drv_AccClamp.h"
#include "drv_Chassis_Solve.h"

#define WHEEL_RADIUS 0.075f      // 麦轮半径 单位米
#define RATED_SPEED 3            // 车额定转速m/s
#define CURRENT_LIMIT 12288.0f   // 最大电流限制，对应实际电流15A，单位 /16384*20 A
#define INSTALL_DIR W_OUTSIDE    // 所有电机安装方向
#define CHASSISMOTOR_RATIO 19.2f // 底盘电机减速比

#define CHASSIS_ACCE_X_ADD 0.003f  // 加速加速度
#define CHASSIS_ACCE_X_DOWN 0.002f // 减速加速度
#define CHASSIS_ACCE_Y_ADD 0.003f
#define CHASSIS_ACCE_Y_DOWN 0.002f
#define CHASSIS_ACCE_W_ADD 30.0f
#define CHASSIS_ACCE_W_DOWN 30.0f

typedef enum
{
    W_OUTSIDE, // 输出轴朝外
    W_INSIDE   // 输出轴朝内

} Wheel_install_e;

typedef struct
{
    ChassisMotor_t set;        // 设定值
    ChassisMotor_t now;        // 当前值
    ChassisMotor_t filter_now; // 滤波后的当前值
    ChassisMotor_t out;        // 电流设定值
} Chassis_t;

typedef struct
{
    Smooth_t x; // x方向加速度限幅
    Smooth_t y; // y方向加速度限幅
    Smooth_t w; // 角速度加速度限幅
} Chassis_AcceClamp_e;

/**
 * @brief   车轮电机初始化
 * @param   None
 * @return  None
 */
void Wheels_Motors_Init(void);

/**
 * @brief 物理量和单位转化为快转子的m/s
 * @param in
 * @param out
 */
void WheelsUnitCverRpm2standard(ChassisMotor_t *in,
                                ChassisMotor_t *out);

/**
 * @brief 物理量和单位转化为慢转子的rpm
 * @param in
 * @param out
 */
void WheelsUnitCverStandard2Rpm(ChassisMotor_t *in,
                                ChassisMotor_t *out);

/**
 * @brief   修正轮子的当前速度(输出麦轮转速 mm/s)
 * @param in
 * @param out
 */
void Wheel_NowSpeed_Revise(ChassisMotor_t *in,
                           ChassisMotor_t *out);

/**
 * @brief   限制轮子的输入设定速度
 * @param   max_limit 最大速度，单位rpm
 * @param   in
 * @param   out
 */
void Wheels_Speed_Limit(float max_limit,
                        ChassisMotor_t *in,
                        ChassisMotor_t *out);

/**
 * @brief 判断电机是否离线
 * @param SpeedNow
 */
void Wheels_Motor_OfflineIf(float SpeedNow[WHEELS_NUM]);

/**
 * @brief 底盘电机PID计算
 * @param now
 * @param set
 * @param out
 */
void WheelsMotPID_Cal(ChassisMotor_t *now,
                      ChassisMotor_t *set,
                      ChassisMotor_t *out);

/**
 * @brief 对Boom电机角度值及速度值进行滤波
 * @param BoomStateData
 */
void WheelsMot_SpeedFilter(ChassisMotor_t *in, ChassisMotor_t *out);

/**
 * @brief 底盘电机数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ChassisMotor_DataValid_If(void);

/***************************加速度限幅*************************/
/**
 * @brief 底盘加速度限幅修改设定值
 * @param set
 */
void Chassis_AccLimit_Process(ChassisMotion_t *set);

/**
 * @brief 获取电机结构体
 * @author mylj
 * @param  WheelMotor         指定需要获取的电机
 * @return void*            返回指向滤波器的指针
 */
void *Get_ChassisMotor(Wheel_local_e WheelMotor);

/**
 * @brief  跟随角pid计算模块
 * @param {float} set_angle 设定角度值，单位 弧度 范围(-180~180°]
 * @param {float} now_a ngle 当前yaw角度值，单位 弧度 范围(-180~180°]
 * @param {rt_bool_t} WNeedReverse 当认为云台逆时针转动角度为正增大时则需要reverse,
 *        解释当set=0,now=30,则底盘需要逆时针转动,0-30=-30,因此需要reverse
 * @param Speed_out {*}角速度：单位1rad /s
 */
void MotModule_FollowPid(float set_angle,
                         float now_angle,
                         rt_bool_t WNeedReverse,
                         float *Speed_out);

/**
 * @brief  机械臂遥控数据数据包序列号读取
 */
int ChassisCtrl_Datanum_Find(void);

#endif /*__FUNC_CHASSISMOTOR_CTRL_H*/
