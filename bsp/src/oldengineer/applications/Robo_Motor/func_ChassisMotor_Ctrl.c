/**
 * @file func_ChassisMotor_Ctrl.c
 * @brief 工程机器人电机控制
 * @brief 主要是电机和pid初始化，以及电机对位
 * @author mylj
 * @version 1.0
 * @date 2022-12-29
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */

#include "func_ChassisMotor_Ctrl.h"
#include "drv_motor.h"
#include "drv_utils.h"
#include "drv_Chassis_Solve.h"

/*电机相关结构体*/
Motor_t wheel[WHEELS_NUM];
Chassis_AcceClamp_e ChassisAcceLimit_s;

pid_t angfol_pid = PID_INIT(4000, 0, 1000, 0, 100000, -100000); // 底盘跟随pid

static uint8_t Wheels_Motor_Offline = 0; // 用于记录底盘电机离线情况,
                                         // 从低到高分别为：右前、左前、左后、右后

#define CHASSISMOTOROUTCLOSE_ENABLE 0

/**
 * @brief   车轮电机初始化
 * @param   None
 * @return  None
 */
void Wheels_Motors_Init(void)
{

    /*初始化电机结构体*/
    /*根据电机安装位置，修正速度方向*/
    motor_init(&wheel[WHEEL_RF], 0,
               CHASSISMOTOR_RATIO, ANGLE_CTRL_EXTRA,
               8192, 180, -180, 0);
    motor_init(&wheel[WHEEL_LF], 0,
               CHASSISMOTOR_RATIO, ANGLE_CTRL_EXTRA,
               8192, 180, -180, 0);
    motor_init(&wheel[WHEEL_LB], 0,
               CHASSISMOTOR_RATIO, ANGLE_CTRL_EXTRA,
               8192, 180, -180, 1);
    motor_init(&wheel[WHEEL_RB], 0,
               CHASSISMOTOR_RATIO, ANGLE_CTRL_EXTRA,
               8192, 180, -180, 1);

    /*pid初始化*/
#if CHASSISMOTOROUTCLOSE_ENABLE
    pid_init(&wheel[WHEEL_RF].spe,
             0, 0, 0, 0, 0, 0);
    pid_init(&wheel[WHEEL_LF].spe,
             0, 0, 0, 0, 0, 0);
    pid_init(&wheel[WHEEL_LB].spe,
             0, 0, 0, 0, 0, 0);
    pid_init(&wheel[WHEEL_RB].spe,
             0, 0, 0, 0, 0, 0);
#else
    pid_init(&wheel[WHEEL_RF].spe,
             150, 1, 30, 2000, 16000, -16000);
    pid_init(&wheel[WHEEL_RB].spe,
             150, 1, 30, 2000, 16000, -16000);
    pid_init(&wheel[WHEEL_LB].spe,
             150, 1, 30, 2000, 16000, -16000);
    pid_init(&wheel[WHEEL_LF].spe,
             150, 1, 30, 2000, 16000, -16000);
#endif
}

/**
 * @brief 物理量和单位转化为快转子的m/s
 * @param in
 * @param out
 */
void WheelsUnitCverRpm2standard(ChassisMotor_t *in,
                                ChassisMotor_t *out)
{
    for (int lo = 0; lo < WHEELS_NUM; lo++)
    {

        out->speed[(Wheel_local_e)lo] = in->speed[(Wheel_local_e)lo] /
                                        CHASSISMOTOR_RATIO /
                                        30 * PI * WHEEL_RADIUS;
    }
}

/**
 * @brief 物理量和单位转化为慢转子的rpm
 * @param in
 * @param out
 */
void WheelsUnitCverStandard2Rpm(ChassisMotor_t *in,
                                ChassisMotor_t *out)
{
    for (int lo = 0; lo < WHEELS_NUM; lo++)
    {
        out->speed[(Wheel_local_e)lo] = in->speed[(Wheel_local_e)lo] *
                                        CHASSISMOTOR_RATIO *
                                        3 / PI / WHEEL_RADIUS;
    }
}

/**
 * @brief   修正轮子的当前速度(输出麦轮转速 m/s)
 * @param in
 * @param out
 */
void Wheel_NowSpeed_Revise(ChassisMotor_t *in,
                           ChassisMotor_t *out)
{
    int dir = 1;
    for (int lo = 0; lo < WHEELS_NUM; lo++)
    {
        dir = 1;
        /*根据电机安装朝向，修正速度方向*/
        if (INSTALL_DIR == W_INSIDE)
        {
            dir = -1;
        }
        out->speed[lo] = dir * in->speed[lo];
    }
}

/**
 * @brief   限制轮子的输入设定速度
 * @param   max_limit 最大速度，单位rpm
 * @param   in
 * @param   out
 */
void Wheels_Speed_Limit(float max_limit,
                        ChassisMotor_t *in,
                        ChassisMotor_t *out)
{
    float set_max, rate;

    /*限制输入为正数*/
    max_limit = fabsf(max_limit);

    /*找到4个值的绝对值的最大值*/
    set_max = utils_max_of_4(fabsf(in->speed[WHEEL_RF]),
                             fabsf(in->speed[WHEEL_LF]),
                             fabsf(in->speed[WHEEL_LB]),
                             fabsf(in->speed[WHEEL_RB]));

    if (set_max > max_limit)
    {
        /*同比例缩小速度设定值*/
        rate = max_limit / set_max;
        for (int lo = 0; lo < WHEELS_NUM; lo++)
        {
            out->speed[lo] = rate * in->speed[lo];
        }
    }
}

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
                         float *Speed_out)
{
    rt_int16_t loop;
    /*将error变换到设定值左右的(-180.0°,180.0°]*/
    loop = (rt_int16_t)(now_angle / (2.0f * PI));
    now_angle = now_angle - loop * 2.0f * PI;

    float error = set_angle - now_angle;

    if (error > PI)
    {
        error -= 2.0f * PI;
    }
    else if (error < -PI)
    {
        error += 2.0f * PI;
    }
    error = RAD2DEG_f(error);

    /*pid计算*/
    PID_Calculate(&angfol_pid, error);

    if (RT_TRUE == WNeedReverse)
        *Speed_out = -DEG2RAD_f(angfol_pid.out / 10.0f);
    else
        *Speed_out = DEG2RAD_f(angfol_pid.out / 10.0f);
}

/**
 * @brief 判断电机是否离线
 * @param SpeedNow
 */
void Wheels_Motor_OfflineIf(float SpeedNow[WHEELS_NUM])
{
    for (int lo = 0; lo < WHEELS_NUM; lo++)
    {
        if ((wheel[lo].dji.FreshTick) && (rt_tick_get() - wheel[lo].dji.FreshTick < 200))
        {
            Motor_SpeedPIDCalculate(&wheel[lo], SpeedNow[lo]); // pid计算结果,单位:电流
            Wheels_Motor_Offline &= ~(1 << lo);
        }
        else
        {
            wheel[lo].spe.out = 0;
            Wheels_Motor_Offline |= (1 << lo);
        }
    }
}

/**
 * @brief 底盘电机PID计算
 * @param now
 * @param set
 * @param out
 */
void WheelsMotPID_Cal(ChassisMotor_t *now,
                      ChassisMotor_t *set,
                      ChassisMotor_t *out)
{

    for (int lo = 0; lo < WHEELS_NUM; lo++)
    {
        Motor_Write_SetSpeed_ABS(&wheel[lo], set->speed[lo]);
        Motor_SpeedPIDCalculate(&wheel[lo], now->speed[lo]);
        out->speed[lo] = Motor_Read_OutSpeed(&wheel[lo]);
    }
}

/**
 * @brief 对Boom电机角度值及速度值进行滤波
 * @param BoomStateData
 */
void WheelsMot_SpeedFilter(ChassisMotor_t *in, ChassisMotor_t *out)
{
    static float filter[WHEELS_NUM] = {0.5f, 0.5f, 0.5f, 0.5f}; // 滤波系数

    for (int lo = 0; lo < WHEELS_NUM; lo++)
    {
        out->speed[lo] = UTILS_LP_FAST(out->speed[lo],
                                       in->speed[lo],
                                       filter[lo]); // 滞后滤波
    }
}

/**
 * @brief 底盘电机数据有效性判断
 * @return 有效则返回1 ，无效则返回0
 */
int ChassisMotor_DataValid_If(void)
{
    /*等待电机第一次通信完毕*/
    rt_uint8_t CheckSum = 0, CheckTimes = 0;

    while (1)
    {
        CheckSum = 0;
        for (int lo = 0; lo < (int)WHEELS_NUM; ++lo)
            CheckSum += wheel[lo].dji.Data_Valid;
        if (CheckSum == (int)WHEELS_NUM)
            return 1;
        // 如果只有一个轮子离线也是勉强可以开始控制的
        else if (CheckSum == (int)WHEELS_NUM - 1)
        {
            ++CheckTimes;
            if (CheckTimes > 20)
                return 1;
            rt_thread_mdelay(50);
        }
        else
        {
            return 0;
        }
    }
}

/***************************加速度限幅*************************/

/**
 * @brief 底盘加速度限幅初始化
 */
static int Chassis_AccLimit_Init(void)
{
    Smooth_Acc_Init(&ChassisAcceLimit_s.x,
                    CHASSIS_ACCE_X_ADD,
                    CHASSIS_ACCE_X_DOWN,
                    SM_NORMAL);
    Smooth_Acc_Init(&ChassisAcceLimit_s.y,
                    CHASSIS_ACCE_Y_ADD,
                    CHASSIS_ACCE_Y_DOWN,
                    SM_NORMAL);
    Smooth_Acc_Init(&ChassisAcceLimit_s.w,
                    CHASSIS_ACCE_W_ADD,
                    CHASSIS_ACCE_W_DOWN,
                    SM_NORMAL);

    return 0;
}
INIT_APP_EXPORT(Chassis_AccLimit_Init);

/**
 * @brief 底盘加速度限幅修改设定值
 * @param set
 */
void Chassis_AccLimit_Process(ChassisMotion_t *set)
{
    Smooth_Acc_Process(&ChassisAcceLimit_s.x,
                       &set->vel.x);
    Smooth_Acc_Process(&ChassisAcceLimit_s.y,
                       &set->vel.y);
    Smooth_Acc_Process(&ChassisAcceLimit_s.w,
                       &set->angvel);
}

/**
 * @brief 获取电机结构体
 * @author mylj
 * @param  WheelMotor         指定需要获取的电机
 * @return void*            返回指向滤波器的指针
 */
void *Get_ChassisMotor(Wheel_local_e WheelMotor)
{
    switch (WheelMotor)
    {
    case WHEEL_RF:
        return (void *)&wheel[WHEEL_RF];
    case WHEEL_RB:
        return (void *)&wheel[WHEEL_RB];
    case WHEEL_LB:
        return (void *)&wheel[WHEEL_LB];
    case WHEEL_LF:
        return (void *)&wheel[WHEEL_LF];
    default:
        return NULL;
    }
}
