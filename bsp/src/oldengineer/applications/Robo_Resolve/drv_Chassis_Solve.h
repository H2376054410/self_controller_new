#ifndef __FUNC_CHASSIS_SOLVE_H__
#define __FUNC_CHASSIS_SOLVE_H__

#include <rtthread.h>
#include "drv_vector.h"

#define TICK_PROBE_STORAGE(name) static Tick_probe_t name = {0, RT_FALSE}

/*时间探针结构体*/
typedef struct
{
    rt_tick_t last_tick;
    rt_bool_t if_init;
} Tick_probe_t;

/*底盘数据信息*/
#define WHEELS_NUM 4
// 麦轮模式
#define MECANUM_WHEEL 1
// 前后轮间距(m)
#define VEHICLE_LONG 0.400f
// 左右轮间距(m)
#define VEHICLE_WIDTH 0.460f

// 输入限幅参数
#define M_MAX_YSPEED 5     // 底盘最大移动速度，单位m/s
#define M_MAX_XSPEED 5     // 底盘最大移动速度，单位m/s
#define M_MAX_ROTATE_AC 100 // 底盘最大自旋角速度
#define M_MAX_POSITION_Y 1 // 旋转轴坐标最大值，单位m
#define M_MAX_POSITION_X 1 // 旋转轴坐标最大值，单位m
#define M_MAX_REVOLVE_AC 8 // 底盘最大公转角速度，单位rad°/s

typedef struct
{
    VectorXY_Str vel; // 平移速度矢量，单位mm/s
    float angvel;     // 角速度，单位0.1°/s
} ChassisMotion_t;

/*公转运动录制结构体,创建时成员值为0即可*/
typedef struct
{
    VectorXY_Str last_pos;
    float last_angvel;
    float incre_theta;
    Tick_probe_t tick;
} ChassisRecord_t;

typedef enum
{
    WHEEL_RF = 0,
    WHEEL_RB,
    WHEEL_LB,
    WHEEL_LF,
} Wheel_local_e;

typedef struct
{
    float speed[WHEELS_NUM];
} ChassisMotor_t;

typedef struct
{
    float mod;      // 矢量大小
    float xy_theta; // 二维矢量偏角,范围为(-180°,180°]，0°为坐标系的y轴，角度值增大方向为z轴旋转正方向
    VectorXYZ_Str Vector;
} ChassisVector_t;

/**
 * @brief 单位换算，遥控器数值转成国际单位
 * @param VxyW_in
 * @param VxyW_out
 */
void UnitCverRemote2standard(ChassisMotion_t *VxyW_in,
                             ChassisMotion_t *VxyW_out);

/**
 * @brief    只有底盘运动
 * @param    VxyW.vel   底盘xy速度矢量，单位mm/s
 * @param    VxyW.angvel   底盘自转角速度大小，单位0.1°/s
 * @return   输出到Resolve函数的VxyW
 */
void MotPack_Only_Chass(ChassisMotion_t *VxyW_in,
                        ChassisMotion_t *VxyW_out);

/**
 * @brief    偏心的只有底盘运动
 * @param    VxyW.vel        底盘xy速度矢量，单位mm/s
 * @param    VxyW.angvel     底盘自转角速度大小，单位0.1°/s
 * @param    pos             偏心坐标点，单位(mm,mm)
 * @param    out             输出到Resolve函数的VxyW
 */
void MotPack_Offset_OnlyChass(ChassisMotion_t *VxyW,
                              VectorXY_Str *pos,
                              ChassisMotion_t *out);

/**
 * @brief    绕点旋转运动
 * @param    Pxy 偏心坐标点，单位(m,m)
 * @param    dot_angvel 公转角速度大小，单位rad/s
 * @return   输出到Resolve函数的VxyW
 */
void MotPack_Spin_Dot(VectorXY_Str *pos, float dot_angvel,
                      ChassisMotion_t *out);

/**
 * @brief   麦轮/全向底盘运动解算,并输出到wheel层
 * @param   chs.vel     底盘xy方向速度大小,单位mm/s
 * @param   chs.angvel  自转角速度,单位0.1°/s,底盘逆时针旋转为正
 * @return  None
 * @author  lfp
 */
void MecanOmni_Resolve(ChassisMotion_t *chs, float *output);

#endif /*__FUNC_CHASSIS_SOLVE_H__*/
