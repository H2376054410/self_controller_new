#ifndef __DRV_PLLFILTER_H__
#define __DRV_PLLFILTER_H__

#include <rtthread.h>

typedef struct
{
    float Pos;
    float Spe;
} PLLFilter_OutData;

typedef struct
{
    float Kp;                 // 锁相环滤波器中使用的 PI 滤波器的比例参数
    float Ki;                 // 锁相环滤波器中使用的 PI 滤波器的积分参数
    rt_uint8_t CrossCircle_Flag; // 是否需要跨圈计算
    float CircleMax;          // 跨圈计算时的圈内最大数值
    float CircleMin;          // 跨圈计算时的圈内最小数值
} PLLFilter_Settings;

typedef struct
{
    PLLFilter_Settings Settings; // 锁相环滤波器的相关配置参数
    float Pos_Sense;             // 当前输入到滤波器中的位置信息
    float PosErr;                // 滤波器传感到的位置误差
    float Err_Integral;          // 输入误差的积分
    rt_tick_t Last_FreshTick;    // 上一次滤波器计算时的时刻
    PLLFilter_OutData OutData;   // 当前滤波器的输出数据
} PLLFilter_Ctrl_Struct;

/**
 * @brief 初始化锁相环滤波器控制块
 * @author fwlh
 * @param  PLLFilter_Ctrl   待初始化的锁相环滤波器结构体
 * @param  Kp               锁相环滤波器的比例系数
 * @param  Ki               锁相环滤波器的积分系数
 * @param  StartPos         锁相环滤波器的初始位置
 */
extern void PLLFilter_Init(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float Kp, float Ki, float StartPos);

/**
 * @brief 进行锁相环滤波器的跨圈设置
 * @author fwlh
 * @param  PLLFilter_Ctrl   待配置的锁相环滤波器结构体
 * @param  CircleMax        跨圈计算时的圈内最大数值
 * @param  CircleMin        跨圈计算时的圈内最小数值
 */
extern void PLLFilter_CrossCircle_Set(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float CircleMax, float CircleMin);

/**
 * @brief 锁相环滤波器的计算
 * @author fwlh
 * @param  PLLFilter_Ctrl   待计算滤波结果的锁相环滤波器控制块
 * @param  NowPos           当前待滤波的数据
 */
extern void PLLFilter_Calc(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float NowPos);

/**
 * @brief 重置锁相环滤波器
 * @author fwlh
 * @param  PLLFilter_Ctrl   需要重置的锁相环滤波器控制块
 * @param  NowPos           重置以后的当前位置
 */
extern void PLLFilter_Restart(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float NowPos);

#endif /* __DRV_PLLFILTER_H__ */
