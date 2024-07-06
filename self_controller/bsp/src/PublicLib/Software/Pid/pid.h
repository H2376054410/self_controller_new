#ifndef __PID_H__
#define __PID_H__

#include <rtdef.h>

typedef struct
{
    // pid三个参数
    float kp;
    float ki;
    float kd;

    // PID 状态参数
    float set;                   // pid 的设定值
    float last_set;              // 上一次设定值
    float err;                   // 偏差值
    float err_deadzone;          // 偏差值死区
    rt_uint8_t err_deadzoneflag; // 偏差值死区标志位
    float err_old;               // 上次偏差值

    // 微分相关参数
    float d_now;                // 这一次的 D 分量
    float d_last;               // 上一次输出的 D 分量
    rt_uint8_t d_filter_enable; // 为 1 时会在计算 D 分量时使用滤波器
    float d_filter_constant;    // D 的输出滤波

    // 积分相关参数
    float i_value;   // 现在的积分值, 该积分已经乘了 ki 参数了
    float i_limit;   // 积分限幅
    rt_int8_t I_Dis; // 为 1 时会停止修改 I_Value

    // 输出相关参数
    float out;            // pid 的输出
    float out_limit_up;   // 输出上限幅
    float out_limit_down; // 输出下限幅
} pid_t;                  // PID 闭环结构体

/**
 * @brief  pid参数设定
 * @param  pid：pid 结构体指针
 */
extern void pid_init(pid_t *pid,
                     float kp, float ki, float kd, // pid 参数
                     float i_limit,                // 积分限幅
                     float out_limit_up,           // 输出上限幅
                     float out_limit_down);        // 输出下限幅

/* pid创建时参数初始化, 但是输入的第一个参数必须是已经定义好的 PID 句柄 */
#define PID_INIT(val_kp, val_ki, val_kd, i_lim, lim_up, lim_down) \
    {                                                             \
        .kp = val_kp,                                             \
        .ki = val_ki,                                             \
        .kd = val_kd,                                             \
        .last_set = 0,                                            \
        .set = 0,                                                 \
        .err = 0,                                                 \
        .err_old = 0,                                             \
        .i_value = 0,                                             \
        .i_limit = i_lim,                                         \
        .I_Dis = 0,                                               \
        .out = 0,                                                 \
        .out_limit_up = lim_up,                                   \
        .out_limit_down = lim_down,                               \
        .d_filter_enable = 0,                                     \
        .err_deadzoneflag = 0                                     \
    }

/**
 * @brief  开启 pid 的微分项滤波器（不完全微分）
 * @author fwlh
 * @param  pid                待操作的 PID 句柄
 * @param  d_filter_constant  准备设置的微分项滤波参数
 * ! 注意该参数应当在 [0, 1] 之间, 0 代表不做滤波
 */
extern void pid_config_d_filter(pid_t *pid, float d_filter_constant);

/**
* @brief：该函数对输入的ERROR进行PID计算
* @param [in]	target:要计算的pid结构体
                error:期望值与实际值的误差（单位rpm
* @return：	无
* @author：mqy-ych-fwlh
*/
extern void PID_Calculate(pid_t *target, float Error);

/**
 * @brief pid误差死区设定
 * @param target
 * @param errdeadzone_flag
 * @param errdeadzone
 */
void pid_ErrorDeadzone_Set(pid_t *target,
                           rt_uint8_t errdeadzone_flag,
                           float errdeadzone);

/**
 * @brief：清除PID的状态
 * @param [in]	target:要清零的PID结构体指针
 * @return：		无
 * @author：mqy-fwlh
 */
extern void pid_clear(pid_t *target);

#endif
