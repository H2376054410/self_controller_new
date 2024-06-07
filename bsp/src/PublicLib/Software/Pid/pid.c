#include "pid.h"

/**
 * @brief  pid参数设定
 * @param  pid：pid结构体指针
 */
void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float i_limit,
              float out_limit_up,
              float out_limit_down)
{
    // 输入参数检查
    if (RT_NULL == pid)
        return;
    // 参数逻辑检查
    if (out_limit_up < out_limit_down)
        return;
    // 参数设置
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_limit_up = out_limit_up;
    pid->out_limit_down = out_limit_down;
    // 状态归零
    pid->last_set = 0;
    pid->err = 0;
    pid->err_old = 0;
    pid->i_value = 0;
    pid->out = 0;
    pid->set = 0;
    pid->I_Dis = 0;
    pid->d_filter_enable = 0;
    pid->err_deadzoneflag = 0;
}

/**
 * @brief  开启 pid 的微分项滤波器（不完全微分）
 * @author fwlh
 * @param  pid                待操作的 PID 句柄
 * @param  d_filter_constant  准备设置的微分项滤波参数
 * ! 注意该参数应当在 [0, 1] 之间, 0 代表不做滤波
 */
void pid_config_d_filter(pid_t *pid, float d_filter_constant)
{
    // 输入参数检查
    if (RT_NULL == pid)
        return;
    // 滤波参数必须在合理范围内
    if ((d_filter_constant < 1.f) && (d_filter_constant > 0.f))
    {
        pid->d_filter_enable = 1;
        pid->d_filter_constant = d_filter_constant;
    }
}

/**
* @brief：该函数对输入的ERROR进行PID计算
* @param [in]	target:要计算的pid结构体
                error:期望值与实际值的误差（单位rpm
* @return：	无
* @author：mqy-ych-fwlh
*/
void PID_Calculate(pid_t *target, float Error)
{
    // 输入参数检查
    if (RT_NULL == target)
        return;

    if (target->err_deadzoneflag)
    {
        if (Error < target->err_deadzone ||
            Error > -target->err_deadzone)
        {
            Error = 0;
        }
        else
        {
            if (Error > target->err_deadzone)
            {
                Error -= target->err_deadzone;
            }
            else if (Error < -target->err_deadzone)
            {
                Error += target->err_deadzone;
            }
        }
    }

    target->err_old = target->err;
    target->err = Error;

    // 计算积分项
    if (target->I_Dis == 0)
    {
        target->i_value += target->ki * Error;
        // 积分限幅
        if (target->i_value < 0)
        {
            if (target->i_value < -target->i_limit)
                target->i_value = -target->i_limit;
        }
        else
        {
            if (target->i_value > target->i_limit)
                target->i_value = target->i_limit;
        }
    }

    // 计算微分项
    if (target->d_filter_enable)
    {
        float temp_d = target->kd * (Error - target->err_old);
        target->d_now = temp_d * (1.f - target->d_filter_constant) +
                        target->d_last * target->d_filter_constant;
        target->d_last = target->d_now;
    }
    else
        target->d_now = target->kd * (Error - target->err_old);

    // 计算输出
    target->out = target->kp * Error + target->i_value + target->d_now;
    // 输出限幅
    if (target->out > target->out_limit_up)
        target->out = target->out_limit_up;
    else if (target->out < target->out_limit_down)
        target->out = target->out_limit_down;
}

/**
 * @brief pid误差死区设定
 * @param target
 * @param errdeadzone_flag
 * @param errdeadzone
 */
void pid_ErrorDeadzone_Set(pid_t *target,
                           rt_uint8_t errdeadzone_flag,
                           float errdeadzone)
{
    if (errdeadzone_flag == 0)
    {
        target->err_deadzoneflag = 0;
    }
    if (errdeadzone < 0)
    {
        while (1)
            ;
    }
    else
    {
        target->err_deadzoneflag = 1;
        target->err_deadzone = errdeadzone;
    }
}

/**
 * @brief：清除PID的状态
 * @param [in]	target:要清零的PID结构体指针
 * @return：		无
 * @author：mqy-fwlh
 */
void pid_clear(pid_t *target)
{
    // 输入参数检查
    if (RT_NULL == target)
        return;
    // 状态归零
    target->err = 0;
    target->err_old = 0;
    target->i_value = 0;
    target->out = 0;
    target->set = 0;
    target->I_Dis = 0;
    target->d_last = 0;
}
