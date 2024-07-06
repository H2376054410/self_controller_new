#include "pid.h"

#define ABS(a) (((a)>0)?(a):(-a))

/**
 * @brief  pid参数设定
 * @param  pid：pid结构体指针
 */
void pid_init(pid_t *pid,
			float kp, float ki, float kd,
			float i_limit,
			float 	out_limit_up,
			float 	out_limit_down)
{
	//参数设置
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_limit = i_limit;
	pid->out_limit_up = out_limit_up;
	pid->out_limit_down = out_limit_down;
	//状态归零
	pid->err = 0;
	pid->err_old = 0;
	pid->i_value = 0;
	pid->out = 0;
	pid->set = 0;
	pid->I_Dis = 0;
}

/**
* @brief：该函数对输入的ERROR进行PID计算
* @param [in]	target:要计算的pid结构体
				error:期望值与实际值的误差（单位rpm
* @return：	无
* @author：mqy-ych
*/
void PID_Calculate(pid_t *target, float Error)
{
	target->err_old = target->err;
	target->err = Error;
	if(target->I_Dis==0)
	{
		target->i_value += target->ki * Error;
		//积分限幅
		if (target->i_value < 0)
		{
			if (target->i_value < -target->i_limit)
			{
				target->i_value = -target->i_limit;
			}
		}
		else
		{
			if (target->i_value > target->i_limit)
			{
				target->i_value = target->i_limit;
			}
		}
	}

	target->out = target->kp * Error + target->i_value + target->kd * (target->err - target->err_old);

	//输出限幅
	if(target->out > target->out_limit_up)
	{
		target->out = target->out_limit_up;
	}
	else if(target->out < target->out_limit_down)
	{
		target->out = target->out_limit_down;
	}
}

/**
* @brief：清除PID的状态
* @param [in]	target:要清零的PID结构体指针
* @return：		无
* @author：mqy
*/
void pid_clear(pid_t* target)
{
	//状态归零
	target->err = 0;
	target->err_old = 0;
	target->i_value = 0;
	target->out = 0;
	target->set = 0;
	target->I_Dis = 0;
}
