#include "pid.h"

#define ABS(a) (((a)>0)?(a):(-a))

/**
 * @brief  pid�����趨
 * @param  pid��pid�ṹ��ָ��
 */
void pid_init(pid_t *pid,
			float kp, float ki, float kd,
			float i_limit,
			float 	out_limit_up,
			float 	out_limit_down)
{
	//��������
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_limit = i_limit;
	pid->out_limit_up = out_limit_up;
	pid->out_limit_down = out_limit_down;
	//״̬����
	pid->err = 0;
	pid->err_old = 0;
	pid->i_value = 0;
	pid->out = 0;
	pid->set = 0;
	pid->I_Dis = 0;
}

/**
* @brief���ú����������ERROR����PID����
* @param [in]	target:Ҫ�����pid�ṹ��
				error:����ֵ��ʵ��ֵ������λrpm
* @return��	��
* @author��mqy-ych
*/
void PID_Calculate(pid_t *target, float Error)
{
	target->err_old = target->err;
	target->err = Error;
	if(target->I_Dis==0)
	{
		target->i_value += target->ki * Error;
		//�����޷�
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

	//����޷�
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
* @brief�����PID��״̬
* @param [in]	target:Ҫ�����PID�ṹ��ָ��
* @return��		��
* @author��mqy
*/
void pid_clear(pid_t* target)
{
	//״̬����
	target->err = 0;
	target->err_old = 0;
	target->i_value = 0;
	target->out = 0;
	target->set = 0;
	target->I_Dis = 0;
}
