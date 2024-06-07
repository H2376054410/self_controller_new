#ifndef __PID_H__
#define __PID_H__

#include <rtthread.h>

typedef struct
{
	float 	kp;					//pid��������
	float 	ki;
	float 	kd;

	float 	last_set;			//�ϴ�pid���趨ֵ
	float 	set;				//pid���趨ֵ
	float 	err;				//ƫ��ֵ
	float 	err_old;			//�ϴ�ƫ��ֵ

	float 	i_value;			//���ڵĻ���ֵ���û����Ѿ�����ki������
	float  	i_limit;			//�����޷�
	rt_int8_t I_Dis;			// Ϊ1ʱ��ֹͣ�޸�I_Value

	float	out;				//pid�����
	float	out_limit_up;		//������޷�
	float	out_limit_down;		//������޷�
}pid_t;
//���PID�ջ��ṹ��

extern void pid_init(pid_t *pid,
	float kp, float ki, float kd, 	//pid����
	float i_limite, 			//�����޷�
	float out_limite_up, 		//������޷�
	float out_limite_down);	//������޷�
		
extern void PID_Calculate(pid_t* target,float Error);
extern void pid_clear(pid_t* target);
	
#endif
