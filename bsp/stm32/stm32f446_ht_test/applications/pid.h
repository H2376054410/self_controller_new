#ifndef __PID_H__
#define __PID_H__

#include <rtthread.h>

typedef struct
{
	float 	kp;					//pid三个参数
	float 	ki;
	float 	kd;

	float 	last_set;			//上次pid的设定值
	float 	set;				//pid的设定值
	float 	err;				//偏差值
	float 	err_old;			//上次偏差值

	float 	i_value;			//现在的积分值（该积分已经乘了ki参数了
	float  	i_limit;			//积分限幅
	rt_int8_t I_Dis;			// 为1时会停止修改I_Value

	float	out;				//pid的输出
	float	out_limit_up;		//输出上限幅
	float	out_limit_down;		//输出下限幅
}pid_t;
//电机PID闭环结构体

extern void pid_init(pid_t *pid,
	float kp, float ki, float kd, 	//pid参数
	float i_limite, 			//积分限幅
	float out_limite_up, 		//输出上限幅
	float out_limite_down);	//输出下限幅
		
extern void PID_Calculate(pid_t* target,float Error);
extern void pid_clear(pid_t* target);
	
#endif
