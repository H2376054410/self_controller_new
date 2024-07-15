#include "CONTROL.h"
#include "CANINIT.h"
static struct rt_timer timer_control;				
struct rt_semaphore control_sem;
BoomMotor_s Boom_States;
BoomState_Data_s Boom_Datas;
BoomMotor_s  Boom_Compension;

static void Boom_Control_Thread(void *parameter)
{
Boom_States.BoomLeft=0;
Boom_States.BoomRight=0;
Boom_States.BoomYaw=0;	
	while(1)
	{
				rt_sem_take(&control_sem, RT_WAITING_FOREVER);

		//数据读取
		Boom_Datas.AngleNow.BoomLeft = rec_data_s.AngleNowBOOM_LEFT;
		Boom_Datas.AngleNow.BoomRight = rec_data_s.AngleNowBOOM_RIGHT;
		Boom_Datas.AngleNow.BoomYaw =  rec_data_s.AngleNowBOOM_YAW ;		
		Boom_Datas.SpeedNow.BoomLeft = rec_data_s.SpeedNowBOOM_LEFT;
		Boom_Datas.SpeedNow.BoomRight= rec_data_s.SpeedNowBOOM_RIGHT;
		Boom_Datas.SpeedNow.BoomYaw =  rec_data_s.SpeedNowBOOM_YAW ;		
		Boom_Datas.AngleSetPlan.BoomLeft = 150.0f;
		Boom_Datas.AngleSetPlan.BoomRight = 150.0f;	
		Boom_Datas.AngleSetPlan.BoomYaw = 150.0f;
		BoomMotDataFilter(&Boom_Datas);
		//开始控制
		
		
		BoomMotor_Ctrl(BoomLeft,&Boom_Datas);
		BoomMotor_Ctrl(BoomRight,&Boom_Datas);
		BoomMotor_Ctrl(BoomYaw,&Boom_Datas);		
		ArmMotorinput_Calculate(&Boom_Compension,&Boom_Datas.MotorCtrl_Out);
//		Boom_States.BoomLeft=Boom_Datas.MotorCtrl_Out.BoomLeft;
//		Boom_States.BoomRight=Boom_Datas.MotorCtrl_Out.BoomRight;
//		Boom_States.BoomYaw=Boom_Datas.MotorCtrl_Out.BoomYaw;	
		Boom_States.BoomLeft=-30;
    Boom_States.BoomRight=20;
    Boom_States.BoomYaw=20;	
	  can_save_handle(&Boom_States);
	}
	
}


static void control_timeout(void *parameter) // 定时器回调函数
{
	/*释放信号量*/
	rt_sem_release(&control_sem);
}

void control_init(void)
{
rt_thread_t ctrl_thread;
rt_sem_init(&control_sem, "control_time", 0, RT_IPC_FLAG_FIFO);
ctrl_thread = rt_thread_create("Boom_Control", Boom_Control_Thread, RT_NULL, 2048, 10, 5);	
if (ctrl_thread != RT_NULL)
	{
		rt_thread_startup(ctrl_thread);
	}	else
	{
		rt_kprintf("create ctrl_thread failed!\n");
	}
	/* 初始化定时器 */
	rt_timer_init(&timer_control, "timer_Control",	   /* 定时器名字是 timer1 */
				  control_timeout,				   /* 超时时回调的处理函数 */
				  RT_NULL,				   /* 超时函数的入口参数 */
				  1,					   /* 定时长度，以 OS Tick 为单位，即 10 个 OS Tick */
				  RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */

	rt_timer_start(&timer_control);


}





