#include <rtthread.h>

//#include "drv_buzzer.h"
#include "drv_remote.h"
#include "drv_thread.h"
#include "drv_Arm_Solve.h"
#include "drv_canthread.h"

#include "func_Gpio_Ctrl.h"
//#include "func_encoderCail.h"
#include "func_MusicPlayer.h"
#include "func_ArmMotor_Ctrl.h"
#include "func_ImageMotor_Ctrl.h"
#include "func_ChassisMotor_Ctrl.h"

#include "mod_Can_data.h"
#include "mod_RefSystem.h"
#include "mod_RoboStateCtrl.h"

#include "app_ui.h"
#include "app_monitor.h"
#include "app_RoboCtrl.h"
#include "app_RoboRemote_Ctrl.h"
#include "app_KeymouseData_Ctrl.h"
#include "func_sd_master.h"
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10
static rt_thread_t tid1 = RT_NULL;			// can接收线程的
static struct rt_semaphore rx_sem;			/* 用于接收消息的信号量 */
static struct rt_semaphore rx_time;			/*用于定时器的信号量*/
static rt_device_t can_dev;					/* CAN 设备句柄 */
static struct rt_timer timer1;				/*定时器1*/
#define CAN_DEV_NAME "can1" /* CAN 设备名称 */
struct rt_can_msg msg = {0}; /* CAN 消息 */ // 这个是自己设定的一个链表名字
int16_t receive_size;
static void timeout1(void *parameter) // 定时器回调函数
{
	/*释放信号量*/
	rt_sem_release(&rx_time);
}
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
	/* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
	rt_sem_release(&rx_sem);

	return RT_EOK;
}
static void thread1_entry(void *parameter) // can发送线程
{
	rt_size_t size;
	while (1)
	{

	msg.id = 0x1FF;			/* ID  */
	msg.ide = RT_CAN_STDID; /* 标准格式 */
	msg.rtr = RT_CAN_DTR;	/* 数据帧 */
	msg.len = 8;			/* 数据长度为 2 */
	/* 待发送的pwm数据 */
		
//*(float*)(&msg.data[0])= 0.1;
//msg.data[7]= 255;
		msg.data[0]=-0x09;
		msg.data[1]=0x00;
		msg.data[2]=0x09;
		msg.data[3]=0x00;
		msg.data[4]=0x00;
		msg.data[5]=0x00;
		msg.data[6]=0x00;
		msg.data[7]=0x00;

		rt_sem_take(&rx_time, RT_WAITING_FOREVER);
		size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
		if (size == 0)
		{
			rt_kprintf("can dev write data failed!\n");
		}
	}
}
static void can_rx_thread(void *parameter)
{
	rt_err_t res;
	struct rt_can_msg rxmsg = {0};
	rxmsg.len=4;
	/* 设置接收回调函数 */
	rt_device_set_rx_indicate(can_dev, can_rx_call);
	(void)res;

	while (1)
	{
		/* 阻塞等待接收信号量 */
		rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
		/* 从 CAN 读取一帧数据 */
		rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
		receive_size = sizeof(rxmsg);
	}
}
int main(void)
{
	/* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();
    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();
    SCB->CACR |= 1 << 2;
    /*电源指示灯*/
    PowerPL_Init();
    Monitor_Init(2048, 1, 2);
    // /*上电延时*/
    rt_thread_mdelay(100);
    /*CAN初始化*/
//   SPICAN_Init();
		/* set LED0 pin mode to output */

	rt_err_t res;

	rt_thread_t thread;
	//    char can_name[RT_NAME_MAX];
	/* 查找 CAN 设备 */
	can_dev = rt_device_find(CAN_DEV_NAME);
	if (!can_dev)
	{
		rt_kprintf("find %s failed!\n", CAN_DEV_NAME);
		return RT_ERROR;
	}

	/* 初始化 CAN 接收信号量 */
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_time, "rx_time", 0, RT_IPC_FLAG_FIFO);
	/* 以中断接收及发送方式打开 CAN 设备 */
	res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);

	/* 初始化定时器 */
	rt_timer_init(&timer1, "timer1",	   /* 定时器名字是 timer1 */
				  timeout1,				   /* 超时时回调的处理函数 */
				  RT_NULL,				   /* 超时函数的入口参数 */
				  1,					   /* 定时长度，以 OS Tick 为单位，即 10 个 OS Tick */
				  RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */

	rt_timer_start(&timer1);

	/* 创建数据接收线程 */
	thread = rt_thread_create("can_rx", can_rx_thread, RT_NULL, 1024, 25, 1);
	if (thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
	else
	{
		rt_kprintf("create can_rx thread failed!\n");
	}
	tid1 = rt_thread_create("thread1",
							thread1_entry, RT_NULL,
							THREAD_STACK_SIZE,
							THREAD_PRIORITY, 1);
	/* 如 果 获 得 线 程 控 制 块， 启 动 这 个 线 程 */
	if (tid1 != RT_NULL)
		rt_thread_startup(tid1);
	
    rt_thread_mdelay(50);
    /*电机初始化*/
    ArmMotor_Init();
    Send_Slave2_Init();

    /*上电延时*/
    rt_thread_mdelay(50);

    /*遥控相关初始化*/
    // 串口初始化
//    remote_uart_init();
//    // 遥控器键鼠接收线程
//    KeymouseData_Thread_Init();
//    // 遥控器键鼠数据处理线程
//    RoboremoteCtrl_Init();

    // /*控制线程初始化*/
    ArmCtrl_Thread_Init();

    return RT_EOK;
}
