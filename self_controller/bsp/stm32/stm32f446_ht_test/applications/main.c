/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <pid.h>
#include <math.h>
/* defined the LED0 pin: PB1 */
#define LED0_PIN GET_PIN(B, 1)
#define CAN_DEV_NAME "can1" /* CAN �豸���� */
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10
int16_t pwm ;
int16_t angle;
int16_t angle2;
int16_t speed;
float speed2;
int16_t speed1;
float angle0;
float target_speed=0,err_s=0,last_angle=0,lvbo_b=0.95;
float target_angle=0,err_a=0,last_speed=0,lvbo_a=0.7;
float kp_a=0.1,kd_a=0,ki_a=0;
float kp_s=100,kd_s=0,ki_s=0;
static rt_thread_t tid1 = RT_NULL;			// can�����̵߳�
static struct rt_semaphore rx_sem;			/* ���ڽ�����Ϣ���ź��� */
static struct rt_semaphore rx_time;			/*���ڶ�ʱ�����ź���*/
static rt_device_t can_dev;					/* CAN �豸��� */
static struct rt_timer timer1;				/*��ʱ��1*/
struct rt_can_msg msg = {0}; /* CAN ��Ϣ */ // ������Լ��趨��һ����������
 pid_t pid_angle;
 pid_t pid_speed;
int16_t set_sp=500;
float kd_temp;
float tor_tmp;


/* �������ݻص����� */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
	/* CAN ���յ����ݺ�����жϣ����ô˻ص�������Ȼ���ͽ����ź��� */
	rt_sem_release(&rx_sem);

	return RT_EOK;
}
int16_t id_ceshi;
int16_t receive_size;
static void can_rx_thread(void *parameter)
{
	rt_err_t res;
	struct rt_can_msg rxmsg = {0};
	rxmsg.len=4;
	/* ���ý��ջص����� */
	rt_device_set_rx_indicate(can_dev, can_rx_call);
	(void)res;

	while (1)
	{
		/* �����ȴ������ź��� */
		rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
		/* �� CAN ��ȡһ֡���� */
		rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
		receive_size = sizeof(rxmsg);
	}
}

static void timeout1(void *parameter) // ��ʱ���ص�����
{
	/*�ͷ��ź���*/
	rt_sem_release(&rx_time);
}
static void thread1_entry(void *parameter) // can�����߳�
{
	rt_size_t size;
	while (1)
	{
#if 0
	msg.id = 0x1FF;			/* ID  */
	msg.ide = RT_CAN_STDID; /* ��׼��ʽ */
	msg.rtr = RT_CAN_DTR;	/* ����֡ */
	msg.len = 8;			/* ���ݳ���Ϊ 2 */
	/* �����͵�pwm���� */
		
//*(float*)(&msg.data[0])= 0.1;
//msg.data[7]= 255;
		msg.data[0]=0x00;
		msg.data[1]=0x00;
		msg.data[2]=0x00;
		msg.data[3]=0x00;
		msg.data[4]=0x00;
		msg.data[5]=0x00;
		msg.data[6]=0x00;
		msg.data[7]=0x00;
#elif 0
	msg.id = 0x7FF;			/* ID  */
	msg.ide = RT_CAN_STDID; /* ��׼��ʽ */
	msg.rtr = RT_CAN_DTR;	/* ����֡ */
	msg.len = 4;			/* ���ݳ���Ϊ 2 */
	/* �����͵�pwm���� */
		
//*(float*)(&msg.data[0])= 0.1;
//msg.data[7]= 255;
		msg.data[0]=0x00;
		msg.data[1]=0x02;
		msg.data[2]=0x00;
		msg.data[3]=0x03;
//		msg.data[4]=0x00;
//		msg.data[5]=0x00;
//		msg.data[6]=0x00;
//		msg.data[7]=0x00;
#else 1
	msg.id = 0x200;			/* ID  */
	msg.ide = RT_CAN_STDID; /* ��׼��ʽ */
	msg.rtr = RT_CAN_DTR;	/* ����֡ */
	msg.len = 8;			/* ���ݳ���Ϊ 2 */
	/* �����͵�pwm���� */
		
//*(float*)(&msg.data[0])= 0.1;
//msg.data[7]= 255;
		msg.data[0]=0x05;
		msg.data[1]=0x10;
		msg.data[2]=0x05;
		msg.data[3]=0x10;
		msg.data[4]=0x05;
		msg.data[5]=0x10;
		msg.data[6]=0x05;
		msg.data[7]=0x10;
#endif		
		rt_sem_take(&rx_time, RT_WAITING_FOREVER);
		size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
		if (size == 0)
		{
			rt_kprintf("can dev write data failed!\n");
		}
	}
}

int main(void)
{

	/* set LED0 pin mode to output */
	rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

	rt_err_t res;

	rt_thread_t thread;
	//    char can_name[RT_NAME_MAX];
	/* ���� CAN �豸 */
	can_dev = rt_device_find(CAN_DEV_NAME);
	if (!can_dev)
	{
		rt_kprintf("find %s failed!\n", CAN_DEV_NAME);
		return RT_ERROR;
	}

	/* ��ʼ�� CAN �����ź��� */
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_time, "rx_time", 0, RT_IPC_FLAG_FIFO);
	/* ���жϽ��ռ����ͷ�ʽ�� CAN �豸 */
	res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
	RT_ASSERT(res == RT_EOK);

	/* ��ʼ����ʱ�� */
	rt_timer_init(&timer1, "timer1",	   /* ��ʱ�������� timer1 */
				  timeout1,				   /* ��ʱʱ�ص��Ĵ����� */
				  RT_NULL,				   /* ��ʱ��������ڲ��� */
				  1,					   /* ��ʱ���ȣ��� OS Tick Ϊ��λ���� 10 �� OS Tick */
				  RT_TIMER_FLAG_PERIODIC); /* �����Զ�ʱ�� */

	rt_timer_start(&timer1);

	/* �������ݽ����߳� */
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
	/* �� �� �� �� �� �� �� �� �飬 �� �� �� �� �� �� */
	if (tid1 != RT_NULL)
		rt_thread_startup(tid1);
	// while (1)
	{
	}
}
