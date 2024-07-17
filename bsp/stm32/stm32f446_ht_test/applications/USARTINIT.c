#include "USARTINIT.h"
#define UART_NAME1       "uart1"  /* 串口设备名称 */
#define UART_NAME2       "uart2"  /* 串口设备名称 */
#define UART_NAME3       "uart3"  /* 串口设备名称 */
#define UART_NAME4       "uart4"  /* 串口设备名称 */
#define UART_NAME5       "uart5"  /* 串口设备名称 */
static struct rt_semaphore rx_sem_u1;
static struct rt_semaphore rx_sem_u2;
static struct rt_semaphore rx_sem_u3;
static struct rt_semaphore rx_sem_u4;
static struct rt_semaphore rx_sem_u5;
static rt_device_t serial_u1;
static rt_device_t serial_u2;
static rt_device_t serial_u3;
static rt_device_t serial_u4;
static rt_device_t serial_u5;
uint8_t top1;
uint8_t top2;
uint8_t top3;
uint8_t top5;
float angle1;
float angle2;
float angle3;
float angle5;
/* 消息队列控制块 */
static struct rt_messagequeue rx_mq1;
static struct rt_messagequeue rx_mq2;
static struct rt_messagequeue rx_mq3;
static struct rt_messagequeue rx_mq4;
static struct rt_messagequeue rx_mq5;
uint8_t rx_buffer1[RT_SERIAL_RB_BUFSZ + 1];
uint8_t rx_buffer2[RT_SERIAL_RB_BUFSZ + 1];
uint8_t rx_buffer3[RT_SERIAL_RB_BUFSZ + 1];
uint8_t rx_buffer4[RT_SERIAL_RB_BUFSZ + 1];
uint8_t rx_buffer5[RT_SERIAL_RB_BUFSZ + 1];
uint8_t sendbuff[39]={0};
int data_length = 30;
int cmd_id = 0x0302;
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10

static struct rt_timer time_usart;

float angle_tranform(uint8_t *buff)
{
  uint32_t temp=0;
  for(int i=0;i<=3;i++)
    ((uint8_t*)(&temp))[i]=buff[3-i];
  return temp/262144.0f*360.0f;
}
/* 串口接收数据回调函数 */
static rt_err_t uart_input1(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq1, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
/* 串口接收数据回调函数 */
static rt_err_t uart_input2(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq2, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
static rt_err_t uart_input3(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq3, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
static rt_err_t uart_input4(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq4, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
static rt_err_t uart_input5(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq5, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
//串口线程
static void usart1_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq1, &msg, sizeof(msg), 200);
        if (result == RT_EOK)
        {
					  /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer1, msg.size);
						if(top1<(uint8_t)(top1+9)&&rx_length-top1==9)
						{
							if(rx_buffer1[top1]==0x00&&rx_buffer1[top1+1]==0x03&&rx_buffer1[top1+2]==0x04)
								angle1=angle_tranform(&rx_buffer1[(uint8_t)(top1+3)]);
						}
						top1=0;
				}
	   }
}
static void usart2_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq2, &msg, sizeof(msg), 200);
        if (result == RT_EOK)
        {
					  /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer2, msg.size);
						if(top2<(uint8_t)(top2+9)&&rx_length-top2==9)
						{
							if(rx_buffer2[top2]==0x00&&rx_buffer2[top2+1]==0x03&&rx_buffer2[top2+2]==0x04)
								angle2=angle_tranform(&rx_buffer2[(uint8_t)(top2+3)]);
						}
						top1=0;
				}
	   }
}
static void usart3_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq3, &msg, sizeof(msg), 200);
        if (result == RT_EOK)
        {
					  /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer3, msg.size);
						if(top3<(uint8_t)(top3+9)&&rx_length-top3==9)
						{
							if(rx_buffer3[top3]==0x00&&rx_buffer3[top3+1]==0x03&&rx_buffer3[top3+2]==0x04)
								angle3=angle_tranform(&rx_buffer3[(uint8_t)(top3+3)]);
						}
						top3=0;
				}
	   }
}
static void usart4_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
	  uint16_t temp=0;
    rt_uint32_t rx_length;
    while (1)
    {
		rt_sem_take(&rx_sem_u4, RT_WAITING_FOREVER);
		
			
		sendbuff[7]=1;
    sendbuff[8]=((uint8_t*)&angle1)[0];
    sendbuff[9]=((uint8_t*)&angle1)[1];
    sendbuff[10]=((uint8_t*)&angle1)[2];
    sendbuff[11]=((uint8_t*)&angle1)[3];

    sendbuff[12]=((uint8_t*)&angle2)[0];
    sendbuff[13]=((uint8_t*)&angle2)[1];
    sendbuff[14]=((uint8_t*)&angle2)[2];
    sendbuff[15]=((uint8_t*)&angle2)[3];

    sendbuff[16]=((uint8_t*)&angle1)[0];
    sendbuff[17]=((uint8_t*)&angle1)[1];
    sendbuff[18]=((uint8_t*)&angle1)[2];
    sendbuff[19]=((uint8_t*)&angle1)[3];

    sendbuff[20]=((uint8_t*)&angle1)[0];
    sendbuff[21]=((uint8_t*)&angle1)[1];
    sendbuff[22]=((uint8_t*)&angle1)[2];
    sendbuff[23]=((uint8_t*)&angle1)[3];

    sendbuff[24]=((uint8_t*)&angle1)[0];
    sendbuff[25]=((uint8_t*)&angle1)[1];
    sendbuff[26]=((uint8_t*)&angle1)[2];
    sendbuff[27]=((uint8_t*)&angle1)[3];

    sendbuff[28]=((uint8_t*)&angle1)[0];
    sendbuff[29]=((uint8_t*)&angle1)[1];
    sendbuff[30]=((uint8_t*)&angle1)[2];
    sendbuff[31]=((uint8_t*)&angle1)[3];

    sendbuff[32] = 0;
    sendbuff[33] = 0;
    sendbuff[34] = 0;
    sendbuff[35] = 0;
    sendbuff[36] = 0;
    temp=Get_CRC16_Check_Sum(sendbuff,37,CRC_INIT);
    sendbuff[38]=temp>>8;
    sendbuff[37]=temp;
		rt_device_write(serial_u4, 0, &sendbuff, sizeof(sendbuff));
	   }
}
static void usart5_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq5, &msg, sizeof(msg), 200);
        if (result == RT_EOK)
        {
					  /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer5, msg.size);
						if(top5<(uint8_t)(top5+9)&&rx_length-top5==9)
						{
							if(rx_buffer5[top5]==0x00&&rx_buffer5[top5+1]==0x03&&rx_buffer5[top5+2]==0x04)
								angle5=angle_tranform(&rx_buffer5[(uint8_t)(top5+3)]);
						}
						top2=0;
				}
	   }
}
static void timeout_usart(void *parameter) // 定时器回调函数
{
	/*释放信号量*/
	rt_sem_release(&rx_sem_u4);
}
static char msg_pool1[256];
static char msg_pool2[256];
static char msg_pool3[256];
static char msg_pool4[256];
static char msg_pool5[256];	
void uart_init(void)
{
  char str[] = "hello RT-Thread!\r\n";
	char str1[] = "AT+MRATE=100\r\n";
	char str2[] = "AT+MODE=1\r\n";
	char str3[] = "AT+PRATE=100\r\n";
	sendbuff[0]=0xA5;
  sendbuff[1]=(int16_t)(data_length);;
  sendbuff[2]=(int16_t)(data_length)>>8;
  sendbuff[3]=0x5C;
  sendbuff[4]=Get_CRC8_Check_Sum(sendbuff,4,CRC8_INIT);
  sendbuff[5]=(int16_t)(cmd_id);
  sendbuff[6]=(int16_t)(cmd_id)>>8;
	rt_thread_t thread1;
	rt_thread_t thread2;
	rt_thread_t thread3;
	rt_thread_t thread4;
	rt_thread_t thread5;	
	serial_u1 = rt_device_find(UART_NAME1);
  serial_u2 = rt_device_find(UART_NAME2);
	serial_u3 = rt_device_find(UART_NAME3);
	serial_u4 = rt_device_find(UART_NAME4);
	serial_u5 = rt_device_find(UART_NAME5);
  struct serial_configure config1 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
	struct serial_configure config2 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
  struct serial_configure config3 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
  struct serial_configure config4 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */	
	struct serial_configure config5 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */	
		if (!serial_u1)
  {
        rt_kprintf("find %s failed!\n", UART_NAME1);
  }
		if (!serial_u2)
  {
        rt_kprintf("find %s failed!\n", UART_NAME2);
  }	
			if (!serial_u3)
  {
        rt_kprintf("find %s failed!\n", UART_NAME3);
  }
			if (!serial_u4)
  {
        rt_kprintf("find %s failed!\n", UART_NAME4);
  }
			if (!serial_u5)
  {
        rt_kprintf("find %s failed!\n", UART_NAME5);
  }
	
    config1.baud_rate = 9600;      // 修改波特率为 9600
    config1.data_bits = DATA_BITS_8; // 数据位 9
    config1.stop_bits = STOP_BITS_1; // 停止位 1
    config1.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config1.parity = PARITY_NONE;    
    rt_device_control(serial_u1, RT_DEVICE_CTRL_CONFIG, &config1);
	
    config2.baud_rate = 9600;      // 修改波特率为 9600
    config2.data_bits = DATA_BITS_8; // 数据位 9
    config2.stop_bits = STOP_BITS_1; // 停止位 1
    config2.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config2.parity = PARITY_NONE;    
    rt_device_control(serial_u2, RT_DEVICE_CTRL_CONFIG, &config2);
	
    config3.baud_rate = 9600;      // 修改波特率为 9600
    config3.data_bits = DATA_BITS_8; // 数据位 9
    config3.stop_bits = STOP_BITS_1; // 停止位 1
    config3.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config3.parity = PARITY_NONE;    
    rt_device_control(serial_u3, RT_DEVICE_CTRL_CONFIG, &config3);
	
	
    config4.baud_rate = 9600;      // 修改波特率为 9600
    config4.data_bits = DATA_BITS_8; // 数据位 9
    config4.stop_bits = STOP_BITS_1; // 停止位 1
    config4.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config4.parity = PARITY_NONE;    
    rt_device_control(serial_u4, RT_DEVICE_CTRL_CONFIG, &config4);	

    config5.baud_rate = 9600;      // 修改波特率为 9600
    config5.data_bits = DATA_BITS_8; // 数据位 9
    config5.stop_bits = STOP_BITS_1; // 停止位 1
    config5.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config5.parity = PARITY_NONE;    
    rt_device_control(serial_u5, RT_DEVICE_CTRL_CONFIG, &config5);	
		
	  rt_sem_init(&rx_sem_u1, "rx_usart1", 0, RT_IPC_FLAG_FIFO);
	  rt_sem_init(&rx_sem_u2, "rx_usart2", 0, RT_IPC_FLAG_FIFO);
	  rt_sem_init(&rx_sem_u3, "rx_usart3", 0, RT_IPC_FLAG_FIFO);
	  rt_sem_init(&rx_sem_u4, "rx_usart4", 0, RT_IPC_FLAG_FIFO);
	  rt_sem_init(&rx_sem_u5, "rx_usart5", 0, RT_IPC_FLAG_FIFO);
	    /* 初始化消息队列 */
    rt_mq_init(&rx_mq1, "rx_mq1",
               msg_pool1,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool1),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
		rt_mq_init(&rx_mq2, "rx_mq2",
               msg_pool2,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool2),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    rt_mq_init(&rx_mq3, "rx_mq3",
               msg_pool3,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool3),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    rt_mq_init(&rx_mq4, "rx_mq4",
               msg_pool4,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool4),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    rt_mq_init(&rx_mq5, "rx_mq5",
               msg_pool5,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool5),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
  /* 以DMA接收及轮询发送模式打开串口设备 */
    rt_device_open(serial_u1, RT_DEVICE_FLAG_DMA_RX);
    rt_device_open(serial_u2, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u3, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u4, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u5, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial_u1, uart_input1);
    rt_device_set_rx_indicate(serial_u2, uart_input2);
	  rt_device_set_rx_indicate(serial_u3, uart_input3);
	  rt_device_set_rx_indicate(serial_u4, uart_input4);
		rt_device_set_rx_indicate(serial_u5, uart_input5);

	rt_device_write(serial_u1, 0, str1, (sizeof(str1)-1));

	rt_device_write(serial_u2, 0, str1, (sizeof(str1)-1));
	rt_device_write(serial_u4, 0, str, (sizeof(str)-1));
							 	rt_device_write(serial_u1, 0, str, (sizeof(str)-1));

	    /* 创建 serial 线程 */
   thread1 = rt_thread_create("u1_thread", usart1_thread_entry, RT_NULL, 1024, 25, 10);
   thread2 = rt_thread_create("u2_thread", usart2_thread_entry, RT_NULL, 1024, 25, 10);
	 thread3 = rt_thread_create("u3_thread", usart3_thread_entry, RT_NULL, 1024, 25, 10);
	 thread4 = rt_thread_create("u4_thread", usart4_thread_entry, RT_NULL, 1024, 25, 10);
	 thread5 = rt_thread_create("u5_thread", usart5_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread1 != RT_NULL)
    {
        rt_thread_startup(thread1);
    }
    if (thread2 != RT_NULL)
    {
        rt_thread_startup(thread2);
    }
		    if (thread3 != RT_NULL)
    {
        rt_thread_startup(thread3);
    }
		    if (thread4 != RT_NULL)
    {
        rt_thread_startup(thread4);
    }
		    if (thread5 != RT_NULL)
    {
        rt_thread_startup(thread5);
    }
		
			/* 初始化定时器 */
	rt_timer_init(&time_usart, "timer_usart",	   /* 定时器名字是 timer1 */
				  timeout_usart,				   /* 超时时回调的处理函数 */
				  RT_NULL,				   /* 超时函数的入口参数 */
				  1,					   /* 定时长度，以 OS Tick 为单位，即 10 个 OS Tick */
				  RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */

	rt_timer_start(&time_usart);
}
	

