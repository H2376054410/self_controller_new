#include "USARTINIT.h"
#define UART_NAME1       "uart1"  /* 串口设备名称 */
#define UART_NAME3       "uart3"  /* 串口设备名称 */
#define UART_NAME4       "uart4"  /* 串口设备名称 */
static struct rt_semaphore rx_sem_u1;
static struct rt_semaphore rx_sem_u3;
static struct rt_semaphore rx_sem_u4;
static rt_device_t serial_u1;
static rt_device_t serial_u3;
static rt_device_t serial_u4;
/* 消息队列控制块 */
static struct rt_messagequeue rx_mq1;
static struct rt_messagequeue rx_mq3;
static struct rt_messagequeue rx_mq4;
uint8_t rx_buffer1[RT_SERIAL_RB_BUFSZ + 1];
uint8_t rx_buffer3[RT_SERIAL_RB_BUFSZ + 1];
uint8_t rx_buffer4[RT_SERIAL_RB_BUFSZ + 1];
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10
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
//            if (rx_length != 18)
//            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
//                continue;
//            }
					rt_device_write(msg.dev, 0, rx_buffer1, msg.size);
//            rx_buffer1[rx_length] = '\0';

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
            if (rx_length != 18)
            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            rx_buffer3[rx_length] = '\0';

				}
	   }
}
static void usart4_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq4, &msg, sizeof(msg), 200);
        if (result == RT_EOK)
        {
					  /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer4, msg.size);
            if (rx_length != 18)
            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            rx_buffer4[rx_length] = '\0';

				}
	   }
}
void uart_init(void)
{
  char str[] = "hello RT-Thread!\r\n";
	rt_thread_t thread1;
	rt_thread_t thread3;
	rt_thread_t thread4;
    static char msg_pool1[256];
    static char msg_pool3[256];
    static char msg_pool4[256];
	
	
	serial_u1 = rt_device_find(UART_NAME1);
	serial_u3 = rt_device_find(UART_NAME3);
	serial_u4 = rt_device_find(UART_NAME4);
  struct serial_configure config1 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
  struct serial_configure config3 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
  struct serial_configure config4 = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */	
		if (!serial_u1)
  {
        rt_kprintf("find %s failed!\n", UART_NAME1);
  }
			if (!serial_u3)
  {
        rt_kprintf("find %s failed!\n", UART_NAME3);
  }
			if (!serial_u4)
  {
        rt_kprintf("find %s failed!\n", UART_NAME4);
  }

	
    config1.baud_rate = 9600;      // 修改波特率为 9600
    config1.data_bits = DATA_BITS_8; // 数据位 9
    config1.stop_bits = STOP_BITS_1; // 停止位 1
    config1.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config1.parity = PARITY_NONE;    
    rt_device_control(serial_u1, RT_DEVICE_CTRL_CONFIG, &config1);

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
	
	rt_sem_init(&rx_sem_u1, "rx_usart1", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_sem_u3, "rx_usart3", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_sem_u4, "rx_usart4", 0, RT_IPC_FLAG_FIFO);
	    /* 初始化消息队列 */
    rt_mq_init(&rx_mq1, "rx_mq",
               msg_pool1,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool1),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    rt_mq_init(&rx_mq3, "rx_mq",
               msg_pool3,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool3),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
    rt_mq_init(&rx_mq4, "rx_mq",
               msg_pool4,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool4),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */
  /* 以DMA接收及轮询发送模式打开串口设备 */
    rt_device_open(serial_u1, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u3, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u4, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial_u1, uart_input1);
	  rt_device_set_rx_indicate(serial_u3, uart_input3);
	  rt_device_set_rx_indicate(serial_u4, uart_input4);
//	rt_device_write(serial_u1, 0, str, (sizeof(str) - 1));

	    /* 创建 serial 线程 */
   thread1 = rt_thread_create("u1_thread", usart1_thread_entry, RT_NULL, 1024, 25, 10);
	 thread3 = rt_thread_create("u1_thread", usart3_thread_entry, RT_NULL, 1024, 25, 10);
	 thread4 = rt_thread_create("u1_thread", usart4_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread1 != RT_NULL)
    {
        rt_thread_startup(thread1);
    }
		    if (thread3 != RT_NULL)
    {
        rt_thread_startup(thread3);
    }
		    if (thread4 != RT_NULL)
    {
        rt_thread_startup(thread4);
    }
}
	

