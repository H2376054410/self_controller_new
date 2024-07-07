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
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10
/* 串口接收数据回调函数 */
static rt_err_t uart_input1(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem_u1);

    return RT_EOK;
}
static rt_err_t uart_input3(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem_u3);

    return RT_EOK;
}
static rt_err_t uart_input4(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem_u4);

    return RT_EOK;
}
//串口线程
static void usart1_thread_entry(void *parameter)
{
    char ch;

    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial_u1, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem_u1, RT_WAITING_FOREVER);
        }
        /* 读取到的数据通过串口错位输出 */
        ch = ch + 1;
        rt_device_write(serial_u1, 0, &ch, 1);
    }
}
static void usart3_thread_entry(void *parameter)
{
    char ch;

    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial_u3, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem_u3, RT_WAITING_FOREVER);
        }
        /* 读取到的数据通过串口错位输出 */
        ch = ch + 1;
        rt_device_write(serial_u3, 0, &ch, 1);
    }
}
static void usart4_thread_entry(void *parameter)
{
    char ch;

    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial_u4, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem_u4, RT_WAITING_FOREVER);
        }
        /* 读取到的数据通过串口错位输出 */
        ch = ch + 1;
        rt_device_write(serial_u4, 0, &ch, 1);
    }
}
void uart_init(void)
{
  char str[] = "hello RT-Thread!\r\n";
	rt_thread_t thread1;
	rt_thread_t thread3;
	rt_thread_t thread4;
	
	
	serial_u1 = rt_device_find(UART_NAME1);
	serial_u3 = rt_device_find(UART_NAME3);
	serial_u4 = rt_device_find(UART_NAME4);
	
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
	
	rt_sem_init(&rx_sem_u1, "rx_usart1", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_sem_u3, "rx_usart3", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_sem_u4, "rx_usart4", 0, RT_IPC_FLAG_FIFO);
  /* 以DMA接收及轮询发送模式打开串口设备 */
    rt_device_open(serial_u1, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u3, RT_DEVICE_FLAG_DMA_RX);
	  rt_device_open(serial_u4, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial_u1, uart_input1);
	    rt_device_set_rx_indicate(serial_u3, uart_input3);
	    rt_device_set_rx_indicate(serial_u4, uart_input4);
	rt_device_write(serial_u1, 0, str, (sizeof(str) - 1));

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
	

