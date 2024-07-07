#include "USARTINIT.h"
#define SAMPLE_UART_NAME       "uart1"  /* 串口设备名称 */
static struct rt_semaphore rx_sem_u1;
static rt_device_t serial_u1;

#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10
/* 串口接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem_u1);

    return RT_EOK;
}
//串口线程
static void serial_thread_entry(void *parameter)
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
void usart1_init(void)
{
  char str[] = "hello RT-Thread!\r\n";
	rt_thread_t thread;
	rt_err_t res;
	serial_u1 = rt_device_find(SAMPLE_UART_NAME);
	rt_sem_init(&rx_sem_u1, "rx_time", 0, RT_IPC_FLAG_FIFO);
	RT_ASSERT(res == RT_EOK);
  /* 以DMA接收及轮询发送模式打开串口设备 */
    rt_device_open(serial_u1, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial_u1, uart_input);
	rt_device_write(serial_u1, 0, str, (sizeof(str) - 1));

	    /* 创建 serial 线程 */
   thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
}
	

