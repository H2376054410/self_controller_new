#include "func_RxHandle.h"
#include "drv_HW_UART.h"

// 缓冲区大小
#define BUFFER_LENGTH 256
// 缓冲区数量
#define RX_UART_NUMBER 2
// 接收串口0
#define RX_UART_0 (&huart1)
// 接收串口1
#define RX_UART_1 (&huart3)
// 标准数据帧中的字节数
#define DATAFRAME_LENGTH 16
// 根据缓冲区大小选择适当的数据形式
typedef uint8_t Buffer_Typedef;

// 接收数据处理结构体
typedef struct
{
    // 串口句柄
    UART_HandleTypeDef *huart;
    // 用于标记数据是否更新
    uint8_t Data_Is_Refreshed;
    // 缓冲数组
    uint8_t Rx_Buffer[BUFFER_LENGTH];
    // 数据帧跨圈后拼包存放
    uint8_t Cross_Buffer[DATAFRAME_LENGTH];
    // 本次接收了多少个字节
    uint16_t Place_Now;
    // 已经使用的缓冲数组的位置（实际上是没有新数据的位置）
    Buffer_Typedef Place_Old;
    // 最新有效的数据帧位置
    Buffer_Typedef Place_Right;
    // 最新有效的数据帧是否跨圈
    uint8_t Is_Circled;
    // 长度错误计数
    uint32_t Error_Len;
    // 和校验错误计数
    uint32_t Error_Sum;
    // 正确个数
    uint32_t Right_Num;
    // 数据个数
    uint32_t Rece_Num;
} Rx_Handle_t;

// 接收数据帧
typedef __packed struct
{
    uint32_t random;
    uint32_t tick;
    uint32_t sum;
    uint32_t data;
} RxData_Frame_t;

// 接收数据处理结构体
Rx_Handle_t Rx_Handle[RX_UART_NUMBER];

// 用于接收的信号量
static struct rt_semaphore Rx_Sem;

// 空闲中断回调函数中释放信号量，执行相应的数据处理
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t Now_BufferNum; // 记录当前写入的缓冲区编号

    // 得到需要处理的缓冲区
    Now_BufferNum = (huart == Rx_Handle[0].huart) ? 0 : 1;
    // 记录本次DMA传输字节数
    Rx_Handle[Now_BufferNum].Place_Now = Size;
    // 标记数据更新
    Rx_Handle[Now_BufferNum].Data_Is_Refreshed = 1;
    Rx_Handle[Now_BufferNum].Rece_Num++;
    // 释放信号量
    rt_sem_release(&Rx_Sem);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t Error_num; // 记录当前出错的缓冲区编号
    Error_num = (huart == Rx_Handle[0].huart) ? 0 : 1;
    // 重启DMA
    HW_UartDma_Start(Rx_Handle[Error_num].huart, Rx_Handle[Error_num].Rx_Buffer, BUFFER_LENGTH);
}

static void Rx_DataHandle(void *parameter)
{
    uint8_t fori, forj;            // 循环使用
    Rx_Handle_t *rx_buffer = NULL; // 需处理的缓冲区指针
    RxData_Frame_t *data_frame;    // 存放本次数据帧的指针
    uint32_t sum;                  // 暂存和校验

    while (1)
    {
        rt_sem_take(&Rx_Sem, RT_WAITING_FOREVER);

        // 获取当前需处理的缓冲区标号
        for (fori = 0; fori < RX_UART_NUMBER; fori++)
        {
            if (Rx_Handle[fori].Data_Is_Refreshed == 1)
            {
                rx_buffer = &Rx_Handle[fori];
                Rx_Handle[fori].Data_Is_Refreshed = 0;
                break;
            }
        }

        // 指针不得为空
        if (rx_buffer == NULL)
            continue;

        // 检查本次数据长度
        if (rx_buffer->Place_Now - rx_buffer->Place_Old != DATAFRAME_LENGTH)
        { // 如果长度不符合，舍弃本次数据
            rx_buffer->Place_Old = rx_buffer->Place_Now;
            rx_buffer->Error_Len++;
            continue;
        }

        // 运行至此说明长度正确
        // 检查是否跨圈
        if (rx_buffer->Place_Now < rx_buffer->Place_Old)
        { // 本次数据跨圈
            // 拼包
            for (fori = 0; fori < BUFFER_LENGTH - rx_buffer->Place_Old; fori++)
            {
                rx_buffer->Cross_Buffer[fori] = rx_buffer->Rx_Buffer[rx_buffer->Place_Old + fori];
            }
            for (forj = 0; forj < DATAFRAME_LENGTH - BUFFER_LENGTH + rx_buffer->Place_Old; forj++)
            {
                rx_buffer->Cross_Buffer[fori + forj] = rx_buffer->Rx_Buffer[forj];
            }
            // 得到本次数据帧
            data_frame = (RxData_Frame_t *)(rx_buffer->Cross_Buffer);
            // 计算和校验
            sum = data_frame->random + data_frame->tick;
            if (data_frame->sum != sum)
            { // 和校验错误，舍弃本次数据
                rx_buffer->Place_Old = rx_buffer->Place_Now;
                rx_buffer->Error_Sum++;
                continue;
            }
            // 和校验正确
            // 记录此次正确数据帧的位置
            rx_buffer->Place_Right = rx_buffer->Place_Old;
            rx_buffer->Is_Circled = 0;
            // 更新缓冲数组位置
            rx_buffer->Place_Old = rx_buffer->Place_Now;
            rx_buffer->Right_Num++;
        }
        else
        { // 本次数据未跨圈
            // 得到本次数据帧
            data_frame = (RxData_Frame_t *)(&rx_buffer->Rx_Buffer[rx_buffer->Place_Old]);
            // 计算和校验
            sum = data_frame->random + data_frame->tick;
            if (data_frame->sum != sum)
            { // 和校验错误，舍弃本次数据
                rx_buffer->Place_Old = rx_buffer->Place_Now;
                rx_buffer->Error_Sum++;
                continue;
            }
            // 和校验正确
            // 记录此次正确数据帧的位置
            rx_buffer->Place_Right = rx_buffer->Place_Old;
            rx_buffer->Is_Circled = 0;
            // 更新缓冲数组位置
            rx_buffer->Place_Old = rx_buffer->Place_Now;
            rx_buffer->Right_Num++;
        }
    }
}

/**
 * @brief 读取串口最新收到的数据
 * @author dty
 * @param uartx 要读取的串口（0,1）
 * @param data_type 数据类型：0为随机码，1为时间戳
 * @return uint32_t
 */
uint32_t Read_RxData(uint8_t uartx, uint8_t data_type)
{
    Rx_Handle_t *rx_buffer = &Rx_Handle[uartx]; // 存储要读取的缓冲区指针
    RxData_Frame_t *data;                       // 存储数据帧指针

    // 得到数据帧
    if (rx_buffer->Is_Circled)
        data = (RxData_Frame_t *)(&rx_buffer->Cross_Buffer);
    else
        data = (RxData_Frame_t *)(&rx_buffer->Rx_Buffer[rx_buffer->Place_Right]);

    // 返回对应数据
    if (data_type)
        return data->tick;
    else
        return data->random;
}

/**
 * @brief 初始化结构体
 * @author dty
 */
static void DataHandle_Struct_Init(void)
{
    rt_memset(Rx_Handle, 0, sizeof(Rx_Handle));
    Rx_Handle[0].huart = Get_UART_Dev(0);
    Rx_Handle[1].huart = Get_UART_Dev(1);
}

/**
 * @brief 初始化接收数据处理
 * @author dty
 */
void RxHandle_Init(void)
{
    uint8_t fori;
    rt_thread_t thread = RT_NULL;

    // 初始化硬件
    HW_UARTandDMA_Init(0);

    // 初始化结构体
    DataHandle_Struct_Init();

    // 初始化信号量
    rt_sem_init(&Rx_Sem, "Rx_Sem", 0, RT_IPC_FLAG_FIFO);

    // 初始化控制线程
    thread = rt_thread_create("Rx_DataHandle",
                              Rx_DataHandle,
                              RT_NULL,
                              1024, // 线程栈大小
                              1,    // 线程优先级
                              10);  // 线程时间片
    if (thread == RT_NULL)
        while (1)
            ;
    if (rt_thread_startup(thread) != RT_EOK)
        while (1)
            ;

    // 打开接收DMA
    for (fori = 0; fori < RX_UART_NUMBER; fori++)
    {
        HW_UartDma_Start(Rx_Handle[fori].huart, Rx_Handle[fori].Rx_Buffer, BUFFER_LENGTH);
    }
}
