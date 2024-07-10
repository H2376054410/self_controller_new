#ifndef __DRV_SPICAN_H__
#define __DRV_SPICAN_H__

#include "rtthread.h"
#include "board.h"
#include "rtdevice.h"

// 当实际所用从机数量小于等于3时，为保证与各个从机通讯不会因为通讯频率太快而导致从机撑不住
// 故设置从机数量为3，保证通讯的准确率

#if NEED_SLAVE_NUM == 0
#error "NEED_SLAVE_NUM can't be 0"
#endif

#ifdef USE_STC_SLAVE

#if NEED_SLAVE_NUM > 3
#define SPI_CAN_SLAVE_NUM (NEED_SLAVE_NUM)
#else
#define SPI_CAN_SLAVE_NUM 3
#endif

#else

#define SPI_CAN_SLAVE_NUM (NEED_SLAVE_NUM)

#endif

// 相应的，为实际上没有使用到的从机分配CS与RST引脚
#if NEED_SLAVE_NUM < 3
#define CS_PORT_2 "E"
#define CS_PIN_2 3
#define RST_PORT_2 "F"
#define RST_PIN_2 3
#endif

#if NEED_SLAVE_NUM < 2
#define CS_PORT_1 "E"
#define CS_PIN_1 2
#define RST_PORT_1 "F"
#define RST_PIN_1 2
#endif

// #define SPI_CAN_SLAVE_NUM (2) // SPI从机个数
#define SPI_CAN_MSG_MAX (4) // 每轮传输时至多传输多少个CAN报文

#define SLAVE_0_CS_PIN ((*CS_PORT_0 - 'A') * 16 + CS_PIN_0)
#define SLAVE_1_CS_PIN ((*CS_PORT_1 - 'A') * 16 + CS_PIN_1)
#define SLAVE_2_CS_PIN ((*CS_PORT_2 - 'A') * 16 + CS_PIN_2)
#define SLAVE_3_CS_PIN ((*CS_PORT_3 - 'A') * 16 + CS_PIN_3)
#define SLAVE_4_CS_PIN ((*CS_PORT_4 - 'A') * 16 + CS_PIN_4)

#define SLAVE_0_RST_PIN ((*RST_PORT_0 - 'A') * 16 + RST_PIN_0)
#define SLAVE_1_RST_PIN ((*RST_PORT_1 - 'A') * 16 + RST_PIN_1)
#define SLAVE_2_RST_PIN ((*RST_PORT_2 - 'A') * 16 + RST_PIN_2)
#define SLAVE_3_RST_PIN ((*RST_PORT_3 - 'A') * 16 + RST_PIN_3)
#define SLAVE_4_RST_PIN ((*RST_PORT_4 - 'A') * 16 + RST_PIN_4)

#if SPI_MASTER_USE_HSPIN == 1
#define HW_SPI_HANDLE hspi1
#elif SPI_MASTER_USE_HSPIN == 2
#define HW_SPI_HANDLE hspi2
#elif SPI_MASTER_USE_HSPIN == 3
#define HW_SPI_HANDLE hspi3
#elif SPI_MASTER_USE_HSPIN == 4
#define HW_SPI_HANDLE hspi4
#endif

#define THREAD_PRIO_SPICAN 2

extern SPI_HandleTypeDef HW_SPI_HANDLE;
/*

    为了提高底层驱动程序的效率，需要尽可能减少内存复制和线程调度次数。

    推荐将SPICAN的收发管理线程的优先级设置为程序内最高优先级，
    这样可以提高实时性，并尽可能减少CAN接收时的线程调度次数。
        （因为接收回调函数中的用户程序可能会释放信号量）

    考虑直接将数据打包放进DMA发送缓冲区的动作安排在供外部线程调用的发送函数中
    由于每轮SPI-CAN的发送量需要固定以便从机使用DMA，且为了提高实时性，
    则需要将其固定为较小的数值，适当提高轮询频率。

    如果多线程集中发送CAN报文，则会出现发送缓冲区存满，则此时必须
    将调用发送接口的线程挂起，等待缓冲区腾出空间，这就会增加线程调度次数
    因此需要适当增加CAN发送缓冲区的长度来保证程序效率。

    综上，要想增大缓冲区，就只能开辟多个缓冲区，上层文件集中申请CAN发送时，
    驱动程序会按顺序把报文紧密排列到后续的多轮SPI发送过程中去。

*/

// SPI-DMA缓冲区个数 (为实现最高的代码运行速度，此处实际有效的Buffer会少一个)
#define SPI_CAN_BUFFER_CNT (10)

#define SPI_DMA_MSG_LEN (16)                                     // SPI通信时单个CAN报文占用的字节数
#define SPI_DMA_DATA_LEN (SPI_CAN_MSG_MAX * SPI_DMA_MSG_LEN + 4) // 与一个从机通信一轮时需要发送的字节数

// SPI通信时的CAN报文表示形式
typedef struct
{
    rt_uint32_t ID : 29;
    rt_uint32_t IsExtID : 1;
    rt_uint32_t IsRemote : 1;
    rt_uint32_t ForCANx : 1; // CAN1:0	CAN2:1
    rt_uint32_t Len : 8;
    rt_uint32_t Check3C : 8;
    rt_uint32_t Sum : 16;
    rt_uint8_t data[8]; // STM32为小端，低字节位于低地址
} SPICAN_MsgOnTransfer_t;

// 内存形式的CAN报文表示形式（用于计算校验）
typedef struct
{
    rt_uint16_t IDData[2];
    rt_uint8_t Len;
    rt_uint8_t Check3C;
    rt_uint16_t Sum;
    rt_uint16_t data[4]; // STM32为小端，低字节位于低地址
} SPICAN_MsgOnMem16_t;

// 内存形式的CAN报文表示形式（用于复制）
typedef struct
{
    rt_uint32_t IDData;
    rt_uint8_t Len;
    rt_uint8_t Check3C;
    rt_uint16_t Sum;
    rt_uint32_t data[2]; // STM32为小端，低字节位于低地址
} SPICAN_MsgOnMem32_t;

typedef union
{
    SPICAN_MsgOnTransfer_t Transfer;
    SPICAN_MsgOnMem16_t Mem16;
    SPICAN_MsgOnMem32_t Mem32;
} SPICAN_Msg_t;

// 一组供DMA循环使用的Buffer集合
typedef struct
{
    // 用于处理读写冲突，访问设备结构体时需要加锁
    struct rt_semaphore Access_Sem;

    // 多段小Buffer类似于循环队列使用
    rt_uint8_t DMASendbuff[SPI_CAN_BUFFER_CNT][SPI_DMA_DATA_LEN]; // DMA-SPI原始数据 发送缓冲区
    rt_uint8_t DMARecbuff[SPI_CAN_BUFFER_CNT][SPI_DMA_DATA_LEN];  // DMA-SPI原始数据 接收缓冲区
    rt_uint16_t BuffNum_Write;                                    // 记录当前正在写入的缓冲区的标号
    rt_uint16_t BuffNum_Send;                                     // 记录当前即将发送的缓冲区的标号
} SPIDMA_Buff_t;

typedef void (*CANReceiveFun_t)(SPICAN_MsgOnTransfer_t *Msg);

// 一个SPI从机设备相关数据对应结构体
typedef struct
{
    // 用信号量剩余的个数表示当前各个Buffer中还能写入的CAN报文数的总和
    struct rt_semaphore Free_Sem;
    // 用于DMA收发的多重缓冲区
    SPIDMA_Buff_t DMABuffer;
    // 从机对应的片选引脚
    rt_base_t CS_PIN;
    // 初始化时可以连接指定的接收回调函数
    CANReceiveFun_t ReceiveFun;
    // 从机对应的片选引脚
    rt_base_t RST_PIN;
    // 帧头校验错误（用于判断从机是否异常）
    rt_uint8_t ErrorCount_FrameHead;
    // 从机工作状态
    rt_uint8_t WorkStatus;

    rt_uint8_t Free_Sem_Empty_Count;
    // 从机是否使用
    rt_uint8_t isUse;
    // 从机和校验错误计数
    rt_uint32_t ErrorCount_CheckSum;
    // 从机报文错位检查错误计数、
    rt_uint32_t ErrorCount_Check3C;
} SPICAN_dev_t;

// CAN发送申请，函数内完成对SPI发送的打包
extern rt_err_t SPI_CAN_SendMsg(rt_uint8_t SlaveNum, SPICAN_MsgOnTransfer_t *ToSend);

// CAN接收回调函数设置
extern void SPI_CAN_Set_ReceiveFun(rt_uint8_t SlaveNum, CANReceiveFun_t Fun);

// 从机复位(从0开始)
extern rt_err_t STC_Reset(rt_int8_t SlaveNum);

// 初始化SPI-CAN从机设备
extern void SPI_CAN_Init(void);

#endif
