#include "drv_SPICAN.h"

#ifdef SOC_SERIES_STM32H7
#include "core_cm7.h"
#endif

#ifndef HW_SPI_HANDLE
#error "Please select the right SPI device for SPI_Master!"
#endif

// DMA传输完成中断中释放, 用于触发SPI控制线程
static struct rt_semaphore DMA_Finish_Sem;

// 用于触发STC从机复位
static struct rt_semaphore Slave_Reset_Sem;

// 从机的设备结构体
SPICAN_dev_t Slave_Dev[SPI_CAN_SLAVE_NUM];

// SPI的DMA中断需要调用此函数来触发SPI控制线程
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UNUSED(hspi);
    // 取走堆积信号量
    while (RT_EOK == rt_sem_trytake(&DMA_Finish_Sem))
        ;
    // 释放一个新的信号量
    rt_sem_release(&DMA_Finish_Sem);
}

// 启动与指定设备的SPI传输 不含片选控制
static void SPI_CAN_StartTransmit(SPICAN_dev_t *NowDev)
{
    rt_uint8_t SendCNT_This;  // 本次启动DMA实际发出去的CAN报文数
    rt_int8_t EmptyCNT_This;  // 本次启动DMA时所用的小SendBuffer中空的报文数
    rt_uint16_t BuffNum_Send; // 本次启动发送所用的Buffer序号

    // 获取Buffer的访问权限（信号量上锁）
    rt_sem_take(&NowDev->DMABuffer.Access_Sem, RT_WAITING_FOREVER);
    rt_sem_trytake(&NowDev->DMABuffer.Access_Sem); // 确保取完

    BuffNum_Send = NowDev->DMABuffer.BuffNum_Send;
    SendCNT_This = NowDev->DMABuffer.DMASendbuff[BuffNum_Send][1];
    NowDev->DMABuffer.DMASendbuff[BuffNum_Send][2] = SendCNT_This;

    // 按照当前使用的Buffer启动DMA
    HAL_SPI_TransmitReceive_DMA(&HW_SPI_HANDLE,
                                &NowDev->DMABuffer.DMASendbuff[BuffNum_Send][0],
                                &NowDev->DMABuffer.DMARecbuff[BuffNum_Send][0], SPI_DMA_DATA_LEN);

    // 启动发送后当前DMA占用的SendBuffer不能继续写入了，等效于总的可写入的报文个数减少了
    for (EmptyCNT_This = SPI_CAN_MSG_MAX - SendCNT_This; EmptyCNT_This > 0; EmptyCNT_This--)
    {
        // 取走相应数量的信号量
        if (RT_EOK != rt_sem_trytake(&NowDev->Free_Sem))
        {
            NowDev->Free_Sem_Empty_Count++;
        }
    }

    // 状态由不是当前从机进行传输变为是当前从机进行传输，相应处理当前从机的发送缓冲区写入位置
    if (NowDev->DMABuffer.BuffNum_Send == NowDev->DMABuffer.BuffNum_Write)
    {
        // 启动DMA前写入的位置和启动DMA的位置相同，此时需要将写入位置向前移动
        if (NowDev->DMABuffer.BuffNum_Write < SPI_CAN_BUFFER_CNT - 1)
        {
            NowDev->DMABuffer.BuffNum_Write++;
        }
        else
        { // 循环使用缓冲区
            NowDev->DMABuffer.BuffNum_Write = 0;
        }
    }

    // 释放Buffer的访问权限（信号量解锁）
    rt_sem_release(&NowDev->DMABuffer.Access_Sem);
}
uint32_t Rece_num1 = 0;
uint32_t Rece_num2 = 0;
uint32_t Erro_num11 = 0; // 报文数校验错误
uint32_t Erro_num12 = 0;
uint32_t Erro_num2 = 0; // 错位检查错误
uint32_t Erro_num3 = 0; // 和校验错误
// 进行接收校验和回调
static void SPI_CAN_ReceiveProcess(SPICAN_dev_t *NowDev, rt_uint8_t *RxBuff)
{
    rt_uint8_t fori1, fori2;
    SPICAN_Msg_t *NowMsg;
    rt_uint16_t CheckSum;
    rt_uint16_t *DataLoad;

    if (NowDev->isUse == 1 && NowDev->WorkStatus != RT_ERROR)
    { // 当前从机正在使用
        // 处理接收到的报文
        if (RxBuff[0] == 0x3C && RxBuff[3] == 0x3C && RxBuff[1] == RxBuff[2])
        {                                     // 报文数校验正常
            NowDev->ErrorCount_FrameHead = 0; // 非连续出错，错误计数置零
            for (fori1 = 0; fori1 < RxBuff[1]; fori1++)
            {
                NowMsg = (SPICAN_Msg_t *)&RxBuff[4 + fori1 * SPI_DMA_MSG_LEN];
                if (NowMsg->Mem16.Check3C == 0x3C)
                {
                    // 错位检查通过
                    CheckSum = 0;
                    DataLoad = NowMsg->Mem16.data;
                    for (fori2 = 0; fori2 < 4; fori2++)
                    {
                        CheckSum += DataLoad[fori2];
                    }
                    CheckSum += NowMsg->Mem16.Len;
                    CheckSum += NowMsg->Mem16.IDData[0];
                    CheckSum += NowMsg->Mem16.IDData[1];
                    if (CheckSum == NowMsg->Mem16.Sum)
                    {
                        // 和校验通过
                        if (NowDev->ReceiveFun != NULL)
                        { // 调用回调函数
                            NowDev->ReceiveFun(&NowMsg->Transfer);
                        }
                    }
                    else
                    {
                        NowDev->ErrorCount_CheckSum++;
                    }
                }
                else
                {
                    NowDev->ErrorCount_Check3C++;
                }
            }
        }
        else
        { // 帧头校验错误
            if (NowDev->ErrorCount_FrameHead > 10)
            { // 连续出错10次，触发从机复位
                NowDev->ErrorCount_FrameHead = 0;
                NowDev->WorkStatus = RT_ERROR;
                rt_sem_release(&Slave_Reset_Sem);
            }
            else
            {
                NowDev->ErrorCount_FrameHead++;
            }
        }
    }
}

// 与指定设备的SPI传输完成后的动作: 接收回调、Buffer变化
static void SPI_CAN_EndTransmit(SPICAN_dev_t *NowDev)
{
    rt_uint8_t SendCNT_Last; // 刚刚完成的是对哪个Buffer的发送
    rt_uint8_t fori;

    // 获取Buffer的访问权限（信号量上锁）
    rt_sem_take(&NowDev->DMABuffer.Access_Sem, RT_WAITING_FOREVER);

    // 修改"将要进行DMA发送的buffer"标号
    SendCNT_Last = NowDev->DMABuffer.BuffNum_Send;
    if (SendCNT_Last < SPI_CAN_BUFFER_CNT - 1)
    {
        NowDev->DMABuffer.BuffNum_Send = SendCNT_Last + 1;
    }
    else
    {
        NowDev->DMABuffer.BuffNum_Send = 0;
    }

    // 清空已经完成发送的SendBuffer中的Msg计数
    NowDev->DMABuffer.DMASendbuff[SendCNT_Last][1] = 0;

    // 新增一个buffer尺寸的缓冲区空位，释放信号量
    for (fori = 0; fori < SPI_CAN_MSG_MAX; fori++)
    {
        if (NowDev->Free_Sem_Empty_Count == 0)
        {
            rt_sem_release(&NowDev->Free_Sem);
        }
        else
        {
            NowDev->Free_Sem_Empty_Count--;
        }
    }

    // 释放Buffer的访问权限（信号量解锁）
    rt_sem_release(&NowDev->DMABuffer.Access_Sem);
#ifdef SOC_SERIES_STM32H7
    SCB_CleanInvalidateDCache();
#endif
    SPI_CAN_ReceiveProcess(NowDev, NowDev->DMABuffer.DMARecbuff[SendCNT_Last]);
}

// SPI-DMA控制线程
static void SPI_CAN_Thread(void *Para)
{
    rt_int8_t SlaveNow = 0;
    SPICAN_dev_t *NowDev = &Slave_Dev[0];
    SPICAN_dev_t *LastDev = &Slave_Dev[SPI_CAN_SLAVE_NUM - 1];

    // 启动从机1通信
    rt_pin_write(NowDev->CS_PIN, PIN_LOW);
    SPI_CAN_StartTransmit(NowDev);

    while (1)
    {
        rt_sem_take(&DMA_Finish_Sem, RT_WAITING_FOREVER);
        // 上一从机通信完成，恢复片选
        rt_pin_write(NowDev->CS_PIN, PIN_HIGH);

        // 寻找下一从机
        LastDev = NowDev;
        if (SlaveNow < SPI_CAN_SLAVE_NUM - 1)
        {
            SlaveNow++;
        }
        else
        {
            SlaveNow = 0;
        }

        NowDev = &Slave_Dev[SlaveNow];

        // 启动下一从机通信
        rt_pin_write(NowDev->CS_PIN, PIN_LOW);

        if (NowDev != LastDev)
        { // 有切换下一个从机，可以直接先启动下一从机通信再处理上一从机数据
            SPI_CAN_StartTransmit(NowDev);
            // 处理上一从机通信完成的后续: 接收回调、Buffer变化
            SPI_CAN_EndTransmit(LastDev);
        }
        else
        {
            // 只有一个从机的情况
            // 处理通信完成的后续: 接收回调、Buffer变化
            SPI_CAN_EndTransmit(LastDev);
            // 开始下一轮通信
            SPI_CAN_StartTransmit(NowDev);
        }
    }
}

// CAN发送申请，函数内完成对SPI发送的打包
rt_err_t SPI_CAN_SendMsg(rt_uint8_t SlaveNum, SPICAN_MsgOnTransfer_t *ToSend)
{
    SPICAN_dev_t *NowDev;
    SPICAN_Msg_t *SendBuff;
    rt_uint16_t Wr_CNT_Buffer; // 当前从机数据写入到第几个Buffer了
    rt_uint16_t Wr_CNT_Msg;    // 当前Buffer写入到第几个报文了

    // 检查参数正确性
    if (SlaveNum > SPI_CAN_SLAVE_NUM - 1)
    {
        return RT_ERROR;
    }
    NowDev = &Slave_Dev[SlaveNum];

    // 获取信号量，检查缓冲区是否已满
    rt_sem_take(&NowDev->Free_Sem, RT_WAITING_FOREVER);
    // 获取Buffer写入权限，信号量上锁
    rt_sem_take(&NowDev->DMABuffer.Access_Sem, RT_WAITING_FOREVER);

    // 找到当前该写哪个buffer了
    Wr_CNT_Buffer = NowDev->DMABuffer.BuffNum_Write;
    // 查看当前buffer原本有多少报文
    Wr_CNT_Msg = NowDev->DMABuffer.DMASendbuff[Wr_CNT_Buffer][1];
    NowDev->DMABuffer.DMASendbuff[Wr_CNT_Buffer][1]++;
    // 如果本次写完就满了，则将Dev中的下次写入的Buffer序号加1
    if (Wr_CNT_Msg >= SPI_CAN_MSG_MAX - 1)
    {
        if (Wr_CNT_Buffer < SPI_CAN_BUFFER_CNT - 1)
        {
            NowDev->DMABuffer.BuffNum_Write++;
        }
        else
        {
            NowDev->DMABuffer.BuffNum_Write = 0;
        }
    }

    // 数据打包
    SendBuff = (SPICAN_Msg_t *)&NowDev->DMABuffer.DMASendbuff[Wr_CNT_Buffer][Wr_CNT_Msg * SPI_DMA_MSG_LEN + 4];

    SendBuff->Mem32.Check3C = 0x3C;
    // 内存已对齐前提下，每次可复制4字节
    SendBuff->Mem32.data[0] = *(rt_uint32_t *)(&ToSend->data[0]);
    SendBuff->Mem32.data[1] = *(rt_uint32_t *)(&ToSend->data[4]);

    SendBuff->Mem32.Len = ToSend->Len;
    SendBuff->Mem32.IDData = ((SPICAN_Msg_t *)ToSend)->Mem32.IDData;
    // 和校验计算
    SendBuff->Mem32.Sum = SendBuff->Mem16.data[0] +
                          SendBuff->Mem16.data[1] +
                          SendBuff->Mem16.data[2] +
                          SendBuff->Mem16.data[3] +
                          SendBuff->Mem16.Len +
                          SendBuff->Mem16.IDData[0] +
                          SendBuff->Mem16.IDData[1];

    // 释放Buffer写入权限，信号量解锁
    rt_sem_release(&NowDev->DMABuffer.Access_Sem);

    return RT_EOK;
}

// CAN接收回调函数设置
void SPI_CAN_Set_ReceiveFun(rt_uint8_t SlaveNum, CANReceiveFun_t Fun)
{
    Slave_Dev[SlaveNum].ReceiveFun = Fun;
}

// 每个从机设备的Buffer最多存储多少CAN报文（不含即将DMA发送的那一个Buffer）
#define FREE_MAX (SPI_CAN_MSG_MAX * (SPI_CAN_BUFFER_CNT - 1))

// 初始化设备结构体相关内容
static void SPI_CAN_Dev_Init(void)
{
    int fori1, fori2;
    SPICAN_dev_t *NowDev;
    char FreeSemName[8] = "SLVFR_A";
    char AccessSemName[8] = "SLVAC_A";

    // 初始化片选引脚与复位引脚(STC复位脚需配置为浮空输入，为高阻态)
#if (SPI_CAN_SLAVE_NUM > 0)
    Slave_Dev[0].CS_PIN = SLAVE_0_CS_PIN;
    rt_pin_mode(SLAVE_0_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SLAVE_0_CS_PIN, PIN_HIGH);

    Slave_Dev[0].RST_PIN = SLAVE_0_RST_PIN;
    rt_pin_mode(SLAVE_0_RST_PIN, PIN_MODE_INPUT);
#endif
#if (SPI_CAN_SLAVE_NUM > 1)
    Slave_Dev[1].CS_PIN = SLAVE_1_CS_PIN;
    rt_pin_mode(SLAVE_1_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SLAVE_1_CS_PIN, PIN_HIGH);

    Slave_Dev[1].RST_PIN = SLAVE_1_RST_PIN;
    rt_pin_mode(SLAVE_1_RST_PIN, PIN_MODE_INPUT);
#endif
#if (SPI_CAN_SLAVE_NUM > 2)
    Slave_Dev[2].CS_PIN = SLAVE_2_CS_PIN;
    rt_pin_mode(SLAVE_2_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SLAVE_2_CS_PIN, PIN_HIGH);

    Slave_Dev[2].RST_PIN = SLAVE_2_RST_PIN;
    rt_pin_mode(SLAVE_2_RST_PIN, PIN_MODE_INPUT);
#endif
#if (SPI_CAN_SLAVE_NUM > 3)
    Slave_Dev[3].CS_PIN = SLAVE_3_CS_PIN;
    rt_pin_mode(SLAVE_3_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SLAVE_3_CS_PIN, PIN_HIGH);

    Slave_Dev[3].RST_PIN = SLAVE_3_RST_PIN;
    rt_pin_mode(SLAVE_3_RST_PIN, PIN_MODE_INPUT);
#endif
#if (SPI_CAN_SLAVE_NUM > 4)
    Slave_Dev[4].CS_PIN = SLAVE_4_CS_PIN;
    rt_pin_mode(SLAVE_4_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(SLAVE_4_CS_PIN, PIN_HIGH);

    Slave_Dev[4].RST_PIN = SLAVE_4_RST_PIN;
    rt_pin_mode(SLAVE_4_RST_PIN, PIN_MODE_INPUT);
#endif

    // 初始化每个从机结构体
    for (fori1 = 0; fori1 < SPI_CAN_SLAVE_NUM; fori1++)
    {
        NowDev = &Slave_Dev[fori1];

        // 初始化信号量
        rt_sem_init(&NowDev->Free_Sem, FreeSemName, FREE_MAX, RT_IPC_FLAG_FIFO);
        FreeSemName[6]++; // 递增信号量名称中的序号
        rt_sem_init(&NowDev->DMABuffer.Access_Sem, AccessSemName, 1, RT_IPC_FLAG_FIFO);
        AccessSemName[6]++; // 递增信号量名称中的序号

        // 初始化发送Buffer
        NowDev->DMABuffer.BuffNum_Send = 0;
        NowDev->DMABuffer.BuffNum_Write = 0;
        for (fori2 = 0; fori2 < SPI_CAN_BUFFER_CNT; fori2++)
        {
            NowDev->DMABuffer.DMASendbuff[fori2][0] = 0x3C;
            NowDev->DMABuffer.DMASendbuff[fori2][1] = 0;
            NowDev->DMABuffer.DMASendbuff[fori2][2] = 0;
            NowDev->DMABuffer.DMASendbuff[fori2][3] = 0x3C;
        }

        // 初始化回调函数
        NowDev->ReceiveFun = NULL;

        NowDev->WorkStatus = RT_EOK;
        NowDev->ErrorCount_FrameHead = 0;
        NowDev->ErrorCount_Check3C = 0;
        NowDev->ErrorCount_CheckSum = 0;

        NowDev->Free_Sem_Empty_Count = 0;
        // 初始化是否使用标记
        if (fori1 < NEED_SLAVE_NUM)
            NowDev->isUse = 1;
        else
            NowDev->isUse = 0;
    }
}

static void Slave_Reset(void *Para)
{ // STC从机需要复位脚在正常情况下为高阻态，故设置为浮空输入
    // 记录需要复位的从机号
    rt_int8_t SlaveNum;
    rt_int8_t fori;
    rt_uint32_t Reset_tick[NEED_SLAVE_NUM] = {0};
    while (1)
    {
        rt_sem_take(&Slave_Reset_Sem, RT_WAITING_FOREVER);
        // 寻找需要复位的从机
        for (fori = 0; fori < SPI_CAN_SLAVE_NUM; fori++)
        {
            if (Slave_Dev[fori].WorkStatus == RT_ERROR)
            {
                SlaveNum = fori;
                break;
            }
        }
        // 确保该从机需要复位
        if (Slave_Dev[SlaveNum].WorkStatus != RT_ERROR)
            continue;
        if ((rt_tick_get() - Reset_tick[SlaveNum]) < 2000)
        { // 确保从机恢复正常工作
            Slave_Dev[SlaveNum].WorkStatus = RT_EOK;
            continue;
        }
        // 复位
        rt_pin_mode(Slave_Dev[SlaveNum].RST_PIN, PIN_MODE_OUTPUT); // 切换为推挽输出
        rt_pin_write(Slave_Dev[SlaveNum].RST_PIN, PIN_HIGH);       // 拉高
        rt_thread_mdelay(1);                                       // 等待1ms
        rt_pin_write(Slave_Dev[SlaveNum].RST_PIN, PIN_LOW);        // 拉低
        rt_pin_mode(Slave_Dev[SlaveNum].RST_PIN, PIN_MODE_INPUT);  // 切换为浮空输入
        // 记录从机复位时刻
        Reset_tick[SlaveNum] = rt_tick_get();
        // 恢复从机状态
        Slave_Dev[SlaveNum].WorkStatus = RT_EOK;
    }
}

// 初始化DMA控制线程，线程优先级尽量高以达到最高运行效率
// 初始化STC从机复位线程
static void SPI_CAN_Thread_Init(void)
{
    rt_thread_t thread;

    rt_sem_init(&DMA_Finish_Sem, "SPIDMA", 0, RT_IPC_FLAG_FIFO);

    thread = rt_thread_create("SPICAN", SPI_CAN_Thread, RT_NULL, 1024, THREAD_PRIO_SPICAN, 1);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }

    rt_sem_init(&Slave_Reset_Sem, "SLAVERESET", 0, RT_IPC_FLAG_FIFO);

    thread = rt_thread_create("SLAVERESET", Slave_Reset, RT_NULL, 1024, 10, 1);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
}

rt_err_t STC_Reset(rt_int8_t SlaveNum)
{
    // 复位的从机没有使用
    if (Slave_Dev[SlaveNum].isUse == 0)
        return RT_ERROR;
    Slave_Dev[SlaveNum].WorkStatus = RT_ERROR;
    rt_sem_release(&Slave_Reset_Sem);
    return RT_EOK;
}

// 初始化SPI-CAN从机设备
void SPI_CAN_Init(void)
{
    // 初始化相关结构体
    SPI_CAN_Dev_Init();
    // 创建线程
    SPI_CAN_Thread_Init();
}
