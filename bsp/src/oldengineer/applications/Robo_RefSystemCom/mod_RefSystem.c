#include "mod_RefSystem.h"
#include "string.h"
#include "drv_CRC.h"
#include "drv_thread.h"
#include "fun_protocolRef.h"
#include "app_monitor.h"

DMA_HandleTypeDef hdma_usart1_rx; // 用于应对CubeMx
DMA_HandleTypeDef hdma_usart3_rx; // 用于应对CubeMx

/* 对裁判系统解析后的数据存放结构体 */
DJI_Data_t DJI_ReadData;
RefID_t refId;
static package_t package;
RefReceiveTime_s RefReceiveTime;
DJI_info_t DJI_info;

static struct rt_messagequeue DJI_mq;                 /* 消息队列控制块 */
static rt_uint8_t msg_pool[512];                      /* 消息队列中用到的放置消息的内存池 存储字节大小和设备供读取串口DMA时使用*/
static rt_uint8_t DJI_buffer[RT_SERIAL_RB_BUFSZ + 1]; // 串口接受的原始数据 DMA

void Cal_frame(DJI_Frame_t *DJI_Frame_Cal) // 计算各个包的频率
{
    uint32_t now_tick = rt_tick_get();
    DJI_Frame_Cal->rec_time = now_tick;
    if ((now_tick - DJI_Frame_Cal->last_time) != 0)
        DJI_Frame_Cal->frame = 1.f / ((float)(now_tick - DJI_Frame_Cal->last_time) / (float)RT_TICK_PER_SECOND * 1000.0f) * 1000; /*单位hz*/
    DJI_Frame_Cal->last_time = now_tick;
}

rt_tick_t GetRefDataRefreshTick(CmdIDIndex_e CmdIDIndex)
{
    switch (CmdIDIndex)
    {
    case Index_game_robot_state:
        return RefReceiveTime.game_robot_state;
    case Index_power_heat_data:
        return RefReceiveTime.power_heat_data;
    case Index_buff_musk: // 机器人增益数据:
        return RefReceiveTime.buff_musk;
    case Index_shoot_data: // 实时射击数据:
        return RefReceiveTime.shoot_data;
    default:
        return 0;
    }
}

static void recordRefID(uint16_t ID)
{
    uint8_t flag = 0;
    for (int i = 0; i < RefIDNUM; i++)
    {
        if (refId.RefID[i] == ID)
        {
            flag = 1;
            refId.refTick[i] = rt_tick_get();
        }
    }
    if (!flag)
    {
        if (refId.IDNum < RefIDNUM)
            refId.RefID[refId.IDNum] = ID;

        refId.IDNum++;
    }
}

/**
 * @brief 解析裁判系统的数据
 * @param {Frame_t} *tempData 指向裁判系统的一包数据，起始地址为帧头
 * @param {uint16_t} PakLen 本包一整包的数据长度 包括帧头以及末尾的crc校验等
 * @param {DJI_Data_t} *pRecBuf 解析后的数据的存放地址
 */
static void RecRefData(Frame_t *tempData, uint16_t PakLen, DJI_Data_t *pRecBuf)
{

    /*得到数据后正常处理*/
    if (Verify_CRC8_Check_Sum((uint8_t *)tempData, LEN_HEADER) &&
        Verify_CRC16_Check_Sum((uint8_t *)tempData, GetNowPackageLen(PakLen)))
    {
        pRecBuf->FrameHeader = tempData->FrameHeader;
        pRecBuf->CmdID = tempData->CmdID;
        recordRefID(tempData->CmdID);
        switch (tempData->CmdID)
        {
        case ID_game_state: // 0x0001
            pRecBuf->ext_game_state = tempData->Data.ext_game_state;
            Cal_frame(&DJI_info.DJI_Frame[0]);
            break;
        case ID_game_result: // 0x0002
            pRecBuf->ext_game_result = tempData->Data.ext_game_result;
            Cal_frame(&DJI_info.DJI_Frame[1]);
            break;
        case ID_game_robot_survivors: // 0x0003
            pRecBuf->ext_game_robot_survivors = tempData->Data.ext_game_robot_survivors;
            Cal_frame(&DJI_info.DJI_Frame[2]);
            Ref_Bulid_If_Hurt(&pRecBuf->ext_game_robot_survivors);
            break;
        case ID_dart_status: // 0x0004
            pRecBuf->ext_dart_status = tempData->Data.ext_dart_status;
            Cal_frame(&DJI_info.DJI_Frame[3]);
            break;
        case ID_ICRA_buff_status: // 0x0005
            pRecBuf->ext_ICRA_buff_status = tempData->Data.ext_ICRA_buff_status;
            Cal_frame(&DJI_info.DJI_Frame[4]);
            break;
        case ID_event_data: // 0x0101
            pRecBuf->ext_event_data = tempData->Data.ext_event_data;
            Cal_frame(&DJI_info.DJI_Frame[5]);
            break;
        case ID_supply_projectile_action: // 0x0102
            pRecBuf->ext_supply_projectile_action = tempData->Data.ext_supply_projectile_action;
            break;
        case ID_referee_warning: // 0x0104
            Cal_frame(&DJI_info.DJI_Frame[6]);
            pRecBuf->ext_referee_warning = tempData->Data.ext_referee_warning;
            break;
        case ID_dart_remaining_time: // 0x0105
            Cal_frame(&DJI_info.DJI_Frame[7]);
            pRecBuf->ext_dart_remaining_time = tempData->Data.ext_dart_remaining_time;
            break;
        case ID_game_robot_state: // 0x0201
            RefReceiveTime.game_robot_state = rt_tick_get();
            Cal_frame(&DJI_info.DJI_Frame[8]);
            pRecBuf->ext_game_robot_state = tempData->Data.ext_game_robot_state;
            break;
        case ID_power_heat_data: // 0x0202
            RefReceiveTime.power_heat_data = rt_tick_get();
            Cal_frame(&DJI_info.DJI_Frame[9]);
            pRecBuf->ext_power_heat_data = tempData->Data.ext_power_heat_data;
            break;
        case ID_game_robot_pos: // 0x0203
            Cal_frame(&DJI_info.DJI_Frame[10]);
            pRecBuf->ext_game_robot_pos = tempData->Data.ext_game_robot_pos;
            break;
        case ID_buff_musk: // 0x0204
            RefReceiveTime.buff_musk = rt_tick_get();
            Cal_frame(&DJI_info.DJI_Frame[11]);
            pRecBuf->ext_buff_musk = tempData->Data.ext_buff_musk;
            break;
        case ID_aerial_robot_energy: // 0x0205
            Cal_frame(&DJI_info.DJI_Frame[12]);
            pRecBuf->aerial_robot_energy = tempData->Data.aerial_robot_energy;
            break;
        case ID_robot_hurt: // 0x0206
            pRecBuf->ext_robot_hurt = tempData->Data.ext_robot_hurt;
            Cal_frame(&DJI_info.DJI_Frame[13]);
            Ref_Robot_Set_Hurt();
            break;
        case ID_shoot_data: // 0x0207
            RefReceiveTime.shoot_data = rt_tick_get();
            Cal_frame(&DJI_info.DJI_Frame[14]);
            pRecBuf->ext_shoot_data = tempData->Data.ext_shoot_data;
            break;
        case ID_bullet_remaining: // 0x0208
            Cal_frame(&DJI_info.DJI_Frame[15]);
            pRecBuf->ext_bullet_remaining = tempData->Data.ext_bullet_remaining;
            break;
        case ID_rfid_status: // 0x0209
            Cal_frame(&DJI_info.DJI_Frame[16]);
            pRecBuf->ext_rfid_status = tempData->Data.ext_rfid_status;
            break;
        case ID_dart_client_cmd: // 0x020A
            Cal_frame(&DJI_info.DJI_Frame[17]);
            pRecBuf->ext_dart_client_cmd = tempData->Data.ext_dart_client_cmd;
            break;
        case ID_student_interactive_data: // 0x0301
            Cal_frame(&DJI_info.DJI_Frame[18]);
            pRecBuf->ext_send_user_data = tempData->Data.ext_send_user_data;
            break;
        case ID_robot_interactive_data: // 0x302
            Cal_frame(&DJI_info.DJI_Frame[19]);
            pRecBuf->robot_interactive_data = tempData->Data.robot_interactive_data;
            break;
        case ID_robot_command_map: // 0x303
            Cal_frame(&DJI_info.DJI_Frame[20]);
            pRecBuf->ext_robot_command_map = tempData->Data.ext_robot_command_map;
            break;
        case ID_robot_command_remote: // 0x304
            Cal_frame(&DJI_info.DJI_Frame[21]);
            pRecBuf->ext_robot_command_remote = tempData->Data.ext_robot_command_remote;
            break;
        default:
            break;
        } // switch
    }
    else
    {
        if (!package.flag)
            DJI_info.err_counts++;
        else
            DJI_info.err_counts_pac++;
    }
}


uint32_t counts_ob;
uint16_t nowFrameLens = 0;
/*解析裁判系统数据的逻辑代码*/
void DJI_protocol(uint8_t *SrcData, uint16_t srcSize, DJI_Data_t *pRecBuf, uint16_t bufSize)
{
    /*指向帧头*/
    uint8_t *pSrcData = SrcData;
    Frame_t *tempData;
    uint16_t length = 0; /*本包纯数据的长度*/
    int16_t realSrcSize = 0;

    volatile uint16_t lenTest = sizeof(FrameHeader_t);

    if (package.flag)
    {
        DJI_info.errState = needPakage;
        if (package.needLen < FRAMEMAXSEIZE)
        {
            memcpy(package.data + package.nowLen, SrcData, package.needLen);
            tempData = (Frame_t *)package.data;
            /*正常处理数据*/
            RecRefData(tempData, package.nowLen + package.needLen, pRecBuf);
        }
        else
        {
            DJI_info.errState = pakageLenErr;
            return;
        }

        package.flag = 0;
    }

    /*首先找到帧头*/
    while (*pSrcData != HEADER_SOF)
    {
        DJI_info.errState = firstHeadErr;
        pSrcData++;
        if (pSrcData - SrcData > srcSize - 1)
        {
            DJI_info.errState = headErr;
            return; /*最大偏移量为Size-1*/
        }
    }

    /*从帧头开始剩余源数长度-包含帧头*/
    realSrcSize = srcSize - (pSrcData - SrcData);
    if (realSrcSize <= 0)
        return;

    while (pSrcData < pSrcData + srcSize - 1)
    {
        /*得到一包数据*/
        tempData = (Frame_t *)pSrcData;

        /*得到本包纯数据的长度*/
        length = tempData->FrameHeader.DataLength;

        if (tempData->CmdID == 0x301||tempData->FrameHeader.DataLength==7)
        {
            counts_ob++;
        }

        /*判断串口数据是否合理*/
        if (length > DATAMAXSEIZE || length == 0)
        {
            DJI_info.errState = lenErr;
            return;
        }
 
        /*得到本次一帧的完整数据长度*/
        nowFrameLens = GetNowPackageLen(length);

        /*当发现本次需要拼包时*/
        if (realSrcSize < nowFrameLens)
        {
            package.nowLen = realSrcSize;
            package.needLen = nowFrameLens - realSrcSize;
            if (package.needLen <= 0 || package.needLen > DATAMAXSEIZE)
            {
                package.needLen = 0;
                DJI_info.errState = pakageLenErr;
                return;
            }
            memcpy(package.data, pSrcData, realSrcSize + 1);
            package.flag = 1;
            return;
        }

        // 判断是否会出现数组越界
        if ((uint32_t)(nowFrameLens) > (uint32_t)bufSize)
        {
            DJI_info.errState = bufSizeErr;
            return;
        }
        // 正常解析数据
        RecRefData(tempData, nowFrameLens, pRecBuf);

        /*指向下一数据头*/
        pSrcData += nowFrameLens;
    }
}


/***
 * @brief	串口回调
 * @param	dev 设备，size 字节数
 * @return	成功or失败
 ***/
static rt_err_t DJI_Callback(rt_device_t dev, rt_size_t size)
{
    struct DJI_Mxg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&DJI_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
        rt_kprintf("DJI: message queue full\n");
    return result;
}

/***
 * @brief:   接收裁判系统的帧数据并进行解析的线程
 *			 接收周期不定,有1Hz的，10Hz的，50Hz的等，具体看裁判系统手册
 * @param:   parameter: None
 * @return:  None
 ***/
static void DJI_Process_thread(void *parameter)
{
    struct DJI_Mxg msg;
    rt_err_t result;
    CREAT_ID(id);
    ADDTOMONITOR_ID("DJI_Process_thread", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);

    CREAT_ID(id1);
    ADDTOMONITOR_ID("RefUsartRx", 500, NULL, ALARM_RED, 1, id1);
    SWDG_START(id1);
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        result = rt_mq_recv(&DJI_mq, &msg, sizeof(msg), 500); // 等待回调函数的消息队列

        if (result == RT_EOK)
        {
            rt_device_read(msg.dev, 0, DJI_buffer, msg.size);
            SWDG_FEED(id1);
            DJI_protocol(DJI_buffer, msg.size, &DJI_ReadData, sizeof(DJI_ReadData)); // 对裁判系统数据进行处理
            rt_memset(&DJI_buffer, 0, sizeof(DJI_buffer));
        }

        SWDG_FEED(id);
    }
}

/**
 * @brief: 初始化裁判系统
 * @param {None}
 * @return {*}
 */
rt_err_t DJI_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_device_t DJI_Serial;

    DJI_info.Debug_UARTEn = 1;
#ifndef BSP_REF_USART_NAME
#error "please Choose ref usart device!"
#endif

    DJI_Serial = rt_device_find(BSP_REF_USART_NAME); // 使用默认的串口配置，配置为波特率 115200,8位数据位,1位停止位,无校验位
    if (!DJI_Serial)
    {
        rt_kprintf("rt_device_find DJI_UART failed !\n"); // while(1);
        return RT_ERROR;
    }

    /* 初始化消息队列 */
    res = rt_mq_init(&DJI_mq, "DJI_mq",
                     msg_pool,
                     sizeof(struct DJI_Mxg),
                     sizeof(msg_pool),
                     RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;

    /* DMA 接收模式 */
    res = rt_device_open(DJI_Serial, RT_DEVICE_FLAG_DMA_RX);
    if (res != RT_EOK)
        return res;

    res = rt_device_set_rx_indicate(DJI_Serial, DJI_Callback);
    if (res != RT_EOK)
        return res;

    rt_thread_t DJI_thread = rt_thread_create("DJI_Read",
                                              DJI_Process_thread,
                                              RT_NULL,
                                              THREAD_STACK_DJI,
                                              THREAD_PRIO_DJI,
                                              THREAD_TICK_DJI);
    if (DJI_thread != RT_NULL)
    {
        rt_thread_startup(DJI_thread);
    }
    else
    {
        return RT_ERROR;
    }

    return RT_EOK;
}

uint8_t RefDataIsOffline(void)
{
    return DJI_info.isValid;
}
