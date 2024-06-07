#include "drv_uiSend.h"
#include "mod_RefSystem.h"
#include "string.h"
#include "drv_CRC.h"
#include "ui_interface.h"
/*
【基本格式 frame_header(5-byte)   cmd_id(2-byte)   data(n-byte)   frame_tai(2-byte)】
frame_header:
    帧头 SOF data_length seq CRC8
data:
    【数据段的内容ID+发送者ID+接收者内容ID+数据段---所以数据长度加6】
        数据段的内容ID(0x0100-0x0104 0x0110)：
            己方机器人间通信 客户端删除图形 客户端绘制【一到七】个图形
        数据内容
            图形名 图形操作 图形操作 图层数 颜色 颜色
            终止角度 线宽 起点x坐标 起点y坐标
            字体大小或者半径 终点x坐标 终点y坐标
frame_tail:
    crc16
*/

/*最终发送给裁判系统的数据带帧头帧尾等数据 可以debug*/
uint8_t referee_intercom_buff[MAX_CUSTOM_UI_BUFF_SIZE];

static rt_device_t serial; // 发送设备

/***
* @name     帧头+ cmd_id (2-byte)+ data (n-byte) +frame_tail
* @brief	向裁判系统传输UI数据
* @param	senderID: 发送者ID:
            红方机器人	英雄 1; 工程 2; 步兵 3/4/5; 空中 6;
            蓝方机器人    英雄 11; 工程 12; 步兵 13/14/15; 空中 16;
* @param	receiverID: 接受者ID
            红方操作手客户端	英雄 0x101; 工程 0x102; 步兵 0x103/104/105; 空中 0x106
            蓝方操作手客户端    英雄 0x111; 工程 0x112; 步兵 0x113/114/115; 空中 0x116
***/
rt_err_t drv_UIdataSend(custom_ui_data_t *UI_data, uint8_t dataLen,
                        uint16_t senderID, uint16_t receiverID)
{
    const uint8_t seq = 0x01; // 包序号
    uint32_t res = RT_ERROR;

    /*命令码 机器人间交互数据，发送方触发发送，上限 10Hz*/
    uint16_t header = ID_student_interactive_data;
    uint16_t datalength = dataLen + 6; // 纯数据长度
    uint8_t frontHalf = 0;             // 除了帧尾的所有字节数

    custom_ui_data_t custom_ui_data;
    memcpy(&custom_ui_data, UI_data, sizeof(custom_ui_data));

    // 初始化为0
    memset(referee_intercom_buff, 0x00, sizeof(referee_intercom_buff));

    /*帧头--SOF data_length seq CRC8---纯帧头5个字节*/
    UI_Frame_t data;
    data.txFrameHeader.SOF = HEADER_SOF;
    data.txFrameHeader.DataLength = datalength;
    data.txFrameHeader.Seq = seq;
    data.txFrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&data.txFrameHeader,
                                                 sizeof(data.txFrameHeader) - sizeof(data.txFrameHeader.CRC8), CRC8_INIT);

    /*数据段的内容ID+发送者ID+接收者内容ID+数据段---所以数据长度加6*/
    data.CmdID = header;
    data.dataFrameHeader.data_cmd_id = custom_ui_data.data_cmd;
    data.dataFrameHeader.send_ID = senderID;
    data.dataFrameHeader.receiver_ID = receiverID;
    memcpy(data.userData.data, custom_ui_data.data_s.buff,
           custom_ui_data.data_s.len); /*数据*/

    // 或者sizeof(data.txFrameHeader) + sizeof(data.CmdID) + sizeof(data.dataFrameHeader) + custom_ui_data.data_s.len;
    frontHalf = sizeof(UI_Frame_t) - (Robot_Interact_DataLen - custom_ui_data.data_s.len) - sizeof(data.FrameTail);

    /*帧尾*/
    data.FrameTail = Get_CRC16_Check_Sum((uint8_t *)&data, frontHalf, CRC_INIT);

    /*因为数据不一定满所以分段赋值*/
    memcpy(referee_intercom_buff, &data, frontHalf);
    memcpy(referee_intercom_buff + frontHalf, (const void *)&data.FrameTail, sizeof(data.FrameTail));

    UI_data->graph_num = 0; // 发送完清0

    res = rt_device_write(serial, 0, referee_intercom_buff, frontHalf + sizeof(data.FrameTail));

   UI_sendEvent(ui_sendReady_eve);   // 防止该函数被打断
   UI_sendEvent(ui_needRefresh_eve); // 线程间同步，通知发送完成
    return res;
}

rt_err_t uiUsartInit(void)
{
    serial = rt_device_find(BSP_REF_USART_NAME);
    if (!serial)
        return RT_ERROR;
    else
        return RT_EOK;
}
