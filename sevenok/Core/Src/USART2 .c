#include "USART2.h"


typedef union
{
    float a;
    uint8_t b[4];
} float2char_u;
uint8_t frame[39];
struct SendPack send_pack;
void sendpack()
	 {
    /// 原：先发高八位再发低八位
    int data_length = 30;
    int fronthalf = 37;
    int cmd_id = 0x0302;
    /// 帧头5字节、指令id两字节、数据30字节、帧尾2字节
    /// 帧头
    /// 数据帧起始字节
    frame[0] = 0xA5;
    send_pack.head.SOF = 0xA5;
    /// 数据帧data长度，这里是30
    frame[2] = (int16_t)(data_length)>>8;
    frame[1] = (int16_t)(data_length);
    send_pack.head.data_length = data_length;
    /// 包序号
    // frame[3] = 0x02;
    frame[3] = 0x5c;
    send_pack.head.seq = 0x5c;
    /// 帧头CRC8校验
    send_pack.head.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&frame,
                                     4, CRC8_INIT);
    frame[4] = send_pack.head.CRC8;
    /*********************************************以上为帧头部分*********************************************/
    // frame[4] = 0x05;
    /// 指令ID
    frame[6] = (int16_t)(cmd_id)>>8;
    frame[5] = (int16_t)cmd_id;
    /// 数据帧
    /// 自定义控制器工作状况
    frame[7] = 0;
    // 坐标
    float2char_u cvt;
    cvt.a = send_pack.BY;
    frame[8] = cvt.b[0];
    frame[9] = cvt.b[1];
    frame[10] = cvt.b[2];
    frame[11] = cvt.b[3];

    cvt.a = send_pack.BP1;
    frame[12] = cvt.b[0];
    frame[13] = cvt.b[1];
    frame[14] = cvt.b[2];
    frame[15] = cvt.b[3];

    cvt.a = send_pack.BP2;
    frame[16] = cvt.b[0];
    frame[17] = cvt.b[1];
    frame[18] = cvt.b[2];
    frame[19] = cvt.b[3];

    cvt.a = send_pack.FY;
    frame[20] = cvt.b[0];
    frame[21] = cvt.b[1];
    frame[22] = cvt.b[2];
    frame[23] = cvt.b[3];

    cvt.a = send_pack.FP;
    frame[24] = cvt.b[0];
    frame[25] = cvt.b[1];
    frame[26] = cvt.b[2];
    frame[27] = cvt.b[3];

    cvt.a = send_pack.FR;
    frame[28] = cvt.b[0];
    frame[29] = cvt.b[1];
    frame[30] = cvt.b[2];
    frame[31] = cvt.b[3];


    /// 剩下用0补齐
    frame[32] = 0;
    frame[33] = 0;
    frame[34] = 0;
    frame[35] = 0;
    frame[36] = 0;
    /// 帧尾
    /// CRC16
    send_pack.tail = Get_CRC16_Check_Sum((uint8_t *) &frame, fronthalf, CRC_INIT);
    frame[38] = (int16_t)(send_pack.tail)>>8;
    frame[37] = (int16_t)(send_pack.tail);

}



