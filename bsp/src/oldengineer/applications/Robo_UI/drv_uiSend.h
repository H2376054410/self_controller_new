#ifndef _DRV_UISEND_H_
#define _DRV_UISEND_H_
#include "board.h"

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

// 一次发送给裁判系统纯数据内容的最大字节数
#define MAX_CUSTOM_UI_BUFF_LENGTH 113

// 一次发送裁判系统的数据最大长度    ui本身一次发送最大长度加上帧头帧尾等
#define MAX_CUSTOM_UI_BUFF_SIZE 128

typedef struct
{
    uint8_t buff[MAX_CUSTOM_UI_BUFF_LENGTH]; // ui本身数据
    uint8_t len;
} UI_array_t;

// 绘制ui本身的数据
typedef struct custom_ui_data
{
    uint16_t data_cmd; // 数据段的内容ID 即是发送几个图形
    UI_array_t data_s; // ui的具体数据不带帧头帧尾等
    uint8_t graph_num; // 统计一次发送的图形数目 目前为只有满7个才发送
} custom_ui_data_t;

// 直接发送数据
rt_err_t drv_UIdataSend(custom_ui_data_t *UI_data, uint8_t dataLen,
                        uint16_t senderID, uint16_t receiverID);
rt_err_t uiUsartInit(void);
#endif
