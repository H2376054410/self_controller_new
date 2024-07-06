#ifndef _DRV_DRAWUI_H_
#define _DRV_DRAWUI_H_
#include "drv_uiSend.h"
#include "fun_protocolRef.h"

/*
自定义控制器交互数据包括一个统一的数据段头结构。数据段为内容数据段，整个交互数据的包总共长最大为
39 个字节，减去 frame_header, cmd_id 和 frame_tail 共 9 个字节，故而发送的内容数据段最大为 30 字
节。整个交互数据 0x0302 的包下行行频率为 30Hz
*/

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

// 一个图形操作对应的数据大小 字节
#define ONE_GRAPH_LENGTH 15

// 一次发送的图形数量的阈值
#define GRAPH_THREADhOLD 7

// 1-7个图像数量对应的内容数据段长度
#define LENGTH_DRAW_1_GRAPH (1 * ONE_GRAPH_LENGTH)
#define LENGTH_DRAW_2_GRAPH (2 * ONE_GRAPH_LENGTH)
#define LENGTH_DRAW_5_GRAPH (5 * ONE_GRAPH_LENGTH)
#define LENGTH_DRAW_7_GRAPH (7 * ONE_GRAPH_LENGTH)
#define LENGTH_DRAW_TEXT (5 * ONE_GRAPH_LENGTH)

// 1-7的图像搡作数量对应的内容ID 和手册一致
#define CMD_ERASE_GRAPH 0x0100  // 客户端删除图形内容ID
#define CMD_DRAW_1_GRAPH 0x0101 // 客户端绘制一个图形内容ID
#define CMD_DRAW_2_GRAPH 0x0102 // 客户端绘制二个图形内容ID
#define CMD_DRAW_5_GRAPH 0x0103 // 客户端绘制五个图形内容ID
#define CMD_DRAW_7_GRAPH 0x0104 // 客户端绘制七个图形内容ID
#define CMD_DRAW_TEXT 0x0110    // 客户端绘制字符内容ID
/*------------------------------------发送图形内容----------------------------------------------------------*/

// 最终的所有图形数据 总共15个字节
typedef __packed struct
{
    // 图形名 在删除，修改等操作中，作为客户端的索引
    uint8_t graphic_name[3]; // 只有三个字节故只能用数字来表示

    // 图形配置 1  将32个位划分为了一组变量，若干个数据。
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;

    // 图形配置 2  将32个位划分为了一组变量，若干个数据。
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;

    // 图形配置  将32个位划分为了一组变量，若干个数据。
    //  字体大小或者半径； Bit 10 - 20：终点 x 坐标； Bit 21 - 31：终点 y 坐标。
    __packed union
    {
        __packed struct // 当为圆或直线时
        {
            uint32_t radious : 10; // 直线时不使用
            uint32_t end_x : 11;
            uint32_t end_y : 11;
        } graph_property;

        int32_t int_val;   // 绘制整数
        int32_t float_val; // 绘制浮点数
    } property;
} graphic_data_struct_t;

// 客户端绘制字符
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;
/*------------------------------------发送图形内容----------------------------------------------------------*/

/*------------------------------------图形操作相关----------------------------------------------------------*/

// 删除图形操作枚举
typedef enum
{
    CLEAR_GRAPH_TYPE_CLEAR_NULL = 0x00,  // 空操作
    CLEAR_GRAPH_TYPE_CLEAR_LAYER = 0x01, // 删除图层
    CLEAR_GRAPH_TYPE_CLEAR_ALL = 0x02,   // 删除所有
} clear_graph_type_t;

// 返回值类型 用于发送时使用
typedef enum
{
    UI_OK = 0,
    UI_FULL, // 当数据满时则释放信号量
    UI_ERR,
    UI_CHAR
} ui_basic_e;

// 图层数：0~9
typedef enum layer_num
{
    BACKGROUND_LAYER = 0, // 背景图形图层号
    DYNAMIC_LAYER,        // 动态图形图层号

    UI_layer_2,
    UI_layer_3,
    UI_layer_4,
    UI_layer_5,
    UI_layer_6,
    UI_layer_7,
    UI_layer_8,
    UI_layer_9,

} layer_num;

// 图形操作枚举
typedef enum
{
    GRAPH_OPERATION_NOOP = 0x00, // 空操作
    ADD_GRAPH = 0x01,            // 增加
    MODIFY_GRAPH = 0x02,         // 修改
    ERASE_GRAPH = 0x03,          // 删除
} graph_operation_t;

// 图形类型枚举
typedef enum
{
    GRAPH_TYPE_LINE = 0x00,    // 直线
    GRAPH_TYPE_RECT = 0x01,    // 矩形
    GRAPH_TYPE_CIRCLE = 0x02,  // 整圆
    GRAPH_TYPE_ELLIPSE = 0x03, // 椭圆
    GRAPH_TYPE_ARC = 0x04,     // 圆弧
    GRAPH_TYPE_FLOAT = 0x05,   // 浮点数
    GRAPH_TYPE_INT = 0x06,     // 整型数
    GRAPH_TYPE_CHAR = 0x07,    // 字符 只能单独发送
} graph_type_t;

/// 图形颜色枚举
typedef enum
{
    GRAPH_COLOR_REDBLUE = 0x00, // 红蓝主色
    GRAPH_COLOR_YELLOW = 0x01,  // 黄色
    GRAPH_COLOR_GREEN = 0x02,   // 绿色；
    GRAPH_COLOR_ORANGE = 0x03,  // 橙色；
    GRAPH_COLOR_VIOLET = 0x04,  // 紫红色；
    GRAPH_COLOR_PINK = 0x05,    // 粉色；
    GRAPH_COLOR_CYAN = 0x06,    // 青色；
    GRAPH_COLOR_BLACK = 0x07,   // 黑色；
    GRAPH_COLOR_WHITE = 0x08,   // 白色；
} graph_color_t;
/*------------------------------------图形操作相关----------------------------------------------------------*/


/*
当为直线时顺序为长和宽
当为点坐标时顺序为x和y
*/
typedef struct
{
    uint16_t x;
    uint16_t y;

} UIVector2_t;

/*------------------------------------函数声明----------------------------------------------------------*/

void writeGraphNum(uint8_t num);

// 得到当前数据是否满
uint8_t getIsCanAddGraph(void);

// 将当前存储的待发送的UI数据全部发送出去
rt_err_t sendUIData(uint16_t senderID, uint16_t receiverID);

custom_ui_data_t *getUISendDataInfo(void);

/*初始化自定义ui绘制包结构体  ui:需要初始化的自定义u1绘制包结构体指针*/
void custom_ui_init(void);

// 将自定义ui绘制包填充为清除图层 关键为前两个--图层和名字
ui_basic_e custom_ui_erase_graph(clear_graph_type_t clear_type, uint8_t layer);

// 绘制字符串   图形名 图层 操作 字符串 字体大小 长 宽 颜色 起始xy  发送字节函数只能单独调用
ui_basic_e custom_ui_draw_text(uint8_t layer, uint32_t name, graph_operation_t operation,
                               char *text, uint16_t font_size,
                               uint16_t textLen, uint16_t width,
                               graph_color_t color, uint16_t start_x, uint16_t start_y);
// 关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_line_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y);

// 向自定义ui绘制包内新增-矩形绘制操作 关键为前两个--图层和名字
ui_basic_e custom_ui_append_rect_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y);

// 向自定义ui绘制包内新增-radious : 半径 关键为前两个--图层和名字
ui_basic_e custom_ui_append_cirle_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t width, graph_color_t color,
                                          uint16_t centre_x, uint16_t centre_y,
                                          uint16_t radious);

// 向自定义ui绘制包内新增-椭圆绘制操作 关键为前两个--图层和名字
ui_basic_e custom_ui_append_ellipse_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                            uint16_t width, graph_color_t color,
                                            uint16_t centre_x, uint16_t centre_y,
                                            uint16_t avis_x, uint16_t avis_y);

// 向自定义ui绘制包内新增-（椭）圆弧绘制操作 关键为前两个--图层和名字
ui_basic_e custom_ui_append_arc_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t start_angle, uint16_t end_angle,
                                        uint16_t width, graph_color_t color,
                                        uint16_t centre_x, uint16_t centre_y,
                                        uint16_t axis_x, uint16_t axis_y);

// 向自定义ui绘制包内新增-浮点数绘制操作 关键为前两个--图层和名字
ui_basic_e custom_ui_append_float_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t font_size, uint16_t digit,
                                          uint16_t width, graph_color_t color,
                                          uint16_t start_x, uint16_t start_y, float value);

// 向自定义ui绘制包内新增-整形绘制操作 关键为前两个--图层和名字
ui_basic_e custom_ui_append_int_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t font_size, uint16_t width, graph_color_t color,
                                        uint16_t start_x, uint16_t start_y, int32_t value);

#endif
