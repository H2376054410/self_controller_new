#include "drv_drawUI.h"
#include "string.h"
// 该文件实现了绘制 矩形 整圆 椭圆 圆弧 浮点数 整型数 字符等的纯数据内容数据
// 除了字符串其他都是7个图形一块发，提高效率
/****
 *
 * 裁判系统绘制自定义uI的图形操作库，定义了通用的自定义ui绘制包结构体，
 * 将线段、矩形、圆、椭圆、圆弧、整形、浮点数和字符串绘制的操作封装成函数，
 * 便于自定义UI绘制的快速开发。
 * 使用说明:
 * 注意图形操作的数量不可大干7
 * 若要进行图像删除,或者字符串绘制的操作,则无法与别的图形操作叠加，须单次调用custom_ui_erase_graph或者 ui_draw_text函数处理
 * 对 custom_ui_buff变量的图像操作完成后,即可通过裁判系统交互发送函数将其直接发送给对应客户端即可,
 * 其中,内容ID,为 custom_ui_buff_t变量内的cmd成员:图像数据长度(不包括3个ID的6个字节),为其中的length成员,全部为自动生成。
 ***/

// 1-7个图像数量对应的内容数据段长度   每次增加数据后更改发送数据长度需要和个数匹配
const uint16_t LENGTH_DRAW_NUM[7] = {LENGTH_DRAW_1_GRAPH, LENGTH_DRAW_2_GRAPH,
                                     LENGTH_DRAW_5_GRAPH, LENGTH_DRAW_5_GRAPH,
                                     LENGTH_DRAW_5_GRAPH, LENGTH_DRAW_5_GRAPH,
                                     LENGTH_DRAW_7_GRAPH};

// 1-7个图像搡作数量对应的内容ID   每次增加数据后更改发送数据ID需要和个数匹配
const uint16_t CMD_DRAW_NUM[7] = {CMD_DRAW_1_GRAPH, CMD_DRAW_2_GRAPH,
                                  CMD_DRAW_5_GRAPH, CMD_DRAW_5_GRAPH,
                                  CMD_DRAW_5_GRAPH, CMD_DRAW_7_GRAPH,
                                  CMD_DRAW_7_GRAPH};

// 发送时需要的结构体信息
custom_ui_data_t UI_data;

// 初始化数据为0
static void ui_data_init(void *data, uint16_t size)
{
    memset(data, 0x00, size);
}
void writeGraphNum(uint8_t num) // 外部接口
{
    //数目发送完成清0
    UI_data.graph_num = num;
}
uint8_t getIsCanAddGraph(void)
{
    // 数目发送完成清0
    return UI_data.graph_num < GRAPH_THREADhOLD ? 1 : 0;
}

/*初始化自定义ui绘制包结构体 */ // 外部接口
void custom_ui_init(void)
{
    UI_data.graph_num = 0;
    UI_data.data_s.len = 0;
    ui_data_init(UI_data.data_s.buff, sizeof(UI_data.data_s.buff));
}
custom_ui_data_t *getUISendDataInfo(void) // 外部接口
{
    return &UI_data;
}

// 将当前存储的待发送的UI数据全部发送出去
rt_err_t sendUIData(uint16_t senderID, uint16_t receiverID)
{
    return drv_UIdataSend(&UI_data, UI_data.data_s.len, senderID, receiverID);
}

/**
 *@brief 将自定义ui绘制包填充为清除图层
 *@param ui:需要修改的自定义ui绘制包结构体,参数是输入
 *@param clear_type:清除类型枚举clear_graph_type_t
 *@param layer：需要清除的图层号，仅在clear_type为CLEAR_GRAPH_TYPE_CLEAR_LAYER时有效
 *@retval UI_FULL:发送队列已满(擦除函数只能单独调用)
 *@retval UI_OK: 操作成功
 */
// 清除图层
ui_basic_e custom_ui_erase_graph(clear_graph_type_t clear_type, uint8_t layer)
{
    // 每个数据内容有 数据的内容ID 发送者的ID 接收者的ID 数据段  (发送者的ID 接收者的ID这个文件里不用)
    if (UI_data.graph_num > 0) // 单独发送
        return UI_FULL;

    /*赋值更新得到相关信息*/
    UI_data.data_s.len = 2;
    UI_data.data_cmd = CMD_ERASE_GRAPH;
    UI_data.graph_num = 1;
    UI_data.data_s.buff[0] = clear_type; // 图层操作
    UI_data.data_s.buff[1] = layer;      // 图层数0-9
    return UI_OK;
}

/**
 *@brief 将自定义ui绘制包填充为图像绘制
 *@param ui:需要修改的自定义ui绘制包结构体,参数是输入;
 *@param txt:绘制的字符串地址,参数是输入
 *@param layer:图层编号
 *@param name:图像名称,最大长度3字节
 *@param operation:图形操作类型枚举graph_operation_t
 *@param text:绘制的字符串地址
 *@param font size字体大小
 *@param length:绘制的字符串长度
 *@param width:绘制线宽
 *@param color:颜色
 *@param pos:字符串绘制坐标
 *@retval UI_FULL:发送队列已满(发送字节函数只能单独调用)
 *@retval UI_CHAR: 发送字节成功
 */
// 绘制字符串   图形名 图层 操作 字符串 字体大小 长 宽 颜色 起始xy  发送字节函数只能单独调用
ui_basic_e custom_ui_draw_text(uint8_t layer, uint32_t name, graph_operation_t operation,
                               char *text, uint16_t font_size,
                               uint16_t textLen, uint16_t width,
                               graph_color_t color, uint16_t start_x, uint16_t start_y)
{
    ext_client_custom_character_t custom_char;
    memset(&custom_char, 0, sizeof(custom_char));

    if (UI_data.graph_num > 0) // 只能单独发送
        return UI_FULL;

    /*按协议得到相关数据*/
    memcpy(custom_char.grapic_data_struct.graphic_name, &name, 3);
    custom_char.grapic_data_struct.layer = layer;
    custom_char.grapic_data_struct.graphic_tpye = GRAPH_TYPE_CHAR;
    custom_char.grapic_data_struct.operate_tpye = operation;
    custom_char.grapic_data_struct.start_angle = font_size;
    custom_char.grapic_data_struct.end_angle = textLen;
    custom_char.grapic_data_struct.width = width;
    custom_char.grapic_data_struct.color = color;
    custom_char.grapic_data_struct.start_x = start_x;
    custom_char.grapic_data_struct.start_y = start_y;
    memcpy(custom_char.data, text, textLen);

    /*赋值更新得到相关信息*/
    memcpy(UI_data.data_s.buff, &custom_char, LENGTH_DRAW_TEXT);
    UI_data.graph_num = 1;
    UI_data.data_s.len = LENGTH_DRAW_TEXT;
    UI_data.data_cmd = CMD_DRAW_TEXT;

    return UI_CHAR;
}

/**
 *@brief 向自定义ui绘制包内新增-线条绘制操作
 *@param ui: 需要修改的自定义ui绘制包结构体
 *@param	layer:图层编号
 *@param	name:图像名称，最大长度3字节
 *@param	operation:图形操作类型枚举graph_operation_t
 *@param	width:绘制线宽
 *@param	color:颜色
 *@param	startPos 起点坐标
 *@param	endPos : 终点坐标
 *@retval UI_FULL:发送队列已满(7个)
 *@retval UI_OK: 操作成功
 */
// 关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_line_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));

    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;

    /*按协议得到相关数据*/
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_LINE;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data.property.graph_property.end_x = end_x;
    graph_data.property.graph_property.end_y = end_y;
    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH,
           &graph_data, ONE_GRAPH_LENGTH);

    /*赋值更新得到相关信息*/
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1]; // 每次增加数据后更改发送数据长度 和个数匹配
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];      // 每次增加数据后更改发送数据ID 和个数匹配
    return UI_OK;
}

/**
 *@brief	向自定义ui绘制包内新增-矩形绘制操作
 *@param ui:需要修改的自定义ui绘制包结构体，参数是输入
 *@param	layer:图层编号
 *@param	name:图像名称，最大长度3字节
 *@param	operat 1on:图形操作类型枚举graph_operation_t
 *@param	width:绘制线宽
 *@param	color:颜色
 *@param	start_ x:起点坐标x
 *@param	start_ y:起点坐标y
 *@param	end_x:对角顶点x坐标
 *@param	end_y:对角顶点y坐标
 *@retval UI_FULL:发送队列已满(7个)
 *@retval UI_OK: 操作成功
 */
// 关键为前两个--图层和名字
ui_basic_e custom_ui_append_rect_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                         uint16_t width, graph_color_t color,
                                         uint16_t start_x, uint16_t start_y,
                                         uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));
    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;

    /*按协议得到相关数据*/
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_RECT;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data.property.graph_property.end_x = end_x;
    graph_data.property.graph_property.end_y = end_y;
    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH,
           &graph_data, ONE_GRAPH_LENGTH);

    /*赋值更新得到相关信息*/
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1];
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-radious : 半径
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
                name:图像名称，最大长度3字节
                operat 1on:图形操作类型枚举graph_operation_t
                width:绘制线宽
                color:颜色
                centre_x:圆心坐标x
                centre_y:圆心坐标y
                radious : 半径
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
////关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_cirle_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t width, graph_color_t color,
                                          uint16_t centre_x, uint16_t centre_y,
                                          uint16_t radious)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));
    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_CIRCLE;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = centre_x;
    graph_data.start_y = centre_y;
    graph_data.property.graph_property.radious = radious;
    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH);
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1];
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-椭圆绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
                name:图像名称，最大长度3字节
                operat 1on:图形操作类型枚举graph_operation_t
                width:绘制线宽
                color:颜色
                centre_x:圆心坐标x
                centre_y:圆心坐标y
                axis_x : x半轴长度
                axis_y : y半轴长度
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
// 关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_ellipse_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                            uint16_t width, graph_color_t color,
                                            uint16_t centre_x, uint16_t centre_y,
                                            uint16_t axis_x, uint16_t axis_y)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));
    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_ELLIPSE;
    graph_data.operate_tpye = operation;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = centre_x;
    graph_data.start_y = centre_y;
    graph_data.property.graph_property.end_x = axis_x;
    graph_data.property.graph_property.end_y = axis_y;
    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH);
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1];
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-（椭）圆弧绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
                name:图像名称，最大长度3字节
                start_angle
                end_angle
                operation:图形操作类型枚举graph_operation_t
                width:绘制线宽
                color:颜色
                centre_x:圆心坐标x
                centre_y:圆心坐标y
                radious : 半径
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
// 关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_arc_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t start_angle, uint16_t end_angle,
                                        uint16_t width, graph_color_t color,
                                        uint16_t centre_x, uint16_t centre_y,
                                        uint16_t axis_x, uint16_t axis_y)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));
    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_ARC;
    graph_data.operate_tpye = operation;
    graph_data.start_angle = start_angle;
    graph_data.end_angle = end_angle;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = centre_x;
    graph_data.start_y = centre_y;
    graph_data.property.graph_property.end_x = axis_x;
    graph_data.property.graph_property.end_y = axis_y;
    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH);
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1];
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-浮点数绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
                name:图像名称，最大长度3字节
                operation:图形操作类型枚举graph_operation_t
                font_size:字体大小
                digit:小数点位数
                width:绘制线宽
                color:颜色
                start_x:起点坐标x
                start_y:起点坐标y
                value : 要绘制的浮点数
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
// 关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_float_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                          uint16_t font_size, uint16_t digit,
                                          uint16_t width, graph_color_t color,
                                          uint16_t start_x, uint16_t start_y, float value)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));
    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_FLOAT;
    graph_data.operate_tpye = operation;
    graph_data.start_angle = font_size;
    graph_data.end_angle = digit;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data.property.float_val = (int32_t)(value * 1000);

    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH);
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1];
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];
    return UI_OK;
}

/*0brief	向自定义ui绘制包内新增-整形绘制操作
@parem[: in] [out] ui:需要修改的自定义ui绘制包结构体，参数是输入
@par am[ in]	layer:图层编号
                name:图像名称，最大长度3字节
                operation:图形操作类型枚举graph_operation_t
                font_size:字体大小
                width:绘制线宽
                color:颜色
                start_x:起点坐标x
                start_y:起点坐标y
                value : 要绘制的整形
*@retval UI_FULL:发送队列已满(7个)
*@retval UI_OK: 操作成功
*/
// 关键为前两个-- 图层和名字
ui_basic_e custom_ui_append_int_operate(uint8_t layer, uint32_t name, graph_operation_t operation,
                                        uint16_t font_size, uint16_t width, graph_color_t color,
                                        uint16_t start_x, uint16_t start_y, int32_t value)
{
    graphic_data_struct_t graph_data;
    ui_data_init(&graph_data, sizeof(graph_data));
    if (UI_data.graph_num >= GRAPH_THREADhOLD)
        return UI_FULL;
    memcpy(graph_data.graphic_name, &name, 3);
    graph_data.layer = layer;
    graph_data.graphic_tpye = GRAPH_TYPE_INT;
    graph_data.operate_tpye = operation;
    graph_data.start_angle = font_size;
    graph_data.width = width;
    graph_data.color = color;
    graph_data.start_x = start_x;
    graph_data.start_y = start_y;
    graph_data.property.int_val = value;

    memcpy(UI_data.data_s.buff + UI_data.graph_num * ONE_GRAPH_LENGTH, &graph_data, ONE_GRAPH_LENGTH);
    UI_data.graph_num++;
    UI_data.data_s.len = LENGTH_DRAW_NUM[UI_data.graph_num - 1];
    UI_data.data_cmd = CMD_DRAW_NUM[UI_data.graph_num - 1];
    return UI_OK;
}
