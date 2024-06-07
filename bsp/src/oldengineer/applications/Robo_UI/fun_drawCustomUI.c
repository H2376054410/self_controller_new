#include "fun_drawCustomUI.h"
#include "ui_interface.h"
#if defined ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif
#include "drv_utils.h"
#include "CustomUI_GraphParameters.h"
#include "stdint.h"
/*该文件实现具体UI功能代码同时实现线程间同步*/

#define M_SETBIT(x, y) ((x) |= (1 << y))  // 将X的第Y位置1 标号从0开始
#define M_CLRBIT(x, y) ((x) &= ~(1 << y)) // 将X的第Y位清0 标号从0开始
#define M_READBIT(x, y) ((x) & (1 << y))  // 读取某一位 标号从0开始

ui_circle_t Debug;

UI_Used_Data_t exData; // UI需要使用到的外部数据

// 所有的动态图层的图层操作变量 首先需要增加图层然后再修改图像，当需要刷新时只需将该变量置为 MODIFY_GRAPH
static graph_operation_t operation;
static rt_uint8_t ui_erase_flg = 0; // ui擦除标志位

// 发送前检查同时含线程间同步
void ui_startSend(ui_basic_e endState)
{
    uint8_t state = getIsCanAddGraph();
    if ((!state) || (endState != UI_OK))
    {
        UI_sendEvent(ui_dataReady_eve);   // 尝试唤醒发送线程将数据发送出去
        UI_waitEvent(ui_needRefresh_eve); // 等待数据发送出去
    }
}

// 发送完成后检查-实现线程间同步
void ui_endSend(ui_basic_e endState)
{
    switch (endState)
    {
    case UI_FULL:
    case UI_CHAR:
        UI_sendEvent(ui_dataReady_eve);   // 尝试 唤醒发送线程将数据发送出去
        UI_waitEvent(ui_needRefresh_eve); // 等待数据发送出去
        break;
    default:
        break;
    }
}

// 得到UI外部需要的数据
void getUIexData(void)
{
    exData = Get_UI_Data();
}

// set the flag when UI finished drawing.
ui_basic_e UI_dynRefresh(void *param)
{
    UI_Set_Init_Flag();
    return UI_OK;
}

// 一轮链表发送完成
ui_basic_e UI_drawEnd(void *param)
{
    ui_endSend(UI_FULL);
    return UI_OK;
}

// 集中操作某些图像
void UI_Clear_Init_Flag(void)
{
    ui_erase_flg = 1;
    operation = ADD_GRAPH;
}

// 集中操作某些图像
void UI_Set_Init_Flag(void)
{
    ui_erase_flg = 0;
    operation = MODIFY_GRAPH;
}

// 填充整个屏幕
ui_basic_e Fill_Full_Screen(void *param)
{
    ui_basic_e state = UI_FULL;

    state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_Fill_Full_Screen,
                                          ADD_GRAPH, 1080, GRAPH_COLOR_GREEN,
                                          0, 540, 1920, 540);

    ui_endSend(state);
    return UI_OK;
}

// 清屏 即删除所有图层
ui_basic_e Clear_Screen(void *param)
{
    ui_basic_e state = UI_FULL;
    ui_startSend(UI_CHAR);
    state = custom_ui_erase_graph(CLEAR_GRAPH_TYPE_CLEAR_ALL, 1);

    ui_endSend(state);

    return UI_OK;
}

/*------------------------------------XXX----------------------------------------------------------*/

#define copyStartPoint(preEnd, nowStart) \
    do                                   \
    {                                    \
        nowStart.x = preEnd.x;           \
        nowStart.y = preEnd.y;           \
    } while (0)

// 角度单位为度 [-180,180]，水平为0,逆时针增大
static void getRotationLine(const ui_point_t *const startPoint, ui_point_t *const endPoint,
                            float angle, float len)
{
    while (angle > 360)
        angle -= 360;
    while (angle < -360)
        angle += 360;

    endPoint->x = startPoint->x + len * cosf(DEG2RAD_f(angle));
    endPoint->y = startPoint->y + len * sinf(DEG2RAD_f(angle));
}

// 绘制气泵状态
ui_basic_e Draw_pump_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;
    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_Pump_bkgd,
                                           ADD_GRAPH, 3, GRAPH_COLOR_ORANGE,
                                           170, 850, 15);
    ui_endSend(state);

    char txt[25] = {"pump"};
    ui_startSend(UI_CHAR);
    state = custom_ui_draw_text(BACKGROUND_LAYER, UI_Name_Pump_bkgd1, ADD_GRAPH,
                                txt, 10, sizeof(txt), 2, GRAPH_COLOR_YELLOW,
                                154, 820);
    ui_endSend(state);

    return UI_OK;
}

// 绘制气泵状态--动态
ui_basic_e Draw_pump_dyn(void *param)
{
    ui_basic_e state = UI_FULL;
    uint8_t nowstate = exData.state.pumpStatus;
    static uint8_t s = 0;
    if (s != nowstate || ui_erase_flg)
    {
        if (nowstate)
        {
            // 异常就绘制
            state = custom_ui_append_cirle_operate(DYNAMIC_LAYER, UI_Name_Pump_dyn,
                                                   ADD_GRAPH, 14, GRAPH_COLOR_GREEN,
                                                   170, 850, 7);
        }
        else
        {
            // 正常就擦除
            state = custom_ui_append_cirle_operate(DYNAMIC_LAYER, UI_Name_Pump_dyn,
                                                   ERASE_GRAPH, 2, GRAPH_COLOR_ORANGE,
                                                   170, 850, 15);
        }
        ui_endSend(state);
        s = nowstate;
    }
    return UI_OK;
}

// 绘制前臂的roll轴状态
ui_basic_e Draw_forearmRoll_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;

    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_forearmRoll_bkgd,
                                           ADD_GRAPH, 3, GRAPH_COLOR_ORANGE,
                                           240, 850, 15);
    ui_endSend(state);

    char txt[25] = {"Roll"};
    ui_startSend(UI_CHAR);
    state = custom_ui_draw_text(BACKGROUND_LAYER, UI_Name_forearmRoll_bkgd1, ADD_GRAPH,
                                txt, 10, sizeof(txt), 2, GRAPH_COLOR_YELLOW,
                                225, 820);
    ui_endSend(state);
    return UI_OK;
}

// 绘制前臂的roll轴状态--动态
ui_basic_e Draw_forearmRoll_dyn(void *param)
{
    ui_basic_e state = UI_FULL;
    uint8_t nowstate = exData.state.ForearmRoll;
    static uint8_t s = 0;
    if (s != nowstate || ui_erase_flg)
    {
        if (nowstate)
        {
            // 异常就绘制
            state = custom_ui_append_cirle_operate(DYNAMIC_LAYER, UI_Name_forearmRoll_dyn,
                                                   ADD_GRAPH, 14, GRAPH_COLOR_GREEN,
                                                   240, 850, 7);
        }
        else
        {
            // 正常就擦除
            state = custom_ui_append_cirle_operate(DYNAMIC_LAYER, UI_Name_forearmRoll_dyn,
                                                   ERASE_GRAPH, 2, GRAPH_COLOR_ORANGE,
                                                   240, 850, 15);
        }
        ui_endSend(state);
        s = nowstate;
    }
    return UI_OK;
}

// 绘制图传状态
ui_basic_e Draw_imageTrans_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;

    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_imageTrans_bkgd,
                                           ADD_GRAPH, 3, GRAPH_COLOR_ORANGE,
                                           310, 850, 15);
    ui_endSend(state);

    char txt[25] = {"image"};

    ui_startSend(UI_CHAR);
    state = custom_ui_draw_text(BACKGROUND_LAYER, UI_Name_imageTrans_bkgd1, ADD_GRAPH,
                                txt, 10, sizeof(txt), 2, GRAPH_COLOR_YELLOW,
                                289, 820);
    ui_endSend(state);

    return UI_OK;
}

// 绘制图传状态--动态
ui_basic_e Draw_imageTrans_dyn(void *param)
{
    ui_basic_e state = UI_FULL;
    uint8_t nowstate = exData.state.ImageTrans;
    static uint8_t s = 0;
    if (s != nowstate || ui_erase_flg)
    {
        if (nowstate)
        {
            state = custom_ui_append_cirle_operate(DYNAMIC_LAYER, UI_Name_imageTrans_dyn,
                                                   ADD_GRAPH, 14, GRAPH_COLOR_GREEN,
                                                   310, 850, 7);
        }
        else
        {
            state = custom_ui_append_cirle_operate(DYNAMIC_LAYER, UI_Name_imageTrans_dyn,
                                                   ERASE_GRAPH, 14, GRAPH_COLOR_ORANGE,
                                                   310, 850, 7);
        }
        ui_endSend(state);
        s = nowstate;
    }
    return UI_OK;
}

// 绘制底盘控制模式的字符串
static void drawCtrlMenuCharbkdg(const ui_point_t line[6])
{
    char txt[6][25] = {"Chassis_Norma", "Chassis_Trim", "Chassis_Extra",
                       "Arm_Pos", "Arm_Angle", "Arm_Ext"};
    ui_basic_e state = UI_FULL;
    ui_point_fix_t fix[6];

    fix[0].x = 15, fix[0].y = 5;
    fix[1].x = -50, fix[1].y = 20;
    fix[2].x = -140, fix[2].y = 5;
    fix[3].x = -72, fix[3].y = -5;
    fix[4].x = -40, fix[4].y = -12;
    fix[5].x = 14, fix[5].y = -5;

    for (int i = 0; i < 6; i++)
    {
        ui_startSend(UI_CHAR);
        state = custom_ui_draw_text(BACKGROUND_LAYER, UI_Name_ctrlMenu_char1 + i, ADD_GRAPH,
                                    txt[i], 10, sizeof(txt[i]), 2, GRAPH_COLOR_ORANGE,
                                    line[i].x + fix[i].x,
                                    line[i].y + fix[i].y);
        ui_endSend(state);
    }
}

// 绘制底盘控制模式
ui_basic_e Draw_ctrlMenu_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;

    ui_circle_t circle = {240, 660, 80};
    const uint16_t len = circle.r / 4;

    // start end
    ui_point_t line[LinePoint][6];

    // 绘制外圆
    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_ctrlMenu_bkgd,
                                           ADD_GRAPH, 5, GRAPH_COLOR_REDBLUE,
                                           circle.c.x, circle.c.y, circle.r);
    ui_endSend(state);

    for (int i = 0; i < 6; i++)
    {
        // 得到起始点
        getRotationLine(&circle.c, &line[PointStart][i], 30 + 60 * i, circle.r - len);

        // 得到末尾点
        getRotationLine(&line[PointStart][i], &line[PointEnd][i], 30 + 60 * i, len);
        // 绘制单线条
        state = custom_ui_append_line_operate(BACKGROUND_LAYER, UI_Name_ctrlMenu_Line1 + i,
                                              ADD_GRAPH, 3, GRAPH_COLOR_REDBLUE,
                                              line[0][i].x, line[0][i].y, line[1][i].x, line[1][i].y);
        ui_endSend(state);
    }

    drawCtrlMenuCharbkdg(line[1]);

    return UI_OK;
}

// 绘制底盘控制模式--动态
ui_basic_e Draw_ctrlMenu_dyn(void *param)
{
    ui_basic_e state = UI_FULL;
    uint8_t nowstate = exData.ctrlMenu; // 0-6 0为消失
    static uint8_t s = 0;

    ui_circle_t circle = {240, 660, 80};
    // 得到起始坐标点

    ui_point_t end;
    if (s != nowstate || ui_erase_flg)
    {
        if (menu_ChassisNoCtrl == exData.ctrlMenu)
        {
            // 清除单线条
            state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_ctrlMenu_Line,
                                                  ERASE_GRAPH, 3, GRAPH_COLOR_YELLOW,
                                                  circle.c.x, circle.c.y, end.x, end.y);
        }
        else
        {
            getRotationLine(&circle.c, &end, 30 + 60 * (exData.ctrlMenu - 1), circle.r / 2);

            // 绘制单线条
            state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_ctrlMenu_Line,
                                                  ADD_GRAPH, 3, GRAPH_COLOR_YELLOW,
                                                  circle.c.x, circle.c.y, end.x, end.y);
        }

        ui_endSend(state);
        s = nowstate;
    }
    return UI_OK;
}

ui_point_t point_debug1 = {980, 525};
ui_point_t point_debug2 = {1375, 140};

// 绘制瞄准框
ui_basic_e Draw_aimingFrame_bkgd(void *param)
{
    // ui_basic_e state = UI_FULL;

    // state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_aimingFrame_bkgd, ADD_GRAPH,
    //                                       1, GRAPH_COLOR_YELLOW,
    //                                       1005, 750,
    //                                       1900, 100);
    // // state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_aimingFrame_bkgd, ADD_GRAPH,
    // //                                       1, GRAPH_COLOR_YELLOW,
    // //                                       point_debug1.x,
    // //                                       point_debug1.y,
    // //                                       point_debug2.x,
    // //                                       point_debug2.y);
    // ui_endSend(state);
     return UI_OK;
}
// 绘制瞄准框
ui_basic_e Draw_aimingFrame_dyn(void *param)
{
    ui_basic_e state = UI_FULL;
    uint8_t nowstate = exData.Ore_State;

    if (nowstate == ore_silver)
    {
        state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_aimingFrame_dyn1, ADD_GRAPH,
                                              1, GRAPH_COLOR_YELLOW,
                                              1030, 535,
                                              1900, 100);
    }
    else
    {
        state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_aimingFrame_dyn1, ERASE_GRAPH,
                                              1, GRAPH_COLOR_YELLOW,
                                              1030, 535,
                                              1900, 100);
    }

    if (nowstate == ore_floor)
    {
        state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_aimingFrame_dyn2, ADD_GRAPH,
                                              1, GRAPH_COLOR_YELLOW,
                                              980, 525,
                                              1375, 140);
    }
    else
    {
        state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_aimingFrame_dyn2, ERASE_GRAPH,
                                              1, GRAPH_COLOR_YELLOW,
                                              980, 525,
                                              1375, 140);
    }

    ui_startSend(state);
    ui_endSend(state);
    
    return UI_OK;
}

// 绘制车道线
ui_basic_e Draw_laneLines_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;
    ui_line_t line[2];
    line[0].start.x = 400;
    line[0].start.y = 100;
    line[0].len = 600;
    line[0].angle = 68;

    line[1].start.x = 600;
    line[1].start.y = 100;
    line[1].len = 600;
    line[1].angle = 112;

    for (int i = 0; i < 1; i++)
    {
        // 根据角度和长度得到末尾点坐标
        getRotationLine(&line[i].start, &line[i].end, line[i].angle, line[i].len);

        // 绘制单线条
        state = custom_ui_append_line_operate(BACKGROUND_LAYER, UI_Name_laneLines_bkgd1 + i,
                                              ADD_GRAPH, 1, GRAPH_COLOR_YELLOW,
                                              line[i].start.x, line[i].start.y,
                                              line[i].end.x, line[i].end.y);

        ui_endSend(state);
    }
    return UI_OK;
}

// 绘制车道线
ui_basic_e Draw_laneLines_dyn(void *param)
{

    return UI_OK;
}

// 绘制机械臂yaw轴的状态
ui_basic_e Draw_armYaw_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;

    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_armYaw_bkgd1,
                                           ADD_GRAPH, 4, GRAPH_COLOR_WHITE,
                                           1140, 80, 4);
    ui_endSend(state);
    state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_armYaw_bkgd2,
                                          ADD_GRAPH, 3, GRAPH_COLOR_WHITE,
                                          1115, 55, 1165, 105);
    ui_startSend(state);
    ui_endSend(state);
    state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_armYaw_bkgd4,
                                          ADD_GRAPH, 4, GRAPH_COLOR_WHITE,
                                          1113, 64, 1117, 70);
    ui_startSend(state);
    ui_endSend(state);
    state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_armYaw_bkgd3,
                                          ADD_GRAPH, 3, GRAPH_COLOR_WHITE,
                                          1163, 64, 1167, 70);
    ui_startSend(state);
    ui_endSend(state);
    state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_armYaw_bkgd6,
                                          ADD_GRAPH, 3, GRAPH_COLOR_WHITE,
                                          1113, 89, 1117, 96);
    ui_startSend(state);
    ui_endSend(state);
    state = custom_ui_append_rect_operate(BACKGROUND_LAYER, UI_Name_armYaw_bkgd5,
                                          ADD_GRAPH, 3, GRAPH_COLOR_WHITE,
                                          1163, 89, 1167, 96);
    ui_startSend(state);
    ui_endSend(state);
    return UI_OK;
}

// 绘制机械臂yaw轴的状态--动态
ui_basic_e Draw_armYaw_dyn(void *param)
{
    ui_basic_e state = UI_FULL;

    // angle state
    static float s[2][2];

    ui_simpleline_t base = {1110, 80, 1170, 80};

    // 得到基本参数
    ui_line_t line[2];
    line[0].start.x = (base.end.x + base.start.x) / 2;
    line[0].start.y = base.start.y;
    line[0].len = 60;
    line[0].angle = RAD2DEG_f(exData.LargeArmYaw);
    line[0].state = exData.state.largeArmYaw;

    line[1].len = 24;
    line[1].angle = RAD2DEG_f(exData.smallArmYaw);
    line[1].state = exData.state.smallArmYaw;

    for (int i = 0; i < 2; i++)
    {
        if (i)
            copyStartPoint(line[i - 1].end, line[i].start); // 得到起始点

        if (s[i][0] != line[i].angle || s[i][1] != line[i].state || ui_erase_flg)
        {
            // 根据角度和长度得到末尾点坐标
            getRotationLine(&line[i].start, &line[i].end, line[i].angle, line[i].len);

            if (line[i].state == lineNormal)
            {
                // 绘制单线条
                state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_armYaw_dyn1 + i,
                                                      operation, 4, GRAPH_COLOR_WHITE,
                                                      line[i].start.x, line[i].start.y,
                                                      line[i].end.x, line[i].end.y);
            }
            else if (line[i].state == lineAbnormal)
            {
                // 绘制单线条
                state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_armYaw_dyn1 + i,
                                                      operation, 4, GRAPH_COLOR_GREEN,
                                                      line[i].start.x, line[i].start.y,
                                                      line[i].end.x, line[i].end.y);
            }
            ui_endSend(state);
            s[i][0] = line[i].angle;
            s[i][1] = line[i].state;
        }
    }
    return UI_OK;
}

// 绘制机械臂pitch轴的状态
ui_basic_e Draw_armPitch_bkgd(void *param)
{
    ui_basic_e state = UI_FULL;

    state = custom_ui_append_line_operate(BACKGROUND_LAYER, UI_Name_armPitch_bkgd1,
                                          ADD_GRAPH, 4, GRAPH_COLOR_WHITE,
                                          1260, 70, 1360, 70);
    ui_endSend(state);
    state = custom_ui_append_line_operate(BACKGROUND_LAYER, UI_Name_armPitch_bkgd2,
                                          ADD_GRAPH, 4, GRAPH_COLOR_WHITE,
                                          1240, 60, 1380, 60);
    ui_endSend(state);
    state = custom_ui_append_line_operate(BACKGROUND_LAYER, UI_Name_armPitch_bkgd3,
                                          ADD_GRAPH, 4, GRAPH_COLOR_WHITE,
                                          1240, 60, 1260, 70);
    ui_endSend(state);
    state = custom_ui_append_line_operate(BACKGROUND_LAYER, UI_Name_armPitch_bkgd4,
                                          ADD_GRAPH, 4, GRAPH_COLOR_WHITE,
                                          1380, 60, 1360, 70);

    ui_endSend(state);
    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_armPitch_bkgd5,
                                           ADD_GRAPH, 3, GRAPH_COLOR_WHITE,
                                           1270, 53, 5);
    ui_endSend(state);

    state = custom_ui_append_cirle_operate(BACKGROUND_LAYER, UI_Name_armPitch_bkgd6,
                                           ADD_GRAPH, 3, GRAPH_COLOR_WHITE,
                                           1350, 53, 5);
    ui_startSend(state);
    ui_endSend(state);
    return UI_OK;
}

// 绘制机械臂pitch轴的状态--动态
ui_basic_e Draw_armPitch_dyn(void *param)
{
    ui_basic_e state = UI_FULL;

    // angle state
    static float s[3][2];
    static uint16_t counts;

    ui_simpleline_t base = {1260, 72, 1360, 72};

    // 得到基本参数
    ui_line_t line[3];
    line[0].start.x = (base.end.x + base.start.x) / 2;
    line[0].start.y = base.start.y;
    line[0].len = 72;
    line[0].angle = RAD2DEG_f(exData.smallArmPitch1);
    line[0].state = exData.state.smallArmPitch1;

    line[1].len = 60;
    line[1].angle = RAD2DEG_f(exData.smallArmPitch2);
    line[1].state = exData.state.smallArmPitch2;

    line[2].len = 24;
    line[2].angle = RAD2DEG_f(exData.smallArmPitch3);
    line[2].state = exData.state.smallArmPitch3;

    for (int i = 0; i < 3; i++)
    {
        if (i)
            copyStartPoint(line[i - 1].end, line[i].start); // 得到起始点

        if (s[i][0] != line[i].angle || s[i][1] != line[i].state || ui_erase_flg || counts++ < 240)
        {
            // 根据角度和长度得到末尾点坐标
            getRotationLine(&line[i].start, &line[i].end, line[i].angle, line[i].len);

            if (line[i].state == lineNormal)
            {
                // 绘制单线条
                state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_armPitch_dyn1 + i,
                                                      operation, 4, GRAPH_COLOR_WHITE,
                                                      line[i].start.x, line[i].start.y,
                                                      line[i].end.x, line[i].end.y);
            }
            else
            {
                // 绘制单线条
                state = custom_ui_append_line_operate(DYNAMIC_LAYER, UI_Name_armPitch_dyn1 + i,
                                                      operation, 4, GRAPH_COLOR_GREEN,
                                                      line[i].start.x, line[i].start.y,
                                                      line[i].end.x, line[i].end.y);
            }
            ui_endSend(state);
            s[i][0] = line[i].angle;
            s[i][1] = line[i].state;
        }
    }
    return UI_OK;
}
