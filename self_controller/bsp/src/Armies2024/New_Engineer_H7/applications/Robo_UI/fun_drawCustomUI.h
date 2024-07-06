#ifndef _FUN_DRAWCUSTOMUI_H_
#define _FUN_DRAWCUSTOMUI_H_

#include "drv_drawUI.h"

// 每个图形的名字枚举 在删除，修改等操作中，作为客户端的索引
typedef enum UI_image_name
{
    UI_Name_clear = 0,
    UI_Name_Fill_Full_Screen,

    /* 绘制气泵状态*/
    UI_Name_Pump_dyn,
    UI_Name_Pump_bkgd,
    UI_Name_Pump_bkgd1,

    /* 绘制前臂的roll轴状态*/
    UI_Name_forearmRoll_dyn,
    UI_Name_forearmRoll_bkgd,
    UI_Name_forearmRoll_bkgd1,

    /* 绘制图传状态*/
    UI_Name_imageTrans_dyn,
    UI_Name_imageTrans_bkgd,
    UI_Name_imageTrans_bkgd1,


    /* 绘制底盘控制模式*/
    UI_Name_ctrlMenu_dyn,
    UI_Name_ctrlMenu_bkgd,
    UI_Name_ctrlMenu_Line, // 中间指针
    UI_Name_ctrlMenu_Line1,
    UI_Name_ctrlMenu_LineEnd = UI_Name_ctrlMenu_Line1 + 5,
    UI_Name_ctrlMenu_char1,
    UI_Name_ctrlMenu_charEnd = UI_Name_ctrlMenu_char1 + 5,

    /* 绘制瞄准框*/
    UI_Name_aimingFrame_dyn,
    UI_Name_aimingFrame_bkgd,
    UI_Name_aimingFrame_dyn1,
    UI_Name_aimingFrame_dyn2,
    /* 绘制车道线*/
    UI_Name_laneLines_dyn,
    UI_Name_laneLines_bkgd1,
    UI_Name_laneLines_bkgd2,

    /* 绘制机械臂yaw轴的状态*/
    UI_Name_armYaw_dyn1,
    UI_Name_armYaw_dyn2,
    UI_Name_armYaw_bkgd1,
    UI_Name_armYaw_bkgd2,
    UI_Name_armYaw_bkgd3,
    UI_Name_armYaw_bkgd4,
    UI_Name_armYaw_bkgd5,
    UI_Name_armYaw_bkgd6,

    /* 绘制机械臂pitch轴的状态*/
    UI_Name_armPitch_dyn1,
    UI_Name_armPitch_dyn2,
    UI_Name_armPitch_dyn3,
    UI_Name_armPitch_bkgd1,
    UI_Name_armPitch_bkgd2,
    UI_Name_armPitch_bkgd3,
    UI_Name_armPitch_bkgd4,
    UI_Name_armPitch_bkgd5,
    UI_Name_armPitch_bkgd6,

    // 空操作
    UI_Name_Null_operation,

    // 对于动态图层改变参数实现修改
    UI_Name_dynRefresh,

    // 一轮循环结束
    UI_Name_Draw_END,

} UI_imageName_e;

typedef enum
{
    PointStart,
    PointEnd,
    LinePoint,
} ui_linePoint_e;

typedef struct
{
    uint16_t x;
    uint16_t y;
} ui_point_t;

typedef struct
{
    int16_t x;
    int16_t y;
} ui_point_fix_t;

typedef struct
{
    ui_point_t c;
    uint16_t r;
} ui_circle_t;

typedef struct
{
    ui_point_t start;
    ui_point_t end;
} ui_simpleline_t;

typedef struct
{
    ui_point_t start;
    ui_point_t end;
    uint16_t len;
    float angle;
    uint8_t state;
} ui_line_t;

// 发送前检查同时含线程间同步
void ui_startSend(ui_basic_e endState);

// 发送完成后检查-实现线程间同步
void ui_endSend(ui_basic_e endState);

// 得到UI外部需要的数据
void getUIexData(void);

// 一轮链表发送完成
ui_basic_e UI_drawEnd(void *param);
void UI_Clear_Init_Flag(void);
void UI_Set_Init_Flag(void);

ui_basic_e Fill_Full_Screen(void *param);

// 清屏
ui_basic_e Clear_Screen(void *param);

// UI结束用以动态刷新
ui_basic_e UI_dynRefresh(void *param);

// 得到UI外部需要的数据
void getUIexData(void);

// 绘制气泵状态--静态
ui_basic_e Draw_pump_bkgd(void *param);

// 绘制气泵状态--动态
ui_basic_e Draw_pump_dyn(void *param);

// 绘制前臂的roll轴状态--静态
ui_basic_e Draw_forearmRoll_bkgd(void *param);

// 绘制前臂的roll轴状态--动态
ui_basic_e Draw_forearmRoll_dyn(void *param);

// 绘制图传状态--静态
ui_basic_e Draw_imageTrans_bkgd(void *param);

// 绘制图传状态--动态
ui_basic_e Draw_imageTrans_dyn(void *param);

// 绘制底盘控制模式--静态
ui_basic_e Draw_ctrlMenu_bkgd(void *param);

// 绘制底盘控制模式--动态
ui_basic_e Draw_ctrlMenu_dyn(void *param);

// 绘制瞄准框--静态
ui_basic_e Draw_aimingFrame_bkgd(void *param);

// 绘制瞄准框--动态
ui_basic_e Draw_aimingFrame_dyn(void *param);

// 绘制车道线--静态
ui_basic_e Draw_laneLines_bkgd(void *param);

// 绘制车道线--动态
ui_basic_e Draw_laneLines_dyn(void *param);

// 绘制机械臂yaw轴的状态--静态
ui_basic_e Draw_armYaw_bkgd(void *param);

// 绘制机械臂yaw轴的状态--动态
ui_basic_e Draw_armYaw_dyn(void *param);

// 绘制机械臂yaw轴的状态--静态
ui_basic_e Draw_armPitch_bkgd(void *param);

// 绘制机械臂pitch轴的状态--动态
ui_basic_e Draw_armPitch_dyn(void *param);
#endif
