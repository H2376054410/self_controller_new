#ifndef _UI_INTERFACE_H__
#define _UI_INTERFACE_H__

#include "board.h"

#define UI_START_DATA_ALL 1

typedef enum
{
    lineNormal = 0,
    lineAbnormal = 1,
} ui_lineState_e;

typedef enum
{
    menu_ChassisNoCtrl,
    menu_ChassisNorma,
    menu_ChassisFineTunin,
    menu_ChassisCustCtrl,
    menu_ArmPos,
    menu_ArmAngle,
    menu_ArmCtrl,
} ui_ctrlMenu_e;

typedef enum
{
    ore_floor = 1,
    ore_silver,
    ore_gold,
    ore_aim,
} ui_oremode_e;

// 一些0/1变量
typedef struct
{
    unsigned UI_Reset_Flag : 1;  // 重置 UI 的标志位, 翻转时需要重置
    unsigned pumpStatus : 1;     // 气泵状态 1为打开 0关闭
    unsigned smallArmYaw : 1;    // 0正常，1超限
    unsigned largeArmYaw : 1;    // 0正常，1超限
    unsigned smallArmPitch1 : 1; // 0正常，1超限
    unsigned smallArmPitch2 : 1; // 0正常，1超限
    unsigned smallArmPitch3 : 1; // 0正常，1超限
    unsigned ForearmRoll : 1;    // 0正常，1超限
    unsigned ImageTrans : 1;     // 0正常，1超限

} robotBoolState;

// UI需要获取的数据
typedef struct UI_Used_Data
{
    robotBoolState state;
    uint8_t ctrlMenu;       // 从低位依次向低位改变模式 0为无控制
    unsigned Ore_State : 3; // 当前矿石模式
    float LargeArmYaw;      // 角度值 单位弧度 水平为0 逆时针增大
    float smallArmYaw;      // 角度值 单位弧度 水平为0 逆时针增大
    float smallArmPitch1;   // 角度值 单位弧度 水平为0 逆时针增大
    float smallArmPitch2;   // 角度值 单位弧度 水平为0 逆时针增大
    float smallArmPitch3;   // 角度值 单位弧度 水平为0 逆时针增大

} UI_Used_Data_t;

typedef enum
{
    ui_50ms_eve = 1 << 1,        // 每50ms检查是否可以发送
    ui_dataReady_eve = 1 << 2,   // 检查是否需要发送
    ui_sendReady_eve = 1 << 3,   // 检查上一次是否发送完成即是否可以发送,用于发送线程自锁
    ui_needRefresh_eve = 1 << 4, // 检查上一次是否发送完成，用于更新线程与发送线程同步
} UI_threadsyn_e;

UI_Used_Data_t Get_UI_Data(void);
rt_uint8_t Get_UI_Reset_Flag(void);
rt_err_t UI_synInit(void);
rt_err_t UI_waitForSend(void);
rt_err_t UI_sendEvent(UI_threadsyn_e event);
rt_err_t UI_waitEvent(UI_threadsyn_e event);
#endif
