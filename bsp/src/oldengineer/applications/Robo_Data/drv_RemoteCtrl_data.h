#ifndef __DRV_REMOTECTRL_DATA_H__
#define __DRV_REMOTECTRL_DATA_H__
#include <rtdevice.h>

typedef enum
{
    Remote_Mode = 0, // 单独遥控器模式
    Client_Mode,     // 客户端模式
    NoCtrl_Mode      // 不控制模式
} RoboCtrl_State_e;  // 机器人控制状态

typedef enum
{
    AirPumpState_Close = 0, // 气泵关闭状态
    AirPumpState_Open       // 气泵关闭状态
} AirPump_State_e;          // 气泵状态

typedef enum
{
    Still_Mode = 0, // 静止
    Running_Mode,   // 有非零设定值
} Running_State_e;  // 底盘当前有非零设定值

typedef enum
{
    NoPush_Mode = 0, // 不推
    PushReady_Mode,  // 推矿准备
    PushStart_Mode,  // 开始推矿
} Push_State_e;      // 推矿状态

typedef enum
{
    NoFloorOre_Mode = 0,     // 不控制
    FloorOreImageReady_Mode, // 地上矿图传准备模式
    FloorOreReady_Mode,      // 地上矿石准备模式
    FloorOrePullup_Mode,     // 地上矿上拔状态
    FloorOreLaydown_Mode,    // 地上矿下放状态
} FloorOre_State_e;          // 地上矿的状态

typedef enum
{
    NormalSpeed_Mode = 0, // 不控制
    Acceleration_Mode,    // 加速模式
    Deceleration_Mode     // 减速模式
} Speed_State_e;          // 速度模式

typedef enum
{
    NormalOre_Mode = 0,  // 正常矿石状态
    FloorOre_Mode,       // 地上矿模式
    SliverOre_Mode,      // 银矿模式
    FrontGoldOre_Mode,   // 正金矿模式
    SideGoldOre_Mode,    // 侧金矿模式
    AimOre3level_Mode,   // 三级兑矿模式
    AimOre4level_L_Mode, // 左四级兑矿模式
    AimOre4level_R_Mode, // 右四级兑矿模式
} Ore_State_e;           // 矿石模式

typedef enum
{
    NoGold_Mode = 0,   // 不控制
    FrontGoldAim_Mode, // 正金矿对准
    SideGoldAim_Mode,  // 侧金矿对准
    GoldPullup_Mode    // 上拔矿石
} GoldOre_State_e;     // 金矿状态

typedef enum
{
    NoSliver_Mode = 0, // 不控制
    SliverAim_Mode,    // 银矿对准状态
    SliverGrip_Mode,   // 银矿吸取状态
    SliverPullup_Mode, // 银矿上拔状态
} SliverOre_State_e;   // 银矿状态
typedef enum
{
    NoExpand_Mode = 0, // 非拓展模式
    EnExpand_Mode,     // 控制小臂Roll或大臂z状态
} ExpandCtrl_State_e;  // 是否控制小臂Roll或大臂z状态

typedef enum
{
    XYXtransReset_Mode = 0, // 坐标系转换复位
    XYXtransing_Mode,       // 正在进行坐标系转换
} XYXtrans_State_e;         // 坐标系转换状态

typedef enum
{
    NoCPUReset_Mode = 0, // 单片机不复位模式
    CPUReset_Mode,       // 单片机复位模式
} CPUReset_State_e;      // 底盘当前有非零设定值

typedef enum
{
    RoboNoCtrl_Mode = 0,    // 机械臂无控制模式
    ChassisNormal_Mode,     // 正常底盘模式（键盘控制）
    ChassisFineTuning_Mode, // 微调底盘模式（鼠标控制）
    ArmPos_Mode,            // 机械臂末端坐标点控制模式
    ArmAngle_Mode,          // 机械臂角度控制模式
    ArmCtrl_Mode,           // 6轴机械臂同时控制模式
    ArmCtrlReady_Mode,      // 6轴机械臂同时控制准备模式
} RoboCtrl_RC_State_e;      // 机械臂遥控模式

typedef enum
{
    Notfollowing_Mode = 0,  // 图传不跟随模式
    Following_Mode,         // 图传跟随模式
} ImageTransFollow_State_e; // 图传跟随状态

typedef enum
{
    UIReset_Mode1 = 0, // UI重置状态1
    UIReset_Mode2,     // UI重置状态2
} UIReset_State_e;     // UI重启标志位

typedef enum
{
    ChassisNoFollowing = 0,  // 底盘不跟随模式
    ChassisFollowing,        // 底盘跟随模式
    GyroMode,                // 陀螺模式
    GyroDisable_activeMode,  // 陀螺模式主动禁用
    GyroDisable_passiveMode, // 陀螺模式被动禁用
} GyroMode_State_e;          // 陀螺模式状态

typedef enum
{
    NoUIResetConfirm = 0, // UI重置状态1
    UIResetConfirm,       // UI重置状态2
} UIResetConfirm_State_e; // UI重启标志位

typedef enum
{
    NoChassisReverse = 0, // 底盘方向不返向
    ChassisReverse,       // 底盘方向反向
} ChassisReverse_State_e; // UI重启标志位

typedef enum
{
    NoArmPosInit = 0, // 机械臂非初始状态
    ArmPosInit,       // 机械臂初始状态
} ArmPosInit_State_e; // UI重启标志位

typedef enum
{
    Obstacleblock_Aim = 0, // 机械臂非初始状态
    Obstacleblock_Pullup,  // 机械臂初始状态
} Obstacleblock_State_e;   // 障碍快状态

typedef enum
{
    Write = 0, // 对结构体的数据进行写入
    Read       // 对结构体的数据进行读取
} RoboCtrl_DateMode_e;

typedef enum
{
    // 这一组数据的返回值为 uint8_t 类型, 使用时为 (uint8_t)Data
    RoboCtrl_State = 0,   // 机器人总控模式
    AirPump_State,        // 是否打开气泵
    Push_State,           // 是否推矿
    FloorOre_State,       // 地上矿的状态
    Speed_State,          // 速度模式
    Ore_State,            // 矿石模式
    GoldOre_State,        // 金矿状态
    SliverOre_State,      // 银矿状态
    ExpandCtrl_State,     // 是否控制小臂Roll或大臂z
    Running_State,        // 底盘当前有非零设定值
    XYXtrans_State,       // 图传坐标系转换复位
    RoboCtrl_RC_State,    // 机械臂控制状态
    ImageFollow_State,    // 图传跟随状态
    UIReset_State,        // UI重启状态
    ImagePos_State,       // 坐标系状态
    CPUReset_State,       // 单片机复位状态
    UIResetConfirm_State, // UI重置确认
    GyroMode_State,       //
    ChassisReverse_State, // 底盘反向
    ArmPosInit_State,
    Obstacleblock_State, // 障碍快状态
} RoboCtrl_Data_e;

/**
 * @brief  从文件外部读取或存入想要的控制数据
 * @author fwlh
 * @author mylj
 * @param  State_Type       指定需要读取的数据
 * @param  DateMode           动作的类型(写入或者读取)
 * @param  Data             需要写入或者读取到的数据
 * ! 对于标志位的修改, 只需要传入 (void *)Number, 返回值使用方法为 (uint32_t)(Control_Robot())
 * ! 对于浮点型设定值的修改, 需要传入 (void *)&Number, 返回值使用方法为 *(float *)Control_Robot()
 * @return void*            为了方便使用, 返回写入或者读出的数据, 使用时注意数据类型
 */
void *Robo_Control(RoboCtrl_Data_e State_Type,
                   RoboCtrl_DateMode_e DateMode,
                   void *Data);

#endif /*__DRV_REMOTECTRL_DATA_H__*/
