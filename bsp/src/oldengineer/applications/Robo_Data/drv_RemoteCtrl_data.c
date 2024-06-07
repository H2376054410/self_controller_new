/**
 * @file drv_RemoteCtrl_data.c
 * @brief 机器人遥控数据
 * @author mylj
 * @version 1.0
 * @date 2023-05-07
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_RemoteCtrl_data.h"

// 机械臂遥控状态
struct RoboCtrl_Data_s
{
    unsigned RoboCtrl_State : 2;       // 0 表示处于单独遥控器模式,
                                       // 1 代表处于客户端模式
                                       // 2 代表不控制模式
    unsigned AirPump_State : 4;        // 0 表示气泵关闭，电磁阀打开
                                       // 1 代表气泵打开，电磁阀关闭
    unsigned Push_State : 3;           // 0 不推
                                       // 1 推矿准备
                                       // 2 开始推矿
    unsigned FloorOre_State : 4;       // 0 不控制
                                       // 1 地上矿石模式
                                       // 2 开始捡地上矿石
                                       // 3 转矿动作1
                                       // 4 转矿动作2
    unsigned Speed_State : 4;          // 0 普通模式
                                       // 1 加速模式
                                       // 2 减速模式
    unsigned Ore_State : 4;            // 0 普通模式
                                       // 1 银矿模式
                                       // 2 金矿模式
                                       // 3 地上矿模式
    unsigned GoldOre_State : 4;        // 0 不控制
                                       // 1 金矿对准
                                       // 2 上拔矿石
    unsigned SliverOre_State : 4;      // 0 不控制
                                       // 1 银矿对准
                                       // 2 上拔矿石
    unsigned ExpandCtrl_State : 2;     // 0 非拓展模式
                                       // 1 控制小臂Roll或大臂z状态
    unsigned Running_State : 2;        // 0 底盘有零设定值
                                       // 1 底盘有非零设定值
    unsigned XYXtrans_State : 3;       // 图传坐标系转换复位
    unsigned RoboCtrl_RC_State : 6;    // 机械臂控制状态
    unsigned ImageFollow_State : 2;    // 图传跟随状态
    unsigned CPUReset_State : 2;       // 单片机复位状态
    unsigned GyroMode_State : 2;       // 陀螺模式状态
    unsigned ChassisReverse_State : 2; // 底盘电机反向
    unsigned ArmPosInit_State : 1;     // 是否为机械臂初始状态
    unsigned Obstacleblock_State : 1;  // 障碍快状态
} RoboCtrl_State_Struct = {
    .RoboCtrl_State = 0,
    .AirPump_State = 0,
    .Push_State = 0,
    .Speed_State = 0,
    .FloorOre_State = 0,
    .Ore_State = 0,
    .GoldOre_State = 0,
    .SliverOre_State = 0,
    .ExpandCtrl_State = 0,
    .Running_State = 0,
    .XYXtrans_State = 0,
    .RoboCtrl_RC_State = 0,
    .ImageFollow_State = 0,
    .CPUReset_State = 0,
    .GyroMode_State = 0,
    .ChassisReverse_State = 0,
    .ArmPosInit_State = 0,
    .Obstacleblock_State = 0,
}
;

static float Reserve; // 本变量没有任何意义, 只是用于保护下面的这个函数不会出现空指针操作
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
                   void *Data)
{
    static void *reserve = (void *)&Reserve;
    if ((Read == DateMode) && (NULL == Data))
        // 如果要读数据但是传入空指针则考虑进行保护
        Data = reserve;

    switch (State_Type)
    {
    case RoboCtrl_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.RoboCtrl_State));
        else
            RoboCtrl_State_Struct.RoboCtrl_State = (uint32_t)Data;
        return Data;
    case AirPump_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.AirPump_State));
        else
            RoboCtrl_State_Struct.AirPump_State = (uint32_t)Data;
        return Data;
    case Push_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.Push_State));
        else
            RoboCtrl_State_Struct.Push_State = (uint32_t)Data;
        return Data;
    case FloorOre_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.FloorOre_State));
        else
            RoboCtrl_State_Struct.FloorOre_State = (uint32_t)Data;
        return Data;
    case Speed_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.Speed_State));
        else
            RoboCtrl_State_Struct.Speed_State = (uint32_t)Data;
        return Data;
    case Ore_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.Ore_State));
        else
            RoboCtrl_State_Struct.Ore_State = (uint32_t)Data;
        return Data;
    case GoldOre_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.GoldOre_State));
        else
            RoboCtrl_State_Struct.GoldOre_State = (uint32_t)Data;
        return Data;
    case SliverOre_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.SliverOre_State));
        else
            RoboCtrl_State_Struct.SliverOre_State = (uint32_t)Data;
        return Data;
    case ExpandCtrl_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.ExpandCtrl_State));
        else
            RoboCtrl_State_Struct.ExpandCtrl_State = (uint32_t)Data;
        return Data;
    case Running_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.Running_State));
        else
            RoboCtrl_State_Struct.Running_State = (uint32_t)Data;
        return Data;
    case XYXtrans_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.XYXtrans_State));
        else
            RoboCtrl_State_Struct.XYXtrans_State = (uint32_t)Data;
        return Data;
    case RoboCtrl_RC_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.RoboCtrl_RC_State));
        else
            RoboCtrl_State_Struct.RoboCtrl_RC_State = (uint32_t)Data;
        return Data;
    case ImageFollow_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.ImageFollow_State));
        else
            RoboCtrl_State_Struct.ImageFollow_State = (uint32_t)Data;
    case CPUReset_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.CPUReset_State));
        else
            RoboCtrl_State_Struct.CPUReset_State = (uint32_t)Data;
        return Data;
    case GyroMode_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.GyroMode_State));
        else
            RoboCtrl_State_Struct.GyroMode_State = (uint32_t)Data;
        return Data;
    case ChassisReverse_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.ChassisReverse_State));
        else
            RoboCtrl_State_Struct.ChassisReverse_State = (uint32_t)Data;
        return Data;
    case ArmPosInit_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.ArmPosInit_State));
        else
            RoboCtrl_State_Struct.ArmPosInit_State = (uint32_t)Data;
        return Data;
    case Obstacleblock_State:
        if (Read == DateMode)
            return (Data = (void *)((uint32_t)RoboCtrl_State_Struct.Obstacleblock_State));
        else
            RoboCtrl_State_Struct.Obstacleblock_State = (uint32_t)Data;
        return Data;
    default:
        return NULL;
    }
}
