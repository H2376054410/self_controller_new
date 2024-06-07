/**
 * @file    app_RoboRemote_Ctrl.c
 * @author  mylj
 * @brief   本文件用于处理键盘的回调函数
 * @version 1.3
 * @date    2023-07-23
 * @copyright Copyright (c) 2023 哈尔滨工业大学(威海)HERO战队
 */
#include "app_RemoteCallback_Set.h"
#include "drv_remote.h"
#include "drv_Key_Set.h"
#include "drv_ExactSmooth.h"
#include "func_Key_Record.h"
#include "func_MusicPlayer.h"
#include "mod_KeyCtrl.h"
#include "mod_RoboStateCtrl.h"
#include "drv_RemoteCtrl_data.h"

/**
 * @brief 吸盘的打开与机械臂一键推矿
 */
static void CatchOreEntry_Callback(void)
{
    // 只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;
    if (Key_GetState(KeyEVT_MouseL))
    {
        Ore_Grabbing();
        Robo_Control(Push_State, Write, (void *)PushReady_Mode); // 开始推矿
        SoundDisplay_AddSound(Test1_EQ);                         // 提示音
    }
    else if (Key_GetState(KeyEVT_MouseR))
    {
        Robo_Control(AirPump_State, Write, (void *)AirPumpState_Close); // 关闭吸盘
        Ore_Drawing();
        SoundDisplay_AddSound(Test2_EQ); // 提示音
    }
}
/**
 * @brief 吸盘的关闭与机械臂一键推矿
 */
static void CatchOreQuit_Callback(void)
{
    // 只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    Robo_Control(Push_State, Write, (void *)NoPush_Mode); // 关闭推矿
    SoundDisplay_AddSound(Test3_EQ);                      // 提示音
}

/**
 * @brief 通过键盘上的热键 Shift 判断控制机器人速度增益
 * @brief 当且仅当按下时，提供速度增益
 * @author mylj
 */
static void HotKey_Enter_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;
    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
        return;

    RoboState_Acceleration(); // 机器人加速模式
}

/**
 * @brief 通过键盘上的热键 Shift 判断控制机器人速度增益
 * @brief 当且仅当抬起时，取消速度增益
 * @author mylj
 */
static void HotKey_Quit_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;
    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
        return;

    RoboState_NormalGain(); //
}

/**
 * @brief 通过键盘上的 Ctrl 键判断是否进入机器人的拓展控制模式
 * @brief 当且仅当按下时，进入拓展控制模式，鼠标控制roll或者z
 * @author mylj
 */
static void ExpandCtrl_Enter_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    Robo_Control(ExpandCtrl_State, Write, (void *)EnExpand_Mode); // 拓展控制状态
}

/**
 * @brief 通过键盘上的 Ctrl 键判断是否进入机器人的拓展控制模式
 * @brief 当且仅当抬起时，取消拓展控制模式
 * @author mylj
 */
static void ExpandCtrl_Quit_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    Robo_Control(ExpandCtrl_State, Write, (void *)NoExpand_Mode); // 正常状态
}

/**
 * @brief 复位与回退入口函数
 */
static void ResetEntry_Callback(void)
{
    // 只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
    {
        rt_uint8_t UI_Reset;
        UIReset_State_Data_Read(&UI_Reset);
        if (UI_Reset == (rt_uint8_t)UIReset_Mode1)
            UIReset_State_Data_Write((rt_uint8_t)UIReset_Mode2);
        else
            UIReset_State_Data_Write((rt_uint8_t)UIReset_Mode1);
        return;
    }

    if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) == FloorOre_Mode) // 地上矿模式
    {
        // 地上矿模式
        switch ((rt_uint32_t)(Robo_Control(FloorOre_State, Read, NULL)))
        {
        case (rt_uint32_t)FloorOreImageReady_Mode:                        // 地上矿图传对准状态
            Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);       // 回退至正常状态
            Robo_Control(FloorOre_State, Write, (void *)NoFloorOre_Mode); // 回退至地上矿初始状态
            ArmPosState_Init();                                           // 机械臂位姿初始化状态

            SoundDisplay_AddSound(Test2_EQ);                        //
            break;                                                  //
        case (rt_uint32_t)FloorOreReady_Mode:                       // 地上矿对准状态
            Robo_Control(Ore_State, Write, (void *)NormalOre_Mode); // 回退至正常状态
            ArmPosState_floorImageInit();                           // 地上矿图传对准状态

            SoundDisplay_AddSound(Test2_EQ);   //
            break;                             //
        case (rt_uint32_t)FloorOrePullup_Mode: // 地上矿上拔状态
            ArmPosState_floorInit();           //

            SoundDisplay_AddSound(Test3_EQ);    //
            break;                              //
        case (rt_uint32_t)FloorOreLaydown_Mode: // 地上矿下放状态
            ArmPosState_floorPullup();

            SoundDisplay_AddSound(Test1_EQ);
            break;
        default:
            break;
        }
    }
    else if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) == FrontGoldOre_Mode ||
             ((uint32_t)(Robo_Control(Ore_State, Read, NULL))) == SideGoldOre_Mode)
    {
        // 金矿模式
        switch ((rt_uint32_t)(Robo_Control(GoldOre_State, Read, NULL)))
        {
        case (rt_uint32_t)FrontGoldAim_Mode:                         // 金矿对准状态
            Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);  // 回退至正常状态
            Robo_Control(GoldOre_State, Write, (void *)NoGold_Mode); // 回退至金矿初始状态
            ArmPosState_Init();                                      // 机械臂位姿初始化状态
            SoundDisplay_AddSound(Test2_EQ);
            break;                                                   //
        case (rt_uint32_t)SideGoldAim_Mode:                          // 金矿对准状态
            Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);  // 回退至正常状态
            Robo_Control(GoldOre_State, Write, (void *)NoGold_Mode); // 回退至金矿初始状态
            ArmPosState_Init();                                      // 机械臂位姿初始化状态
            SoundDisplay_AddSound(Test2_EQ);
            break;                         //
        case (rt_uint32_t)GoldPullup_Mode: // 金矿上拔状态
            if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) == FrontGoldOre_Mode)
            {
                Robo_Control(GoldOre_State, Write, (void *)FrontGoldAim_Mode); // 回退至金矿对准状态
                SoundDisplay_AddSound(Test3_EQ);
                ArmPosState_FrontGoldInit(); // 金矿对准状态
            }
            else
            {
                Robo_Control(GoldOre_State, Write, (void *)FrontGoldAim_Mode); // 回退至金矿对准状态
                SoundDisplay_AddSound(Test3_EQ);
                ArmPosState_FrontGoldInit(); // 金矿对准状态
            }
            break;
        default:
            break;
        }
    }
    else if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) == SliverOre_Mode) // 银矿模式
    {
        // 银矿模式
        switch ((rt_uint32_t)(Robo_Control(SliverOre_State, Read, NULL)))
        {
        case (rt_uint32_t)SliverAim_Mode:                                // 银矿对准状态
            Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);      // 回退至正常状态
            Robo_Control(SliverOre_State, Write, (void *)NoSliver_Mode); // 回退至银矿初始状态

            ArmPosState_Init(); // 机械臂位姿初始化状态
            SoundDisplay_AddSound(Test2_EQ);
            break;                           //
        case (rt_uint32_t)SliverPullup_Mode: // 银矿上拔状态
            ArmPosState_SilverInit();        // 银矿初始状态

            SoundDisplay_AddSound(Test3_EQ);
            break;
        case (rt_uint32_t)SliverGrip_Mode: // 银矿上拔状态
            ArmPosState_SilverInit();      // 银矿初始状态

            SoundDisplay_AddSound(Test3_EQ);
            break;
        default:
            break;
        }
    }
    else
    {
        ArmPosState_Init(); // 机械臂位姿初始化状态

        Robo_Control(RoboCtrl_RC_State, Write, (void *)ChassisNormal_Mode); // 底盘控制模式
        Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);             // 普通矿石模式
    }
}

/**
 * @brief 地上矿模式回调函数
 */
static void FloorOreEntry_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
    {
        // 三级兑矿模式
        ArmPosState_3levelOre_Init();
        SmoothFilter_Restart(AllSmoothFilter);
        SoundDisplay_AddSound(Test1_EQ);
        return;
    }

    if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) != FloorOre_Mode)
    {
        Robo_Control(Ore_State, Write, (void *)FloorOre_Mode);        // 地上矿模式
        Robo_Control(FloorOre_State, Write, (void *)NoFloorOre_Mode); // 地上矿初始化状态
    }
    Robo_Control(GyroMode_State, Write, (void *)GyroDisable_passiveMode); // 被动禁用小陀螺
    // 地上矿模式
    switch ((rt_uint32_t)(Robo_Control(FloorOre_State, Read, NULL)))
    {
        // 循环式
    case (rt_uint32_t)NoFloorOre_Mode:                                // 地上矿初始状态
        ArmPosState_floorImageInit();                                 //
        SoundDisplay_AddSound(Test1_EQ);                              //
        break;                                                        //
    case (rt_uint32_t)FloorOreImageReady_Mode:                        // 地上矿图传对准状态
        ArmPosState_floorInit();                                      //
        Ore_Grabbing();                                               //
        SoundDisplay_AddSound(Test1_EQ);                              //
        break;                                                        //
    case (rt_uint32_t)FloorOreReady_Mode:                             // 地上矿对准状态
        ArmPosState_floorPullup();                                    //
        SoundDisplay_AddSound(Test2_EQ);                              //
        break;                                                        //
    case (rt_uint32_t)FloorOrePullup_Mode:                            // 地上矿上拔状态
        ArmPosState_floorLaydown();                                   //
        SoundDisplay_AddSound(Test3_EQ);                              //
        break;                                                        //
    case (rt_uint32_t)FloorOreLaydown_Mode:                           // 地上矿下放状态
        Robo_Control(FloorOre_State, Write, (void *)NoFloorOre_Mode); // 回退至地上矿上拔状态
        Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);       // 回退至正常状态

        SoundDisplay_AddSound(Test1_EQ); //
        break;                           //
    default:                             //
        break;                           //
    }
}

/**
 * @brief 银矿模式回调函数
 */
static void SilverOreEntry_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;
    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
    {
        // 左四级兑矿模式
        ArmPosState_4levelOreL_Init();
        Robo_Control(RoboCtrl_RC_State, Write, (void *)ChassisFineTuning_Mode); //
        SmoothFilter_Restart(AllSmoothFilter);
        SoundDisplay_AddSound(Test2_EQ);
        return;
    }

    if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) != SliverOre_Mode)
    {
        Robo_Control(Ore_State, Write, (void *)SliverOre_Mode);      // 银矿模式
        Robo_Control(SliverOre_State, Write, (void *)NoSliver_Mode); // 银矿初始化状态
    }

    // 银矿模式
    switch ((rt_uint32_t)(Robo_Control(SliverOre_State, Read, NULL)))
    {
        // 循环式
    case (rt_uint32_t)NoSliver_Mode:                                            // 银矿初始状态
        ArmPosState_SilverInit();                                               //
        Robo_Control(RoboCtrl_RC_State, Write, (void *)ChassisFineTuning_Mode); //
        SoundDisplay_AddSound(Test1_EQ);
        break;                        //
    case (rt_uint32_t)SliverAim_Mode: // 银矿对准状态
        Ore_Grabbing();
        Robo_Control(SliverOre_State, Write, (void *)SliverGrip_Mode); // 回退至银矿上拔状态
        SoundDisplay_AddSound(Test2_EQ);
        break;
    case (rt_uint32_t)SliverGrip_Mode: // 银矿对准状态
        ArmPosState_SilverPullup();    //

        SoundDisplay_AddSound(Test3_EQ);
        break;                                                       //
    case (rt_uint32_t)SliverPullup_Mode:                             // 银矿下放状态
        Robo_Control(SliverOre_State, Write, (void *)NoSliver_Mode); // 回退至银矿上拔状态
        Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);      // 回退至正常状态

        SoundDisplay_AddSound(Test3_EQ);
        break;
    default:
        break;
    }
}

/**
 * @brief 正金矿模式回调函数
 */
static void FrontGoldOreEntry_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;
    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
    {
        // // 右四级兑矿模式
        // ArmPosState_4levelOreR_Init();
        switch ((rt_uint32_t)(Robo_Control(Obstacleblock_State, Read, NULL)))
        {
        case (rt_uint32_t)Obstacleblock_Aim: // 金矿下放状态
            ArmPosState_ObstacleblockPullup();
            SoundDisplay_AddSound(Test2_EQ);
            break; //
        default:   //
            ArmPosState_ObstacleblockAim();
            SoundDisplay_AddSound(Test3_EQ);
            break; //
        }
        SmoothFilter_Restart(AllSmoothFilter);
        return;
    }

    if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) != FrontGoldOre_Mode)
    {
        Robo_Control(Ore_State, Write, (void *)FrontGoldOre_Mode); // 金矿模式
        Robo_Control(GoldOre_State, Write, (void *)NoGold_Mode);   // 金矿初始化状态
    }
    Robo_Control(GyroMode_State, Write, (void *)GyroDisable_passiveMode); //        // 被动禁用小陀螺
    // 金矿模式
    switch ((rt_uint32_t)(Robo_Control(GoldOre_State, Read, NULL)))
    {
        // 循环式
    case (rt_uint32_t)NoGold_Mode:   // 金矿初始状态
        ArmPosState_FrontGoldInit(); //

        SoundDisplay_AddSound(Test1_EQ);
        break;                           //
    case (rt_uint32_t)FrontGoldAim_Mode: // 金矿对准状态
        ArmPosState_FrontGoldPullup();   //

        SoundDisplay_AddSound(Test2_EQ);
        break;                                                   //                                                         //
    case (rt_uint32_t)GoldPullup_Mode:                           // 金矿下放状态
        Robo_Control(GoldOre_State, Write, (void *)NoGold_Mode); // 回退至金矿上拔状态
        Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);  // 回退至正常状态

        SoundDisplay_AddSound(Test3_EQ);
        break; //
    default:   //
        break; //
    }
}

/**
 * @brief 侧金矿模式回调函数
 */
static void SideGoldOreEntry_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;
    if (((uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL))) == EnExpand_Mode)
    {

        if ((uint32_t)(Robo_Control(ChassisReverse_State, Read, NULL)) == ChassisReverse)
        {
            Robo_Control(ChassisReverse_State, Write, (void *)NoChassisReverse); // 底盘不返向
        }
        else
        {
            Robo_Control(ChassisReverse_State, Write, (void *)ChassisReverse); // 底盘不返向
        }
        return;
    }

    if (((uint32_t)(Robo_Control(Ore_State, Read, NULL))) != SideGoldAim_Mode)
    {
        Robo_Control(Ore_State, Write, (void *)SideGoldAim_Mode); // 金矿模式
        Robo_Control(GoldOre_State, Write, (void *)NoGold_Mode);  // 金矿初始化状态
    }
    // 金矿模式
    switch ((rt_uint32_t)(Robo_Control(GoldOre_State, Read, NULL)))
    {
        // 循环式
    case (rt_uint32_t)NoGold_Mode:  // 金矿初始状态
        ArmPosState_SideGoldInit(); //

        SoundDisplay_AddSound(Test1_EQ);
        break;                          //
    case (rt_uint32_t)SideGoldAim_Mode: // 侧金矿对准状态
        ArmPosState_SideGoldPullup();   //

        SoundDisplay_AddSound(Test2_EQ);
        break;                                                   //                                                         //
    case (rt_uint32_t)GoldPullup_Mode:                           // 金矿下放状态
        Robo_Control(GoldOre_State, Write, (void *)NoGold_Mode); // 回退至金矿上拔状态
        Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);  // 回退至正常状态

        SoundDisplay_AddSound(Test3_EQ);
        break; //
    default:   //
        break; //
    }
}

/**
 * @brief 机械臂控制模式回调函数
 */
static void RoboArmAngleMode_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    Robo_Control(RoboCtrl_RC_State, Write, (void *)ArmAngle_Mode);  // 末端角度控制模式
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    SmoothFilter_Restart(AllSmoothFilter);
    SoundDisplay_AddSound(Test1_EQ);
}

/**
 * @brief 机械臂控制模式回调函数
 */
static void RoboArmPosMode_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    Robo_Control(RoboCtrl_RC_State, Write, (void *)ArmPos_Mode); // 末端位置控制模式

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    SmoothFilter_Restart(AllSmoothFilter);
    SoundDisplay_AddSound(Test2_EQ);
}

/**
 * @brief 机械臂控制模式回调函数
 */
static void RoboArmCtrlMode_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode)
        return;

    Robo_Control(RoboCtrl_RC_State, Write, (void *)ArmCtrl_Mode); // 自定义控制器模式

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    SmoothFilter_Restart(AllSmoothFilter);
    SoundDisplay_AddSound(Test3_EQ);
}

/**
 * @brief 底盘普通模式
 */
static void ChassisNormal_Mode_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if ((((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode))
        return;

    Robo_Control(RoboCtrl_RC_State, Write, (void *)ChassisNormal_Mode); // 底盘控制模式

    Robo_Control(ImageFollow_State, Write, (void *)Notfollowing_Mode); // 图传跟随模式
    Robo_Control(Ore_State, Write, (void *)NormalOre_Mode);            // 正常矿石状态
    SmoothFilter_Restart(AllSmoothFilter);
    SoundDisplay_AddSound(Test3_EQ);
}

/**
 * @brief 底盘微调模式
 */
static void ChassisFineTuning_Mode_Callback(void)
{
    //  只有客户端模式才需要处理键盘数据
    if ((((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode))
        return;

    Robo_Control(RoboCtrl_RC_State, Write, (void *)ChassisFineTuning_Mode); //

    SmoothFilter_Restart(AllSmoothFilter);
    SoundDisplay_AddSound(Test3_EQ);
}

// Key_Menu_Str_t MainMenu;           // 主菜单
// Key_Menu_Str_t ExpandCtrlModeMenu; // 拓展控制模式选择菜单

// /**
//  * @brief 进入拓展控制模式——二维按键回调函数
//  */
// static void ExpandModeSet_Callback(int TrigSource)
// {
//     //  只有客户端模式才需要处理键盘数据
//     if ((((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL))) != Client_Mode))
//         return;
//     rt_uint8_t UI_Reset;
//     switch (TrigSource)
//     {
//     case KeyEVT_Z:
//         UIReset_State_Data_Read(&UI_Reset);
//         if (UI_Reset == (rt_uint8_t)UIReset_Mode1)
//             UIReset_State_Data_Write((rt_uint8_t)UIReset_Mode2);
//         else
//             UIReset_State_Data_Write((rt_uint8_t)UIReset_Mode1);
//         break;
//     case KeyEVT_X:
//         // 三级兑矿模式
// ArmPosState_3levelOre_Init();
//         SmoothFilter_Restart(AllSmoothFilter);
//         SoundDisplay_AddSound(Test3_EQ);
//         break;
//     case KeyEVT_C:
//         // 左四级兑矿模式
//         break;
//     case KeyEVT_V:
//         // 右四级兑矿模式
//         break;
//     case KeyEVT_B:
//         break;
//     default:
//         break;
//     }
// }

// /**
//  * @brief 退出拓展控制模式——二维按键回调函数
//  */
// static void ExpandModeQuit_Callback(int TrigSource)
// {
//     // TODO:待测试
//     MenuCmd_SetMainMenu(&MainMenu); // 设置为初始菜单
// }

/**
 * @brief 根据只需要读取跳变沿的按键设置回调函数
 */
static int KeyMouseData_Init(void)
{
    /*****一维按键*****/
    Key_SetCallBack(KeyEVT_MouseL, CatchOreEntry_Callback, CatchOreQuit_Callback);
    Key_SetCallBack(KeyEVT_MouseR, CatchOreEntry_Callback, NULL);

    Key_SetCallBack(KeyEVT_CTRL, ExpandCtrl_Enter_Callback, ExpandCtrl_Quit_Callback);
    Key_SetCallBack(KeyEVT_SHIFT, HotKey_Enter_Callback, HotKey_Quit_Callback);

    Key_SetCallBack(KeyEVT_Z, ResetEntry_Callback, NULL);        // 回退
    Key_SetCallBack(KeyEVT_X, FloorOreEntry_Callback, NULL);     // 地上矿
    Key_SetCallBack(KeyEVT_C, SilverOreEntry_Callback, NULL);    // 银矿
    Key_SetCallBack(KeyEVT_V, FrontGoldOreEntry_Callback, NULL); // 正金矿模式
    Key_SetCallBack(KeyEVT_B, SideGoldOreEntry_Callback, NULL);  // 侧金矿模式

    Key_SetCallBack(KeyEVT_E, RoboArmAngleMode_Callback, NULL); // 机械臂角度模式
    Key_SetCallBack(KeyEVT_Q, RoboArmPosMode_Callback, NULL);   // 机械臂坐标模式
    Key_SetCallBack(KeyEVT_R, RoboArmCtrlMode_Callback, NULL);  // 机械臂角度和坐标模式

    Key_SetCallBack(KeyEVT_F, ChassisNormal_Mode_Callback, NULL);     // 机械臂角度和坐标模式
    Key_SetCallBack(KeyEVT_G, ChassisFineTuning_Mode_Callback, NULL); // 机械臂角度和坐标模式

    /*****二维按键*****/
    // KeyCtrl_MenuStr_Init(&MainMenu);           // 初始化主菜单结构体
    // KeyCtrl_MenuStr_Init(&ExpandCtrlModeMenu); // 初始化拓展控制菜单结构体
    // MenuCmd_SetMainMenu(&MainMenu);            // 设置为初始菜单

    // MenuSet_Add_SubMenu(&MainMenu, &ExpandCtrlModeMenu, KeyEVT_CTRL);
    // MenuSet_Add_EntryFun(&ExpandCtrlModeMenu,
    //                      KeyEVT_ALL,
    //                      NULL);
    // MenuSet_Add_QuitFun(&ExpandCtrlModeMenu,
    //                     KeyEVT_Z |
    //                         KeyEVT_X | KeyEVT_C |
    //                         KeyEVT_V | KeyEVT_B,
    //                     ExpandModeSet_Callback);
    // MenuSet_Add_QuitFun(&ExpandCtrlModeMenu,
    //                     KeyEVT_ALL,
    //                     ExpandModeQuit_Callback);
    // MenuSet_Add_SubMenu(&ExpandCtrlModeMenu,
    //                     &MainMenu,
    //                     KeyEVT_Z | KeyEVT_X |
    //                         KeyEVT_C | KeyEVT_V |
    //                         KeyEVT_B | EXIT_KEY);
    // Key_Menu_Start();

    return 0;
}
INIT_APP_EXPORT(KeyMouseData_Init);
