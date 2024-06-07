/**
 * @file drv_Robo_data.c
 * @brief 机器人数据服务器初始化与遥控数据
 * @author mylj
 * @version 1.0
 * @date 2022-12-29
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_Robo_data.h"
#include "drv_dataserve.h"
#include <string.h>

static void *Engineer_RoboData; // 承接数据服务器的中间变量

/**
 * @brief 工程数据服务器初始化函数
 * @brief 自动初始化
 */
static int Engineer_Data_Init(void)
{
    /*机械臂电机当前角度值（滤波后） 单位；*/
    Request_Add_Package("F_Yaw_rad_now", strlen("F_Yaw_rad_now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Pitch_rad_now", strlen("F_Pitch_rad_now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Roll_rad_now", strlen("F_Roll_rad_now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Yaw_rad_now", strlen("B_Yaw_rad_now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch1_rad_now", strlen("B_Pitch1_rad_now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch2_rad_now", strlen("B_Pitch2_rad_now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*小机械臂电机当前角度值 单位；*/
    Request_Add_Package("F_Yaw_AngleNow", strlen("F_Yaw_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Pitch_AngleNow", strlen("F_Pitch_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Roll_AngleNow", strlen("F_Roll_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*小机械臂电机期望角度值 单位；*/
    Request_Add_Package("F_Yaw_AngleHope", strlen("F_Yaw_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Pitch_AngleHope", strlen("F_Pitch_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Roll_AngleHope", strlen("F_Roll_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*小机械臂电机当前速度值 单位；m/s*/
    Request_Add_Package("F_Yaw_SpeedNow", strlen("F_Yaw_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Pitch_SpeedNow", strlen("F_Pitch_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("F_Roll_SpeedNow", strlen("F_Roll_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*大机械臂电机当前角度值 单位；°*/
    Request_Add_Package("B_Yaw_AngleNow", strlen("B_Yaw_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch1_AngleNow", strlen("B_Pitch1_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch2_AngleNow", strlen("B_Pitch2_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*大机械臂yaw滤波速度*/
    Request_Add_Package("B_Yaw_SpeFilterNow", strlen("B_Yaw_SpeFilterNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*大机械臂电机解算（期望）角度值 单位；°*/
    Request_Add_Package("B_Yaw_AngleHope", strlen("B_Yaw_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch1_AngleHope", strlen("B_Pitch1_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch2_AngleHope", strlen("B_Pitch2_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*大机械臂电机当前速度值 单位；m/s*/
    Request_Add_Package("B_Yaw_SpeedNow", strlen("B_Yaw_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch1_SpeedNow", strlen("B_Pitch1_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("B_Pitch2_SpeedNow", strlen("B_Pitch2_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*机械臂末端当前坐标值 单位；°*/
    Request_Add_Package("ArmTop_x_Now", strlen("ArmTop_x_Now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ArmTop_y_Now", strlen("ArmTop_y_Now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ArmTop_z_Now", strlen("ArmTop_z_Now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*机械臂末端设定坐标值 单位；°*/
    Request_Add_Package("ArmTop_x_Set", strlen("ArmTop_x_Set"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ArmTop_y_Set", strlen("ArmTop_y_Set"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ArmTop_z_Set", strlen("ArmTop_z_Set"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*底盘运动速度当前 单位；°*/
    Request_Add_Package("ChassisSpeNow_RF", strlen("ChassisSpeNow_RF"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisSpeNow_LF", strlen("ChassisSpeNow_LF"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisSpeNow_RB", strlen("ChassisSpeNow_RB"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisSpeNow_LB", strlen("ChassisSpeNow_LB"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*底盘运动速度设定 单位；°*/
    Request_Add_Package("ChassisSpeSet_RF", strlen("ChassisSpeSet_RF"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisSpeSet_LF", strlen("ChassisSpeSet_LF"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisSpeSet_RB", strlen("ChassisSpeSet_RB"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisSpeSet_LB", strlen("ChassisSpeSet_LB"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*机器人速度增益 单位：1*/
    Request_Add_Package("Robo_SpeedGain", strlen("Robo_SpeedGain"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*自定义控制器数据*/
    Request_Add_Package("CustCtrler_Limit", strlen("CustCtrler_Limit"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("CustCtrler_x", strlen("CustCtrler_x"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("CustCtrler_y", strlen("CustCtrler_y"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("CustCtrler_z", strlen("CustCtrler_z"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("CustCtrler_yaw", strlen("CustCtrler_yaw"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("CustCtrler_pitch", strlen("CustCtrler_pitch"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("CustCtrler_roll", strlen("CustCtrler_roll"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*图传云台电机当前角度值*/
    Request_Add_Package("I_Yaw_AngleNow", strlen("I_Yaw_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("I_Pitch_AngleNow", strlen("I_Pitch_AngleNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*图传云台电机当前角度值*/
    Request_Add_Package("I_Yaw_AngleHope", strlen("I_Yaw_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("I_Pitch_AngleHope", strlen("I_Pitch_AngleHope"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*图传云台电机当前角度值*/
    Request_Add_Package("I_Yaw_SpeedNow", strlen("I_Yaw_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("I_Pitch_SpeedNow", strlen("I_Pitch_SpeedNow"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    /*图传云台当前角度值*/
    Request_Add_Package("IGim_Yaw_Angle", strlen("IGim_Yaw_Angle"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("IGim_Pitch_Angle", strlen("IGim_Pitch_Angle"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*图传当前位置状态*/
    Request_Add_Package("Image_PosState", strlen("Image_PosState"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*编码器角度值*/
    Request_Add_Package("BYaw_CoderOrigin", strlen("BYaw_CoderOrigin"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData)); // 机械臂大Yaw
    Request_Add_Package("BYaw_CoderCaliRad", strlen("BYaw_CoderCaliRad"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData)); // 机械臂大Yaw
    Request_Add_Package("IYaw_CoderAngle", strlen("IYaw_CoderAngle"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData)); // 图传小Yaw

    /*机械臂超限判断*/
    Request_Add_Package("Arm_IfLimit", strlen("Arm_IfLimit"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*UI复位数据*/
    Request_Add_Package("UIReset_State", strlen("UIReset_State"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*陀螺仪Yaw角度和角速度*/
    Request_Add_Package("Gyro_YawSpeed", strlen("Gyro_YawSpeed"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("Gyro_YawAngle", strlen("Gyro_YawAngle"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("Gyro_YawRadFilter", strlen("Gyro_YawRadFilter"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*编码器标定状态位*/
    Request_Add_Package("EncoderCali_State", strlen("EncoderCali_State"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    /*云台系与底盘系角度及云台期望角度*/
    Request_Add_Package("GimbalYawRad_Now", strlen("GimbalYawRad_Now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("GimbalYawRad_Set", strlen("GimbalYawRad_Set"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisYawRad_Now", strlen("ChassisYawRad_Now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));
    Request_Add_Package("ChassisYawSpe_Now", strlen("ChassisYawSpe_Now"),
                        &Engineer_RoboData, sizeof(Engineer_RoboData));

    return 0;
}
DATASERVER_DATAPACKAGE_INIT(Engineer_Data_Init);
