/**
 * @file func_Can_data.c
 * @brief 处理can报文及SPI初始化
 * @author mylj
 * @version 1.0
 * @date 2022-12-30
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "mod_Can_data.h"
#include "drv_utils.h"
#include "drv_dataserve.h"
#include "func_ArmMotor_Ctrl.h"
#include "func_ImageMotor_Ctrl.h"
#include "func_ChassisMotor_Ctrl.h"

/*序列号*/
/*小机械臂当前角度值序列号*/
static rt_int8_t ForearmYaw_AngleNow_num = 0;
static rt_int8_t ForearmPitch_AngleNow_num = 0;
static rt_int8_t ForearmRoll_AngleNow_num = 0;
/*小机械臂当前速度值序列号*/
static rt_int8_t ForearmYaw_SpeedNow_num = 0;
static rt_int8_t ForearmPitch_SpeedNow_num = 0;
static rt_int8_t ForearmRoll_SpeedNow_num = 0;
/*大机械臂当前角度值序列号*/
static rt_int8_t BoomYaw_AngleNow_num = 0;
static rt_int8_t BoomPitch1_AngleNow_num = 0;
static rt_int8_t BoomPitch2_AngleNow_num = 0;
/*大机械臂当前速度值序列号*/
static rt_int8_t BoomYaw_SpeedNow_num = 0;
static rt_int8_t BoomPitch1_SpeedNow_num = 0;
static rt_int8_t BoomPitch2_SpeedNow_num = 0;
/*底盘当前运动速度序列号*/
static rt_int8_t ChassisSpe_RF_Now_num = 0;
static rt_int8_t ChassisSpe_RB_Now_num = 0;
static rt_int8_t ChassisSpe_LF_Now_num = 0;
static rt_int8_t ChassisSpe_LB_Now_num = 0;
/*编码器当前角度序列号*/
// static rt_int8_t BoomYaw_CoderAngleOrigin_num = 0;
static rt_int8_t ImageYaw_CoderAngle_num = 0;
/*图传云台数据序列号*/
static rt_int8_t ImageYaw_AngleNow_num = 0;
static rt_int8_t ImagePitch_AngleNow_num = 0;
static rt_int8_t ImageYaw_SpeedNow_num = 0;
static rt_int8_t ImagePitch_SpeedNow_num = 0;
///*陀螺仪Yaw角度和角速度*/
// static rt_int8_t Gyro_YawAngle_num = 0;
// static rt_int8_t Gyro_YawSpeed_num = 0;
///*自定义控制器数据序列号*/
// static rt_int8_t CustCtrler_x_num = 0;
// static rt_int8_t CustCtrler_y_num = 0;
// static rt_int8_t CustCtrler_z_num = 0;
// static rt_int8_t CustCtrler_yaw_num = 0;
// static rt_int8_t CustCtrler_pitch_num = 0;
// static rt_int8_t CustCtrler_roll_num = 0;
// static rt_int8_t CustCtrler_Limit_num = 0;

/*can报文*/
static SPICAN_MsgOnTransfer_t ForearmMsg_Can1, // 标识符0x1FF  小机械臂电机
    BoomMsg_Can2,                              // 标识符0x2FF  大机械臂电机
    ChassisMsg_Can1,                           // 标识符0x2FF  底盘电机
    ImageMsg_Can2,                             // 标识符0x2FF  机械臂陀螺仪
    Debug_Can;                                 // 调试报文

/**
 * @brief 发送can报文初始化
 * @param Msg       can报文
 * @param CanID     标识符
 * @param CanNum    选择can的编号 (0：CAN1	1：CAN2)
 */
static void SPICan_SendInit(SPICAN_MsgOnTransfer_t *Msg,
                            rt_int32_t CanID,
                            CanNum_e CanNum)
{
    rt_int8_t i;

    Msg->ID = CanID;
    Msg->IsExtID = 0;
    Msg->IsRemote = 0;
    Msg->Len = 8;
    Msg->ForCANx = (rt_uint32_t)CanNum;

    for (i = 0; i < 8; i++)
        Msg->data[i] = 0;
}

/**
 * @brief 电机单报文写入处理函数
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID         电机ID （1~4）
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
static void MotorCan_Write(SPICAN_MsgOnTransfer_t *Msg,
                           rt_int8_t Motor_ID,
                           rt_uint16_t CtrlSetData)
{
    rt_int8_t index;

    index = 2 * (Motor_ID - 1);

    Msg->data[index] = (CtrlSetData) >> 8;
    Msg->data[index + 1] = CtrlSetData;
}

/**
 * @brief 电机报文接收处理函数
 */
static void MotorCan_Receive(Motor_t *Motor,
                             rt_uint8_t rxmsg[],
                             rt_int8_t AngleData_num,
                             rt_int8_t SpeedData_num)
{
    float AngleNow;
    float SpeedNow;

    motor_readmsg(rxmsg, &Motor->dji);

    if (AngleData_num != -1)
    {
        AngleNow = Motor_Read_NowAngle(Motor);
        Package_Write_All_Data(AngleData_num,
                               &AngleNow,
                               sizeof(AngleNow));
    }
    else
    {
        AngleNow = 0;
    }

    if (SpeedData_num != -1)
    {
        SpeedNow = Motor_Read_NowSpeed(Motor);
        Package_Write_All_Data(SpeedData_num,
                               &SpeedNow,
                               sizeof(SpeedNow));
    }
    else
    {
        SpeedNow = 0;
    }
}

rt_uint16_t angle_debug;
rt_uint8_t flag_debug;

/**
 * @brief 编码器报文接收处理函数
 * @param encoder
 * @param rxmsg
 * @param AngleData_num
 */
static void EncoderCan_Receive(Encoder_s *encoder,
                               rt_uint8_t rxmsg[],
                               rt_int8_t AngleData_num)
{
    static float angle_now;
    Encoder_readmsg(rxmsg, encoder);
    flag_debug = rxmsg[6];
    if (AngleData_num != -1 && rxmsg[6] == 1)
    {
        angle_now = Encoder_Radnow_Read(encoder);
        angle_debug = (rt_uint16_t)(rxmsg[1] << 8 | rxmsg[0]);
        Package_Write_All_Data(AngleData_num,
                               &angle_now,
                               sizeof(angle_now));
    }
    else
    {
        angle_now = 0;
    }
}

///**
// * @brief 陀螺仪报文接收处理函数
// * @param encoder
// * @param rxmsg
// * @param AngleData_num
// */
// static void GyroScopeCan_Receive(Encoder_s *encoder,
//                                 rt_uint8_t rxmsg[],
//                                 rt_int8_t AngleData_num,
//                                 rt_int8_t GyroAngleData_num,
//                                 rt_int8_t GyroSpeedData_num)
//{
//    float data_temp;

//    if ((GyroAngleData_num != -1) && (rxmsg[7] == 1))
//    {
//        data_temp = (rt_int16_t)(rxmsg[0] << 8 | rxmsg[1]);
//        Package_Write_All_Data(GyroAngleData_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//    if ((GyroSpeedData_num != -1) && (rxmsg[7] == 1))
//    {
//        data_temp = (rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]);
//        Package_Write_All_Data(GyroSpeedData_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }

//    if ((AngleData_num != -1) && (rxmsg[6] == 1))
//    {

//        Encoder_readmsg(rxmsg, encoder);
//        data_temp = Encoder_Radnow_Read(encoder);
//        Package_Write_All_Data(AngleData_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//}

///**
// * @brief 自定义控制器yaw、pitch、roll数据报文接收解算
// */
// static void CustCtrl_ypr_Receive(rt_uint8_t rxmsg[],
//                                 rt_int8_t CustCtrl_yaw_num,
//                                 rt_int8_t CustCtrl_pitch_num,
//                                 rt_int8_t CustCtrl_roll_num)
//{
//    float data_temp;

//    if (CustCtrl_yaw_num != -1)
//    {
//        data_temp = (rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]);
//        Package_Write_All_Data(CustCtrl_yaw_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//    if (CustCtrl_pitch_num != -1)
//    {
//        data_temp = (rt_int16_t)(rxmsg[4] << 8 | rxmsg[5]);
//        Package_Write_All_Data(CustCtrl_pitch_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//    if (CustCtrl_roll_num != -1)
//    {
//        data_temp = (rt_int16_t)(rxmsg[6] << 8 | rxmsg[7]);
//        Package_Write_All_Data(CustCtrl_roll_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//}

///**
// * @brief 自定义控制器xyz数据报文接收解算
// */
// static void CustCtrl_xyz_Receive(rt_uint8_t rxmsg[],
//                                 rt_int8_t CustCtrl_x_num,
//                                 rt_int8_t CustCtrl_y_num,
//                                 rt_int8_t CustCtrl_z_num,
//                                 rt_int8_t CustCtrl_limit_num)
//{
//    float data_temp;
//    rt_uint8_t data_temp2;

//    if (CustCtrl_x_num != -1)
//    {
//        data_temp = (rt_int16_t)(rxmsg[0] << 8 | rxmsg[1]);
//        Package_Write_All_Data(CustCtrl_x_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//    if (CustCtrl_y_num != -1)
//    {
//        data_temp = (rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]);
//        Package_Write_All_Data(CustCtrl_y_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }
//    if (CustCtrl_z_num != -1)
//    {
//        data_temp = (rt_int16_t)(rxmsg[4] << 8 | rxmsg[5]);
//        Package_Write_All_Data(CustCtrl_z_num,
//                               &data_temp,
//                               sizeof(data_temp));
//    }

//    if (CustCtrl_limit_num != -1)
//    {
//        data_temp2 = (rt_int16_t)(rxmsg[6] << 8 | rxmsg[7]);
//        Package_Write_All_Data(CustCtrl_limit_num,
//                               &data_temp2,
//                               sizeof(data_temp2));
//    }
//}

/**
 * @brief 从机1的报文发送处理函数
 * @brief 底盘部分与图传部分电机报文发送
 * @param Chassis_in
 */
void Send_Slave1_Handle(ChassisMotor_t *Chassis_in)
{
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_RF + 1),
                   (rt_int16_t)Chassis_in->speed[WHEEL_RF]);
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_RB + 1),
                   (rt_int16_t)Chassis_in->speed[WHEEL_RB]);
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_LB + 1),
                   (rt_int16_t)Chassis_in->speed[WHEEL_LB]);
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_LF + 1),
                   (rt_int16_t)Chassis_in->speed[WHEEL_LF]);

    SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ChassisMsg_Can1);
}

/**
 * @brief 从机2的报文发送处理函数
 * @brief 机械臂部分电机报文发送
 * @param in
 */
void Send_Slave2_Handle(BoomMotor_s *Boom_in,
                        ForearmMotor_s *Forearm_in,
                        ImageMotor_s *Image_in)
{
    MotorCan_Write(&ForearmMsg_Can1,
                   1, (rt_int16_t)Forearm_in->ForearmYaw);
    MotorCan_Write(&ForearmMsg_Can1,
                   2, (rt_int16_t)Forearm_in->ForearmPitch);
    MotorCan_Write(&ForearmMsg_Can1,
                   3, (rt_int16_t)Forearm_in->ForearmRoll);
    MotorCan_Write(&ForearmMsg_Can1,
                   4, (rt_int16_t)Image_in->ImageYaw);

    MotorCan_Write(&BoomMsg_Can2,
                   1, (rt_int16_t)Boom_in->BoomYaw);
    MotorCan_Write(&BoomMsg_Can2,
                   2, (rt_int16_t)Boom_in->BoomPitch2);
    MotorCan_Write(&BoomMsg_Can2,
                   3, (rt_int16_t)Boom_in->BoomPitch1);
    MotorCan_Write(&BoomMsg_Can2,
                   4, (rt_int16_t)Image_in->ImagePitch);

    SPI_CAN_SendMsg((rt_uint8_t)Slave2, &ForearmMsg_Can1);
    SPI_CAN_SendMsg((rt_uint8_t)Slave2, &BoomMsg_Can2);
}

/**
 * @brief 从机1的报文发送初始化函数
 * @brief 底盘部分电机报文发送
 * @param in
 */
void Send_Slave1_Init(void)
{
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_RF + 1), 0);
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_RB + 1), 0);
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_LB + 1), 0);
    MotorCan_Write(&ChassisMsg_Can1,
                   (rt_int8_t)(WHEEL_LF + 1), 0);

    MotorCan_Write(&ImageMsg_Can2,
                   1, 0);
    MotorCan_Write(&ImageMsg_Can2,
                   2, 0);
    MotorCan_Write(&ImageMsg_Can2,
                   3, 0);

    SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ChassisMsg_Can1);
    SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ImageMsg_Can2);
}

/**
 * @brief 从机2的报文发送初始化函数
 * @brief 机械臂部分电机报文发送
 * @param in
 */
void Send_Slave2_Init(void)
{
    MotorCan_Write(&ForearmMsg_Can1,
                   1, 0);
    MotorCan_Write(&ForearmMsg_Can1,
                   2, 0);
    MotorCan_Write(&ForearmMsg_Can1,
                   3, 0);

    MotorCan_Write(&BoomMsg_Can2,
                   1, 0);
    MotorCan_Write(&BoomMsg_Can2,
                   2, 0);
    MotorCan_Write(&BoomMsg_Can2,
                   3, 0);

    SPI_CAN_SendMsg((rt_uint8_t)Slave2, &ForearmMsg_Can1);
    SPI_CAN_SendMsg((rt_uint8_t)Slave2, &BoomMsg_Can2);
}

/**
 * @brief 给10020电机或4310电机设置数据
 * @brief 机械臂部分电机报文发送
 * @param in
 */
void Send_Slave_Debug(void)
{
    Debug_Can.ID = DEBUG_CANID;
    Debug_Can.IsExtID = 0;
    Debug_Can.IsRemote = 0;
    Debug_Can.Len = 4;
    Debug_Can.ForCANx = can1;

    Debug_Can.data[0] = 0X00; // ID高8位
    Debug_Can.data[1] = 0X01; // ID低8位
    Debug_Can.data[2] = 0X00;
    Debug_Can.data[3] = 0X03;

    SPI_CAN_SendMsg((rt_uint8_t)Slave2, &Debug_Can);
}

/**
 * @brief 从机1的报文处理函数
 * @brief 底盘部分电机CAN报文的接收
 * @param Msg   can报文
 */
void Receive_Slave1_Handler(SPICAN_MsgOnTransfer_t *Msg)
{
    if (Msg->ForCANx == (rt_uint32_t)can1)
    {
        switch (Msg->ID)
        {
        case ChassisRF_MotorID: // 底盘右前轮电机
            MotorCan_Receive(Get_ChassisMotor(WHEEL_RF),
                             Msg->data,
                             DATA_NUM_NULL,
                             ChassisSpe_RF_Now_num);
            break;
        case ChassisRB_MotorID: // 底盘右后轮电机
            MotorCan_Receive(Get_ChassisMotor(WHEEL_RB),
                             Msg->data,
                             DATA_NUM_NULL,
                             ChassisSpe_RB_Now_num);
            break;
        case ChassisLB_MotorID: // 底盘左后轮电机
            MotorCan_Receive(Get_ChassisMotor(WHEEL_LB),
                             Msg->data,
                             DATA_NUM_NULL,
                             ChassisSpe_LB_Now_num);
            break;
        case ChassisLF_MotorID: // 底盘左前轮电机
            MotorCan_Receive(Get_ChassisMotor(WHEEL_LF),
                             Msg->data,
                             DATA_NUM_NULL,
                             ChassisSpe_LF_Now_num);
            break;
        default:
            break;
        }
    }
    // else if (Msg->ForCANx == (rt_uint32_t)can2)
    // {
    //     switch (Msg->ID)
    //     {
    //     // case IMAGEYAW_MOTORID: // 图传云台左侧电机
    //     //     MotorCan_Receive(Get_ImageMotor(ImageYaw),
    //     //                      Msg->data,
    //     //                      ImageYaw_AngleNow_num,
    //     //                      ImageYaw_SpeedNow_num);
    //     //     break;
    //     // case IMAGEPITCH_MOTORID: // 图传云台右侧电机
    //     //     MotorCan_Receive(Get_ImageMotor(ImagePitch),
    //     //                      Msg->data,
    //     //                      ImagePitch_AngleNow_num,
    //     //                      ImagePitch_SpeedNow_num);
    //     //     break;
    //     // case CHASSISYAW_GYROSCOPEID: // 编码器数据
    //     //     GyroScopeCan_Receive(Get_ArmEncoder(BoomYaw),
    //     //                          Msg->data,
    //     //                          BoomYaw_CoderAngleOrigin_num,
    //     //                          Gyro_YawAngle_num,
    //     //                          Gyro_YawSpeed_num);

    //     //     break;
    //     default:
    //         break;
    //     }
    // }
}

/**
 * @brief 从机2的报文处理函数
 * @brief 小机械臂与图传部分CAN报文的接收
 * @brief 大机械臂部分CAN报文的接收
 * @param Msg   can报文
 */
void Receive_Slave2_Handler(SPICAN_MsgOnTransfer_t *Msg)
{
    if (Msg->ForCANx == (rt_uint32_t)can1)
    {
        switch (Msg->ID)
        {
        case ForearmYaw_MotorID: // 小机械臂Yaw轴电机
            MotorCan_Receive(Get_ArmMotor(ForearmYaw),
                             Msg->data,
                             ForearmYaw_AngleNow_num,
                             ForearmYaw_SpeedNow_num);
            break;
        case ForearmPitch_MotorID: // 小机械臂Pitch轴电机
            MotorCan_Receive(Get_ArmMotor(ForearmPitch),
                             Msg->data,
                             ForearmPitch_AngleNow_num,
                             ForearmPitch_SpeedNow_num);
            break;
        case ForearmRoll_MotorID: // 小机械臂Roll轴电机
            MotorCan_Receive(Get_ArmMotor(ForearmRoll),
                             Msg->data,
                             ForearmRoll_AngleNow_num,
                             ForearmRoll_SpeedNow_num);
            break;
        case ImageYaw_MotorID: // 图传yaw、pitch轴电机
            MotorCan_Receive(Get_ImageMotor(ImageYaw),
                             Msg->data,
                             ImageYaw_AngleNow_num,
                             ImageYaw_SpeedNow_num);
            break;
        default:
            break;
        }
    }
    else if (Msg->ForCANx == (rt_uint32_t)can2)
    {
        switch (Msg->ID)
        {
        case BoomYaw_MotorID: // 大机械臂Yaw轴电机
            MotorCan_Receive(Get_ArmMotor(BoomYaw),
                             Msg->data,
                             BoomYaw_AngleNow_num,
                             BoomYaw_SpeedNow_num);
            break;
        case BoomPitch1_MotorID: // 大机械臂Pitch1轴电机
            MotorCan_Receive(Get_ArmMotor(BoomPitch1),
                             Msg->data,
                             BoomPitch1_AngleNow_num,
                             BoomPitch1_SpeedNow_num);
            break;
        case BoomPitch2_MotorID: // 大机械臂Pitch2轴电机
            MotorCan_Receive(Get_ArmMotor(BoomPitch2),
                             Msg->data,
                             BoomPitch2_AngleNow_num,
                             BoomPitch2_SpeedNow_num);
            break;
        case ImagePitch_MotorID: // 大机械臂Pitch2轴电机
            MotorCan_Receive(Get_ImageMotor(ImagePitch),
                             Msg->data,
                             ImagePitch_AngleNow_num,
                             ImagePitch_SpeedNow_num);
            break;
        case IMAGEYAW_EncoderID: // 图传yaw轴编码器数据
            EncoderCan_Receive(Get_ImageEncoder(ImageYaw),
                               Msg->data,
                               ImageYaw_CoderAngle_num);
            break;
        // case CUSTCTRL_YPR_ID: // 自定义控制器yaw、pitch、roll数据
        //     CustCtrl_ypr_Receive(Msg->data,
        //                          CustCtrler_yaw_num,
        //                          CustCtrler_pitch_num,
        //                          CustCtrler_roll_num);
        //     break;
        // case CUSTCTRL_XYZ_ID: // 自定义控制器xyz数据
        //     CustCtrl_xyz_Receive(Msg->data,
        //                          CustCtrler_x_num,
        //                          CustCtrler_y_num,
        //                          CustCtrler_z_num,
        //                          CustCtrler_Limit_num);
        //     break;
        default:
            break;
        }
    }
}

/**
 * @brief SPICAN初始化
 */
void SPICAN_Init(void)
{
    /*初始化SPI主机*/
    drv_HW_SPI_Init();
    SPI_CAN_Init();
    /*线程初始化*/

    /*发送can报文初始化*/
    SPICan_SendInit(&ForearmMsg_Can1,
                    ARM_CANID, can1);
    SPICan_SendInit(&BoomMsg_Can2,
                    ARM_CANID, can2);
    SPICan_SendInit(&ChassisMsg_Can1,
                    CHASSIS_CANID, can1);
    SPICan_SendInit(&ImageMsg_Can2,
                    IMAGE_CANID, can2);

    /*设置SPI主机接收回调*/
    SPI_CAN_Set_ReceiveFun((rt_uint8_t)Slave1, &Receive_Slave1_Handler);
    SPI_CAN_Set_ReceiveFun((rt_uint8_t)Slave2, &Receive_Slave2_Handler);
    rt_thread_delay(100);
}

/**
 * @brief  电机Can报文处理数据包序列号读取
 */
int MotorCan_Datanum_Find(void)
{
    ForearmYaw_AngleNow_num = Package_Find_Num("F_Yaw_AngleNow");
    ForearmPitch_AngleNow_num = Package_Find_Num("F_Pitch_AngleNow");
    ForearmRoll_AngleNow_num = Package_Find_Num("F_Roll_AngleNow");

    ForearmYaw_SpeedNow_num = Package_Find_Num("F_Yaw_SpeedNow");
    ForearmPitch_SpeedNow_num = Package_Find_Num("F_Pitch_SpeedNow");
    ForearmRoll_SpeedNow_num = Package_Find_Num("F_Roll_SpeedNow");

    BoomYaw_AngleNow_num = Package_Find_Num("B_Yaw_AngleNow");
    BoomPitch1_AngleNow_num = Package_Find_Num("B_Pitch1_AngleNow");
    BoomPitch2_AngleNow_num = Package_Find_Num("B_Pitch2_AngleNow");

    BoomYaw_SpeedNow_num = Package_Find_Num("B_Yaw_SpeedNow");
    BoomPitch1_SpeedNow_num = Package_Find_Num("B_Pitch1_SpeedNow");
    BoomPitch2_SpeedNow_num = Package_Find_Num("B_Pitch2_SpeedNow");

    ChassisSpe_RF_Now_num = Package_Find_Num("ChassisSpeNow_RF");
    ChassisSpe_RB_Now_num = Package_Find_Num("ChassisSpeNow_RB");
    ChassisSpe_LF_Now_num = Package_Find_Num("ChassisSpeNow_LF");
    ChassisSpe_LB_Now_num = Package_Find_Num("ChassisSpeNow_LB");

    // 编码器角度
    //    BoomYaw_CoderAngleOrigin_num = Package_Find_Num("BYaw_CoderOrigin");
    ImageYaw_CoderAngle_num = Package_Find_Num("IYaw_CoderAngle");

    // 图传电机
    ImageYaw_AngleNow_num = Package_Find_Num("I_Yaw_AngleNow");
    ImagePitch_AngleNow_num = Package_Find_Num("I_Pitch_AngleNow");

    ImageYaw_SpeedNow_num = Package_Find_Num("I_Yaw_SpeedNow");
    ImagePitch_SpeedNow_num = Package_Find_Num("I_Pitch_SpeedNow");

    //    /*陀螺仪Yaw角度和角速度*/
    //    Gyro_YawAngle_num = Package_Find_Num("Gyro_YawAngle");
    //    Gyro_YawSpeed_num = Package_Find_Num("Gyro_YawSpeed");

    //    /*自定义控制器数据*/
    //    CustCtrler_Limit_num = Package_Find_Num("CustCtrler_Limit");

    //    CustCtrler_x_num = Package_Find_Num("CustCtrler_x");
    //    CustCtrler_y_num = Package_Find_Num("CustCtrler_y");
    //    CustCtrler_z_num = Package_Find_Num("CustCtrler_z");

    //    CustCtrler_yaw_num = Package_Find_Num("CustCtrler_yaw");
    //    CustCtrler_pitch_num = Package_Find_Num("CustCtrler_pitch");
    //    CustCtrler_roll_num = Package_Find_Num("CustCtrler_roll");

    return 0;
}
