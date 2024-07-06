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
#include "drv_remote.h"
#define recieve_UART_NAME "uart3" /* 串口设备名称 */
#define send_UART_NAME "uart4"    /* 串口设备名称 */
/*自定义控制器相关*/
/* 串口接收消息结构*/
///* 消息队列控制块 */
// static struct rt_messagequeue rx_mq;
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
/* 串口设备句柄 */
static rt_device_t recieve;
/* 用于接收消息的信号量 */
static struct rt_semaphore uart3_sem;
static struct rt_semaphore uart3_1ms_sem; // 用于控制的信号量
// 接收数据长度
#define Data_len sizeof(RecData_t)

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
static rt_int8_t BoomMove_AngleNow_num = 0;
/*大机械臂当前速度值序列号*/
static rt_int8_t BoomYaw_SpeedNow_num = 0;
static rt_int8_t BoomPitch1_SpeedNow_num = 0;
static rt_int8_t BoomPitch2_SpeedNow_num = 0;
static rt_int8_t BoomMove_SpeedNow_num = 0;
/*底盘当前运动速度序列号*/
static rt_int8_t ChassisSpe_RF_Now_num = 0;
static rt_int8_t ChassisSpe_RB_Now_num = 0;
static rt_int8_t ChassisSpe_LF_Now_num = 0;
static rt_int8_t ChassisSpe_LB_Now_num = 0;
/*编码器当前角度序列号*/
static rt_int8_t ImageYaw_CoderAngle_num = 0;
static rt_int8_t ImageHigh_CoderAngle_num = 0;
/*图传云台数据序列号*/
static rt_int8_t ImageYaw_AngleNow_num = 0;
static rt_int8_t ImagePitch_AngleNow_num = 0;
static rt_int8_t ImageYaw_SpeedNow_num = 0;
static rt_int8_t ImagePitch_SpeedNow_num = 0;
static rt_int8_t ImageHigh_AngleNow_num = 0;
static rt_int8_t ImageHigh_SpeedNow_num = 0;
/*自定义控制器数据序列号*/
static rt_int8_t CustCtrler_x_num = 0;
static rt_int8_t CustCtrler_y_num = 0;
static rt_int8_t CustCtrler_z_num = 0;
static rt_int8_t CustCtrler_yaw_num = 0;
static rt_int8_t CustCtrler_pitch_num = 0;
static rt_int8_t CustCtrler_roll_num = 0;
static rt_int8_t CustCtrler_Limit_num = 0;

/*can报文*/
static SPICAN_MsgOnTransfer_t ForearmPitchMsg_Can2, // 标识符0x003  DM6006电机
    ForearmRollMsg_Can2,                            // 标识符0x002  DM6006电机
    BoomMsg_Can1,                                   // 标识符0x1FF  大机械臂和小臂yaw电机
    ChassisMsg_Can1,                                // 标识符0x2FF  底盘电机
    ImageMsg_Can2;                                  // 标识符0x2FF  图传电机和move电机

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
 * @brief 电机单报文写入函数
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
static void MotorCan_Write(SPICAN_MsgOnTransfer_t *Msg,
                           rt_int8_t Motor_ID,
                           rt_uint16_t CtrlSetData)
{
    rt_int8_t index;
    if (Motor_ID < 5)
    {
        index = 2 * (Motor_ID - 1);
        Msg->data[index] = (CtrlSetData) >> 8;
        Msg->data[index + 1] = CtrlSetData;
    }
    else
    {
        index = 2 * (Motor_ID % 5);
        Msg->data[index] = (CtrlSetData) >> 8;
        Msg->data[index + 1] = CtrlSetData;
    }
}
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
 * @brief 达妙电机单报文写入函数
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID         电机ID （1~4）
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
rt_uint16_t data_tran;
static void DM_MotorCan_Write(SPICAN_MsgOnTransfer_t *Msg,
                              rt_int8_t Motor_ID,
                              float CtrlSetData)
{

    data_tran = float_to_uint(CtrlSetData, -1000, 1000.0, 12);
    Msg->data[0] = 0x00;
    Msg->data[1] = 0x00;
    Msg->data[2] = 0x00;
    Msg->data[3] = 0x00;
    Msg->data[4] = 0x00;
    Msg->data[5] = 0x00;
    Msg->data[6] = (data_tran >> 8) | ((0x00 & 0x0F) << 4);
    Msg->data[7] = data_tran;
}

/**
 * @brief 达妙电机使能函数//每次上电都需要使能
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID         电机ID （1~4）
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
static void DM_MotorCan_Init(SPICAN_MsgOnTransfer_t *Msg)
{
    Msg->data[0] = 0xFF;
    Msg->data[1] = 0xFF;
    Msg->data[2] = 0xFF;
    Msg->data[3] = 0xFF;
    Msg->data[4] = 0xFF;
    Msg->data[5] = 0xFF;
    Msg->data[6] = 0xFF;
    Msg->data[7] = 0xFC;
}
/**
 * @brief 达妙电机设置零点函数
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID         电机ID （1~4）
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
static void DM_MotorCan_Zero(SPICAN_MsgOnTransfer_t *Msg)
{
    Msg->data[0] = 0xFF;
    Msg->data[1] = 0xFF;
    Msg->data[2] = 0xFF;
    Msg->data[3] = 0xFF;
    Msg->data[4] = 0xFF;
    Msg->data[5] = 0xFF;
    Msg->data[6] = 0xFF;
    Msg->data[7] = 0xFE;
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
        AngleNow = 0;

    if (SpeedData_num != -1)
    {
        SpeedNow = Motor_Read_NowSpeed(Motor);
        Package_Write_All_Data(SpeedData_num,
                               &SpeedNow,
                               sizeof(SpeedNow));
    }
    else
        SpeedNow = 0;
}

/**
 * @brief 达妙电机报文接收处理函数
 */
static void DM_MotorCan_Receive(Motor_t *Motor,
                                rt_uint8_t rxmsg[],
                                rt_int8_t AngleData_num,
                                rt_int8_t SpeedData_num)
{
    float AngleNow;
    float SpeedNow;

    motor_readmsg_DM(rxmsg, &Motor->dji);

    if (AngleData_num != -1)
    {
        AngleNow = Motor_Read_NowAngle(Motor);
        Package_Write_All_Data(AngleData_num,
                               &AngleNow,
                               sizeof(AngleNow));
    }
    else
        AngleNow = 0;

    if (SpeedData_num != -1)
    {
        SpeedNow = Motor_Read_NowSpeed(Motor);
        Package_Write_All_Data(SpeedData_num,
                               &SpeedNow,
                               sizeof(SpeedNow));
    }
    else
        SpeedNow = 0;
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
        angle_now = 0;
}

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
 * @brief 从机1的报文发送函数
 * @brief 底盘部分与图传部分电机报文发送
 */
void Send_Slave1_Handle(ChassisMotor_t *Chassis_in,
                        ImageMotor_s *Image_in,
                        BoomMotor_s *Boom_in)
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

        MotorCan_Write(&ImageMsg_Can2,
                       5, (rt_int16_t)Boom_in->BoomMove);
//    MotorCan_Write(&ImageMsg_Can2,
//                   5, 0);
    MotorCan_Write(&ImageMsg_Can2,
                   6, (rt_int16_t)Image_in->ImageYaw);
//        MotorCan_Write(&ImageMsg_Can2,
//                       6, 0);
//    MotorCan_Write(&ImageMsg_Can2,
//                   7,0);
    MotorCan_Write(&ImageMsg_Can2,
                   7, (rt_int16_t)Image_in->ImagePitch);
    MotorCan_Write(&ImageMsg_Can2,
                   8, (rt_int16_t)Image_in->ImageHigh);
//    MotorCan_Write(&ImageMsg_Can2,
//                   8,0);
     SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ChassisMsg_Can1);
     SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ImageMsg_Can2);
}

/**
 * @brief 从机2的报文发送函数
 * @brief 机械臂部分电机报文发送
 * @param in
 */
rt_uint8_t dm_zero_debug = 1;
float debug_pwm;
void Send_Slave2_Handle(BoomMotor_s *Boom_in,
                        ForearmMotor_s *Forearm_in)
{
	if(dm_zero_debug==1)
	{
    DM_MotorCan_Write(&ForearmRollMsg_Can2,
                      2, Forearm_in->ForearmRoll);
    DM_MotorCan_Write(&ForearmPitchMsg_Can2,
                      3, Forearm_in->ForearmPitch);
	}
	else
	{
    DM_MotorCan_Write(&ForearmRollMsg_Can2,
                      2,debug_pwm);
    DM_MotorCan_Write(&ForearmPitchMsg_Can2,
                      3,-debug_pwm);
	}		
    MotorCan_Write(&BoomMsg_Can1,
                   1, (rt_int16_t)Boom_in->BoomYaw);
    MotorCan_Write(&BoomMsg_Can1,
                   4, (rt_int16_t)Forearm_in->ForearmYaw);
    MotorCan_Write(&BoomMsg_Can1,
                   3, (rt_int16_t)Boom_in->BoomPitch1);
    MotorCan_Write(&BoomMsg_Can1,
                   2, (rt_int16_t)Boom_in->BoomPitch2);

        SPI_CAN_SendMsg((rt_uint8_t)Slave2, &ForearmPitchMsg_Can2);
        SPI_CAN_SendMsg((rt_uint8_t)Slave2, &ForearmRollMsg_Can2);
        SPI_CAN_SendMsg((rt_uint8_t)Slave2, &BoomMsg_Can1);
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
                   5, 0);
    MotorCan_Write(&ImageMsg_Can2,
                   6, 0);
    MotorCan_Write(&ImageMsg_Can2,
                   7, 0);
    MotorCan_Write(&ImageMsg_Can2,
                   8, 0);

    SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ChassisMsg_Can1);
    SPI_CAN_SendMsg((rt_uint8_t)Slave1, &ImageMsg_Can2);
}

/**
 * @brief 从机2的报文发送初始化函数
 * @brief 机械臂部分电机报文发送
 * @param in
 */
rt_uint8_t init_debug = 0;
void Send_Slave2_Init(void)
{

    MotorCan_Write(&BoomMsg_Can1,
                   1, 0);
    MotorCan_Write(&BoomMsg_Can1,
                   2, 0);
    MotorCan_Write(&BoomMsg_Can1,
                   3, 0);
    MotorCan_Write(&BoomMsg_Can1,
                   4, 0);
    // if(init_debug==0)
    //{
    rt_thread_mdelay(10);
    DM_MotorCan_Init(&ForearmRollMsg_Can2);
    DM_MotorCan_Init(&ForearmPitchMsg_Can2);
    //	init_debug++;
    //}
    // else
    //{
    // DM_MotorCan_Zero(&ForearmRollMsg_Can2);
    // DM_MotorCan_Zero(&ForearmPitchMsg_Can2);
    // init_debug++;
    //}
    for (int i = 0; i < 10; i++)
    {
        SPI_CAN_SendMsg((rt_uint8_t)Slave2, &ForearmRollMsg_Can2);
        rt_thread_mdelay(2);

        SPI_CAN_SendMsg((rt_uint8_t)Slave2, &ForearmPitchMsg_Can2);
        rt_thread_mdelay(2);
    }
        SPI_CAN_SendMsg((rt_uint8_t)Slave2, &BoomMsg_Can1);
		
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
    else if (Msg->ForCANx == (rt_uint32_t)can2)
    {
        switch (Msg->ID)
        {
        case ImagePitch_MotorID:
            MotorCan_Receive(Get_ImageMotor(ImagePitch),
                             Msg->data,
                             ImagePitch_AngleNow_num,
                             ImagePitch_SpeedNow_num);
            break;
        case ImageYaw_MotorID: // 图传yaw
            MotorCan_Receive(Get_ImageMotor(ImageYaw),
                             Msg->data,
                             ImageYaw_AngleNow_num,
                             ImageYaw_SpeedNow_num);
            break;
        case ImageHigh_MotorID:
            MotorCan_Receive(Get_ImageMotor(ImageHigh),
                             Msg->data,
                             ImageHigh_AngleNow_num,
                             ImageHigh_SpeedNow_num);
            break;
        case BoomMove_MotorID:
            MotorCan_Receive(Get_ArmMotor(BoomMove),
                             Msg->data,
                             BoomMove_AngleNow_num,
                             BoomMove_SpeedNow_num);
            break;
        default:
            break;
        }
    }
}
/**
 * @brief 从机2的报文处理函数
 * @brief 小机械臂与图传部分CAN报文的接收
 * @brief 大机械臂部分CAN报文的接收
 * @param Msg   can报文
 */
SPICAN_MsgOnTransfer_t Msg1;
int last_id;
void Receive_Slave2_Handler(SPICAN_MsgOnTransfer_t *Msg)
{
    if (Msg->ForCANx == (rt_uint32_t)can2)
    {
        switch (Msg->ID)
        {
        case ForearmPitch_MotorID: // 小机械臂Pitch轴电机
            DM_MotorCan_Receive(Get_ArmMotor(ForearmPitch),
                                Msg->data,
                                ForearmPitch_AngleNow_num,
                                ForearmPitch_SpeedNow_num);
            Msg1 = *Msg;
            break;
        case ForearmRoll_MotorID: // 小机械臂Roll轴电机
            DM_MotorCan_Receive(Get_ArmMotor(ForearmRoll),
                                Msg->data,
                                ForearmRoll_AngleNow_num,
                                ForearmRoll_SpeedNow_num);
            break;
        default:
            break;
        }
    }
    else if (Msg->ForCANx == (rt_uint32_t)can1)
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
        case BoomPitch2_MotorID:
            MotorCan_Receive(Get_ArmMotor(BoomPitch2),
                             Msg->data,
                             BoomPitch2_AngleNow_num,
                             BoomPitch2_SpeedNow_num);
            break;
        case ForearmYaw_MotorID: // 小机械臂Yaw轴电机
            MotorCan_Receive(Get_ArmMotor(ForearmYaw),
                             Msg->data,
                             ForearmYaw_AngleNow_num,
                             ForearmYaw_SpeedNow_num);
            break;
        default:
            last_id = Msg->ID;
            break;
        }
    }
}

typedef union
{
    float a;
    uint8_t b[4];
} float2int_u;
/**
 * @brief  自定义控制器控制定时器回调函数
 */
static void reuart3_1ms_Handler(void *parameter)
{
    // 每隔1ms 释放一次发送信号量
    rt_sem_release(&uart3_1ms_sem);
}
/**
 * @brief 自定义控制器 uart 接收yaw、pitch、roll，x，y，z数据报文接收解算
 */
static void CustCtrl_xyzvpr_Receive(RecData_t *RecData_buff,
                                    rt_int8_t CustCtrl_yaw_num,
                                    rt_int8_t CustCtrl_pitch_num,
                                    rt_int8_t CustCtrl_roll_num,
                                    rt_int8_t CustCtrl_x_num,
                                    rt_int8_t CustCtrl_y_num,
                                    rt_int8_t CustCtrl_z_num)
{
    //    float data_temp;
    float data_x, data_y, data_z, data_yaw, data_roll, data_pitch;
    float2int_u cvt;

    if (CustCtrl_yaw_num != -1)
    {
        cvt.b[0] = RecData_buff->yaw[0];
        cvt.b[1] = RecData_buff->yaw[1];
        cvt.b[2] = RecData_buff->yaw[2];
        cvt.b[3] = RecData_buff->yaw[3];
        data_yaw = cvt.a;
        Package_Write_All_Data(CustCtrl_yaw_num,
                               &data_yaw,
                               sizeof(data_yaw));
    }
    if (CustCtrl_pitch_num != -1)
    {
        cvt.b[0] = RecData_buff->pitch[0];
        cvt.b[1] = RecData_buff->pitch[1];
        cvt.b[2] = RecData_buff->pitch[2];
        cvt.b[3] = RecData_buff->pitch[3];
        data_pitch = cvt.a;
        Package_Write_All_Data(CustCtrl_pitch_num,
                               &data_pitch,
                               sizeof(data_pitch));
    }
    if (CustCtrl_roll_num != -1)
    {
        cvt.b[0] = RecData_buff->roll[0];
        cvt.b[1] = RecData_buff->roll[1];
        cvt.b[2] = RecData_buff->roll[2];
        cvt.b[3] = RecData_buff->roll[3];
        data_roll = cvt.a;
        Package_Write_All_Data(CustCtrl_roll_num,
                               &data_roll,
                               sizeof(data_roll));
    }
    if (CustCtrl_x_num != -1)
    {
        cvt.b[0] = RecData_buff->x[0];
        cvt.b[1] = RecData_buff->x[1];
        cvt.b[2] = RecData_buff->x[2];
        cvt.b[3] = RecData_buff->x[3];
        data_x = cvt.a;
        Package_Write_All_Data(CustCtrl_x_num,
                               &data_x,
                               sizeof(data_x));
    }
    if (CustCtrl_y_num != -1)
    {
        cvt.b[0] = RecData_buff->y[0];
        cvt.b[1] = RecData_buff->y[1];
        cvt.b[2] = RecData_buff->y[2];
        cvt.b[3] = RecData_buff->y[3];
        data_y = cvt.a;
        Package_Write_All_Data(CustCtrl_y_num,
                               &data_y,
                               sizeof(data_y));
    }
    if (CustCtrl_z_num != -1)
    {
        cvt.b[0] = RecData_buff->z[0];
        cvt.b[1] = RecData_buff->z[1];
        cvt.b[2] = RecData_buff->z[2];
        cvt.b[3] = RecData_buff->z[3];
        data_z = cvt.a;
        Package_Write_All_Data(CustCtrl_z_num,
                               &data_z,
                               sizeof(data_z));
    }
}
/* 数据解析线程uart */
static void data_parsing(RecData_t *RecData_buff)
{
    CustCtrl_xyzvpr_Receive(RecData_buff, CustCtrler_yaw_num, CustCtrler_pitch_num,
                            CustCtrler_roll_num, CustCtrler_x_num,
                            CustCtrler_y_num, CustCtrler_z_num);
}
static void Key_Data_Parsing(Remote_Control_t *RecData_buff,RC_Ctrl_t *RC_CtrlData) 
{
    RC_CtrlData->Mouse_Data.x_speed =RecData_buff->mouse_x;
    RC_CtrlData->Mouse_Data.y_speed = RecData_buff->mouse_y;
    RC_CtrlData->Mouse_Data.z_speed = RecData_buff->mouse_z;
    RC_CtrlData->Mouse_Data.press_l = RecData_buff->left_button_down;
    RC_CtrlData->Mouse_Data.press_r = RecData_buff->right_button_down;
    RC_CtrlData->v = RecData_buff->keyboard_value;
    RC_CtrlData->Key_Data.W = (RC_CtrlData->v & 0x0001) == 0x0001;
    RC_CtrlData->Key_Data.S = (RC_CtrlData->v & 0x0002) == 0x0002;
    RC_CtrlData->Key_Data.A = (RC_CtrlData->v & 0x0004) == 0x0004;
    RC_CtrlData->Key_Data.D = (RC_CtrlData->v & 0x0008) == 0x0008;
    RC_CtrlData->Key_Data.shift = (RC_CtrlData->v & 0x0010) == 0x0010;
    RC_CtrlData->Key_Data.ctrl = (RC_CtrlData->v & 0x0020) == 0x0020;
    RC_CtrlData->Key_Data.Q = (RC_CtrlData->v & 0x0040) == 0x0040;
    RC_CtrlData->Key_Data.E = (RC_CtrlData->v & 0x0080) == 0x0080;
    RC_CtrlData->Key_Data.R = (RC_CtrlData->v & 0x0100) == 0x0100;
    RC_CtrlData->Key_Data.F = (RC_CtrlData->v & 0x0200) == 0x0200;
    RC_CtrlData->Key_Data.G = (RC_CtrlData->v & 0x0400) == 0x0400;
    RC_CtrlData->Key_Data.Z = (RC_CtrlData->v & 0x0800) == 0x0800;
    RC_CtrlData->Key_Data.X = (RC_CtrlData->v & 0x1000) == 0x1000;
    RC_CtrlData->Key_Data.C = (RC_CtrlData->v & 0x2000) == 0x2000;
    RC_CtrlData->Key_Data.V = (RC_CtrlData->v & 0x4000) == 0x4000;
    RC_CtrlData->Key_Data.B = (RC_CtrlData->v & 0x8000) == 0x8000;        
}
/* 接收数据回调函数uart */
static char size_custom;
char rx_uart_data[39];
uint8_t read_buffer_3[200];
int16_t debug_rx3=0;
static rt_err_t uart3_input(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        size_custom = size;
        rt_device_read(dev, 0, &rx_uart_data[0], size_custom);
//        if (rx_uart_data[5] != 0x04 && rx_uart_data[6] != 0x03)
//        {
//            if (debug_rx3 < 200&&size_custom!=21)
//            {
//                for (int i = 0; i < size; i++)
//                {
//                    read_buffer_3[i + debug_rx3] = rx_uart_data[i];
//                    debug_rx3 += size;
//                }
//            }
//        }
        rt_sem_release(&uart3_sem);
    }
    return RT_EOK;
}
int debug_uart3_flag;
rt_tick_t mid_uart3;
float real_frequency_uart3;
static void receive_thread_entry(void *parameter) // 数据接收线程
{
	      rt_tick_t tick_uart3=rt_tick_get();
    while (1)
    {
        rt_sem_take(&uart3_sem, RT_WAITING_FOREVER);
        if (size_custom == 39)
        {
            if (Verify_CRC8_Check_Sum((uint8_t *)rx_uart_data, 5) &&
                Verify_CRC16_Check_Sum((uint8_t *)rx_uart_data, 39))
            {
		debug_uart3_flag++;
		if((debug_uart3_flag%100)==0)
		{
			mid_uart3=tick_uart3;
			tick_uart3=rt_tick_get();
			real_frequency_uart3 = 1/((tick_uart3-mid_uart3)/100.0f)*1000.0f;
		}//用来检测当前实时计算的频率
                data_parsing((RecData_t *)&rx_uart_data[0]);
            }
        }
		else if(size_custom == 21&&Remote_Tanslate!=1)
		{
            Key_Data_Parsing((Remote_Control_t *)&rx_uart_data[0],&RC_data);
		}
    }
}
void Uart_Init_Custom(void) // 自定义控制器串口初始化
{
    struct serial_configure configr = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */
    configr.baud_rate = 115200;                                 // 修改波特率为 1500000
    configr.data_bits = DATA_BITS_8;                            // 数据位 8
    configr.stop_bits = STOP_BITS_1;                            // 停止位 1
    configr.bufsz = 128;                                        // 修改缓冲区 buff size 为 128
    configr.parity = PARITY_NONE;                               // 无奇偶校验位
    /* 查找串口设备 */
    recieve = rt_device_find(recieve_UART_NAME); /*接收串口1*/
    if (!recieve)
    {
        rt_kprintf("find %s failed!\n", recieve_UART_NAME);
    }

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(recieve, RT_DEVICE_CTRL_CONFIG, &configr);
    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(recieve, RT_DEVICE_FLAG_DMA_RX);
}

/**
 * @brief SPICAN初始化
 */
void SPICAN_Init(void)
{
    /*初始化SPI主机*/
    drv_HW_SPI_Init();
    SPI_CAN_Init();
    rt_err_t res_uart3 = RT_EOK;
    /*初始化信号量*/
    rt_sem_init(&uart3_1ms_sem, "uart3_1ms_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&uart3_sem, "uart3_sem", 0, RT_IPC_FLAG_FIFO);
    /*线程初始化*/
    Uart_Init_Custom(); // 自定义控制器串口
    /*发送can报文初始化*/
    SPICan_SendInit(&ForearmRollMsg_Can2,
                    FOREARM_ROLL_CANID, can2);
    SPICan_SendInit(&ForearmPitchMsg_Can2,
                    FOREARM_PITCH_CANID, can2);
    SPICan_SendInit(&BoomMsg_Can1,
                    BOOM_CANID, can1);
    SPICan_SendInit(&ChassisMsg_Can1,
                    CHASSIS_CANID, can1);
    SPICan_SendInit(&ImageMsg_Can2,
                    IMAGE_CANID, can2);

    /*设置SPI主机接收回调*/
    SPI_CAN_Set_ReceiveFun((rt_uint8_t)Slave1, &Receive_Slave1_Handler);
    SPI_CAN_Set_ReceiveFun((rt_uint8_t)Slave2, &Receive_Slave2_Handler);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(recieve, uart3_input);
    /* 创建 serial 线程 */
    rt_thread_t recieve_rt = rt_thread_create("recieve_rt", receive_thread_entry, RT_NULL, 1024, 30, 5);
    /* 创建成功则启动线程 */
    if (recieve_rt != RT_NULL)
    {
        rt_thread_startup(recieve_rt);
    }
    /*创建线程定时器*/
    rt_timer_t timer_uart3 = rt_timer_create("reuart3_1ms_timer",
                                             reuart3_1ms_Handler, // 回调函数
                                             RT_NULL,
                                             ROBOCTRL_TIMER_PIRIOD, // 定时时间（ms）
                                             RT_TIMER_FLAG_PERIODIC);
    res_uart3 = rt_timer_start(timer_uart3); // 启动定时器
                                             //    if (res_uart3 != RT_EOK)
                                             //        return RT_ERROR; // 开启失败
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
    BoomMove_AngleNow_num = Package_Find_Num("B_Move_AngleNow");

    BoomYaw_SpeedNow_num = Package_Find_Num("B_Yaw_SpeedNow");
    BoomPitch1_SpeedNow_num = Package_Find_Num("B_Pitch1_SpeedNow");
    BoomPitch2_SpeedNow_num = Package_Find_Num("B_Pitch2_SpeedNow");
    BoomMove_SpeedNow_num = Package_Find_Num("B_Move_SpeedNow");

    ChassisSpe_RF_Now_num = Package_Find_Num("ChassisSpeNow_RF");
    ChassisSpe_RB_Now_num = Package_Find_Num("ChassisSpeNow_RB");
    ChassisSpe_LF_Now_num = Package_Find_Num("ChassisSpeNow_LF");
    ChassisSpe_LB_Now_num = Package_Find_Num("ChassisSpeNow_LB");

    // 编码器角度
    ImageYaw_CoderAngle_num = Package_Find_Num("IYaw_CoderAngle");
    ImageHigh_CoderAngle_num = Package_Find_Num("IHigh_CoderAngle");
    // 图传电机
    ImageYaw_AngleNow_num = Package_Find_Num("I_Yaw_AngleNow");
    ImagePitch_AngleNow_num = Package_Find_Num("I_Pitch_AngleNow");
    ImageHigh_AngleNow_num = Package_Find_Num("I_High_AngleNow");
    ImageYaw_SpeedNow_num = Package_Find_Num("I_Yaw_SpeedNow");
    ImagePitch_SpeedNow_num = Package_Find_Num("I_Pitch_SpeedNow");
    ImageHigh_SpeedNow_num = Package_Find_Num("I_High_SpeedNow");

    /*自定义控制器数据*/
    CustCtrler_Limit_num = Package_Find_Num("CustCtrler_Limit");

    CustCtrler_x_num = Package_Find_Num("CustCtrler_x");
    CustCtrler_y_num = Package_Find_Num("CustCtrler_y");
    CustCtrler_z_num = Package_Find_Num("CustCtrler_z");

    CustCtrler_yaw_num = Package_Find_Num("CustCtrler_yaw");
    CustCtrler_pitch_num = Package_Find_Num("CustCtrler_pitch");
    CustCtrler_roll_num = Package_Find_Num("CustCtrler_roll");

    return 0;
}
