#include "func_Dataserver.h"
#include "drv_dataserve.h"

/*序列号*/
/*小机械臂当前角度值序列号*/
static rt_int8_t ForearmYaw_AngleNow_num = 0;
static rt_int8_t ForearmPitch_AngleNow_num = 0;
static rt_int8_t ForearmRoll_AngleNow_num = 0;
/*小机械臂当前速度值序列号*/
static rt_int8_t ForearmYaw_SpeedNow_num = 0;
static rt_int8_t ForearmPitch_SpeedNow_num = 0;
static rt_int8_t ForearmRoll_SpeedNow_num = 0;
/*小机械臂期望角度值序列号*/
static rt_int8_t ForearmYaw_AngleHope_num = 0;
static rt_int8_t ForearmPitch_AngleHope_num = 0;
static rt_int8_t ForearmRoll_AngleHope_num = 0;

/*小机械臂当前弧度值序列号*/
static rt_int8_t ForearmYaw_AngleFilter_num = 0;
static rt_int8_t ForearmPitch_AngleFilter_num = 0;
static rt_int8_t ForearmRoll_AngleFilter_num = 0;

/*大机械臂当前弧度值序列号*/
static rt_int8_t BoomYaw_AngleCircle_num = 0;
static rt_int8_t BoomPitch1_AngleFilter_num = 0;
static rt_int8_t BoomPitch2_AngleFilter_num = 0;

/*大机械臂当前角度值序列号*/
static rt_int8_t BoomYaw_AngleNow_num = 0;
static rt_int8_t BoomPitch1_AngleNow_num = 0;
static rt_int8_t BoomPitch2_AngleNow_num = 0;

/*大机械臂当前速度值序列号*/
static rt_int8_t BoomYaw_SpeedNow_num = 0;
static rt_int8_t BoomPitch1_SpeedNow_num = 0;
static rt_int8_t BoomPitch2_SpeedNow_num = 0;
static rt_int8_t BoomYaw_SpeedFilter_num = 0;

/*机械臂末端当前坐标值序列号*/
static rt_int8_t ArmTop_x_Now_num = 0;
static rt_int8_t ArmTop_y_Now_num = 0;
static rt_int8_t ArmTop_z_Now_num = 0;
/*机械臂末端设定坐标值序列号*/
static rt_int8_t ArmTop_x_Set_num = 0;
static rt_int8_t ArmTop_y_Set_num = 0;
static rt_int8_t ArmTop_z_Set_num = 0;

/*底盘设定运动速度序列号*/
static rt_int8_t ChassisSpe_RF_Set_num = 0;
static rt_int8_t ChassisSpe_RB_Set_num = 0;
static rt_int8_t ChassisSpe_LF_Set_num = 0;
static rt_int8_t ChassisSpe_LB_Set_num = 0;

/*底盘当前运动速度序列号*/
static rt_int8_t ChassisSpe_RF_Now_num = 0;
static rt_int8_t ChassisSpe_RB_Now_num = 0;
static rt_int8_t ChassisSpe_LF_Now_num = 0;
static rt_int8_t ChassisSpe_LB_Now_num = 0;

/*自定义控制器数据序列号*/
static rt_int8_t CustCtrler_x_num = 0;
static rt_int8_t CustCtrler_y_num = 0;
static rt_int8_t CustCtrler_z_num = 0;
static rt_int8_t CustCtrler_yaw_num = 0;
static rt_int8_t CustCtrler_pitch_num = 0;
static rt_int8_t CustCtrler_roll_num = 0;
static rt_int8_t CustCtrler_Limit_num = 0;

/*图传云台数据序列号*/
static rt_int8_t ImageYaw_AngleNow_num = 0;
static rt_int8_t ImagePitch_AngleNow_num = 0;

static rt_int8_t ImageYaw_AngleHope_num = 0;
static rt_int8_t ImagePitch_AngleHope_num = 0;

static rt_int8_t ImageYaw_SpeedNow_num = 0;
static rt_int8_t ImagePitch_SpeedNow_num = 0;

static rt_int8_t Image_PosState_num = 0;

/*编码器角度数据*/
static rt_int8_t BoomYaw_CoderAngleOrigin_num = 0;
static rt_int8_t BoomYaw_CoderCaliRad_num = 0;
static rt_int8_t ImageYaw_CoderAngle_num = 0;

static rt_int8_t ImageGimbal_YawAngle_num = 0;
static rt_int8_t ImageGimbal_PitchAngle_num = 0;

/*机器人速度增益序列号*/
static rt_int8_t Robo_SpeedGain_num = 0;

/*机械臂超限*/
static rt_int8_t Arm_IfLimit_num = 0;

/*UI复位数据*/
static rt_int8_t UIReset_State_num = 0;

/*陀螺仪Yaw角度和角速度*/
static rt_int8_t Gyro_YawAngle_num = 0;
static rt_int8_t Gyro_YawSpeed_num = 0;
static rt_int8_t Gyro_YawRadFilter_num = 0;

/*编码器标定状态位*/
static rt_int8_t EncoderCali_State_num = 0;

/*云台系与底盘系角度及云台期望角度*/
static rt_int8_t GimbalYawRad_Now_num = 0;
static rt_int8_t GimbalYawRad_Set_num = 0;
static rt_int8_t ChassisYawRad_Now_num = 0;
static rt_int8_t ChassisYawSpeed_Now_num = 0;

/**
 * @brief   从数据服务器读取机械臂电机的速度当前值
 * @param   out
 */
void ArmMotor_NowSpeed_Read(BoomMotor_s *Boom_out,
                            ForearmMotor_s *Forearm_out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_SpeedNow_num, float);
    Boom_out->BoomYaw = *address;
    Package_Write_Pionter_End(BoomYaw_SpeedNow_num, float);

    address = Package_Pionter_Single(BoomPitch1_SpeedNow_num, float);
    Boom_out->BoomPitch1 = *address;
    Package_Write_Pionter_End(BoomPitch1_SpeedNow_num, float);

    address = Package_Pionter_Single(BoomPitch2_SpeedNow_num, float);
    Boom_out->BoomPitch2 = *address;
    Package_Write_Pionter_End(BoomPitch2_SpeedNow_num, float);

    address = Package_Pionter_Single(ForearmYaw_SpeedNow_num, float);
    Forearm_out->ForearmYaw = *address;
    Package_Write_Pionter_End(ForearmYaw_SpeedNow_num, float);

    address = Package_Pionter_Single(ForearmPitch_SpeedNow_num, float);
    Forearm_out->ForearmPitch = *address;
    Package_Write_Pionter_End(ForearmPitch_SpeedNow_num, float);

    address = Package_Pionter_Single(ForearmRoll_SpeedNow_num, float);
    Forearm_out->ForearmRoll = *address;
    Package_Write_Pionter_End(ForearmRoll_SpeedNow_num, float);
}

/**
 * @brief   从数据服务器读取机械臂电机的角度当前值
 * @param   out
 */
void ArmMotor_NowAngle_Read(BoomMotor_s *Boom_out,
                            ForearmMotor_s *Forearm_out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_AngleNow_num, float);
    Boom_out->BoomYaw = *address;
    Package_Write_Pionter_End(BoomYaw_AngleNow_num, float);

    address = Package_Pionter_Single(BoomPitch1_AngleNow_num, float);
    Boom_out->BoomPitch1 = *address;
    Package_Write_Pionter_End(BoomPitch1_AngleNow_num, float);

    address = Package_Pionter_Single(BoomPitch2_AngleNow_num, float);
    Boom_out->BoomPitch2 = *address;
    Package_Write_Pionter_End(BoomPitch2_AngleNow_num, float);

    address = Package_Pionter_Single(ForearmYaw_AngleNow_num, float);
    Forearm_out->ForearmYaw = *address;
    Package_Write_Pionter_End(ForearmYaw_AngleNow_num, float);

    address = Package_Pionter_Single(ForearmPitch_AngleNow_num, float);
    Forearm_out->ForearmPitch = *address;
    Package_Write_Pionter_End(ForearmPitch_AngleNow_num, float);

    address = Package_Pionter_Single(ForearmRoll_AngleNow_num, float);
    Forearm_out->ForearmRoll = *address;
    Package_Write_Pionter_End(ForearmRoll_AngleNow_num, float);
}

/**
 * @brief   从数据服务器读取机械臂电机的姿态设定值
 * @param   out
 */
void ArmMotor_SetPos_Read(VectorXYZ_Str *Pos_out,
                          ForearmMotor_s *Forearm_out)
{
    float *address;

    address = Package_Pionter_Single(ArmTop_x_Set_num, float);
    Pos_out->x = *address;
    Package_Write_Pionter_End(ArmTop_x_Set_num, float);

    address = Package_Pionter_Single(ArmTop_y_Set_num, float);
    Pos_out->y = *address;
    Package_Write_Pionter_End(ArmTop_y_Set_num, float);

    address = Package_Pionter_Single(ArmTop_z_Set_num, float);
    Pos_out->z = *address;
    Package_Write_Pionter_End(ArmTop_z_Set_num, float);

    address = Package_Pionter_Single(ForearmYaw_AngleHope_num, float);
    Forearm_out->ForearmYaw = *address;
    Package_Write_Pionter_End(ForearmYaw_AngleHope_num, float);

    address = Package_Pionter_Single(ForearmPitch_AngleHope_num, float);
    Forearm_out->ForearmPitch = *address;
    Package_Write_Pionter_End(ForearmPitch_AngleHope_num, float);

    address = Package_Pionter_Single(ForearmRoll_AngleHope_num, float);
    Forearm_out->ForearmRoll = *address;
    Package_Write_Pionter_End(ForearmRoll_AngleHope_num, float);
}

/**
 * @brief   机械臂电机的姿态当前值写入数据服务器
 * @param   out
 */
void ArmMotor_NowPos_Write(VectorXYZ_Str *Pos_out)
{
    float *address;

    address = Package_Pionter_Single(ArmTop_x_Now_num, float);
    *address = Pos_out->x;
    Package_Write_Pionter_End(ArmTop_x_Now_num, float);

    address = Package_Pionter_Single(ArmTop_y_Now_num, float);
    *address = Pos_out->y;
    Package_Write_Pionter_End(ArmTop_y_Now_num, float);

    address = Package_Pionter_Single(ArmTop_z_Now_num, float);
    *address = Pos_out->z;
    Package_Write_Pionter_End(ArmTop_z_Now_num, float);
}

/**
 * @brief   机械臂电机的姿态设定值写入数据服务器
 * @param   out
 */
void ArmMotor_SetPos_Write(VectorXYZ_Str *Pos_out)
{
    float *address;

    address = Package_Pionter_Single(ArmTop_x_Set_num, float);
    *address = Pos_out->x;
    Package_Write_Pionter_End(ArmTop_x_Set_num, float);

    address = Package_Pionter_Single(ArmTop_y_Set_num, float);
    *address = Pos_out->y;
    Package_Write_Pionter_End(ArmTop_y_Set_num, float);

    address = Package_Pionter_Single(ArmTop_z_Set_num, float);
    *address = Pos_out->z;
    Package_Write_Pionter_End(ArmTop_z_Set_num, float);
}

/**
 * @brief   机械臂电机的角度设定值写入数据服务器
 * @param   in
 */
void ArmMotor_SetAngle_Write(ForearmMotor_s *Forearm_in)
{
    float *address;

    address = Package_Pionter_Single(ForearmYaw_AngleHope_num, float);
    *address = Forearm_in->ForearmYaw;
    Package_Write_Pionter_End(ForearmYaw_AngleHope_num, float);

    address = Package_Pionter_Single(ForearmPitch_AngleHope_num, float);
    *address = Forearm_in->ForearmPitch;
    Package_Write_Pionter_End(ForearmPitch_AngleHope_num, float);

    address = Package_Pionter_Single(ForearmRoll_AngleHope_num, float);
    *address = Forearm_in->ForearmRoll;
    Package_Write_Pionter_End(ForearmRoll_AngleHope_num, float);
}

/**
 * @brief   机械臂电机的弧度滤波值写入数据服务器
 * @param   in
 */
void ArmMotor_RadFilter_Write(BoomMotor_s *Boom_in,
                              ForearmMotor_s *Forearm_in)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_AngleCircle_num, float);
    *address = Boom_in->BoomYaw;
    Package_Write_Pionter_End(BoomYaw_AngleCircle_num, float);

    address = Package_Pionter_Single(BoomPitch1_AngleFilter_num, float);
    *address = Boom_in->BoomPitch1;
    Package_Write_Pionter_End(BoomPitch1_AngleFilter_num, float);

    address = Package_Pionter_Single(BoomPitch2_AngleFilter_num, float);
    *address = Boom_in->BoomPitch2;
    Package_Write_Pionter_End(BoomPitch2_AngleFilter_num, float);

    address = Package_Pionter_Single(ForearmYaw_AngleFilter_num, float);
    *address = Forearm_in->ForearmYaw;
    Package_Write_Pionter_End(ForearmYaw_AngleFilter_num, float);

    address = Package_Pionter_Single(ForearmPitch_AngleFilter_num, float);
    *address = Forearm_in->ForearmPitch;
    Package_Write_Pionter_End(ForearmPitch_AngleFilter_num, float);

    address = Package_Pionter_Single(ForearmRoll_AngleFilter_num, float);
    *address = Forearm_in->ForearmRoll;
    Package_Write_Pionter_End(ForearmRoll_AngleFilter_num, float);
}

/**
 * @brief   从数据服务器读取机械臂电机的弧度滤波值
 * @param   out
 */
void ArmMotor_RadFilter_Read(BoomMotor_s *Boom_out,
                             ForearmMotor_s *Forearm_out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_AngleCircle_num, float);
    Boom_out->BoomYaw = *address;
    Package_Write_Pionter_End(BoomYaw_AngleCircle_num, float);

    address = Package_Pionter_Single(BoomPitch1_AngleFilter_num, float);
    Boom_out->BoomPitch1 = *address;
    Package_Write_Pionter_End(BoomPitch1_AngleFilter_num, float);

    address = Package_Pionter_Single(BoomPitch2_AngleFilter_num, float);
    Boom_out->BoomPitch2 = *address;
    Package_Write_Pionter_End(BoomPitch2_AngleFilter_num, float);

    address = Package_Pionter_Single(ForearmYaw_AngleFilter_num, float);
    Forearm_out->ForearmYaw = *address;
    Package_Write_Pionter_End(ForearmYaw_AngleFilter_num, float);

    address = Package_Pionter_Single(ForearmPitch_AngleFilter_num, float);
    Forearm_out->ForearmPitch = *address;
    Package_Write_Pionter_End(ForearmPitch_AngleFilter_num, float);

    address = Package_Pionter_Single(ForearmRoll_AngleFilter_num, float);
    Forearm_out->ForearmRoll = *address;
    Package_Write_Pionter_End(ForearmRoll_AngleFilter_num, float);
}

/**
 * @brief   从数据服务器读取大机械臂电机的弧度滤波值
 * @param   out
 */
void BoomMotor_RadFilter_Read(BoomMotor_s *Boom_out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_AngleCircle_num, float);
    Boom_out->BoomYaw = *address;
    Package_Write_Pionter_End(BoomYaw_AngleCircle_num, float);

    address = Package_Pionter_Single(BoomPitch1_AngleFilter_num, float);
    Boom_out->BoomPitch1 = *address;
    Package_Write_Pionter_End(BoomPitch1_AngleFilter_num, float);

    address = Package_Pionter_Single(BoomPitch2_AngleFilter_num, float);
    Boom_out->BoomPitch2 = *address;
    Package_Write_Pionter_End(BoomPitch2_AngleFilter_num, float);
}

/****************************机械臂姿态相关**************************/

/**
 * @brief   从数据服务器读取机械臂电机的姿态设定值
 * @param   out
 */
void ArmPosSet_Read(VectorXYZ_Str *Pos_out)
{
    float *address;

    address = Package_Pionter_Single(ArmTop_x_Set_num, float);
    Pos_out->x = *address;
    Package_Write_Pionter_End(ArmTop_x_Set_num, float);

    address = Package_Pionter_Single(ArmTop_y_Set_num, float);
    Pos_out->y = *address;
    Package_Write_Pionter_End(ArmTop_y_Set_num, float);

    address = Package_Pionter_Single(ArmTop_z_Set_num, float);
    Pos_out->z = *address;
    Package_Write_Pionter_End(ArmTop_z_Set_num, float);
}

/**
 * @brief 机械臂姿态数据读取
 * @param out
 */
void ArmPosStateNowRead(ArmPos_Remote_s *out)
{
    float *x_address;
    float *y_address;
    float *z_address;
    float *yaw_address;
    float *pitch_address;
    float *roll_address;

    x_address = Package_Pionter_Single(ArmTop_x_Now_num, float);
    y_address = Package_Pionter_Single(ArmTop_y_Now_num, float);
    z_address = Package_Pionter_Single(ArmTop_z_Now_num, float);
    yaw_address = Package_Pionter_Single(ForearmYaw_AngleFilter_num, float);
    pitch_address = Package_Pionter_Single(ForearmPitch_AngleFilter_num, float);
    roll_address = Package_Pionter_Single(ForearmRoll_AngleFilter_num, float);

    out->Pos.x = *x_address;
    out->Pos.y = *y_address;
    out->Pos.z = *z_address;
    out->Angle.ForearmYaw = *yaw_address;
    out->Angle.ForearmPitch = *pitch_address;
    out->Angle.ForearmRoll = *roll_address;

    Package_Write_Pionter_End(ArmTop_x_Now_num, float);
    Package_Write_Pionter_End(ArmTop_y_Now_num, float);
    Package_Write_Pionter_End(ArmTop_z_Now_num, float);
    Package_Write_Pionter_End(ForearmYaw_AngleFilter_num, float);
    Package_Write_Pionter_End(ForearmPitch_AngleFilter_num, float);
    Package_Write_Pionter_End(ForearmRoll_AngleFilter_num, float);
}

/**
 * @brief 小机械臂Yaw姿态绝对式设定
 * @param in
 */
void ForearmYawSet_ABS(float *in)
{
    float *yaw_address;

    yaw_address = Package_Pionter_Single(ForearmYaw_AngleHope_num, float);
    *yaw_address = *in;
    Package_Write_Pionter_End(ForearmYaw_AngleHope_num, float);
}

/**
 * @brief 机械臂姿态绝对式设定
 * @param in
 */
void ForearmStateSet_Read(ForearmMotor_s *Forearm_out)
{
    float *yaw_address;
    float *pitch_address;
    float *roll_address;

    yaw_address = Package_Pionter_Single(ForearmYaw_AngleHope_num, float);
    pitch_address = Package_Pionter_Single(ForearmPitch_AngleHope_num, float);
    roll_address = Package_Pionter_Single(ForearmRoll_AngleHope_num, float);

    Forearm_out->ForearmYaw = *yaw_address;
    Forearm_out->ForearmPitch = *pitch_address;
    Forearm_out->ForearmRoll = *roll_address;

    Package_Write_Pionter_End(ForearmYaw_AngleHope_num, float);
    Package_Write_Pionter_End(ForearmPitch_AngleHope_num, float);
    Package_Write_Pionter_End(ForearmRoll_AngleHope_num, float);
}

/**
 * @brief 机械臂姿态增量式设定
 * @param in
 */
void ArmPosStateSet_ADD(ArmPos_Remote_s *in)
{
    float *x_address;
    float *y_address;
    float *z_address;
    float *yaw_address;
    float *pitch_address;
    float *roll_address;
    float *imagepitch_address;

    x_address = Package_Pionter_Single(ArmTop_x_Set_num, float);
    y_address = Package_Pionter_Single(ArmTop_y_Set_num, float);
    z_address = Package_Pionter_Single(ArmTop_z_Set_num, float);
    yaw_address = Package_Pionter_Single(ForearmYaw_AngleHope_num, float);
    pitch_address = Package_Pionter_Single(ForearmPitch_AngleHope_num, float);
    roll_address = Package_Pionter_Single(ForearmRoll_AngleHope_num, float);
    imagepitch_address = Package_Pionter_Single(ImagePitch_AngleHope_num, float);

    *x_address += in->Pos.x;
    *y_address += in->Pos.y;
    *z_address += in->Pos.z;
    *yaw_address += in->Angle.ForearmYaw;
    *pitch_address += in->Angle.ForearmPitch;
    *roll_address += in->Angle.ForearmRoll;
    *imagepitch_address += in->ImagePitch;

    Package_Write_Pionter_End(ArmTop_x_Set_num, float);
    Package_Write_Pionter_End(ArmTop_y_Set_num, float);
    Package_Write_Pionter_End(ArmTop_z_Set_num, float);
    Package_Write_Pionter_End(ForearmYaw_AngleHope_num, float);
    Package_Write_Pionter_End(ForearmPitch_AngleHope_num, float);
    Package_Write_Pionter_End(ForearmRoll_AngleHope_num, float);
    Package_Write_Pionter_End(ImagePitch_AngleHope_num, float);
}

/**
 * @brief 向数据服务器写入图传云台pitch角度设定值
 * @param in
 */
void ImageGimbalPitchSet_Write(float in)
{
    float *imagepitch_address;
    imagepitch_address = Package_Pionter_Single(ImagePitch_AngleHope_num, float);
    *imagepitch_address = in;
    Package_Write_Pionter_End(ImagePitch_AngleHope_num, float);
}

/**
 * @brief 机械臂姿态绝对式设定
 * @param in
 */
void ArmPosStateSet_ABS(ArmPos_Remote_s *in)
{
    float *x_address;
    float *y_address;
    float *z_address;
    float *yaw_address;
    float *pitch_address;
    float *roll_address;

    x_address = Package_Pionter_Single(ArmTop_x_Set_num, float);
    y_address = Package_Pionter_Single(ArmTop_y_Set_num, float);
    z_address = Package_Pionter_Single(ArmTop_z_Set_num, float);
    yaw_address = Package_Pionter_Single(ForearmYaw_AngleHope_num, float);
    pitch_address = Package_Pionter_Single(ForearmPitch_AngleHope_num, float);
    roll_address = Package_Pionter_Single(ForearmRoll_AngleHope_num, float);

    *x_address = in->Pos.x;
    *y_address = in->Pos.y;
    *z_address = in->Pos.z;
    *yaw_address = in->Angle.ForearmYaw;
    *pitch_address = in->Angle.ForearmPitch;
    *roll_address = in->Angle.ForearmRoll;

    Package_Write_Pionter_End(ArmTop_x_Set_num, float);
    Package_Write_Pionter_End(ArmTop_y_Set_num, float);
    Package_Write_Pionter_End(ArmTop_z_Set_num, float);
    Package_Write_Pionter_End(ForearmYaw_AngleHope_num, float);
    Package_Write_Pionter_End(ForearmPitch_AngleHope_num, float);
    Package_Write_Pionter_End(ForearmRoll_AngleHope_num, float);
}

/*******************************底盘数据*****************************/
/**
 * @brief 底盘运动速度绝对式设定
 * @param in
 */
void ChassisSpeStateSet_ABS(ChassisMotor_t *in)
{
    float *address;

    address = Package_Pionter_Single(ChassisSpe_RF_Set_num, float);
    *address = in->speed[WHEEL_RF];
    Package_Write_Pionter_End(ChassisSpe_RF_Set_num, float);

    address = Package_Pionter_Single(ChassisSpe_RB_Set_num, float);
    *address = in->speed[WHEEL_RB];
    Package_Write_Pionter_End(ChassisSpe_RB_Set_num, float);

    address = Package_Pionter_Single(ChassisSpe_LF_Set_num, float);
    *address = in->speed[WHEEL_LF];
    Package_Write_Pionter_End(ChassisSpe_LF_Set_num, float);

    address = Package_Pionter_Single(ChassisSpe_LB_Set_num, float);
    *address = in->speed[WHEEL_LB];
    Package_Write_Pionter_End(ChassisSpe_LB_Set_num, float);
}

/**
 * @brief   从数据服务器读取轮子的速度设定值
 * @param   out
 */
void Chassis_SetSpeed_Read(ChassisMotor_t *out)
{
    float *address;

    address = Package_Pionter_Single(ChassisSpe_RF_Set_num, float);
    out->speed[WHEEL_RF] = *address;
    Package_Write_Pionter_End(ChassisSpe_RF_Set_num, float);

    address = Package_Pionter_Single(ChassisSpe_RB_Set_num, float);
    out->speed[WHEEL_RB] = *address;
    Package_Write_Pionter_End(ChassisSpe_RB_Set_num, float);

    address = Package_Pionter_Single(ChassisSpe_LB_Set_num, float);
    out->speed[WHEEL_LB] = *address;
    Package_Write_Pionter_End(ChassisSpe_LB_Set_num, float);

    address = Package_Pionter_Single(ChassisSpe_LF_Set_num, float);
    out->speed[WHEEL_LF] = *address;
    Package_Write_Pionter_End(ChassisSpe_LF_Set_num, float);
}

/**
 * @brief   从数据服务器读取轮子的速度当前值
 * @param   out
 */
void Chassis_NowSpeed_Read(ChassisMotor_t *out)
{
    float *address;

    address = Package_Pionter_Single(ChassisSpe_RF_Now_num, float);
    out->speed[WHEEL_RF] = *address;
    Package_Write_Pionter_End(ChassisSpe_RF_Now_num, float);

    address = Package_Pionter_Single(ChassisSpe_RB_Now_num, float);
    out->speed[WHEEL_RB] = *address;
    Package_Write_Pionter_End(ChassisSpe_RB_Now_num, float);

    address = Package_Pionter_Single(ChassisSpe_LB_Now_num, float);
    out->speed[WHEEL_LB] = *address;
    Package_Write_Pionter_End(ChassisSpe_LB_Now_num, float);

    address = Package_Pionter_Single(ChassisSpe_LF_Now_num, float);
    out->speed[WHEEL_LF] = *address;
    Package_Write_Pionter_End(ChassisSpe_LF_Now_num, float);
}

/**
 * @brief 向数据服务器设定速度增益
 * @param speedgain
 */
void Robo_SpeedGain_Set(float speedgain)
{
    float *speedgain_address;

    speedgain_address = Package_Pionter_Single(Robo_SpeedGain_num, float);

    *speedgain_address = speedgain;

    Package_Write_Pionter_End(Robo_SpeedGain_num, float);
}

/**
 * @brief 从数据服务器读取速度增益
 * @return speedgain 速度增益
 */
float Robo_SpeedGain_Get(void)
{
    float speedgain;
    float *speedgain_address;

    speedgain_address = Package_Pionter_Single(Robo_SpeedGain_num, float);

    speedgain = *speedgain_address;

    Package_Write_Pionter_End(Robo_SpeedGain_num, float);
    return speedgain;
}

/**************************************自定义控制器数据*****************************************/

/**
 * @brief 从数据服务器读取自定义控制器数据
 */
void CustCtrler_Data_Read(CustCtrler_Data_s *out)
{
    float *address;

    address = Package_Pionter_Single(CustCtrler_Limit_num, float);
    out->Limit = *address;
    Package_Write_Pionter_End(CustCtrler_Limit_num, float);

    address = Package_Pionter_Single(CustCtrler_x_num, float);
    out->pos.x = *address;
    Package_Write_Pionter_End(CustCtrler_x_num, float);

    address = Package_Pionter_Single(CustCtrler_y_num, float);
    out->pos.y = *address;
    Package_Write_Pionter_End(CustCtrler_y_num, float);

    address = Package_Pionter_Single(CustCtrler_z_num, float);
    out->pos.z = *address;
    Package_Write_Pionter_End(CustCtrler_z_num, float);

    address = Package_Pionter_Single(CustCtrler_yaw_num, float);
    out->angle.yaw = *address;
    Package_Write_Pionter_End(CustCtrler_yaw_num, float);

    address = Package_Pionter_Single(CustCtrler_pitch_num, float);
    out->angle.pitch = *address;
    Package_Write_Pionter_End(CustCtrler_pitch_num, float);

    address = Package_Pionter_Single(CustCtrler_roll_num, float);
    out->angle.roll = *address;
    Package_Write_Pionter_End(CustCtrler_roll_num, float);
}

/**
 * @brief 向数据服务器写自定义控制器数据
 */
void CustCtrler_Data_Write(CustCtrler_Data_s *in)
{
    float *address;

    address = Package_Pionter_Single(CustCtrler_Limit_num, float);
    *address = in->Limit;
    Package_Write_Pionter_End(CustCtrler_Limit_num, float);

    address = Package_Pionter_Single(CustCtrler_x_num, float);
    *address = in->pos.x;
    Package_Write_Pionter_End(CustCtrler_x_num, float);

    address = Package_Pionter_Single(CustCtrler_y_num, float);
    *address = in->pos.y;
    Package_Write_Pionter_End(CustCtrler_y_num, float);

    address = Package_Pionter_Single(CustCtrler_z_num, float);
    *address = in->pos.z;
    Package_Write_Pionter_End(CustCtrler_z_num, float);

    address = Package_Pionter_Single(CustCtrler_yaw_num, float);
    *address = in->angle.yaw;
    Package_Write_Pionter_End(CustCtrler_yaw_num, float);

    address = Package_Pionter_Single(CustCtrler_pitch_num, float);
    *address = in->angle.pitch;
    Package_Write_Pionter_End(CustCtrler_pitch_num, float);

    address = Package_Pionter_Single(CustCtrler_roll_num, float);
    *address = in->angle.roll;
    Package_Write_Pionter_End(CustCtrler_roll_num, float);
}

/**
 * @brief 从数据服务器读取自定义控制器按键数据
 */
void CustCtrlerKey_Data_Read(uint8_t *out)
{
    float *address;

    address = Package_Pionter_Single(CustCtrler_Limit_num, float);
    *out = *address;
    Package_Write_Pionter_End(CustCtrler_Limit_num, float);
}

/**
 * @brief 从数据服务器读取机械臂超限数据
 */
void Arm_IfLimit_Data_Read(uint8_t *out)
{
    float *address;

    address = Package_Pionter_Single(Arm_IfLimit_num, float);
    *out = *address;
    Package_Write_Pionter_End(Arm_IfLimit_num, float);
}

/**
 * @brief 向数据服务器写入机械臂超限数据
 */
void Arm_IfLimit_Data_Write(uint8_t *in)
{
    float *address;

    address = Package_Pionter_Single(Arm_IfLimit_num, float);
    *address = *in;
    Package_Write_Pionter_End(Arm_IfLimit_num, float);
}

/**
 * @brief 从数据服务器读取UI复位数据
 */
void UIReset_State_Data_Read(uint8_t *out)
{
    float *address;

    address = Package_Pionter_Single(UIReset_State_num, float);
    *out = *address;
    Package_Write_Pionter_End(UIReset_State_num, float);
}

/**
 * @brief 向数据服务器写入UI复位数据
 */
void UIReset_State_Data_Write(uint8_t in)
{
    float *address;

    address = Package_Pionter_Single(UIReset_State_num, float);
    *address = in;
    Package_Write_Pionter_End(UIReset_State_num, float);
}

/**
 * @brief   从数据服务器读取图传电机的速度当前值
 * @param   out
 */
void ImageMotor_NowSpeed_Read(ImageMotor_s *Image_out)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_SpeedNow_num, float);
    Image_out->ImageYaw = *address;
    Package_Write_Pionter_End(ImageYaw_SpeedNow_num, float);

    address = Package_Pionter_Single(ImagePitch_SpeedNow_num, float);
    Image_out->ImagePitch = *address;
    Package_Write_Pionter_End(ImagePitch_SpeedNow_num, float);
}

/**
 * @brief   从数据服务器读取图传电机的角度当前值
 * @param   out
 */
void ImageMotor_NowAngle_Read(ImageMotor_s *Image_out)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_AngleNow_num, float);
    Image_out->ImageYaw = *address;
    Package_Write_Pionter_End(ImageYaw_AngleNow_num, float);

    address = Package_Pionter_Single(ImagePitch_AngleNow_num, float);
    Image_out->ImagePitch = *address;
    Package_Write_Pionter_End(ImagePitch_AngleNow_num, float);
}

/**
 * @brief   从数据服务器读取图传电机的角度期望值
 * @param   out
 */
void ImageMotor_AngleHope_Read(ImageMotor_s *Image_out)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_AngleHope_num, float);
    Image_out->ImageYaw = *address;
    Package_Write_Pionter_End(ImageYaw_AngleHope_num, float);

    address = Package_Pionter_Single(ImagePitch_AngleHope_num, float);
    Image_out->ImagePitch = *address;
    Package_Write_Pionter_End(ImagePitch_AngleHope_num, float);
}

/**
 * @brief 向数据服务器写自定义控制器数据
 */
void ImageMotor_AngleHope_Write(ImageMotor_s *in)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_AngleHope_num, float);
    *address = in->ImageYaw;
    Package_Write_Pionter_End(ImageYaw_AngleHope_num, float);

    address = Package_Pionter_Single(ImagePitch_AngleHope_num, float);
    *address = in->ImagePitch;
    Package_Write_Pionter_End(ImagePitch_AngleHope_num, float);
}

/**
 * @brief 向数据服务器写图传Yaw角度设定值
 */
void ImageMotor_YawAngleHope_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_AngleHope_num, float);
    *address = in;
    Package_Write_Pionter_End(ImageYaw_AngleHope_num, float);
}

/**
 * @brief 从数据服务器读取图传位置状态
 */
void Image_PosState_Read(uint8_t *out)
{
    float *address;

    address = Package_Pionter_Single(Image_PosState_num, float);
    *out = *address;
    Package_Write_Pionter_End(Image_PosState_num, float);
}

/**
 * @brief 向数据服务器写入图传位置状态
 */
void Image_PosState_Write(uint8_t in)
{
    float *address;

    address = Package_Pionter_Single(Image_PosState_num, float);
    *address = in;
    Package_Write_Pionter_End(Image_PosState_num, float);
}

/**
 * @brief 从数据服务器读取机械臂大yaw编码器角度数据
 */
void BoomYaw_CoderRad_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_CoderAngleOrigin_num, float);
    *out = *address;
    Package_Write_Pionter_End(BoomYaw_CoderAngleOrigin_num, float);
}

/**
 * @brief 向数据服务器写入机械臂大yaw编码器角度数据
 */
void BoomYaw_CoderAngle_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_CoderAngleOrigin_num, float);
    *address = in;
    Package_Write_Pionter_End(BoomYaw_CoderAngleOrigin_num, float);
}

/**
 * @brief 从数据服务器读取图传yaw编码器角度数据
 */
void ImageYaw_CoderAngle_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_CoderAngle_num, float);
    *out = *address;
    Package_Write_Pionter_End(ImageYaw_CoderAngle_num, float);
}

/**
 * @brief 向数据服务器写入图传yaw编码器角度数据
 */
void ImageYaw_CoderAngle_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(ImageYaw_CoderAngle_num, float);
    *address = in;
    Package_Write_Pionter_End(ImageYaw_CoderAngle_num, float);
}

/**
 * @brief 从数据服务器读取图传云台角度数据
 */
void ImageGimbal_Angle_Read(ImageMotor_s *out)
{
    float *address;

    address = Package_Pionter_Single(ImageGimbal_YawAngle_num, float);
    out->ImageYaw = *address;
    Package_Write_Pionter_End(ImageGimbal_YawAngle_num, float);

    address = Package_Pionter_Single(ImageGimbal_PitchAngle_num, float);
    out->ImagePitch = *address;
    Package_Write_Pionter_End(ImageGimbal_PitchAngle_num, float);
}

/**
 * @brief 向数据服务器写入图传云台角度数据
 */
void ImageGimbal_Angle_Write(ImageMotor_s *in)
{
    float *address;

    address = Package_Pionter_Single(ImageGimbal_YawAngle_num, float);
    *address = in->ImageYaw;
    Package_Write_Pionter_End(ImageGimbal_YawAngle_num, float);

    address = Package_Pionter_Single(ImageGimbal_PitchAngle_num, float);
    *address = in->ImagePitch;
    Package_Write_Pionter_End(ImageGimbal_PitchAngle_num, float);
}

/**
 * @brief 从数据服务器读取大机械臂yaw编码器速度滤波数据
 */
void BoomYaw_SpeedFilter_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_SpeedFilter_num, float);
    *out = *address;
    Package_Write_Pionter_End(BoomYaw_SpeedFilter_num, float);
}

/**
 * @brief 向数据服务器写入大机械臂yaw编码器速度滤波数据
 */
void BoomYaw_SpeedFilter_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_SpeedFilter_num, float);
    *address = in;
    Package_Write_Pionter_End(BoomYaw_SpeedFilter_num, float);
}

/**
 * @brief 从数据服务器读取陀螺仪yaw角度
 */
void GyroYaw_AngleNow_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(Gyro_YawAngle_num, float);
    *out = *address;
    Package_Write_Pionter_End(Gyro_YawAngle_num, float);
}

/**
 * @brief 从数据服务器读取陀螺仪yaw角速度
 */
void GyroYaw_SpeedNow_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(Gyro_YawSpeed_num, float);
    *out = *address;
    Package_Write_Pionter_End(Gyro_YawSpeed_num, float);
}

/**
 * @brief 从数据服务器读取陀螺仪yaw角度弧度值
 */
void GyroYaw_radNow_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(Gyro_YawRadFilter_num, float);
    *out = *address;
    Package_Write_Pionter_End(Gyro_YawRadFilter_num, float);
}

/**
 * @brief 从数据服务器读取陀螺仪yaw角速度值
 */
void GyroYaw_radNow_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(Gyro_YawRadFilter_num, float);
    *address = in;
    Package_Write_Pionter_End(Gyro_YawRadFilter_num, float);
}

/**
 * @brief 从数据服务器读取编码器标定状态位
 */
void EncoderCali_State_Read(uint8_t *out)
{
    float *address;

    address = Package_Pionter_Single(EncoderCali_State_num, float);
    *out = *address;
    Package_Write_Pionter_End(EncoderCali_State_num, float);
}

/**
 * @brief 从数据服务器写入编码器标定状态位
 */
void EncoderCali_State_Write(uint8_t in)
{
    float *address;

    address = Package_Pionter_Single(EncoderCali_State_num, float);
    *address = in;
    Package_Write_Pionter_End(EncoderCali_State_num, float);
}

/**
 * @brief 从数据服务器读取当前云台角度
 */
void GimbalYawRad_Now_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(GimbalYawRad_Now_num, float);
    *out = *address;
    Package_Write_Pionter_End(GimbalYawRad_Now_num, float);
}

/**
 * @brief 向数据服务器写入当前云台角度
 */
void GimbalYawRad_Now_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(GimbalYawRad_Now_num, float);
    *address = in;
    Package_Write_Pionter_End(GimbalYawRad_Now_num, float);
}

/**
 * @brief 从数据服务器读取当前底盘角度
 */
void ChassisYawRad_Now_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(ChassisYawRad_Now_num, float);
    *out = *address;
    Package_Write_Pionter_End(ChassisYawRad_Now_num, float);
}

/**
 * @brief 向数据服务器写入当前底盘角度
 */
void ChassisYawRad_Now_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(ChassisYawRad_Now_num, float);
    *address = in;
    Package_Write_Pionter_End(ChassisYawRad_Now_num, float);
}

/**
 * @brief 从数据服务器读取当前底盘转速
 */
void ChassisYawSpeed_Now_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(ChassisYawSpeed_Now_num, float);
    *out = *address;
    Package_Write_Pionter_End(ChassisYawSpeed_Now_num, float);
}

/**
 * @brief 向数据服务器写入当前底盘转速
 */
void ChassisYawSpeed_Now_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(ChassisYawSpeed_Now_num, float);
    *address = in;
    Package_Write_Pionter_End(ChassisYawSpeed_Now_num, float);
}

/**
 * @brief 从数据服务器读取设定云台角度
 */
void GimbalYawRad_Set_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(GimbalYawRad_Set_num, float);
    *out = *address;
    Package_Write_Pionter_End(GimbalYawRad_Set_num, float);
}

/**
 * @brief 向数据服务器写入设定云台角度
 */
void GimbalYawRad_Set_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(GimbalYawRad_Set_num, float);
    *address = in;
    Package_Write_Pionter_End(GimbalYawRad_Set_num, float);
}

/**
 * @brief 向数据服务器增量式写入设定云台角度
 */
void GimbalYawRad_Set_ADD(float in)
{
    float *address;

    address = Package_Pionter_Single(GimbalYawRad_Set_num, float);
    *address += in;
    Package_Write_Pionter_End(GimbalYawRad_Set_num, float);
}

/**
 * @brief 从数据服务器读取大机械臂编码器校正数据
 */
void BoomYaw_CoderCaliRad_Read(float *out)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_CoderCaliRad_num, float);
    *out = *address;
    Package_Write_Pionter_End(BoomYaw_CoderCaliRad_num, float);
}

/**
 * @brief 向数据服务器写入大机械臂编码器校正数据
 */
void BoomYaw_CoderCaliRad_Write(float in)
{
    float *address;

    address = Package_Pionter_Single(BoomYaw_CoderCaliRad_num, float);
    *address = in;
    Package_Write_Pionter_End(BoomYaw_CoderCaliRad_num, float);
}

/**
 * @brief  工程机器人控制线程数据包序列号读取
 */
int EngineerRobo_Datanum_Find(void)
{
    ForearmYaw_AngleNow_num = Package_Find_Num("F_Yaw_AngleNow");
    ForearmPitch_AngleNow_num = Package_Find_Num("F_Pitch_AngleNow");
    ForearmRoll_AngleNow_num = Package_Find_Num("F_Roll_AngleNow");

    ForearmYaw_SpeedNow_num = Package_Find_Num("F_Yaw_SpeedNow");
    ForearmPitch_SpeedNow_num = Package_Find_Num("F_Pitch_SpeedNow");
    ForearmRoll_SpeedNow_num = Package_Find_Num("F_Roll_SpeedNow");

    ForearmYaw_AngleHope_num = Package_Find_Num("F_Yaw_AngleHope");
    ForearmPitch_AngleHope_num = Package_Find_Num("F_Pitch_AngleHope");
    ForearmRoll_AngleHope_num = Package_Find_Num("F_Roll_AngleHope");

    BoomYaw_AngleNow_num = Package_Find_Num("B_Yaw_AngleNow");
    BoomPitch1_AngleNow_num = Package_Find_Num("B_Pitch1_AngleNow");
    BoomPitch2_AngleNow_num = Package_Find_Num("B_Pitch2_AngleNow");

    BoomYaw_SpeedNow_num = Package_Find_Num("B_Yaw_SpeedNow");
    BoomPitch1_SpeedNow_num = Package_Find_Num("B_Pitch1_SpeedNow");
    BoomPitch2_SpeedNow_num = Package_Find_Num("B_Pitch2_SpeedNow");
    BoomYaw_SpeedFilter_num = Package_Find_Num("B_Yaw_SpeFilterNow");

    ArmTop_x_Now_num = Package_Find_Num("ArmTop_x_Now");
    ArmTop_y_Now_num = Package_Find_Num("ArmTop_y_Now");
    ArmTop_z_Now_num = Package_Find_Num("ArmTop_z_Now");

    ArmTop_x_Set_num = Package_Find_Num("ArmTop_x_Set");
    ArmTop_y_Set_num = Package_Find_Num("ArmTop_y_Set");
    ArmTop_z_Set_num = Package_Find_Num("ArmTop_z_Set");

    BoomYaw_AngleCircle_num = Package_Find_Num("B_Yaw_rad_now");
    BoomPitch1_AngleFilter_num = Package_Find_Num("B_Pitch1_rad_now");
    BoomPitch2_AngleFilter_num = Package_Find_Num("B_Pitch2_rad_now");

    ForearmYaw_AngleFilter_num = Package_Find_Num("F_Yaw_rad_now");
    ForearmPitch_AngleFilter_num = Package_Find_Num("F_Pitch_rad_now");
    ForearmRoll_AngleFilter_num = Package_Find_Num("F_Roll_rad_now");

    ChassisSpe_RF_Set_num = Package_Find_Num("ChassisSpeSet_RF");
    ChassisSpe_RB_Set_num = Package_Find_Num("ChassisSpeSet_RB");
    ChassisSpe_LF_Set_num = Package_Find_Num("ChassisSpeSet_LF");
    ChassisSpe_LB_Set_num = Package_Find_Num("ChassisSpeSet_LB");

    ChassisSpe_RF_Now_num = Package_Find_Num("ChassisSpeNow_RF");
    ChassisSpe_RB_Now_num = Package_Find_Num("ChassisSpeNow_RB");
    ChassisSpe_LF_Now_num = Package_Find_Num("ChassisSpeNow_LF");
    ChassisSpe_LB_Now_num = Package_Find_Num("ChassisSpeNow_LB");

    Robo_SpeedGain_num = Package_Find_Num("Robo_SpeedGain");

    /*自定义控制器数据系列号读取*/
    CustCtrler_Limit_num = Package_Find_Num("CustCtrler_Limit");

    CustCtrler_x_num = Package_Find_Num("CustCtrler_x");
    CustCtrler_y_num = Package_Find_Num("CustCtrler_y");
    CustCtrler_z_num = Package_Find_Num("CustCtrler_z");

    CustCtrler_yaw_num = Package_Find_Num("CustCtrler_yaw");
    CustCtrler_pitch_num = Package_Find_Num("CustCtrler_pitch");
    CustCtrler_roll_num = Package_Find_Num("CustCtrler_roll");

    /*图传云台数据序列号*/
    ImageYaw_AngleNow_num = Package_Find_Num("I_Yaw_AngleNow");
    ImagePitch_AngleNow_num = Package_Find_Num("I_Pitch_AngleNow");

    ImageYaw_AngleHope_num = Package_Find_Num("I_Yaw_AngleHope");
    ImagePitch_AngleHope_num = Package_Find_Num("I_Pitch_AngleHope");

    ImageYaw_SpeedNow_num = Package_Find_Num("I_Yaw_SpeedNow");
    ImagePitch_SpeedNow_num = Package_Find_Num("I_Pitch_SpeedNow");

    /*图传位置状态*/
    Image_PosState_num = Package_Find_Num("Image_PosState");

    /*编码器yaw轴数据*/
    BoomYaw_CoderAngleOrigin_num = Package_Find_Num("BYaw_CoderOrigin");
    BoomYaw_CoderCaliRad_num = Package_Find_Num("BYaw_CoderCaliRad");
    ImageYaw_CoderAngle_num = Package_Find_Num("IYaw_CoderAngle");

    ImageGimbal_YawAngle_num = Package_Find_Num("IGim_Yaw_Angle");
    ImageGimbal_PitchAngle_num = Package_Find_Num("IGim_Pitch_Angle");

    /*陀螺仪Yaw角度和角速度*/
    Gyro_YawAngle_num = Package_Find_Num("Gyro_YawAngle");
    Gyro_YawSpeed_num = Package_Find_Num("Gyro_YawSpeed");
    Gyro_YawRadFilter_num = Package_Find_Num("Gyro_YawRadFilter");

    /*机械臂超限*/
    Arm_IfLimit_num = Package_Find_Num("Arm_IfLimit");

    /*UI复位数据*/
    UIReset_State_num = Package_Find_Num("UIReset_State");

    /*编码器标定状态位*/
    EncoderCali_State_num = Package_Find_Num("EncoderCali_State");

    /*云台系与底盘系角度及云台期望角度*/
    GimbalYawRad_Now_num = Package_Find_Num("GimbalYawRad_Now");
    GimbalYawRad_Set_num = Package_Find_Num("GimbalYawRad_Set");
    ChassisYawRad_Now_num = Package_Find_Num("ChassisYawRad_Now");
    ChassisYawSpeed_Now_num = Package_Find_Num("ChassisYawSpe_Now");

    return 0;
}
