#include "func_UI_data.h"
#include "drv_utils.h"
#include "drv_Arm_Solve.h"
#include "drv_RemoteCtrl_data.h"
#include "func_Dataserver.h"

BoomMotor_s Boom_radvalue;
ForearmMotor_s Forearm_radvalue;
Arm_Limit_s Arm_Limit_data;
float mid_pitch;
float mid_roll;
ImageMotor_s image_data_value;
/**
 * @brief 从数据服务器中获取UI相关数据
 */
void Get_UIData_fromServer(void)
{
    ArmMotor_RadFilter_Read(&Boom_radvalue,
                            &Forearm_radvalue);
    mid_pitch = Forearm_radvalue.ForearmPitch;
    mid_roll = Forearm_radvalue.ForearmRoll;
    Forearm_radvalue.ForearmPitch = (mid_roll-mid_pitch)/2.0f;
    Forearm_radvalue.ForearmRoll = (mid_roll+mid_pitch);
    Arm_IfLimit_Data_Read((uint8_t *)&Arm_Limit_data);
	  ImageMotor_AngleHope_Read(&image_data_value);
}

/*******************************获取关节弧度******************************/


/**
 * @brief  获取机械臂关节弧度信息
 * @return rt_uint8_t
 */
float GetState_Armrad(Arm_e in)
{
    switch (in)
    {
    case BoomYaw:
        return Boom_radvalue.BoomYaw-(PI/2.0f);
    case BoomPitch1:
        return Boom_radvalue.BoomPitch1-(45.0f/180.0f*PI);
    case BoomPitch2:
        return Boom_radvalue.BoomPitch2;
    case ForearmYaw:
        return Forearm_radvalue.ForearmYaw - (PI / 2.0f);
    case ForearmPitch:
        return Forearm_radvalue.ForearmPitch;
    case ForearmRoll:
        return Forearm_radvalue.ForearmRoll;
    case BoomMove:
        return Boom_radvalue.BoomMove;
    default:
        return 0;
    }
}
/**
* @brief  获取图传电机位置信息
 * @return rt_uint8_t
 */
float Get_Image_Data(IMAGE_E in)
{
    switch (in)
    {
    case image_high:
        return image_data_value.ImageHigh;
    case image_pitch:
        return image_data_value.ImagePitch;
    default:
        return 0;
    }
}
/**
 * @brief  获取机械臂关节超限信息
 * @return rt_uint8_t
 */
rt_uint8_t GetState_ArmIfLimit(Arm_e in)
{
    switch (in)
    {
    case BoomYaw:
        return Arm_Limit_data.BoomYaw_Iflimit;
    case BoomPitch1:
        return Arm_Limit_data.BoomPitch1_Iflimit;
    case BoomPitch2:
        return Arm_Limit_data.BoomPitch2_Iflimit;
    case BoomPitch:
        return Arm_Limit_data.BoomPitch_Iflimit;
    case ForearmYaw:
        return Arm_Limit_data.ForearmYaw_Iflimit;
    case ForearmPitch:
        return Arm_Limit_data.ForearmPitch_Iflimit;
    case ForearmRoll:
        return Arm_Limit_data.ForearmRoll_Iflimit;
    case BoomMove:
        return Arm_Limit_data.BoomMove_Iflimit;
    default:
        return 0;
    }
}

/**
 * @brief  获取气泵开关信息
 * @return rt_uint8_t
 */
rt_uint8_t GetState_AirPumpIfOpen(void)
{
    return (rt_uint8_t)(uint32_t)Robo_Control(AirPump_State, Read, NULL);
}

/**
 * @brief  获取机器人遥控控制状态
 * @return rt_uint8_t
 */
rt_uint8_t GetState_RoboRCCtrl(void)
{
    return (rt_uint8_t)(uint32_t)Robo_Control(RoboCtrl_RC_State, Read, NULL);
}

/**
 * @brief  获取机器人UI复位状态
 * @return rt_uint8_t
 */
rt_uint8_t GetState_UIReset(void)
{
    rt_uint8_t UIReset_State;
    UIReset_State_Data_Read(&UIReset_State);

    return (rt_uint8_t)UIReset_State;
}

/**
 * @brief  获取机器人矿石模式
 * @return rt_uint8_t
 */
rt_uint8_t GetState_OreState(void)
{
    return (rt_uint8_t)(uint32_t)Robo_Control(Ore_State, Read, NULL);
}
