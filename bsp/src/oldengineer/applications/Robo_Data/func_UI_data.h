#ifndef __FUNC_UI_DATA_H__
#define __FUNC_UI_DATA_H__

#include <rtdevice.h>

typedef enum
{
    BoomYaw = 0,  // 大机械臂Yaw
    BoomPitch1,   // 大机械臂Pitch1
    BoomPitch2,   // 大机械臂Pitch2
    BoomPitch,    // 大机械臂两个pitch的夹角
    ForearmYaw,   // 小机械臂Yaw
    ForearmPitch, // 小机械臂pitch
    ForearmRoll,  // 小机械臂roll
    ImageTrans,   // 图传电机
} Arm_e;

/**
 * @brief 从数据服务器中获取UI相关数据
 */
void Get_UIData_fromServer(void);

/**
 * @brief  获取机械臂关节弧度信息
 * @return rt_uint8_t
 */
float GetState_Armrad(Arm_e in);

/**
 * @brief  获取机械臂关节超限信息
 * @return rt_uint8_t
 */
rt_uint8_t GetState_ArmIfLimit(Arm_e in);

/**
 * @brief  获取气泵开关信息
 * @return rt_uint8_t
 */
rt_uint8_t GetState_AirPumpIfOpen(void);

/**
 * @brief  获取机器人遥控控制状态
 * @return rt_uint8_t
 */
rt_uint8_t GetState_RoboRCCtrl(void);

/**
 * @brief  获取机器人UI复位状态
 * @return rt_uint8_t
 */
rt_uint8_t GetState_UIReset(void);

/**
 * @brief  获取机器人矿石模式
 * @return rt_uint8_t
 */
rt_uint8_t GetState_OreState(void);

#endif /*__FUNC_UI_DATA_H__*/
