#include "fun_protocolRef.h"

extern DJI_Data_t DJI_ReadData;

rt_uint8_t gim_hurt = 0;
rt_uint16_t REF_ROBO_ID;   // 该机器人ID
rt_uint16_t REF_CLIENT_ID; // 该客户端ID
static Ref_Hurt_t hurt =
    {
        .robot_if_hurt = 0,
        .build_if_attacked = 0,
        .sentry_hp = 0,
        .outpost_hp = 0,
        .base_hp = 0};

/**
 * @brief    枪口热量冷却加速
 *		  	发送频率： 1Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   剩余弹量
 */
rt_uint8_t Ref_Power_Rune_Buff(void)
{
    return (rt_uint8_t)(DJI_ReadData.ext_buff_musk.power_rune_buff >> 1 & 0x01);
}

/**
 * @brief    获取剩余弹量
 *		  	发送频率： 10Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   剩余弹量
 */
rt_uint16_t Ref_AmmoRemain(void)
{
#ifdef CORE_USING_HERO
    return DJI_ReadData.ext_bullet_remaining.bullet_remaining_num_42mm;
#else
    return DJI_ReadData.ext_bullet_remaining.bullet_remaining_num_17mm;
#endif
}

/******************************************************************************************************/
/**
 * @brief    获取上一发子弹射速,发送频率：射击后发送
 * @param    None
 * @return   单位 cm/s
 */
rt_uint16_t Ref_Bullet_Speed(void)
{
    return (rt_uint16_t)(DJI_ReadData.ext_shoot_data.bullet_speed * 100);
}

/**
 * @brief    获取枪口上限速度
 * @param    None
 * @return   单位 cm/s
 */
rt_uint16_t Ref_Bullet_Speed_Limit(void)
{
#ifdef CORE_USING_HERO
    return DJI_ReadData.ext_game_robot_state.shooter_id1_42mm_speed_limit * 100;
#else
    return DJI_ReadData.ext_game_robot_state.shooter_id1_17mm_speed_limit * 100;
#endif
}

/**
 * @brief    获取弹速档位
 * @param    None
 * @return   Ref_bullet_speed_mode_e
 */
rt_uint8_t Ref_Bullet_Speed_Mode(void)
{
#ifdef CORE_USING_HERO
    switch (DJI_ReadData.ext_game_robot_state.shooter_id1_42mm_speed_limit)
    {
    case HERO_BS_1:
        return LOW_SPEED;
    case HERO_BS_2:
        return MID_SPEED;
    default:
        return LOW_SPEED;
    }
#else
    switch (DJI_ReadData.ext_game_robot_state.shooter_id1_17mm_speed_limit)
    {
    case INFANTRY_BS_1:
        return LOW_SPEED;
    case INFANTRY_BS_2:
        return MID_SPEED;
    case INFANTRY_BS_3:
        return HIGH_SPEED;
    default:
        return LOW_SPEED;
    }
#endif
}
/******************************************************************************************************/
/**
 * @brief    获取枪口热量
 * @param    None
 * @return   原始数据，单位不变
 */
rt_uint16_t Ref_Shooter_Cooling_Heat(void)
{
#ifdef CORE_USING_HERO
    return DJI_ReadData.ext_power_heat_data.shooter_id1_42mm_cooling_heat;
#else
    return DJI_ReadData.ext_power_heat_data.shooter_id1_17mm_cooling_heat;
#endif
}

/**
 * @brief    获取枪口每秒冷却值
 * @param    None
 * @return   原始数据，单位不变
 */
rt_uint16_t Ref_Shooter_Cooling_Rate(void)
{
#ifdef CORE_USING_HERO
    return DJI_ReadData.ext_game_robot_state.shooter_id1_42mm_cooling_rate;
#else
    return DJI_ReadData.ext_game_robot_state.shooter_id1_17mm_cooling_rate;
#endif
}

/**
 * @brief    获取枪口热量上限
 * @param    None
 * @return   原始数据，单位不变
 */
rt_uint16_t Ref_Shooter_Cooling_Limit(void)
{
#ifdef CORE_USING_HERO
    return DJI_ReadData.ext_game_robot_state.shooter_id1_42mm_cooling_limit;
#else
    return DJI_ReadData.ext_game_robot_state.shooter_id1_17mm_cooling_limit;
#endif
}
/******************************************************************************************************/

/**
 * @brief    获取红蓝方
 * @param    None
 * @return   Ref_team_color_e
 */
Ref_team_color_e Ref_Team_Color(void)
{
    // 从机器人id判断红蓝方
    switch (DJI_ReadData.ext_game_robot_state.robot_id)
    {
    case REF_ROBO_R1_HERO:
    case REF_ROBO_R2_ENGINEER:
    case REF_ROBO_R3_STANDARD:
    case REF_ROBO_R4_STANDARD:
    case REF_ROBO_R5_STANDARD:
    case REF_ROBO_R6_AERIAL:
    case REF_ROBO_R7_SENTRY:
    case REF_ROBO_R8_DART:
    case REF_ROBO_R9_RADAR:
        return RED_TEAM;
    case REF_ROBO_B1_HERO:
    case REF_ROBO_B2_ENGINEER:
    case REF_ROBO_B3_STANDARD:
    case REF_ROBO_B4_STANDARD:
    case REF_ROBO_B5_STANDARD:
    case REF_ROBO_B6_AERIAL:
    case REF_ROBO_B7_SENTRY:
    case REF_ROBO_B8_DART:
    case REF_ROBO_B9_RADAR:
        return BLUE_TEAM;
    default:
        return REF_ERROR;
    }
}

/**
 * @brief    获取当前机器人ID和客户端ID
 * @param    None
 * @return   Ref_ROBO_CLIENT_ID_e
 */
void Ref_Robot_ID(void)
{
    switch (DJI_ReadData.ext_game_robot_state.robot_id)
    {
    case REF_ROBO_R1_HERO:
        REF_ROBO_ID = REF_ROBO_R1_HERO;
        REF_CLIENT_ID = REF_CLIENT_R1_HERO;
        break;
    case REF_ROBO_R2_ENGINEER:
        REF_ROBO_ID = REF_ROBO_R2_ENGINEER;
        REF_CLIENT_ID = REF_CLIENT_R2_ENGINEER;
        break;
    case REF_ROBO_R3_STANDARD:
        REF_ROBO_ID = REF_ROBO_R3_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_R3_STANDARD;
        break;
    case REF_ROBO_R4_STANDARD:
        REF_ROBO_ID = REF_ROBO_R4_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_R4_STANDARD;
        break;
    case REF_ROBO_R5_STANDARD:
        REF_ROBO_ID = REF_ROBO_R5_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_R5_STANDARD;
        break;
    case REF_ROBO_R6_AERIAL:
        REF_ROBO_ID = REF_ROBO_R6_AERIAL;
        REF_CLIENT_ID = REF_CLIENT_R6_AERIAL;
        break;
    case REF_ROBO_R7_SENTRY:
        REF_ROBO_ID = REF_ROBO_R7_SENTRY;
        break;
    case REF_ROBO_B1_HERO:
        REF_ROBO_ID = REF_ROBO_B1_HERO;
        REF_CLIENT_ID = REF_CLIENT_B1_HERO;
        break;
    case REF_ROBO_B2_ENGINEER:
        REF_ROBO_ID = REF_ROBO_B2_ENGINEER;
        REF_CLIENT_ID = REF_CLIENT_B2_ENGINEER;
        break;
    case REF_ROBO_B3_STANDARD:
        REF_ROBO_ID = REF_ROBO_B3_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_B3_STANDARD;
        break;
    case REF_ROBO_B4_STANDARD:
        REF_ROBO_ID = REF_ROBO_B4_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_B4_STANDARD;
        break;
    case REF_ROBO_B5_STANDARD:
        REF_ROBO_ID = REF_ROBO_B5_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_B5_STANDARD;
        break;
    case REF_ROBO_B6_AERIAL:
        REF_ROBO_ID = REF_ROBO_B6_AERIAL;
        REF_CLIENT_ID = REF_CLIENT_B6_AERIAL;
        break;
    case REF_ROBO_B7_SENTRY:
        REF_ROBO_ID = REF_ROBO_B7_SENTRY;
        break;
    default:
        return;
    }
}

//得到哨兵的血量
uint16_t getSentryHP(void)
{
    if (REF_ROBO_R7_SENTRY == REF_ROBO_ID)
        return DJI_ReadData.ext_game_robot_survivors.red_7_robot_HP;
    else if (REF_ROBO_B7_SENTRY == REF_ROBO_ID)
        return DJI_ReadData.ext_game_robot_survivors.blue_7_robot_HP;
    else
        return 0;
}

/******************************************************************************************************/
// 获取当前机器人的等级 1，2，3分别对应一,二，三级
uint8_t RefGetRobotLevel(void)
{
    return DJI_ReadData.ext_game_robot_state.robot_level;
}

// 获取当前机器人是否飞坡增益
uint8_t RefGetIsFlySlope(void)
{
    return (uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status &= 1 << 3);
}

/**
 * @brief    获取当前底盘输出功率
 * @param    None
 * @return   单位 W 瓦
 */
float Ref_Chassis_Power(void)
{
    return DJI_ReadData.ext_power_heat_data.chassis_power;
}

/**
 * @brief    获取当前底盘输出电压
 * @param    None
 * @return   单位 mv
 */
float Ref_Chassis_Voltage(void)
{
    return DJI_ReadData.ext_power_heat_data.chassis_volt;
}
/**
 * @brief    获取当前底盘输出电流
 * @param    None
 * @return   单位 ma
 */
float Ref_Chassis_Current(void)
{
    return DJI_ReadData.ext_power_heat_data.chassis_current;
}

/**
 * @brief    获取机器人底盘功率限制上限
 * @param    None
 * @return   单位 W 瓦
 */
rt_uint16_t Ref_Chassis_Power_Limit(void)
{
    return DJI_ReadData.ext_game_robot_state.chassis_power_limit;
}

/**
 * @brief    是否超功率
 * @param    None
 * @return   1超功率
 */
rt_uint8_t Ref_Chassis_IsOverPower(void)
{
    return (DJI_ReadData.ext_robot_hurt.hurt_type >> 4) == 0x4;
}

/**
 * @brief    获取底盘功率缓冲
 * @note 	飞坡根据规则增加至 250J
 * @param    None
 * @return   单位 J 焦耳
 */
rt_uint16_t Ref_Chassis_Power_Buffer(void)
{
    return DJI_ReadData.ext_power_heat_data.chassis_power_buffer;
}

/**
 * @brief    获取伤害状态,发送频率：伤害发生后发送
 * @param    None
 * @return   ext_robot_hurt_t
 */
ext_robot_hurt_t Ref_Robot_Hurt_Data(void)
{
    return DJI_ReadData.ext_robot_hurt;
}

/**
 * @brief    获取当前机器人是否受伤
 * @param    None
 * @return   1：受伤，0：未受伤
 */
rt_uint8_t Ref_Get_Robot_If_Hurt(void)
{
    return hurt.robot_if_hurt;
}

// 清除受击打标志位
void gimClrRobotHurt(void)
{
    gim_hurt = 0;
}

rt_uint8_t gimGetRobotHurt(void)
{
    return gim_hurt;
}
/**
 * @brief    设置机器人状态为受伤
 * @param    None
 * @return   None
 */
void Ref_Robot_Set_Hurt(void)
{
    gim_hurt = 1;
    hurt.robot_if_hurt = 1;
}

// 当装甲板受攻击时返回装甲板id
uint8_t getArmorHurtID(void)
{
    if (DJI_ReadData.ext_robot_hurt.hurt_type == 0x0)
        return DJI_ReadData.ext_robot_hurt.armor_id;
    else
        return 0xFF;
}

// 得到当前是否开始比赛
uint8_t getIsStartGame(void)
{
    return DJI_ReadData.ext_game_state.game_progress == 4 ? 1 : 0;
}
/**
 * @brief    设置机器人状态为未受伤
 * @param    None
 * @return   None
 */
void Ref_Robot_Reset_Hurt(void)
{
    hurt.robot_if_hurt = 0;
}

/**
 * @brief    判断当前建筑(前哨站，哨兵，基地)是否掉血
 * @param    None
 * @return   None
 */
void Ref_Bulid_If_Hurt(ext_game_robot_survivors_t *robot_survivors)
{
    if (Ref_Team_Color() == RED_TEAM)
    {
        if (robot_survivors->red_7_robot_HP < hurt.sentry_hp)
        {
            hurt.build_if_attacked |= 1 << 0;
            hurt.sentry_hp = robot_survivors->red_7_robot_HP;
        }
        if (robot_survivors->red_outpost_HP < hurt.outpost_hp)
        {
            hurt.build_if_attacked |= 1 << 1;
            hurt.outpost_hp = robot_survivors->red_outpost_HP;
        }
        if (robot_survivors->red_base_HP < hurt.base_hp)
        {
            hurt.build_if_attacked |= 1 << 2;
            hurt.base_hp = robot_survivors->red_base_HP;
        }
    }
    else if (Ref_Team_Color() == BLUE_TEAM)
    {
        if (robot_survivors->blue_7_robot_HP < hurt.sentry_hp)
        {
            hurt.build_if_attacked |= 1 << 0;
            hurt.sentry_hp = robot_survivors->blue_7_robot_HP;
        }
        if (robot_survivors->blue_outpost_HP < hurt.outpost_hp)
        {
            hurt.build_if_attacked |= 1 << 1;
            hurt.outpost_hp = robot_survivors->blue_outpost_HP;
        }
        if (robot_survivors->blue_base_HP < hurt.base_hp)
        {
            hurt.build_if_attacked |= 1 << 2;
            hurt.base_hp = robot_survivors->blue_base_HP;
        }
    }
}

/**
 * @brief    获取建筑物是否掉血
 * @param    None
 * @return   低三位代表前哨站，哨兵，基地。1：掉血，0：不掉血
 */
rt_uint8_t Ref_Get_Bulid_If_Hurt(void)
{
    return hurt.build_if_attacked;
}

/***
 * @brief 清除建筑受攻击标志位
 * @param type : 建筑类型
 ***/
void Ref_Reset_Bulid_If_Hurt(Ref_Build_e type)
{
    hurt.build_if_attacked &= ~(1 << type);
}
