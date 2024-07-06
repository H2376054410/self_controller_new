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
        .base_hp = 0,
        .tick = 0,
};
static EnemyInfo_t EnemyInfo;

radarData_t radar; /*接收到的雷达站的数据*/
/**
 * @brief    枪口热量冷却加速
 *		  	发送频率： 1Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
 */
rt_uint8_t Ref_Power_Rune_Buff(void)
{
    return DJI_ReadData.ext_buff_musk.cooling_buff;
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
 * @return   单位 m/s
 */
rt_uint16_t Ref_Bullet_Speed_Limit(void)
{
#ifdef CORE_USING_HERO
    return 16;
#else
    return 30;
#endif
}

/**
 * @brief    获取弹速档位
 * @param    None
 * @return   Ref_bullet_speed_mode_e
 */
rt_uint8_t Ref_Bullet_Speed_Mode(void)
{ // v1.6.1协议更新后无弹速上限数据
#ifdef CORE_USING_HERO
    switch (HERO_BS_2)
    {
    case HERO_BS_1:
        return LOW_SPEED;
    case HERO_BS_2:
        return MID_SPEED;
    default:
        return LOW_SPEED;
    }
#else
    switch (INFANTRY_BS_3)
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
    return DJI_ReadData.ext_game_robot_state.shooter_cooling_rate;
}

/**
 * @brief    获取枪口热量上限
 * @param    None
 * @return   原始数据，单位不变
 */
rt_uint16_t Ref_Shooter_Cooling_Limit(void)
{
    return DJI_ReadData.ext_game_robot_state.shooter_cooling_limit;
}
/******************************************************************************************************/

// 得到当前己方的红蓝方
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

// 得到己方哨兵的血量
uint16_t getSentryHP(void)
{
    if (REF_ROBO_R7_SENTRY == REF_ROBO_ID)
        return DJI_ReadData.ext_game_robot_survivors.red_7_robot_HP;
    else if (REF_ROBO_B7_SENTRY == REF_ROBO_ID)
        return DJI_ReadData.ext_game_robot_survivors.blue_7_robot_HP;
    else
        return 0;
}
// 己方补给区的占领状态
uint8_t get_selfsupplyarea_status(void)
{
    return (DJI_ReadData.ext_event_data.event_type >> 2) & 0x01;
}
// 中心增益点的占领情况
uint8_t get_Occupation_of_center_gain_points(void)
{
    return (DJI_ReadData.ext_event_data.event_type >> 30) & 0x03;
}
/******************************************************************************************************/
// 获取当前机器人的等级 1，2，3分别对应一,二，三级
uint8_t RefGetRobotLevel(void)
{
    return DJI_ReadData.ext_game_robot_state.robot_level;
}

/**
 * @brief 获取当前机器人是否飞坡增益
 *        根据规则：同一台机器人需在 10 秒内检测到一方场地两处飞坡增益点的场地交互模块卡，才能触发飞坡增益
 *        飞坡增益持续时间：20秒
 * @return uint8_t
 */
uint8_t RefGetIsFlySlope(void)
{
    uint8_t ret_flag = 0;
    static rt_tick_t flyslope_tick = 0; // 最近一次获取到飞坡增益的时刻
    static rt_tick_t tick1 = 0, tick2 = 0, tick3 = 0, tick4 = 0, ticktmp = 0;
    ticktmp = GetRefDataRefreshTick(Index_rfid_status);
    if (rt_tick_get() - ticktmp <= 10 * 1000) // 对于判断飞坡，10s内此数据有效
    {
        if ((uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status >> 8 & 0x01))
        { // 己方飞坡增益区飞坡前
            tick1 = ticktmp;
        }
        else if ((uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status >> 9 & 0x01))
        { // 己方飞坡增益区飞坡后
            tick2 = ticktmp;
        }
        else if ((uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status >> 10 & 0x01))
        { // 对方飞坡增益区飞坡前
            tick3 = ticktmp;
        }
        else if ((uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status >> 11 & 0x01))
        { // 对方飞坡增益区飞坡后
            tick4 = ticktmp;
        }
    }
    if (tick1 && tick2) // 确保数据非零有效
    {
        if (tick2 - tick1 <= 10 * 1000)
        { // 己方飞坡增益区触发
            ret_flag = 1;
            flyslope_tick = tick2;
        }
    }
    if (tick3 && tick4) // 确保数据非零有效
    {
        if (tick4 - tick3 <= 10 * 1000)
        { // 对方飞坡增益区触发
            ret_flag = 1;
            flyslope_tick = tick4;
        }
    }
    if (flyslope_tick && rt_tick_get() - flyslope_tick < 20 * 1000) // 20秒增益时间
        ret_flag = 1;
    else // 20秒过后增益消失
        ret_flag = 0;
    return ret_flag;
}

// 中心增益点RFID检测情况
uint8_t RFID_center_gain_points(void)
{
    uint8_t ret_flag = 0;
    if (rt_tick_get() - GetRefDataRefreshTick(Index_rfid_status) > 500)
        ret_flag = 0;
    else if ((uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status >> 19 & 0x01))
        ret_flag = 1;
    return ret_flag;
}

//  己方补血点RFID
uint8_t RFID_self_supply_points(void)
{
    uint8_t ret_flag = 0;
    if (rt_tick_get() - GetRefDataRefreshTick(Index_rfid_status) > 500)
        ret_flag = 0;
    else if ((uint8_t)(DJI_ReadData.ext_rfid_status.rfid_status >> 13 & 0x01))
        ret_flag = 1;
    return ret_flag;
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
 * @brief    是否超功率扣血
 * @param    传入tick时实时刷新当前的扣血状态，否则只返回裁判系统最近一次发送过来的伤害数据状态
 * @param    注：裁判系统特性：受伤数据只在受伤害时发送，不受伤时不发送
 * @return   1超功率
 */
rt_uint8_t Ref_Chassis_IsOverPower(uint32_t *reftick)
{
    hurt.tick = GetRefDataRefreshTick(Index_robot_hurt);
    if (reftick)
    {
        if (rt_tick_get() - hurt.tick <= 20) // 20ms内都相信此数据
        {
            if (DJI_ReadData.ext_robot_hurt.hurt_type == 0x4)
            {
                *reftick = hurt.tick;
                return 1;
            }
            else
                return 0;
        }
        else if (*reftick != hurt.tick && DJI_ReadData.ext_robot_hurt.hurt_type == 0x4)
        {
            *reftick = hurt.tick;
            return 1;
        }
        else
            return 0;
    }
    else if (DJI_ReadData.ext_robot_hurt.hurt_type == 0x4)
        return 1;
    else
        return 0;
}
/**
 * @brief    获取底盘缓冲能量
 * @note 	飞坡根据规则增加至 250J
 * @param    None
 * @return   单位 J 焦耳
 */
rt_uint16_t
Ref_Chassis_Power_Buffer(void)
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
    hurt.tick = GetRefDataRefreshTick(Index_robot_hurt);
    if (rt_tick_get() - hurt.tick <= 20) // 20ms内都相信此数据
    {
        if (DJI_ReadData.ext_robot_hurt.hurt_type == 0x0)
            return DJI_ReadData.ext_robot_hurt.armor_id;
    }
    return 0xFF;
}

// 得到当前是否开始比赛
uint8_t getIsStartGame(void)
{
    return DJI_ReadData.ext_game_state.game_progress == 4 ? 1 : 0;
}

// 得到当前游戏剩余时间
uint16_t getGameRemainime(void)
{
    return DJI_ReadData.ext_game_state.stage_remain_time;
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
        hurt.isValid = 1;
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
        hurt.isValid = 1;
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

// 得到己方前哨战血量
uint16_t getOurOutpostHP(void)
{
    if (hurt.isValid)
        return hurt.outpost_hp;
    else
        return 0;
}

// 得到敌方的相关信息 例如前哨战血量
EnemyInfo_t GetEnemyInfo(void)
{
    if (Ref_Team_Color() == RED_TEAM)
    {
        EnemyInfo.outpost_hp = DJI_ReadData.ext_game_robot_survivors.blue_outpost_HP;
    }
    else if (Ref_Team_Color() == BLUE_TEAM)
    {
        EnemyInfo.outpost_hp = DJI_ReadData.ext_game_robot_survivors.red_outpost_HP;
    }
    return EnemyInfo;
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

void protocolRadarData(const Frame_t *const frame)
{
    radar.data = *(uint8_t *)frame->Data.ext_rec_user_data.userData.data;
    radar.tick = rt_tick_get();
}

uint8_t getRadarDataFlag(uint32_t *tick)
{
    if (tick)
        *tick = radar.tick;

    return radar.data;
}
// 解析雷达站数据
// void protocolRadarData(const Frame_t *const frame)
// {
//     uint8_t robotPosNum = 0;
//     m_radarData_t *temp = (m_radarData_t *)frame->Data.ext_rec_user_data.userData.data;
//     radar.info.tick = rt_tick_get();
//     switch (frame->Data.ext_rec_user_data.dataFrameHeader.data_cmd_id)
//     {
//     case radar_robotPos:
//         robotPosNum = (frame->FrameHeader.DataLength - sizeof(ext_student_interactive_header_data_t)) /
//                       sizeof(m_radarData_t);
//         if (robotPosNum > 7)
//         {
//             radar.info.errCounts++;
//         }
//         else
//         {
//             for (int i = 0; i < robotPosNum; i++)
//             {
//                 if (temp->robotID > 7)
//                 {
//                     radar.info.errCounts++;
//                     continue;
//                 }
//                 else
//                 {
//                     rt_memcpy(&radar.data[temp->robotID], temp, sizeof(m_radarData_t));
//                     radar.dataTick[temp->robotID] = rt_tick_get();
//                     if (radar.dataIsValid[temp->robotID])
//                     {
//                         radar.filterData[temp->robotID].x = 0.99f * radar.filterData[temp->robotID].x + (1 - 0.99f) * radar.data[temp->robotID].x;
//                         radar.filterData[temp->robotID].y = 0.99f * radar.filterData[temp->robotID].y + (1 - 0.99f) * radar.data[temp->robotID].y;
//                         radar.filterData[temp->robotID].z = 0.99f * radar.filterData[temp->robotID].z + (1 - 0.99f) * radar.data[temp->robotID].z;
//                     }
//                     radar.dataIsValid[temp->robotID] = 1;
//                 }
//                 temp += 1;
//             }
//         }
//         break;
//     case radar_alarm:
//         radar.alarmArea = *(uint8_t *)frame->Data.ext_rec_user_data.userData.data;
//         radar.robotID = *(uint8_t *)(frame->Data.ext_rec_user_data.userData.data + 1);
//         break;
//     default:
//         break;
//     }
// }

// 小地图下发信息
ext_robot_command_map_t getMapData(void)
{
    return DJI_ReadData.ext_robot_command_map;
}
