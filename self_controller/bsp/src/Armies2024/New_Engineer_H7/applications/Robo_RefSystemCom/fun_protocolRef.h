#ifndef __APP_GETREF_H__
#define __APP_GETREF_H__
#include <rtthread.h>
#include "HRef_ID.h"
#include "mod_RefSystem.h"

/* 英雄弹速上限档位 */
#define HERO_BS_1 10
#define HERO_BS_2 16

/* 步兵弹速上限档位 */
#define INFANTRY_BS_1 15
#define INFANTRY_BS_2 18
#define INFANTRY_BS_3 30

typedef enum
{
    LOW_SPEED = 0,
    MID_SPEED,
    HIGH_SPEED,

} Ref_bullet_speed_mode_e;

// 红蓝方
typedef enum
{
    REF_ERROR,
    RED_TEAM = 1,
    BLUE_TEAM,

} Ref_team_color_e;

typedef enum
{
    REF_OUTPOST,  // 前哨战
    REF_SENTRY,   // 哨兵
    REF_BASEMENT, // 基地
    BUILDING_NUM
} Ref_Build_e;
typedef struct
{
    rt_uint8_t robot_if_hurt;     // 机器人是否受伤
    rt_uint8_t build_if_attacked; // 建筑物是否受伤
    rt_uint16_t sentry_hp;        // 哨兵血量
    rt_uint16_t outpost_hp;       // 前哨站血量
    rt_uint16_t base_hp;          // 基地血量
    rt_uint8_t isValid;
    uint32_t tick;

} Ref_Hurt_t;

typedef struct
{
    rt_uint16_t outpost_hp; // 前哨站血量
} EnemyInfo_t;

/*------------------------------------XXX----------------------------------------------------------*/

#define RADAR_ROBOT_NUM_MAX 10
typedef enum
{
    radar_robotPos = 0x201, // 坐标
    radar_alarm = 0x202,    // 预警

} radarInfo_e;

typedef enum
{
    robot_ID1,
    robot_ID2,
    robot_ID3,
    robot_ID4,
    robot_ID5,
    robot_IDSentry,
    robot_IDAll,
} radarRobotID_e;

// 位置信息
typedef __packed struct
{
    uint16_t robotID;
    float x;
    float y;
    float z;
} m_radarData_t;

// 关于雷达站信息的描述
// typedef struct
// {
//     uint8_t nowRobotNum;
//     uint8_t errCounts;
//     uint32_t tick;
// } m_radarInfo_t;

// typedef struct
// {
//     m_radarData_t data[RADAR_ROBOT_NUM_MAX];       // 实时数据
//     m_radarData_t filterData[RADAR_ROBOT_NUM_MAX]; // 滤波数据
//     uint32_t dataTick[RADAR_ROBOT_NUM_MAX];        // 实时的时间戳
//     uint32_t dataIsValid[RADAR_ROBOT_NUM_MAX];        // 实时的时间戳
//     uint8_t alarmArea;            // 警戒区域
//     uint8_t robotID;              // 警戒区域内的机器人的ID，0-6每一位代表一个机器人
//     uint8_t alarmTick;            // 警戒的时间戳
//     m_radarInfo_t info;
// } radarData_t;

typedef struct
{
    uint8_t data;
    uint32_t tick;
} radarData_t;
/*------------------------------------XXX----------------------------------------------------------*/

// 枪口热量冷却加速
rt_uint8_t Ref_Power_Rune_Buff(void);

// 获取剩余弹量
rt_uint16_t Ref_AmmoRemain(void);

// 获取上一发子弹射速,发送频率：射击后发送 单位 cm/s
rt_uint16_t Ref_Bullet_Speed(void);

// 获取枪口上限速度 单位 m/s
rt_uint16_t Ref_Bullet_Speed_Limit(void);

// 获取弹速档位 单位 cm/s
rt_uint8_t Ref_Bullet_Speed_Mode(void);

// 获取枪口热量  原始数据，单位不变
rt_uint16_t Ref_Shooter_Cooling_Heat(void);

// 获取枪口每秒冷却值 原始数据，单位不变
rt_uint16_t Ref_Shooter_Cooling_Rate(void);

// 获取枪口热量上限 原始数据，单位不变
rt_uint16_t Ref_Shooter_Cooling_Limit(void);

// 获取己方红蓝方
Ref_team_color_e Ref_Team_Color(void);

// 获取当前机器人ID和客户端ID
void Ref_Robot_ID(void);

// 获取当前机器人的等级
uint8_t RefGetRobotLevel(void);

// 获取当前机器人是否飞坡增益
uint8_t RefGetIsFlySlope(void);

// 获取当前底盘输出功率
float Ref_Chassis_Power(void);

// 获取当前底盘输出电压   单位 mv
float Ref_Chassis_Voltage(void);

// 获取当前底盘输出电流   单位 ma
float Ref_Chassis_Current(void);

// 是否超功率   1超功率
rt_uint8_t Ref_Chassis_IsOverPower(uint32_t *reftick);

// 获取机器人底盘功率限制上限  单位 W 瓦
rt_uint16_t Ref_Chassis_Power_Limit(void);

// 获取底盘功率缓冲  单位 J 焦耳 <飞坡根据规则增加至 250J>
rt_uint16_t Ref_Chassis_Power_Buffer(void);

// 获取伤害状态,发送频率：伤害发生后发送
ext_robot_hurt_t Ref_Robot_Hurt_Data(void);

// 清除受击打标志位
void gimClrRobotHurt(void);
rt_uint8_t gimGetRobotHurt(void);

// 得到己方哨兵的血量
uint16_t getSentryHP(void);

// 己方补给区的占领状态
uint8_t get_selfsupplyarea_status(void);

// 中心增益点的占领情况
uint8_t get_Occupation_of_center_gain_points(void);

// 中心增益点RFID检测情况
uint8_t RFID_center_gain_points(void);

//  己方补血点RFID
uint8_t RFID_self_supply_points(void);

// 得到己方前哨战血量
uint16_t getOurOutpostHP(void);

// 当装甲板受攻击时返回装甲板id
uint8_t getArmorHurtID(void);

// 得到当前是否开始比赛
uint8_t getIsStartGame(void);

// 得到当前游戏剩余时间
uint16_t getGameRemainime(void);

// 获取当前机器人是否受伤  1：受伤，0：未受伤
rt_uint8_t Ref_Get_Robot_If_Hurt(void);

// 设置机器人状态为受伤
void Ref_Robot_Set_Hurt(void);

// 设置机器人状态为未受伤
void Ref_Robot_Reset_Hurt(void);

// 判断当前建筑(前哨站，哨兵，基地)是否掉血
void Ref_Bulid_If_Hurt(ext_game_robot_survivors_t *robot_survivors);

// 获取建筑物是否掉血 低三位代表前哨站，哨兵，基地。1：掉血，0：不掉血
rt_uint8_t Ref_Get_Bulid_If_Hurt(void);

// 清除建筑受攻击标志位     type : 建筑类型
void Ref_Reset_Bulid_If_Hurt(Ref_Build_e type);

// 得到敌方的相关信息 例如前哨战血量
EnemyInfo_t GetEnemyInfo(void);

// 解析雷达站数据
void protocolRadarData(const Frame_t *const frame);

// 小地图下发信息
ext_robot_command_map_t getMapData(void);

uint8_t getRadarDataFlag(uint32_t *tick);
#endif
