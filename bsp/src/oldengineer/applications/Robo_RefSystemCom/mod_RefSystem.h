#ifndef __MOD_REFSYSTEM_H__
#define __MOD_REFSYSTEM_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "rtconfig.h"

#define BSP_REF_USART_NAME "uart1"

#define FRAMEMAXSEIZE 40 // 31+9加上帧头的数据
#define DATAMAXSEIZE 30  // 一次数据最大长度

#define HEADER_SOF 0xA5 // 帧头

#define LEN_HEADER 5 // 帧头长度
#define LEN_CMDID 2  // 命名码长度
#define LEN_TAIL 2   // 帧尾CRC16

#define LEN_game_state 3               // 0x0001
#define LEN_game_result 1              // 0x0002
#define LEN_game_robot_survivors 32    // 0x0003
#define LEN_dart_status 3              // 0x0004
#define LEN_ICRA_buff_status 3         // 0x0005
#define LEN_event_data 4               // 0x0101
#define LEN_supply_projectile_action 3 // 0x0102
/* #define LEN_supply_projectile_booking 2 //0x0103   */
#define LEN_referee_warning 2     // 0x0104
#define LEN_dart_remaining_time 1 // 0x0105
#define LEN_game_robot_state 18   // 0x0201
#define LEN_power_heat_data 16    // 0x0202
#define LEN_game_robot_pos 16     // 0x0203
#define LEN_buff_musk 1           // 0x0204
#define LEN_aerial_robot_energy 3 // 0x0205
#define LEN_robot_hurt 1          // 0x0206
#define LEN_shoot_data 6          // 0x0207
#define LEN_bullet_remaining 2    // 0x0208
#define LEN_rfid_status 4         // 0x0209
#define LEN_dart_client_cmd 12    // 0x020A

#define Robot_Interact_DataLen 113         // 0x0301 机器人间交互数据 包括UI 最大为113字节
#define Custom_Control_Interact_DataLen 30 // 0x0302 发送的内容数据段最大为 30 字节

typedef struct
{
    uint32_t rec_time;
    uint32_t last_time;

    float frame;
} DJI_Frame_t;

typedef enum
{
    noErr,
    lenErr,
    bufSizeErr,
    pakageLenErr,
    headErr,
    firstHeadErr,
    needPakage,
} DJI_errState_e;

/* 数据帧字节偏移 */
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7,

} FrameOffset_e;

/* 数据帧帧头字节偏移 */
typedef enum
{
    SOF = 0,         // 数据帧起始字节
    DATA_LENGTH = 1, // 数据帧中data的长度
    SEQ = 3,         // 包序号
    CRC8 = 4         // CRC8

} FrameHeaderOffset_e;

/* cmd_id 命令码 ID */
typedef enum
{
    ID_game_state = 0x0001,                // 比赛状态数据
    ID_game_result = 0x0002,               // 比赛结果数据
    ID_game_robot_survivors = 0x0003,      // 比赛机器人血量数据
    ID_dart_status = 0x0004,               // 飞镖发射状态
    ID_ICRA_buff_status = 0x0005,          // ICRA
    ID_event_data = 0x0101,                // 场地事件数据
    ID_supply_projectile_action = 0x0102,  // 场地补给站动作标志数据
    ID_supply_projectile_booking = 0x0103, // 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
    ID_referee_warning = 0x0104,           // 裁判警告数据
    ID_dart_remaining_time = 0x0105,       // 飞镖发射口倒计时
    ID_game_robot_state = 0x0201,          // 机器人状态数据
    ID_power_heat_data = 0x0202,           // 实时功率热量数据
    ID_game_robot_pos = 0x0203,            // 机器人位置数据
    ID_buff_musk = 0x0204,                 // 机器人增益数据
    ID_aerial_robot_energy = 0x0205,       // 空中机器人能量状态数据
    ID_robot_hurt = 0x0206,                // 伤害状态数据
    ID_shoot_data = 0x0207,                // 实时射击数据
    ID_bullet_remaining = 0x0208,          // 弹丸剩余发射数
    ID_rfid_status = 0x0209,               // 机器人RFID状态
    ID_dart_client_cmd = 0x020A,           // 飞镖机器人客户端数据
    ID_student_interactive_data = 0x0301,  // 机器人间交互数据
    ID_robot_interactive_data = 0x0302,    // 自定义控制器交互数据接口
    ID_robot_command_map = 0x0303,         // 客户端小地图交互数据，触发发送
    ID_robot_command_remote = 0x0304,      // 键盘、鼠标信息，通过图传串口发送
    ID_client_mini_map_receieve = 0x0305,  // 客户端小地图接收信息

} CmdID_e;

typedef enum
{
    Index_game_state,                // 比赛状态数据
    Index_game_result,               // 比赛结果数据
    Index_game_robot_survivors,      // 比赛机器人血量数据
    Index_dart_status,               // 飞镖发射状态
    Index_ICRA_buff_status,          // ICRA
    Index_event_data,                // 场地事件数据
    Index_supply_projectile_action,  // 场地补给站动作标志数据
    Index_supply_projectile_booking, // 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
    Index_referee_warning,           // 裁判警告数据
    Index_dart_remaining_time,       // 飞镖发射口倒计时
    Index_game_robot_state,          // 机器人状态数据
    Index_power_heat_data,           // 实时功率热量数据
    Index_game_robot_pos,            // 机器人位置数据
    Index_buff_musk,                 // 机器人增益数据
    Index_aerial_robot_energy,       // 空中机器人能量状态数据
    Index_robot_hurt,                // 伤害状态数据
    Index_shoot_data,                // 实时射击数据
    Index_bullet_remaining,          // 弹丸剩余发射数
    Index_rfid_status,               // 机器人RFID状态
    Index_dart_client_cmd,           // 飞镖机器人客户端数据
    Index_student_interactive_data,  // 机器人间交互数据
    Index_robot_interactive_data,    // 自定义控制器交互数据接口
    Index_robot_command_map,         // 客户端小地图交互数据，触发发送
    Index_robot_command_remote,      // 键盘、鼠标信息，通过图传串口发送
    Index_client_mini_map_receieve,  // 客户端小地图接收信息
    refIndex_all,
} CmdIDIndex_e;

typedef struct
{
    rt_tick_t game_robot_state; // 机器人状态数据
    rt_tick_t power_heat_data;  // 实时功率热量数据
    rt_tick_t buff_musk;        // 机器人增益数据
    rt_tick_t shoot_data;       // 实时射击数据
} RefReceiveTime_s;             // 各类数据接收到的时间

/* 帧头 */
typedef __packed struct
{
    uint8_t SOF;         // 数据帧起始字节
    uint16_t DataLength; // 数据帧中data的长度
    uint8_t Seq;         // 包序号
    uint8_t CRC8;        // CRC8校验

} FrameHeader_t;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct
{
    uint8_t game_type : 4;      // 比赛类型
    uint8_t game_progress : 4;  // 当前比赛阶段
    uint16_t stage_remain_time; // 当前阶段剩余时间
    uint64_t SyncTimeStamp;     // 机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效

} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct
{
    uint8_t winner; // 0 平局 1 红方胜利 2 蓝方胜利

} ext_game_result_t;

/* ID: 0x0003  28bits    机器人血量数据 */
typedef __packed struct
{
    uint16_t red_1_robot_HP;  // 红 1 英雄机器人血量，未上场以及罚下血量为 0
    uint16_t red_2_robot_HP;  // 红 2 工程机器人血量
    uint16_t red_3_robot_HP;  // 红 3 步兵机器人血量
    uint16_t red_4_robot_HP;  // 红 4 步兵机器人血量
    uint16_t red_5_robot_HP;  // 红 5 步兵机器人血量
    uint16_t red_7_robot_HP;  // 红 7 哨兵机器人血量
    uint16_t red_outpost_HP;  // 红方前哨战血量
    uint16_t red_base_HP;     // 红方基地血量
    uint16_t blue_1_robot_HP; // 蓝 1 英雄机器人血量
    uint16_t blue_2_robot_HP; // 蓝 2 工程机器人血量
    uint16_t blue_3_robot_HP; // 蓝 2 工程机器人血量
    uint16_t blue_4_robot_HP; // 蓝 2 工程机器人血量
    uint16_t blue_5_robot_HP; // 蓝 5 步兵机器人血量
    uint16_t blue_7_robot_HP; // 蓝 7 哨兵机器人血量
    uint16_t blue_outpost_HP; // 蓝方前哨站血量

    uint16_t blue_base_HP; // 红方基地血量

} ext_game_robot_survivors_t;

/* ID: 0x0004  3bits  飞镖发射状态 */
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;

} ext_dart_status_t;

/* ID: 0x0005 3bits ICRA */
typedef __packed struct
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left; // 红方 1 号剩余弹量
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;

} ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct
{
    uint32_t event_type;

} ext_event_data_t;

/* ID: 0x0102  Byte:  4    补给站动作标识 */
typedef __packed struct
{
    uint8_t supply_projectile_id;   // 补给站口 ID
    uint8_t supply_robot_id;        // 补给站口 ID
    uint8_t supply_projectile_step; // 出弹口开闭状态
    uint8_t supply_projectile_num;  // 补弹数量

} ext_supply_projectile_action_t;

/* ID: 0X0104  Byte:  2    裁判警告数据 */
typedef __packed struct
{
    uint8_t level;         // 1：黄牌 2：红牌 3：判负
    uint8_t foul_robot_id; // 犯规机器人 ID

} ext_referee_warning_t;

/* ID: 0X0105  Byte:  1    飞镖发射口倒计时 */
typedef __packed struct
{
    uint8_t dart_remaining_time; // 15s 倒计时

} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    比赛机器人状态 */
typedef __packed struct
{
    uint8_t robot_id;                        // 机器人id 可用于进行数据校验
    uint8_t robot_level;                     // 机器人等级
    uint16_t remain_HP;                      // 剩余血量
    uint16_t max_HP;                         // 血量上限
    uint16_t shooter_id1_17mm_cooling_rate;  // 机器人 1 号 17mm 枪口每秒冷却值
    uint16_t shooter_id1_17mm_cooling_limit; // 机器人 1 号 17mm 枪口热量上限
    uint16_t shooter_id1_17mm_speed_limit;   // 机器人 1 号 17mm 枪口上限速度 单位 m/s
    uint16_t shooter_id2_17mm_cooling_rate;  // 机器人 2 号 17mm 枪口每秒冷却值
    uint16_t shooter_id2_17mm_cooling_limit; // 机器人 2 号 17mm 枪口热量上限
    uint16_t shooter_id2_17mm_speed_limit;   // 机器人 2 号 17mm 枪口上限速度 单位 m/s
    uint16_t shooter_id1_42mm_cooling_rate;  // 机器人 42mm 枪口每秒冷却值
    uint16_t shooter_id1_42mm_cooling_limit; // 机器人 42mm 枪口热量上限
    uint16_t shooter_id1_42mm_speed_limit;   // 机器人 42mm 枪口上限速度 单位 m/s
    uint16_t chassis_power_limit;            // 机器人底盘功率限制上限

    uint8_t mains_power_gimbal_output : 1; // 电源管理模块输出情况
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;

} ext_game_robot_status_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct
{
    uint16_t chassis_volt;                  // 底盘输出电压 单位 毫伏
    uint16_t chassis_current;               // 底盘输出电流 单位 毫安
    float chassis_power;                    // 底盘输出功率 单位 W 瓦
    uint16_t chassis_power_buffer;          // 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
    uint16_t shooter_id1_17mm_cooling_heat; // 1 号 17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat; // 2 号 17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat; // 42mm 枪口热量

} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct
{
    float x; // 位置 x 坐标，单位 m
    float y;
    float z;
    float yaw; // 位置枪口，单位度

} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct
{
    uint8_t power_rune_buff; // 机器人攻击 防御 冷却加成 补血

} ext_buff_musk_t;

/* ID: 0x0205  Byte:  3    空中机器人能量状态 */
typedef __packed struct
{
    uint8_t attack_time; // 可攻击时间 单位 s。30s 递减至 0

} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态 */
typedef __packed struct
{
    uint8_t armor_id : 4;  // 当血量变化类型为装甲伤害，代表装甲 ID
    uint8_t hurt_type : 4; // 血量变化类型 6种

} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  6    实时射击信息 */
typedef __packed struct
{
    uint8_t bullet_type; // 子弹类型: 1：17mm 弹丸 2：42mm 弹丸
    uint8_t shooter_id;  // 发射机构 ID
    uint8_t bullet_freq; // 子弹射频 单位 Hz
    float bullet_speed;  // 子弹射速 单位 m/s

} ext_shoot_data_t;

/* ID: 0x0208  Byte:  2    子弹剩余发射数 */
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm; // 17mm 子弹剩余发射数目
    uint16_t bullet_remaining_num_42mm; // 42mm 子弹剩余发射数目
    uint16_t coin_remaining_num;        // 剩余金币数量

} ext_bullet_remaining_t;

/* ID：0X0209  Byte:  4    机器人RFID状态 */
typedef __packed struct
{
    uint32_t rfid_status;

} ext_rfid_status_t;

/* ID: 0x020A  Byte:  12   飞镖机器人客户端指令数据 */
typedef __packed struct
{
    uint8_t dart_launch_opening_status; // 当前飞镖发射口的状态
    uint8_t dart_attack_target;         // 飞镖的打击目标，默认为前哨站
    uint16_t target_change_time;        // 切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0
    uint16_t operate_launch_cmd_time;   // 最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0

} ext_dart_client_cmd_t;

/* ID: 0x0301  Byte:  n    机器人间交互数据 */
typedef __packed struct
{
    uint16_t data_cmd_id; // 数据段的内容 ID  0x0200~0x02FF为己方机器人通信  0x0100~0x004和0x110为客户端绘制图形
    uint16_t send_ID;     // 发送者的 ID
    uint16_t receiver_ID; // 接收者的 ID

} ext_student_interactive_header_data_t;

/* ID: 0x0302  Byte:  n    自定义控制器交互数据 */
typedef __packed struct
{
    uint8_t data[Custom_Control_Interact_DataLen];

} robot_interactive_data_t;

/* ID: 0x0301 UI 最大数据长度 */
typedef __packed struct
{
    uint8_t data[Robot_Interact_DataLen];

} UI_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t masks;

} user_custom_data_t;

/* ID: 0x0303  Byte:  15    小地图交互信息 */
typedef __packed struct
{
    float target_position_x; // 目标 x 位置坐标，单位 m
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;   // 发送指令时，云台手按下的键盘信息
    uint16_t target_robot_ID; // 要作用的目标机器人 ID

} ext_robot_command_map_t;

/* ID: 0x0304  Byte:  14    图传遥控信息 发送频率：30Hz */
typedef __packed struct
{
    int16_t mouse_x; // 鼠标 X 轴信息
    int16_t mouse_y;
    int16_t mouse_z;         // 鼠标滚轮信息
    int8_t left_button_down; // 鼠标左键
    int8_t right_button_down;
    uint16_t keyboard_value; // 键盘信息
    uint16_t reserved;       // 键盘信息

} ext_robot_command_remote_t;

// 小地图接收信息标识：0x0305。最大接收频率：10Hz
typedef __packed struct
{
    uint16_t target_robot_ID; // 目标机器人 ID
    float target_position_x;  // 目标 x 位置坐标，单位 m 当 x,y 超出界限时则不显示
    float target_position_y;  // 目标 y 位置坐标，单位 m 当 x,y 超出界限时则不显示。
} ext_client_map_command_t;

// 这里用于0x0200 ~0x02FF为己方机器人通信
typedef __packed struct
{
    FrameHeader_t txFrameHeader; // 帧头
    uint16_t CmdID;              // 机器人间通信为0x301
    ext_student_interactive_header_data_t dataFrameHeader;
    robot_interactive_data_t userData; // 这里用于0x0200 ~0x02FF为己方机器人通信
    uint16_t FrameTail;                // CRC16校验

    uint16_t dataLength; // 数据长度

} ext_send_user_data_t;

typedef __packed struct
{
    FrameHeader_t txFrameHeader; // 帧头
    uint16_t CmdID;              // 机器人间通信为0x301
    ext_student_interactive_header_data_t dataFrameHeader;
    UI_data_t userData; // UI最大数据长度
    uint16_t FrameTail; // CRC16校验

} UI_Frame_t;

/*------------------------------------可用结构体----------------------------------------------------------*/

// 暂存单个裁判系统数据
typedef __packed struct
{
    FrameHeader_t FrameHeader;
    uint16_t CmdID;
    __packed union
    {
        ext_game_state_t ext_game_state;                             // 0x0001
        ext_game_result_t ext_game_result;                           // 0x0002
        ext_game_robot_survivors_t ext_game_robot_survivors;         // 0x0003
        ext_dart_status_t ext_dart_status;                           // 0x0004
        ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_status;     // 0x0005
        ext_event_data_t ext_event_data;                             // 0x0101
        ext_supply_projectile_action_t ext_supply_projectile_action; // 0x0102
        // ext_supply_projectile_booking_t		  ext_supply_projectile_booking;
        ext_referee_warning_t ext_referee_warning;           // 0x0104
        ext_dart_remaining_time_t ext_dart_remaining_time;   // 0x0105
        ext_game_robot_status_t ext_game_robot_state;        // 0x0201
        ext_power_heat_data_t ext_power_heat_data;           // 0x0202
        ext_game_robot_pos_t ext_game_robot_pos;             // 0x0203
        ext_buff_musk_t ext_buff_musk;                       // 0x0204
        aerial_robot_energy_t aerial_robot_energy;           // 0x0205
        ext_robot_hurt_t ext_robot_hurt;                     // 0x0206
        ext_shoot_data_t ext_shoot_data;                     // 0x0207
        ext_bullet_remaining_t ext_bullet_remaining;         // 0x0208
        ext_rfid_status_t ext_rfid_status;                   // 0x0209
        ext_dart_client_cmd_t ext_dart_client_cmd;           // 0x020A
        ext_send_user_data_t ext_send_user_data;             // 0x0301
        robot_interactive_data_t robot_interactive_data;     // 0x0302
        ext_robot_command_map_t ext_robot_command_map;       // 0x0303
        ext_robot_command_remote_t ext_robot_command_remote; // 0x0304
    } Data;
    uint16_t CRC16;

} Frame_t;

// 所有裁判系统数据
typedef __packed struct
{
    FrameHeader_t FrameHeader;
    uint16_t CmdID;
    ext_game_state_t ext_game_state;                             // 0x0001
    ext_game_result_t ext_game_result;                           // 0x0002
    ext_game_robot_survivors_t ext_game_robot_survivors;         // 0x0003
    ext_dart_status_t ext_dart_status;                           // 0x0004
    ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_status;     // 0x0005
    ext_event_data_t ext_event_data;                             // 0x0101
    ext_supply_projectile_action_t ext_supply_projectile_action; // 0x0102
    // ext_supply_projectile_booking_t ext_supply_projectile_booking;
    ext_referee_warning_t ext_referee_warning;           // 0x0104
    ext_dart_remaining_time_t ext_dart_remaining_time;   // 0x0105
    ext_game_robot_status_t ext_game_robot_state;        // 0x0201
    ext_power_heat_data_t ext_power_heat_data;           // 0x0202
    ext_game_robot_pos_t ext_game_robot_pos;             // 0x0203
    ext_buff_musk_t ext_buff_musk;                       // 0x0204
    aerial_robot_energy_t aerial_robot_energy;           // 0x0205
    ext_robot_hurt_t ext_robot_hurt;                     // 0x0206
    ext_shoot_data_t ext_shoot_data;                     // 0x0207
    ext_bullet_remaining_t ext_bullet_remaining;         // 0x0208
    ext_rfid_status_t ext_rfid_status;                   // 0x0209
    ext_dart_client_cmd_t ext_dart_client_cmd;           // 0x020A
    ext_send_user_data_t ext_send_user_data;             // 0x0301
    robot_interactive_data_t robot_interactive_data;     // 0x0302
    ext_robot_command_map_t ext_robot_command_map;       // 0x0303
    ext_robot_command_remote_t ext_robot_command_remote; // 0x0304

} DJI_Data_t;

#define DATATYPENUM refIndex_all        // 一次数据最大长度
extern RefReceiveTime_s RefReceiveTime; // 裁判系统相关有效数据接收到的时间

typedef struct
{
    DJI_Frame_t DJI_Frame[DATATYPENUM]; // 计算帧率
    uint32_t err_counts;
    uint32_t err_counts_pac;
    uint8_t Debug_UARTEn;

    uint32_t lastT; // 检测裁判系统是否正常运行
    uint16_t threadT;
    uint16_t confirmNum;
    uint8_t isValid;
    DJI_errState_e errState;
} DJI_info_t;

/* rtt串口驱动 */
struct DJI_Mxg
{
    rt_device_t dev;
    rt_size_t size;
};

typedef struct
{
    void (*Ref_BulidIfHurt_f)(ext_game_robot_survivors_t *robot_survivors);
    void (*Ref_RobotSetHurt_f)(void);

} RefCallBack_t;

typedef struct
{
    uint16_t processLen;         /*单位字节*/
    uint16_t nowLen;             /*已经有了多少字节*/
    uint16_t needLen;            /*还需多少字节*/
    uint8_t flag;                // 是否需要拼包
    uint8_t data[FRAMEMAXSEIZE]; /*一个完整包的长度存储*/
} package_t;

#define RefIDNUM (20) // 单位ms
typedef struct
{
    uint16_t RefID[RefIDNUM];
    uint8_t IDNum;
    uint32_t refTick[RefIDNUM];

} RefID_t;

#define GetNowPackageLen(dataLen) (sizeof(FrameHeader_t) + LEN_CMDID + dataLen + LEN_TAIL)
#define ERR_PERIOD (50)      // 单位ms
#define ERR_CONFIRM_NUM (10) // 单位ms
/**
 * @brief: 初始化裁判系统
 * @param {None}
 * @return {*}
 */
rt_err_t DJI_Init(void);
rt_tick_t GetRefDataRefreshTick(CmdIDIndex_e CmdIDIndex);
uint8_t RefDataIsOffline(void);
void Cal_frame(DJI_Frame_t *DJI_Frame_Cal); // 计算各个包的频率;
/*解析裁判系统数据的逻辑代码*/
void DJI_protocol(uint8_t *SrcData, uint16_t srcSize,
                  DJI_Data_t *pRecBuf, uint16_t bufSize);
#endif
