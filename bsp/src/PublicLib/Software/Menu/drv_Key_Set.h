#ifndef __DRV_KEY_SET_H__
#define __DRV_KEY_SET_H__

#include <rtdef.h>

#define KEY_COUNT_ALL (20) // 按键总数

// 对按键进行编号
typedef enum
{
    KeyEVT_MouseL = (rt_uint32_t)1,
    KeyEVT_MouseR = (rt_uint32_t)1 << 1,
    KeyEVT_W = (rt_uint32_t)1 << 2,
    KeyEVT_S = (rt_uint32_t)1 << 3,
    KeyEVT_A = (rt_uint32_t)1 << 4,
    KeyEVT_D = (rt_uint32_t)1 << 5,
    KeyEVT_SHIFT = (rt_uint32_t)1 << 6,
    KeyEVT_CTRL = (rt_uint32_t)1 << 7,
    KeyEVT_Q = (rt_uint32_t)1 << 8,
    KeyEVT_E = (rt_uint32_t)1 << 9,
    KeyEVT_R = (rt_uint32_t)1 << 10,
    KeyEVT_F = (rt_uint32_t)1 << 11,
    KeyEVT_G = (rt_uint32_t)1 << 12,
    KeyEVT_Z = (rt_uint32_t)1 << 13,
    KeyEVT_X = (rt_uint32_t)1 << 14,
    KeyEVT_C = (rt_uint32_t)1 << 15,
    KeyEVT_V = (rt_uint32_t)1 << 16,
    KeyEVT_B = (rt_uint32_t)1 << 17,
    KeyEVT_MouseZ_P = (rt_uint32_t)1 << 18,       // 鼠标滚轮
    KeyEVT_MouseZ_N = (rt_uint32_t)1 << 19,       // 鼠标滚轮
    KeyEVT_End = (rt_uint32_t)1 << KEY_COUNT_ALL, // 在一些循环中用作结尾判断
    KeyEVT_ALL = (int)0xFFFFFFFF,
} Key_Select_E;

// 对按键进行编号
typedef enum
{
    KeyNum_MouseL = 0,
    KeyNum_MouseR = 1,
    KeyNum_W = 2,
    KeyNum_S = 3,
    KeyNum_A = 4,
    KeyNum_D = 5,
    KeyNum_SHIFT = 6,
    KeyNum_CTRL = 7,
    KeyNum_Q = 8,
    KeyNum_E = 9,
    KeyNum_R = 10,
    KeyNum_F = 11,
    KeyNum_G = 12,
    KeyNum_Z = 13,
    KeyNum_X = 14,
    KeyNum_C = 15,
    KeyNum_V = 16,
    KeyNum_B = 17,
    KeyNum_MouseZ_P = 18,
    KeyNum_MouseZ_N = 19,
    KeyNum_End = KEY_COUNT_ALL // 在一些循环中用作结尾判断
} Key_Num_E;

#define AIMBOT_KEY KeyEVT_MouseR  // 自瞄
//#define PROBE_KEY KeyEVT_V        // 探头模式
#define LIUCHAO_KEY KeyEVT_V        // 刘超模式
#define SCAP_RESERVE_KEY KeyEVT_C // 开启/关闭超级电容的保留能量
#define JUMP_KEY KeyEVT_B

#define GUN_MODE1 KeyEVT_MouseZ_N // 单发模式 (修改模式时查询其它菜单的状态，区分枪模式设置和热量限制设置)
#define GUN_MODE2 KeyEVT_MouseZ_P // 三连发模式

// 底盘模式二维按键
// 选择底盘模式
#define CHASSISMODE_KEY_ENTRY KeyEVT_R
// 选择底盘模式菜单后选择具体模式

#define CHASSISMODE_KEY_MODE_NO_FOLLOW KeyEVT_G  // 不跟随
#define CHASSISMODE_KEY_MODE_FOLLOW KeyEVT_W     // 跟随
#define CHASSISMODE_KEY_MODE_SMALL_GYRO KeyEVT_R // 慢陀螺
#define CHASSISMODE_KEY_MODE_FAST_GYRO KeyEVT_E  // 快陀螺
#define CHASSISMODE_KEY_MODE_MOVE_BACK KeyEVT_C  // 倒车模式（回头）

#ifdef CORE_USING_BALANCING_INFANTRY
// 平步按键
#define CHASSISMODE_KEY_MODE_WHEEL_LEGGED_FOLDED KeyEVT_Q // 轮腿抱紧
#define CHASSISMODE_KEY_MODE_VERTICAL_FOLLOW KeyEVT_Z    // 正交跟随
#define CHASSISMODE_KEY_MODE_HEIGHT_STATE_ONE KeyEVT_MouseZ_N   // 高度一档
#define CHASSISMODE_KEY_MODE_HEIGHT_STATE_THREE KeyEVT_MouseZ_P  // 高度三档
#else 
#define CHASSISMODE_KEY_MODE_MOVE_BACK KeyEVT_C  // 倒车模式（回头）
#endif

// 瞄准模式设置
#define AIMMODE_SET_ENTRY KeyEVT_G
// 具体模式设置
#define AIMMODE_SET_AIMBOT KeyEVT_G           // 常规自瞄
#define AIMMODE_SET_ROTATING_OUTPOST KeyEVT_Z // 动态击打前哨站
#define AIMMODE_SET_STATIC_OUTPOST KeyEVT_X   // 静态击打前哨站
#ifdef CORE_USING_HERO
#define AIMMODE_SET_OUTPOST_F KeyEVT_C     // 第三种击打前哨战的模式
#define AIMMODE_SET_DANGLING_MODE KeyEVT_V // 吊射模式
#elif defined CORE_USING_INFANTRY
#define AIMMODE_SET_AIMBUFF_CONST_SPEED KeyEVT_V // 小能量机关
#define AIMMODE_SET_AIMBUFF_VARY_SPEED KeyEVT_Q  // 大能量机关
#endif

// 弹速选择设置
#define GUNSPEED_SET_ENTRY KeyEVT_Q
// 具体模式设置
#if defined CORE_USING_HERO
#define GUNSPEED_SET_10 KeyEVT_Z // 弹速10
#define GUNSPEED_SET_16 KeyEVT_X // 弹速16
#elif defined CORE_USING_INFANTRY
#define GUNSPEED_SET_15 KeyEVT_Z // 弹速15
#define GUNSPEED_SET_18 KeyEVT_X // 弹速18
#define GUNSPEED_SET_30 KeyEVT_C // 弹速30
#elif defined CORE_USING_SENTRY
#define GUNSPEED_SET_30 KeyEVT_C // 弹速30
#endif

// 杂项设置
#define MISC_KEY_ENTRY KeyEVT_E

#define MISC_KEY_UI_RST KeyEVT_Z // 具体项: 重置UI显示
#ifdef CORE_USING_INFANTRY
#define MISC_KEY_HATCH_OPEN KeyEVT_C // 具体项: 打开弹舱盖子
#define MISC_KEY_HATCH_CLSE KeyEVT_V // 具体项: 关闭弹舱盖子
#elif defined CORE_USING_WALL_E
#define MISC_KEY_HATCH_OPEN KeyEVT_C // 具体项: 打开弹舱盖子
#define MISC_KEY_HATCH_CLSE KeyEVT_V // 具体项: 关闭弹舱盖子
#elif defined CORE_USING_HERO
#define MISC_KEY_BURST_FIRE KeyEVT_E
#endif
#define AIMBOT_COLOR_RED KeyEVT_MouseZ_N
#define AIMBOT_COLOR_BLUE KeyEVT_MouseZ_P

#define MISC_KEY_RESET KeyEVT_R // 具体项: 复位机器人上的单片机

#define MISC_KEY_GIMBAL_RESET KeyEVT_G  // 复位云台单片机
#define MISC_KEY_CHASSIS_RESET KeyEVT_C // 复位底盘单片机
#define MISC_KEY_TOTAL_RESET KeyEVT_Q   // 复位云台及底盘单片机

#define MISC_KEY_CLOSE_CAP KeyEVT_G // 具体项: 关闭或开启超级电容

#define MISC_KEY_CAP_OPEN KeyEVT_C // 具体项: 打开超级电容充电
#define MISC_KEY_CAP_CLSE KeyEVT_V // 具体项: 关闭超级电容充电

// 性能设置
#define PERFORMANCE_KEY_ENTRY KeyEVT_V

#define PERFORMANCE_KEY_LEVEL1 KeyEVT_Z
#define PERFORMANCE_KEY_LEVEL2 KeyEVT_X
#define PERFORMANCE_KEY_LEVEL3 KeyEVT_C

#define PERFORMANCE_TYPE_KEY_ENTRY KeyEVT_V

#define PERFORMANCE_TYPE_KEY_OUTBREAK KeyEVT_Z // 爆发优先
#ifndef CORE_USING_HERO
#define PERFORMANCE_TYPE_KEY_COOLING KeyEVT_X // 冷却优先
#endif
#define PERFORMANCE_TYPE_KEY_POWER KeyEVT_V          // 功率优先
#define PERFORMANCE_TYPE_KEY_BLOOD KeyEVT_Q          // 血量优先
#define PERFORMANCE_TYPE_KEY_FORCED_OFFLINE KeyEVT_G // 强制裁判系统数据离线
#define PERFORMANCE_TYPE_KEY_FORCED_ONLINE KeyEVT_R  // 强制裁判系统数据在线

// 运动设置
#define ACCL_KEY KeyEVT_SHIFT // 加速
#define SLOW_KEY KeyEVT_CTRL  // 减速

#define FOREWORD_KEY KeyEVT_W // 前进
#define BACK_KEY KeyEVT_S     // 后退
#define LEFT_KEY KeyEVT_A     // 左移
#define RIGHT_KEY KeyEVT_D    // 右移

#ifdef CORE_USING_BALANCING_INFANTRY
#define CHASSISMODE_KEY_MODE_STOP_STOP KeyEVT_F //  停（平步）
#define CHASSISMODE_KEY_MODE_JUMP KeyEVT_B //  跳（平步）
#endif

// 发弹设置
#define MOUSE_L KeyEVT_MouseL // 左键发弹

#endif
