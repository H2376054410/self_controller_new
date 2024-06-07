#ifndef __MOD_ROBOSTATECTRL_H
#define __MOD_ROBOSTATECTRL_H

#include "drv_Chassis_Solve.h"
#include "func_Dataserver.h"

#define CPU_RESER_THRESHOLD1 464
#define CPU_RESER_THRESHOLD2 1584

#define AIRPUMP_CLOSEDALEYTIME 500
#define CPURESET_DALEYTIME 500

#define REMOTECONNECT_TIMEINTERVAL 1000 // 一秒

/*平滑结构体枚举*/
typedef enum
{
    Arm_X,
    Arm_Y,
    Arm_Z,
    Arm_Yaw,
    Arm_Pitch,
    Arm_Roll,
    Chassis_X,
    Chassis_Y,
    Chassis_Yaw,
    ImagePitch_Ctrl
} Filter_Enum;
/*平滑结构体枚举*/

typedef struct
{
    VectorXYZ_Str BoomPos_Set;
    VectorXYZ_Str BoomPos_SetOld;
    BoomMotor_s BoomAngle_Set;
    float ImageTransPitch_SetOld;
    ArmPos_Remote_s ArmStateNow_Filter;
    ArmPos_Remote_s ArmState_Add;
    ArmPos_Remote_s ArmState_Set;
    ArmPos_Remote_s ArmState_SetOld;
} ArmState_s;

/****************************结构体初始化**************************/

/**
 * @brief 底盘遥控状态结构体初始化
 * @param ChassisSpeState
 */
void ChassisSpeRemote_Struct_Init(ChassisMotion_t *ChassisSpeState);

/**
 * @brief 机械臂遥控状态结构体初始化
 * @param ArmPosState
 */
void ArmPosRemote_Struct_Init(ArmPos_Remote_s *ArmState);

/****************************存取矿石********************************/
/**
 * @brief 存放矿石
 */
void Ore_Drawing(void);

/**
 * @brief 抓取矿石
 */
void Ore_Grabbing(void);

/*************************************************************/

/*机械臂初始位置*/
#define ARMPOS_INIT_X 0
#define ARMPOS_INIT_Y 0.31f
#define ARMPOS_INIT_Z 0.24f
#define ARMPOS_INIT_YAW 1.57f
#define ARMPOS_INIT_PITCH -1.57f
#define ARMPOS_INIT_ROLL 0.0f

/*机械臂底盘模式位置*/
#define ARMPOS_CHASSIS_X 0.0f
#define ARMPOS_CHASSIS_Y 0.48f
#define ARMPOS_CHASSIS_Z 0.0f
#define ARMPOS_CHASSIS_YAW 1.57f
#define ARMPOS_CHASSIS_PITCH -1.57f
#define ARMPOS_CHASSIS_ROLL 0.0f

/*机械臂银矿准备姿态*/
#define ARMPOS_SILVERINIT_X 0.00f
#define ARMPOS_SILVERINIT_Y 0.42f
#define ARMPOS_SILVERINIT_Z 0.32f
#define ARMPOS_SILVERINIT_YAW 1.57f
#define ARMPOS_SILVERINIT_PITCH 0.13f
#define ARMPOS_SILVERINIT_ROLL -0.00f

/*机械臂银矿上拔姿态*/
#define ARMPOS_SILVERPULLUP_X 0.00f
#define ARMPOS_SILVERPULLUP_Y 0.39f
#define ARMPOS_SILVERPULLUP_Z 0.50f
#define ARMPOS_SILVERPULLUP_YAW 1.57f
#define ARMPOS_SILVERPULLUP_PITCH 0.21f
#define ARMPOS_SILVERPULLUP_ROLL -0.00f

/*机械臂二级矿瞄准姿态*/
#define ARMPOS_2LEVELORE_X 0.00f
#define ARMPOS_2LEVELORE_Y 0.365f
#define ARMPOS_2LEVELORE_Z 0.40f
#define ARMPOS_2LEVELORE_YAW 1.57f
#define ARMPOS_2LEVELORE_PITCH 0.09f
#define ARMPOS_2LEVELORE_ROLL -0.07f

/*机械臂三级矿瞄准姿态*/
#define ARMPOS_3LEVELORE_X 0.00f
#define ARMPOS_3LEVELORE_Y 0.31f
#define ARMPOS_3LEVELORE_Z 0.62f
#define ARMPOS_3LEVELORE_YAW 1.57f
#define ARMPOS_3LEVELORE_PITCH 0.09f
#define ARMPOS_3LEVELORE_ROLL -0.07f

/*机械臂左四级矿瞄准姿态*/
#define ARMPOS_4LEVELORE_L_X 0.00f
#define ARMPOS_4LEVELORE_L_Y 0.31f
#define ARMPOS_4LEVELORE_L_Z 0.62f
#define ARMPOS_4LEVELORE_L_YAW 1.57f
#define ARMPOS_4LEVELORE_L_PITCH 0.09f
#define ARMPOS_4LEVELORE_L_ROLL -0.07f

/*机械臂右四级矿瞄准姿态*/
#define ARMPOS_4LEVELORE_R_X 0.00f
#define ARMPOS_4LEVELORE_R_Y 0.31f
#define ARMPOS_4LEVELORE_R_Z 0.62f
#define ARMPOS_4LEVELORE_R_YAW 1.57f
#define ARMPOS_4LEVELORE_R_PITCH 0.09f
#define ARMPOS_4LEVELORE_R_ROLL -0.07f

/*机械臂正金矿准备姿态*/
#define ARMPOS_FRONTGOLDINIT_X -0.03f
#define ARMPOS_FRONTGOLDINIT_Y 0.74f
#define ARMPOS_FRONTGOLDINIT_Z 0.31f
#define ARMPOS_FRONTGOLDINIT_YAW 1.58f
#define ARMPOS_FRONTGOLDINIT_PITCH -0.1f
#define ARMPOS_FRONTGOLDINIT_ROLL 0

/*机械臂侧金矿准备姿态*/
#define ARMPOS_SIDEGOLDINIT_X 0.10f
#define ARMPOS_SIDEGOLDINIT_Y 0.68f
#define ARMPOS_SIDEGOLDINIT_Z 0.37f
#define ARMPOS_SIDEGOLDINIT_YAW 1.02f
#define ARMPOS_SIDEGOLDINIT_PITCH -0.36f
#define ARMPOS_SIDEGOLDINIT_ROLL 0.60f

/*机械臂金矿上拔姿态*/
#define ARMPOS_SIDE_GOLDPULLUP_X 0.15f
#define ARMPOS_SIDE_GOLDPULLUP_Y 0.59f
#define ARMPOS_SIDE_GOLDPULLUP_Z 0.45f
#define ARMPOS_SIDE_GOLDPULLUP_YAW 1.09f
#define ARMPOS_SIDE_GOLDPULLUP_PITCH -0.08f
#define ARMPOS_SIDE_GOLDPULLUP_ROLL 0.60f

/*机械臂金矿上拔姿态*/
#define ARMPOS_FRONT_GOLDPULLUP_X 0.00f
#define ARMPOS_FRONT_GOLDPULLUP_Y 0.68f
#define ARMPOS_FRONT_GOLDPULLUP_Z 0.48f
#define ARMPOS_FRONT_GOLDPULLUP_YAW 1.56f
#define ARMPOS_FRONT_GOLDPULLUP_PITCH 0.17f
#define ARMPOS_FRONT_GOLDPULLUP_ROLL 0.00f

/*机械臂地上矿图传准备姿态*/
#define ARMPOS_FLOORIMAGEINIT_X 0.01f
#define ARMPOS_FLOORIMAGEINIT_Y 0.50f
#define ARMPOS_FLOORIMAGEINIT_Z 0.30f
#define ARMPOS_FLOORIMAGEINIT_YAW 1.5707f
#define ARMPOS_FLOORIMAGEINIT_PITCH -1.5707f
#define ARMPOS_FLOORIMAGEINIT_ROLL 0

/*机械臂吸取地上矿姿态*/
#define ARMPOS_FLOORINIT_X 0.01f
#define ARMPOS_FLOORINIT_Y 0.50f
#define ARMPOS_FLOORINIT_Z -0.04f
#define ARMPOS_FLOORINIT_YAW 1.5707f
#define ARMPOS_FLOORINIT_PITCH -1.5707f
#define ARMPOS_FLOORINIT_ROLL 0

/*机械臂转矿上抬状态*/
#define ARMPOS_FLOORPULLUP_X 0.01f
#define ARMPOS_FLOORPULLUP_Y 0.50f
#define ARMPOS_FLOORPULLUP_Z 0.24f
#define ARMPOS_FLOORPULLUP_YAW 1.5707f
#define ARMPOS_FLOORPULLUP_PITCH 0
#define ARMPOS_FLOORPULLUP_ROLL 1.5707f

/*机械臂转矿下放状态*/
#define ARMPOS_FLOORLAYDOWN_X 0.0f
#define ARMPOS_FLOORLAYDOWN_Y 0.54f
#define ARMPOS_FLOORLAYDOWN_Z 0.01f
#define ARMPOS_FLOORLAYDOWN_YAW 1.5707f
#define ARMPOS_FLOORLAYDOWN_PITCH 0
#define ARMPOS_FLOORLAYDOWN_ROLL 1.5707f

/*机械臂障碍快瞄准姿态*/
#define ARMPOS_OBSTACLE_X 0.0f
#define ARMPOS_OBSTACLE_Y 0.54f
#define ARMPOS_OBSTACLE_Z 0.01f
#define ARMPOS_OBSTACLE_YAW 1.5707f
#define ARMPOS_OBSTACLE_PITCH 0
#define ARMPOS_OBSTACLE_ROLL 1.5707f

/*机械臂障碍快瞄准姿态*/
#define ARMPOS_OBSTACLE_PULLUP_X 0.0f
#define ARMPOS_OBSTACLE_PULLUP_Y 0.54f
#define ARMPOS_OBSTACLE_PULLUP_Z 0.01f
#define ARMPOS_OBSTACLE_PULLUP_YAW 1.5707f
#define ARMPOS_OBSTACLE_PULLUP_PITCH 0
#define ARMPOS_OBSTACLE_PULLUP_ROLL 1.5707f

/**
 * @brief 机械臂初始状态
 */
void ArmPosState_Init(void);

/**
 * @brief 机械臂抛矿状态
 */
void ArmPosState_ThrowingOre(void);

/**
 * @brief 机械臂初始状态
 */
void ArmPosState_Chassis(void);

/**
 * @brief 机械臂银矿初始状态
 */
void ArmPosState_SilverInit(void);

/**
 * @brief 机械臂三级矿状态
 */
void ArmPosState_3levelOre_Init(void);

/**
 * @brief 机械臂左四级矿状态
 */
void ArmPosState_4levelOreL_Init(void);

/**
 * @brief 机械臂右四级矿状态
 */
void ArmPosState_4levelOreR_Init(void);

/**
 * @brief 正金矿模式
 */
void ArmPosState_FrontGold_Init(void);

/**
 * @brief 侧金矿模式
 */
void ArmPosState_SideGold_Init(void);

/**
 * @brief 机械臂银矿吸取状态
 */
void ArmPosState_SilverGrip(void);

/**
 * @brief 机械臂银矿上拔状态
 */
void ArmPosState_SilverPullup(void);

/**
 * @brief 机械臂金矿初始状态
 */
void ArmPosState_FrontGoldInit(void);

/**
 * @brief 机械臂侧金矿初始状态
 */
void ArmPosState_SideGoldInit(void);

/**
 * @brief 机械臂银矿上拔状态
 */
void ArmPosState_SideGoldPullup(void);

/**
 * @brief 机械臂正金矿上拔状态
 */
void ArmPosState_FrontGoldPullup(void);

/**
 * @brief 机械臂地上矿图传初始状态
 */
void ArmPosState_floorImageInit(void);

/**
 * @brief 机械臂地上矿初始状态
 */
void ArmPosState_floorInit(void);

/**
 * @brief 机械臂转矿上抬状态
 */
void ArmPosState_floorPullup(void);

/**
 * @brief 机械臂转矿下放状态
 */
void ArmPosState_floorLaydown(void);

/**
 * @brief 机械臂障碍快瞄准姿态
 */
void ArmPosState_ObstacleblockAim(void);


/**
 * @brief 机械臂障碍快上拔姿态
 */
void ArmPosState_ObstacleblockPullup(void);
/**********************************机器人速度增益设定*******************************/
/*底盘*/
#define GAIN_CHASSIS_KEY_MOVE 300.0f
#define GAIN_CHASSIS_KEY_ROTATE 3.0f

#define GAIN_CHASSIS_MOUSE_MOVE_X 2.0f
#define GAIN_CHASSIS_MOUSE_MOVE_Y 2.0f
#define GAIN_CHASSIS_MOUSE_ROTATE_Y 1.0f

#define GAIN_CHASSIS_REMOTE 0.6f
// #define GAIN_CHASSIS_REMOTE_ROTATE_Y 0.00002f    //直接控制云台
#define GAIN_CHASSIS_REMOTE_ROTATE_Y 0.7f
/*大机械臂*/
#define GAIN_BOOM_KEY_MOVE 1.0f

#define GAIN_BOOM_MOUSE_MOVE_X 0.000015f
#define GAIN_BOOM_MOUSE_MOVE_Y 0.000015f
#define GAIN_BOOM_MOUSE_MOVE_Z 0.000015f

#define GAIN_BOOM_REMOTE 0.0000016f
/*小机械臂*/
#define GAIN_FOREARM_KEY_MOVE 0.000002f

#define GAIN_FOREARM_MOUSE_MOVE_YAW 0.000038f
#define GAIN_FOREARM_MOUSE_MOVE_PITCH 0.000090f
#define GAIN_FOREARM_MOUSE_MOVE_ROLL 0.0001f
#define GAIN_FOREARM_MOUSE_MOVE_IMAGE 0.000090f

#define GAIN_FOREARM_REMOTE 0.000016f

/*自定义控制器数据*/
#define GAIN_CUSTCTRLER_X 0.000016f
#define GAIN_CUSTCTRLER_Y 0.000016f
#define GAIN_CUSTCTRLER_Z 0.000016f
#define GAIN_CUSTCTRLER_YAW 0.000016f
#define GAIN_CUSTCTRLER_PITCH 0.000016f
#define GAIN_CUSTCTRLER_ROLL 0.000016f

/*图传电机*/
#define GAIN_IMAGETRANS_REMOTE 0.000015f

/*速度增益*/
#define ROBO_SPEEDGAIN_ACCE 2.0f
#define ROBO_SPEEDGAIN_DCCE 0.5f
#define ROBO_SPEEDGAIN_NORM 1.0f

/**
 * @brief 正常状态
 */
void RoboState_NormalGain(void);

/**
 * @brief 加速状态
 */
void RoboState_Acceleration(void);

/**
 * @brief 减速状态
 */
void RoboState_Deceleration(void);

/**
 * @brief 机器人增益下速度计算
 * @param A_in
 * @param C_in
 * @param GimbalYaw_in
 * @param SpeedGain
 * @param A_out
 * @param C_out
 * @param GimbalYaw_out
 */
void Robo_GainSpeedCalculate(ArmPos_Remote_s *A_in,
                             ChassisMotion_t *C_in,
                             float GimbalYaw_in,
                             float SpeedGain,
                             ArmPos_Remote_s *A_out,
                             ChassisMotion_t *C_out,
                             float *GimbalYaw_out);

/*****************************数据判0*****************************/

/**
 * @brief 小机械臂角度增量
 * @param in
 * @return int
 */
int ForearmAngleAdd_ZeroIf(ForearmMotor_s *in, float tolerance);

/********************************滤波相关********************************/

typedef enum
{
    ChassisSmoothFilter = 0, // 底盘平滑滤波器
    BoomSmoothFilter,        // 大机械臂平滑滤波器
    ForearmSmoothFilter,     // 小机械臂平滑滤波器
    AllSmoothFilter,         // 小机械臂平滑滤波器
} SmoothFilter_Restart_e;    // 平滑滤波器清零枚举体

/**
 * @brief 平滑滤波器清零设置
 * @param type
 */
void SmoothFilter_Restart(SmoothFilter_Restart_e type);

/**
 * @brief 获取滤波器结构体
 * @author fwlh
 * @author mylj
 * @param  filter           指定需要获取的滤波器
 * @return void*            返回指向滤波器的指针
 */
void *Get_Filter(Filter_Enum filter);

/*********************************模拟量********************************/
typedef enum
{
    Client_Chassis = 0, // 客户端底盘
    Remote_Chassis,     // 遥控器底盘
    Client_Boom,        // 客户端大机械臂
    Remote_Boom,        // 遥控器大机械臂
    Client_Forearm,     // 客户端小机械臂
    Remote_Forearm,     // 遥控器小机械臂
} RoboCtrl_Target_e;

/**
 * @brief 设定平滑滤波设定数据
 * @author mylj
 */
void SmootFilterhData_Set(void);

/**
 * @brief 对机器人的控制模式进行配置
 * @author mylj
 */
void RoboCtrl_State_Judge(void);

/**
 * @brief 获取平滑滤波后的数据
 * @author mylj
 */
void SmootFilterhData_Get(ArmPos_Remote_s *ArmState_Set,
                          VectorXY_Str *ChassisSpeXY_Set,
                          float *GimbalYaw_Set,
                          CustCtrler_Data_s *CustCtrlerState_Set);

/**
 * @brief 输出当前BoomYaw和小臂矢量
 * @brief 用于机器人姿态锁存
 * @param ArmState_in
 * @param BoomYaw_in
 * @param ForearmVector_out1 考虑机械偏移
 * @param ForearmVector_out2 不考虑机械偏移
 * @param BoomYaw_out
 * @param ForearmYaw_out
 */
void RoboPos_Latch(ArmPos_Remote_s *ArmState_in,
                   float BoomYaw_in,
                   VectorXYZ_Str *ForearmVector_out1,
                   VectorXYZ_Str *ForearmVector_out2,
                   float *BoomYaw_out,
                   float *ForearmYaw_out);

/**
 * @brief 沿当前小臂矢量方向，求解机械臂的坐标增量值
 * @param ArmState
 * @param Set_in
 * @param BoomYaw
 * @param BoomYawlatch
 * @param ForearmVector_latch1 考虑机械偏移
 * @param ForearmVector_latch2 不考虑机械偏移
 * @param SpeedSet      设定增量的速度，单位：m/s
 * @param dt            时间，单位：s
 * @param DeltaPos_out
 */
void ForearmVector2Arm_DeltaGet(VectorXYZ_Str *ArmPos_Set,
                                float BoomYaw, float BoomYawlatch,
                                float ForearmYawlatch,
                                VectorXYZ_Str *ForearmVector_latch1,
                                VectorXYZ_Str *ForearmVector_latch2,
                                float SpeedSet, float dt,
                                ArmPos_Remote_s *DeltaPos_out);

/**
 * @brief 坐标系转换——由小臂坐标系转换到机器人的坐标系
 * @param in
 * @param pitch   推荐使用图传pitch
 * @param ForearmYaw
 * @param out
 */
void XYZ_Forearm2Robo(VectorXYZ_Str *in,
                      float pitch, float ForearmYaw,
                      VectorXYZ_Str *out);

// /**
//  * @brief 遥控坐标以图传视野为基础坐标系控制
//  * @param Now_in
//  * @param PosSetlast_in
//  * @param BoomYaw
//  * @param ArmPosDelta_in
//  * @param Delta_out
//  */
// void ArmPosRemote_XYZtrans(ArmPos_Remote_s *Now_in,
//                            VectorXYZ_Str *PosSetlast_in,
//                            float BoomYaw,
//                            VectorXYZ_Str *ArmPosDelta_in,
//                            ArmPos_Remote_s *Delta_out);

/**
 * @brief  与遥控器连接情况
 * @return 1：成功连接状态  0：连接失败状态
 */
int RemoteConnectState_Get(void);

#endif /*__MOD_ROBOSTATECTRL_H*/
