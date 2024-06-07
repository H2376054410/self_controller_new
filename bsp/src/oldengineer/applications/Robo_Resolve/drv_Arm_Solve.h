#ifndef __FUNC_ARM_POS_H__
#define __FUNC_ARM_POS_H__

#include <rtthread.h>
#include "drv_Setplanning3D.h"

/*
 *x轴正方向水平向右
 *y轴正方向水平向前
 *z轴正方向竖直向上
 */

/*大小臂之间初始坐标补偿*/
#define BoomToFoerarm_y 0.07f  // 大臂螺丝到小臂yaw间y轴方向的坐标点平移
#define BoomToFoerarm_z 0.056f // 大臂螺丝到小臂yaw间z轴方向的坐标点平移

/*大机械臂机械尺寸宏定义，单位：m,精确度：1mm，即0.001*/
#define BoomMaster_P1_L 0.360f                             // 第一个pitch的大驱动臂的长度
#define BoomMaster_P2_L 0.060f                             // 第二个pitch的小驱动臂的长度
#define BoomSlave1_P2_L 0.360f                             // 第二个pitch的联动臂1的长度
#define BoomSLave2_P2_L 0.060f                             // 第二个pitch的联动臂2的长度
#define BoomSLave3_P2_L 0.300f                             // 第二个pitch的联动臂3的长度
#define BoomSLave_P2_L (BoomSLave2_P2_L + BoomSLave3_P2_L) // 第二个pitch的“横梁”联动臂的长度

#define BoomMaster_P1_L_Square 0.1296f // SQUARE(BoomMaster_P1_L)
#define BoomSLave3_P2_L_Square 0.09f   // SQUARE(BoomSLave3_P2_L)

/*小机械臂机械尺寸宏定义，单位：m,精确度：1mm，即0.001*/

#define ForearmMaster_Y_L 0.067f // yaw轴到pitch轴水平距离
#define Forearm_YtoP_L 0.052f    // yaw轴到pitch轴竖直距离
#define ForearmMaster_P_L 0.152f // pitch轴到矿石两个吸盘中心的距离

/*图传尺寸宏定义，单位*/
#define BOOMSLAVE2IMAGE_L 0.155f  // 第二个pitch的联动臂3到图传按照位置的臂长
#define IMAGE_LEVEL_L 0.2061f     // 机械臂到安装位置的水平距离
#define IMAGE_VERTIUCAL_L 0.0015f // 机械臂到安装位置的竖直距离
/*********************************扭矩补偿*********************************/
#define gravity_AC 9.8f // 重力加速度
/*机械臂等效重力，单位为：N，精确度：0.01N，即0.01*/
#define BoomMaster_EquiG1 12.15f  // 大机械臂主动臂的等效重力
#define BoomSlave2_3_EquiG2 8.92f // 大机械臂联动臂2&3的等效重力
#define Forearm_EquiG3 11.76f     // 小机械臂的等效重力(估算)
#define BoomSlave1_EquiG4 3.61f   // 大机械臂联动臂1的等效重力
#define ForearmPitch_EquiG5 5.2f  // 小机械臂pitch轴之后的等效重力(估算)
#define BoomSlave3_EquiG6 8.04f   // 大机械臂联动臂3的等效重力

/*机械臂等效力臂，单位：m,精确度：1mm，即0.001*/
#define Boom_Forearm_EquiL2 0.120f // 小机械臂等效重心与大机械臂末端的距离
#define ForearmPitch_EquiL3 0.100f // 小机械臂pitch轴之后等效重心到pitch轴的距离

/*转矩电流对应电机的电流设定值 单位：/A*/
#define TorqueCurrentToSet_2006 1000
#define TorqueCurrentToSet_3508 819.2f
#define TorqueCurrentToSet_4310 100
#define TorqueCurrentToSet_10020 100
/*电机对应的扭矩常数 单位：N·m/A*/
#define TorqueConstant_2006 0.18f
#define TorqueConstant_3508 0.3f
#define TorqueConstant_4310 1.4f
#define TorqueConstant_10020 2.5f
/*扭矩向电流设定值的转换比 单位/N/m*/
#define Conversion_ratio_2006 (TorqueCurrentToSet_2006 / TorqueConstant_2006)
#define Conversion_ratio_3508 (TorqueCurrentToSet_3508 / TorqueConstant_3508)
#define Conversion_ratio_4310 (TorqueCurrentToSet_4310 / TorqueConstant_4310)
#define Conversion_ratio_10020 (TorqueCurrentToSet_10020 / TorqueConstant_10020)
/*各关节所用的电机对应的扭矩电流设定值的转换比*/
#define BoomYaw_M1_Ratio Conversion_ratio_4310
#define BoomPitch1_M2_Ratio Conversion_ratio_10020
#define BoomPitch2_M3_Ratio Conversion_ratio_10020
#define ForearmPitch_M5_Ratio Conversion_ratio_4310

/************************控制角度限度******************************/
#define BoomYaw_AngleMax 2.60f
#define BoomYaw_AngleMin 0.50f
#define BoomPitch1_AngleMax 2.315f
#define BoomPitch1_AngleMin 0.92f
#define BoomPitch2_AngleMax 1.25f
#define BoomPitch2_AngleMin -1.20f

#define ForearmYaw_AngleMax 2.75f
#define ForearmYaw_AngleMin 0.37f
#define ForearmPitch_AngleMax 0.21f
#define ForearmPitch_AngleMin -1.6f
#define ForearmRoll_AngleMax 2.90f
#define ForearmRoll_AngleMin -3.33f

/************************机械角度限幅******************************/
#define BoomYaw_LimitMax BoomYaw_AngleMax
#define BoomYaw_LimitMin BoomYaw_AngleMin
#define BoomPitch1_LimitMax BoomPitch1_AngleMax
#define BoomPitch1_LimitMin BoomPitch1_AngleMin
#define BoomPitch2_LimitMax BoomPitch2_AngleMax
#define BoomPitch2_LimitMin BoomPitch2_AngleMin

#define FOREARMYAW_LIMITMAX ForearmYaw_AngleMax
#define FOREARMYAW_LIMITMIN ForearmYaw_AngleMin
#define FOREARMPITCH_LIMITMAX ForearmPitch_AngleMax
#define FOREARMPITCH_LIMITMIN ForearmPitch_AngleMin
#define FOREARMROLL_LIMITMAX ForearmRoll_AngleMax
#define FOREARMROLL_LIMITMIN ForearmRoll_AngleMin

#define IMAGETRANS_LIMITMAX ImageTrans_AngleMax
#define IMAGETRANS_LIMITMIN ImageTrans_AngleMin

/************************机械逼近******************************/
#define APPROACHING_ORIGIN_X 0
#define APPROACHING_ORIGIN_Y 0.29f
#define APPROACHING_ORIGIN_Z 0.19f

// 当BoomPitch2达到最大角度机械限位的情况下的最大夹角
#define DeltaAngle_LimitMax 3.08f
// 大臂和小臂的机械结构限位下的最小夹角
#define BoomPitch1_Critical1 1.34f // 临界角度值
#define DeltaAngle_Limit1Min 0.68f
// 小臂连杆和大臂中间连接板子限位下的最小夹角
#define BoomPitch1_Critical2 2.01f // 临界角度值
#define DeltaAngle_Limit2Min 0.70f

typedef struct
{
    float BoomYaw;
    float BoomPitch1;
    float BoomPitch2;
} BoomMotor_s;

typedef struct
{
    float ForearmYaw;
    float ForearmPitch;
    float ForearmRoll;
} ForearmMotor_s;

typedef struct
{
    BoomMotor_s AngleNow;               // 不可以用JScope查看此变量
    BoomMotor_s SpeedNow;               // 不可以用JScope查看此变量
    BoomMotor_s AngleHope;              // 设定值规划前的位置设定值
    BoomMotor_s AngleNowFilter;         //
    BoomMotor_s AngleNow_CrossCircle;   // 当前角度（跨圈）
    BoomMotor_s SpeedNowFilter;         //
    BoomMotor_s AngleSetPlan;           // 设定值规划之后
    BoomMotor_s SpeedFeedforward;       // 设定值规划后的速度前馈
    BoomMotor_s SpeedFeedforwardFilter; // 设定值规划后的速度前馈

    BoomMotor_s Compensation;  // 力矩补偿
    BoomMotor_s MotorCtrl_Out; // 设定值规划后的速度前馈
} BoomState_Data_s;

typedef struct
{
    ForearmMotor_s AngleNow;
    ForearmMotor_s AngleHope;
    ForearmMotor_s AngleHopeOld;
    ForearmMotor_s AngleNowFilter;
    ForearmMotor_s SpeedNow;
    ForearmMotor_s SpeedNowFilter;
    ForearmMotor_s AngleSetPlan;
    ForearmMotor_s SpeedFeedforward;
    ForearmMotor_s Compensation;  // 力矩补偿
    ForearmMotor_s MotorCtrl_Out; // 设定值规划后的速度前馈
} ForearmState_Data_s;

typedef struct
{
    VectorXYZ_Str ArmPos_Now;        // 机械臂末端坐标当前值
    VectorXYZ_Str ArmPos_Set;        // 机械臂末端坐标设定值
    VectorXYZ_Str ArmPos_SetOld;     // 机械臂末端坐标上一次设定值
    VectorXYZ_Str ArmPos_SetPlan;    // 机械臂末端坐标设定规划值
    VectorXYZ_Str ArmPos_SetPlanOld; // 机械臂末端坐标上一次设定规划值

    VectorXYZ_Str BoomPos_Now;    // 大机械臂末端坐标当前值
    VectorXYZ_Str BoomPos_Set;    // 大机械臂末端坐标设定值
    VectorXYZ_Str BoomPos_SetOld; // 大机械臂末端坐标设定值
} ArmPosState_s;

typedef __packed struct
{
    rt_uint8_t BoomYaw_Iflimit : 1;      // 大机械臂Yaw
    rt_uint8_t BoomPitch1_Iflimit : 1;   // 大机械臂Pitch1
    rt_uint8_t BoomPitch2_Iflimit : 1;   // 大机械臂Pitch2
    rt_uint8_t ForearmYaw_Iflimit : 1;   // 小机械臂Yaw
    rt_uint8_t ForearmPitch_Iflimit : 1; // 小机械臂pitch
    rt_uint8_t ForearmRoll_Iflimit : 1;  // 小机械臂roll
    rt_uint8_t BoomPitch_Iflimit : 1;    // 大机械臂两个pitch的夹角
} Arm_Limit_s;

/****************************************机械臂姿态解算***************************************/

/**
 * @brief 用机械臂的角度解算小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量
 * @param Angle1
 * @param BoomYaw
 * @param out
 */
void ArmAngle2ForearmVector1(ForearmMotor_s *Angle1,
                             float BoomYaw,
                             VectorXYZ_Str *out);

/**
 * @brief 用机械臂的角度解算小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量（忽略机械z向变动）
 * @param Angle1
 * @param BoomYaw
 * @param out
 */
void ArmAngle2ForearmVector2(ForearmMotor_s *Angle1,
                             float BoomYaw,
                             VectorXYZ_Str *out);

/**
 * @brief 用机械臂的角度和大臂位姿，解算机械臂小臂末端的位置坐标
 * @param Angle1
 * @param BoomYaw
 * @param BoomPosin
 * @param out
 */
void ArmAngle2ArmPos(ForearmMotor_s *Angle1,
                     float BoomYaw,
                     VectorXYZ_Str *BoomPosin,
                     VectorXYZ_Str *ArmPos_out);
/**
 * @brief 小机械臂的末端相对于小机械臂Yaw轴的顶部坐标矢量
 * @param ForearmPos
 * @param Angle
 * @param out
 * @return  1 正常 0：非正常
 */
int ForearmPos2ForearmVector(VectorXYZ_Str *ForearmPos,
                             ForearmMotor_s *Angle,
                             VectorXYZ_Str *out);

/**
 * @brief 小机械臂Yaw轴的顶部相对于大机械臂始端坐标矢量解算
 * @param Angle
 * @param VectorOut
 */
void BoomAngle2BoomPos(BoomMotor_s *Angle, VectorXYZ_Str *VectorOut);

/**
 * @brief 根据Forearm末端位置解算小机械臂Yaw轴的顶部
 * @param ForearmPos
 * @param Angle
 * @param BoomPos
 * @return  1 正常 0：非正常
 */
int Forearm2BoomPos(VectorXYZ_Str *ForearmPos,
                    ForearmMotor_s *Angle,
                    VectorXYZ_Str *BoomPos);

/**
 * @brief 根据电机角度和转速，解算小机械臂Yaw轴的顶部三维速度矢量
 * @param Angle
 * @param Speed
 * @param BoomPos
 * @param BoomSpe
 */
void BoomMot2BoomSpe(BoomMotor_s *Angle,
                     BoomMotor_s *Speed,
                     VectorXYZ_Str *BoomPos,
                     VectorXYZ_Str *BoomSpe);

/**
 * @brief 根据小机械臂Yaw轴的顶部解算Boom电机角度值
 * @param BoomPos
 * @param LastYaw
 * @param Angle_Out
 */
void BoomPos2BoomMotAngle(VectorXYZ_Str *BoomPos,
                          float LastYaw,
                          BoomMotor_s *Angle_Out);

/**
 * @brief 根据小机械臂Yaw轴的顶部解算当前Boom电机Yaw角度值
 * @param BoomPos
 * @param LastYaw
 * @param Angle_Out
 */
void BoomPos2BoomYawangle(VectorXYZ_Str *BoomPos,
                          float LastYaw,
                          float *Angle_Out);

/**
 * @brief 由Boom末端速度求解Boom电机的速度值
 */
void BoomSpe2BoomMotSpe(VectorXYZ_Str *BoomPos,
                        BoomMotor_s *BoomAngle,
                        VectorXYZ_Str *BoomSpe,
                        BoomMotor_s *Spe_Out);

/**
 * @brief 通过大机械臂角度解算图传位置坐标
 * @param Angle
 * @param Imagepos_out
 */
void ImagePos_Resolve(BoomMotor_s *Angle,
                      VectorXYZ_Str *Imagepos_out);

/**
 * @brief BoomPos设定值逼近算法
 * @param BoomPos_Set_in
 * @param BoomPos_SetOld
 * @param BoomYaw
 * @param BoomPos_Set_out
 */
int BoomPos_Limit(VectorXYZ_Str *BoomPos_Set_in,
                  VectorXYZ_Str *BoomPos_SetOld,
                  float BoomYaw,
                  VectorXYZ_Str *BoomPos_Set_out);

/*逼近算法*/
#define IFLIMIT_NUM_MAX 5 // 逼近次数上限
/**
 * @brief 机械限幅判断函数
 * @brief 根据机械臂角度判断是否超限
 * @param BoomAngle
 * @param ForearmAngle
 * @param Arm_Limit_data
 * @return int 1 则表示超限 0 则表示没有超限
 */
int ArmMachinelimit_If(BoomMotor_s *BoomAngle,
                       ForearmMotor_s *ForearmAngle,
                       Arm_Limit_s *Arm_Limit_data);

/*****************************************扭矩补偿解算***************************************/
/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 * @param   period            缓启动的时长 单位：ms
 * @return  int
 */
int ArmComp_Slow(int period,
                 BoomMotor_s *boom_anglenow,
                 ForearmMotor_s *forearm_anglenow,
                 BoomMotor_s *boom_compout,
                 ForearmMotor_s *forearm_compout);

/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 */
void ArmComp(BoomMotor_s *boom_anglenow,
             ForearmMotor_s *forearm_anglenow,
             BoomMotor_s *boom_compout,
             ForearmMotor_s *forearm_compout);

/**
 * @brief  姿态解算数据包序列号读取
 */
extern int Arm_Pos_Datanum_Find(void);

#endif /*#__FUNC_ARM_POS_H__*/
