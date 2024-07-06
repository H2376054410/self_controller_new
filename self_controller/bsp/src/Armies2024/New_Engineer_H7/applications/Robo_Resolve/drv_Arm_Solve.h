#ifndef __FUNC_ARM_POS_H__
#define __FUNC_ARM_POS_H__

#include <rtthread.h>
#include "drv_Setplanning3D.h"
// #include "func_Dataserver.h"
/*
 *x轴正方向水平向右
 *y轴正方向水平向前
 *z轴正方向竖直向上
 */

/*大小臂之间初始坐标补偿*/
#define BoomToFoerarm_y 0.07f  // 大臂螺丝到小臂yaw间y轴方向的坐标点平移
#define BoomToFoerarm_z 0.056f // 大臂螺丝到小臂yaw间z轴方向的坐标点平移

/*大机械臂机械尺寸宏定义，单位：m,精确度：1mm，即0.001*/
#define Motor_Y0_P1 0.196f    // 竖直杆长度
#define Motor_P2_P1_L 0.260f  // 第二根杆长度pitch1
#define Motor_P2_Y1_L 0.200f  // 第三根杆长度pitch2
#define Motor_Y1_p3_L 0.068f  // 第四根杆yaw->pitch（末端执行器）
#define Pitch3_Last_L 0.100f  // 第五根杆pitch_>小臂末端点
#define Last_Gold_L 0.100f    // 小臂末端点到矿石中心距离
#define Motorp3_Gold_L 0.200f // pitch电机到矿石中心距离
#define g0_link       0.05f   //平行四连杆和电机相连杆的长度
/*小机械臂机械尺寸宏定义，单位：m,精确度：1mm，即0.001*/

#define Forearm_YtoP_L 0.052f    // yaw轴到pitch轴竖直距离
#define ForearmMaster_P_L 0.152f // pitch轴到矿石两个吸盘中心的距离

/*图传尺寸宏定义，单位*/
#define BOOMSLAVE2IMAGE_L 0.155f  // 第二个pitch的联动臂3到图传按照位置的臂长
#define IMAGE_LEVEL_L 0.2061f     // 机械臂到安装位置的水平距离
#define IMAGE_VERTIUCAL_L 0.0015f // 机械臂到安装位置的竖直距离
/*********************************扭矩补偿*********************************/
#define gravity_AC 9.8f // 重力加速度
/*机械臂等效重力，单位为：N，精确度：0.01N，即0.01*/
#define Motor_Y0P1_Glink 1.15f // 竖直杆平行四边形结构重力
#define Motor_Y0P1_GL 12.15f    // 竖直杆的重力
#define Motor_P1P2_Glink 1.0f  // 控制pitch1电机到控制pitch2电机的杆的重力
#define Motor_P2Y1_Glink 0.96f // 控制pitch2电机到控制y1电机杆的重力
#define Motor_Y1P3_Glink 0.2f  // 控制小臂y1到p3的杆的重力
#define Motor_P3Last_Glink 1.2f // 小机械臂pitch轴之后的等效重力(估算)
#define Gold_G 0.14f            // 矿石的重力
#define moduan 1.2f            //整个末端执行器重量
#define Gp2_moduan 2.25f
/*机械臂等效力臂，单位：m,精确度：1mm，即0.001*/
#define Boom_Forearm_EquiL2 0.120f // 小机械臂等效重心与大机械臂末端的距离
// #define ForearmPitch_EquiL3 0.100f // 小机械臂pitch轴之后等效重心到pitch轴的距离

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
#define BoomPitch2_M3_Ratio Conversion_ratio_4310
#define ForearmPitch_M5_Ratio Conversion_ratio_4310

/************************控制角度限度******************************/
#define BoomYaw_AngleMax (179.0f/180.0f)*PI // 2024没校准
#define BoomYaw_AngleMin (0.5f/180.0f)*PI
#define BoomPitch1_AngleMax (78.0f/180.0f)*PI
#define BoomPitch1_AngleMin (0.5f/180.0f)*PI
#define BoomPitch2_AngleMax (135.0f/180.0f)*PI
#define BoomPitch2_AngleMin (0.5f/180.0f)*PI
#define BoomMove_AngleMax 0.1
#define BoomMove_AngleMin -220

#define ForearmYaw_AngleMax (180.0f/57.3f)
#define ForearmYaw_AngleMin (0.05f/57.3f)
#define ForearmPitch_AngleMax (90.0f/57.3f)
#define ForearmPitch_AngleMin (-90.0f/57.3f)
#define ForearmRoll_AngleMax (90.0f/57.3f)
#define ForearmRoll_AngleMin (-90.0f/57.3f)

/************************机械角度限幅******************************/
#define BoomYaw_LimitMax BoomYaw_AngleMax
#define BoomYaw_LimitMin BoomYaw_AngleMin
#define BoomPitch1_LimitMax BoomPitch1_AngleMax
#define BoomPitch1_LimitMin BoomPitch1_AngleMin
#define BoomPitch2_LimitMax BoomPitch2_AngleMax
#define BoomPitch2_LimitMin BoomPitch2_AngleMin
#define BoomMove_LimitMax BoomMove_AngleMax
#define BoomMove_LimitMin BoomMove_AngleMin

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
    float BoomMove;
} BoomMotor_s;

typedef struct
{
    float ForearmYaw;
    float ForearmPitch;
    float ForearmRoll;
} ForearmMotor_s;

typedef struct 
{
    float arm_yaw;
    float arm_pitch;
    float arm_roll;
}Arm_Angle_Gold;

typedef struct
{
    BoomMotor_s AngleNow;               // 不可以用JScope查看此变量
    BoomMotor_s SpeedNow;               // 不可以用JScope查看此变量
    BoomMotor_s AngleHope;              // 目标角度值
    BoomMotor_s AngleHopeOld;           //上一次目标角度值
    BoomMotor_s AngleNowFilter;         //
    BoomMotor_s AngleNow_CrossCircle;   // 当前角度（跨圈）
    BoomMotor_s SpeedNowFilter;         //
    BoomMotor_s AngleSetPlan;           // 设定值规划之后
    BoomMotor_s SpeedFeedforward;       // 设定值规划后的速度前馈
    BoomMotor_s Compensation;  // 力矩补偿
    BoomMotor_s MotorCtrl_Out; // 设定值规划后的速度前馈
} BoomState_Data_s;
typedef struct
{
    ForearmMotor_s Data;           
    uint8_t IfConvert;
} Forearm_Mid_Data;
typedef struct
{
	  Forearm_Mid_Data AngleNow;
	  Forearm_Mid_Data SpeedNow;
	  Forearm_Mid_Data AngleSetPlan;
}Forearm_Mid_Data_s;


typedef struct
{
    BoomMotor_s Data;          
    uint8_t IfConvert;
} Boom_Mid_Data;
typedef struct
{
    Boom_Mid_Data AngleNow;
	  Boom_Mid_Data SpeedNow;
	  Boom_Mid_Data AngleSetPlan;
}Boom_Mid_Data_s;

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
    ForearmMotor_s SpeedHope;
} ForearmState_Data_s;

typedef struct
{
    Arm_Angle_Gold AngleNow;
    Arm_Angle_Gold AngleHope;
    Arm_Angle_Gold AngleHopeOld;
    Arm_Angle_Gold AngleNowFilter;
    Arm_Angle_Gold SpeedNow;
    Arm_Angle_Gold SpeedNowFilter;
    Arm_Angle_Gold AngleSetPlan;
    Arm_Angle_Gold SpeedFeedforward;
    Arm_Angle_Gold Compensation;  // 力矩补偿
    Arm_Angle_Gold MotorCtrl_Out; // 设定值规划后的速度前馈
} Gold_Data_s;


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
    rt_uint8_t BoomMove_Iflimit : 1;     // 大机械臂底座移动
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


int LimitIfIn(float *in,float a1_now,float a2_set);
/**
 * @brief 根据小机械臂Yaw轴的顶部解算当前Boom电机Yaw角度值
 * @param BoomPos
 * @param LastYaw
 * @param Angle_Out
 */
void BoomPos2BoomYawangle(VectorXYZ_Str *BoomPos,
                          float LastYaw,
                          float *Angle_Out);
/*机械臂输入输出结构体定义*/
typedef union
{
    struct
    {
        float x;
        float y;
        float z;
        float pitch;
        float yaw;
    } arm_end;
    float data[5];
} arm_output;

typedef union
{
    struct
    {
        float th0;
        float th1;
        float th2;
        float th3;
        float th4;
    } arm_begin;
    float data[5];
} arm_input;
/**
 * @brief 求前三个机械臂dangxiaweizhi1
 * 
 */
void Three_Postion_Now(VectorXYZ_Str *Pos_Now,BoomMotor_s *Boom_Data);

/**
 * @brief 求前三个机械臂到达的位置
 * 
 */
void Three_Postion(VectorXYZ_Str *Pos_Hope,VectorXYZ_Str *Pos_Now,BoomMotor_s *Boom_Data);
/**
* @brief 实际角度转化为算法角度
 */
void True_To_Compute(BoomMotor_s *Boom_Data,ForearmMotor_s *Forearm_Data,arm_input *output);
/**
* @brief 算法角度转化为实际角度
 */
void Compute_To_True(BoomMotor_s *Boom_Data,ForearmMotor_s *Forearm_Data,arm_input *output,float roll);
/**
 * @brief 机械臂电机限幅判断
 * @param arm_get 解算得到的电机的角度
 */
int Arm_Limit_Determine(arm_input *arm_get);
/**
 * @brief   正运动学解算，输入是弧度制，输出是三个轴的位置和pitch,yaw
 * @param   input   五个电机的角度
 * @return  output
 */
arm_output forward2(arm_input *input);
/**
 * @brief 辅助雅可比矩阵的函数
 * @param u0
 * @param u1
 * @return float
 */
static float rt_powd_snf(float u0, float u1);
// 获取机械臂正运动学的雅可比矩阵
/**
 * @brief 根据当前的位姿求解当前的雅可比矩阵求解各个关节电机角度
 * @param J 雅可比矩阵
 * @param input 输入的各个关节电机角度
 */
void get_J(float J[5][5], arm_input *input);
/**
 * @brief 用于解算的矩阵结构体初始化
 *
 */
void Calculate_Struct_Init(void);
/**
 * @brief 存储三维设定值规划速度到arm_output结构体中
 *
 */
void Set_Plan_To_Input(VectorXYZ_Str *Pos_out,Arm_Angle_Gold *Angle_Out,arm_output *str);
/**
 * @brief 根据输入位姿和输出位姿进行逆解算输出角度
 * @param th1 输入的角度结构体
 * @param pos_hop 目标机械臂位姿
 */
int Angle_Inverse(arm_input th1, arm_output pos_hop,arm_input *out,float *th2_now);

/**
 * @brief 转化机械臂设定位置到arm_output中
 *
 */
void Arm_Pos_Storage(VectorXYZ_Str *Pos_out,
                     ForearmMotor_s *Forearm_out, arm_output *mid);
/**
 * @brief 转化arm_input结构体数据到机械臂角度数据中
 *
 */
void Angle_Arm_Storage(Boom_Mid_Data_s *Boom_out,
                       Forearm_Mid_Data_s *Forearm_out, arm_input *mid);

/**
 * @brief 转化机械臂角度数据到arm_input结构体中
 *
 */
void Arm_Angle_Storage(Boom_Mid_Data_s *Boom_out,
                       Forearm_Mid_Data_s *Forearm_out, arm_input *mid);
/*逼近算法*/
#define IFLIMIT_NUM_MAX 5 // 逼近次数上限

/**
* @brief 判断机械臂是否缓启动完成
* @param arm_set 设定的角度(角度制度)
* @param Boom_state 大机械臂的当前角度
* @param ForearmMotor_s 小机械臂的当前角度
 */
int Arm_Start_Determine(arm_input *arm_set,BoomMotor_s *Boom_state,ForearmMotor_s *Forearm_state);
/**
* @brief 为arm_input结构体赋值
 */
void Arminput_Setvalue(arm_input *arm_set,float a,float b,float c,float d,float e);
/**
* @brief 初始化设定SETPLAN
 */
void Arm_Start_SetPlan(arm_input *arm_set,BoomMotor_s *Boom_state,ForearmMotor_s *Forearm_state);
/**
*@brief 控制限幅
*/
void Control_Limit(BoomMotor_s *Boom_Data,ForearmMotor_s *Forearm_Data,Arm_Limit_s *Arm_Limit_data);
/**
* @brief 存储当前的关节电机速度
 *
 */
void Storage_Speed_Now(ForearmMotor_s *forearm,BoomMotor_s *boom,arm_input *input1);
/**
 * @brief 将解算得到的当前位置存入当前位置的结构体中
 *
 */
void Solve_Pos_To_Now(arm_output *solve,VectorXYZ_Str *data);
/**
*@brief 大机械臂弧度转化为角度贴上标签
*/
void Boom_Fact_Angle(Boom_Mid_Data_s *BoomMath,BoomMotor_s *BoomFact);

/*
*@brief 小机械臂pitch和roll目标角度转化成实际值
*@brief 锥齿轮数据转化 两个电机同向同速控制pitch，反向同速控制roll
*/
void Forearm_Old_Angle(ForearmMotor_s *Forearmangle_New,ForearmMotor_s *Forearmangle_Old);
/*
*@brief 小机械臂pitch和rollshiji角度转化成suanfa
*@brief 锥齿轮数据转化 两个电机同向同速控制pitch，反向同速控制roll
*/
void Forearm_Suanfa_Angle(ForearmMotor_s *ForearmMath,ForearmMotor_s *ForearmFact);
/**
*@brief 小机械臂pitch和roll目标角度转化成实际值
*@brief 锥齿轮数据转化 两个电机同向同速控制pitch，反向同速控制roll
*/
void Forearm_Fact_Angle(Forearm_Mid_Data_s *ForearmMath,ForearmMotor_s *ForearmFact);
int Comp_SlowStart1(int dt, int period, float *out);
/*****************************************扭矩补偿解算***************************************/
/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 * @param   period            缓启动的时长 单位：ms
 * @return  int
 */
void ArmComp_Slow(int period,int *SlowStart_flag,
                 BoomState_Data_s *Boom_Slow,
                 ForearmState_Data_s *Forearm_Slow);


/**
 * @brief  姿态解算数据包序列号读取
 */
extern int Arm_Pos_Datanum_Find(void);
extern int time_flag;
#endif /*#__FUNC_ARM_POS_H__*/
