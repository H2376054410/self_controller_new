/**
 * @file mod_RoboStateCtrl.c
 * @brief 工程机器人模拟量数据处理文件
 * @author mylj
 * @version 1.0
 * @date 2023-03-16
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */

#include "mod_RoboStateCtrl.h"
#include "Cpu_reset.h"
#include "drv_utils.h"
#include "drv_remote.h"
#include "drv_Vector.h"
#include "drv_Arm_Solve.h"
#include "drv_ExactSmooth.h"
#include "drv_RemoteCtrl_data.h"
#include "func_Dataserver.h"
#include "func_Gpio_Ctrl.h"
#include "func_MusicPlayer.h"
#include "func_ArmMotor_Ctrl.h"
#include <stddef.h>
#include <stdint.h>

/*平滑*/
static ExactSmth_CTRL_S ArmX_Filter, ArmY_Filter, ArmZ_Filter;               // 机械臂末端坐标数据（大机械臂三自由度）
static ExactSmth_CTRL_S ArmMove_Filter;                                      //底部平移数据
static ExactSmth_CTRL_S ArmYaw_Filter, ArmPitch_Filter, ArmRoll_Filter;      // 小机械臂Yaw，Pitch、Roll数据
static ExactSmth_CTRL_S ChassisX_Filter, ChassisY_Filter, ChassisYaw_Filter; // 用于对底盘的 X、Y和Yaw轴方向数据进行滤波
static ExactSmth_CTRL_S ImagePitch_Filter,ImageYaw_Filter,ImageHigh_Filter;// 图传电机pitch,yaw,roll方向的数据进行滤波

/**
 * @brief 获取滤波器结构体
 * @author fwlh
 * @author mylj
 * @param  filter           指定需要获取的滤波器
 * @return void*            返回指向滤波器的指针
 */
void *Get_Filter(Filter_Enum filter)
{
    switch (filter)
    {
    case Arm_X:
        return (void *)&ArmX_Filter;
    case Arm_Y:
        return (void *)&ArmY_Filter;
    case Arm_Z:
        return (void *)&ArmZ_Filter;
    case Arm_Move:
        return (void *)&ArmMove_Filter;
    case Arm_Yaw:
        return (void *)&ArmYaw_Filter;
    case Arm_Pitch:
        return (void *)&ArmPitch_Filter;
    case Arm_Roll:
        return (void *)&ArmRoll_Filter;
    case Chassis_X:
        return (void *)&ChassisX_Filter;
    case Chassis_Y:
        return (void *)&ChassisY_Filter;
    case Chassis_Yaw:
        return (void *)&ChassisYaw_Filter;
    case ImagePitch_Ctrl:
        return (void *)&ImagePitch_Filter;
    case ImageYaw_Ctrl:
        return (void *)&ImageYaw_Filter;
    case ImageHigh_Ctrl:
        return (void *)&ImageHigh_Filter;
    default:
        return NULL;
    }
}
/*****************************数据判0*****************************/

/**
 * @brief 小机械臂角度增量
 * @param in
 * @return int
 */
int ForearmAngleAdd_ZeroIf(ForearmMotor_s *in, float tolerance)
{
    if ((SQUARE(in->ForearmYaw) +
         SQUARE(in->ForearmPitch) +
         SQUARE(in->ForearmRoll)) <
        SQUARE(tolerance))
        return 1; // 当平方和小于一定值的时候，判断为0
    else
        return 0;
}

/****************************结构体初始化**************************/

/**
 * @brief 底盘遥控状态结构体初始化
 * @param ChassisSpeState
 */
void ChassisSpeRemote_Struct_Init(ChassisMotion_t *ChassisSpeState)
{
    ChassisSpeState->vel.x = 0;
    ChassisSpeState->vel.y = 0;
    ChassisSpeState->angvel = 0;
}

/**
 * @brief 机械臂遥控状态结构体初始化
 * @param ArmPosState
 */
void ArmPosRemote_Struct_Init(ArmPos_Remote_s *ArmState)
{
    ArmState->Pos.x = 0;
    ArmState->Pos.y = 0;
    ArmState->Pos.z = 0;

    ArmState->Angle.arm_roll = 0;
    ArmState->Angle.arm_pitch = 0;
    ArmState->Angle.arm_yaw = 0;

    ArmState->Boommove = 0;
}

/****************************存取矿石相关**************************/

/**
 * @brief 存放矿石
 */
void Ore_Drawing(void)
{
    SuckerState_Set(main_beng_close);
}

/**
 * @brief 抓取矿石
 */
void Ore_Grabbing(void)
{
    
    SuckerState_Set(main_beng_open);

}

/**
 * @brief 机械臂初始状态
 */
void ArmPosState_Init(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(ImageFollow_State, Write, (void *)Notfollowing_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)ArmPosInit);         // 机械臂初始状态

    ArmState_Set.Pos.x = ARMPOS_INIT_X;
    ArmState_Set.Pos.y = ARMPOS_INIT_Y;
    ArmState_Set.Pos.z = ARMPOS_INIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_INIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_INIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_INIT_ROLL;
		ArmState_Set.Boommove = -100.0f;
	
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂底盘状态
 */
void ArmPosState_Chassis(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(ImageFollow_State, Write, (void *)Notfollowing_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);       // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_CHASSIS_X;
    ArmState_Set.Pos.y = ARMPOS_CHASSIS_Y;
    ArmState_Set.Pos.z = ARMPOS_CHASSIS_Z;

    ArmState_Set.Angle.arm_yaw = ARMPOS_CHASSIS_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_CHASSIS_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_CHASSIS_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂三级矿状态
 */
void ArmPosState_3levelOre_Init(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(Ore_State, Write, (void *)AimOre3level_Mode);      //
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_3LEVELORE_X;
    ArmState_Set.Pos.y = ARMPOS_3LEVELORE_Y;
    ArmState_Set.Pos.z = ARMPOS_3LEVELORE_Z;

    ArmState_Set.Angle.arm_yaw = ARMPOS_3LEVELORE_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_3LEVELORE_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_3LEVELORE_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂左四级矿状态
 */
void ArmPosState_4levelOreL_Init(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(Ore_State, Write, (void *)AimOre4level_L_Mode);    //
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_4LEVELORE_L_X;
    ArmState_Set.Pos.y = ARMPOS_4LEVELORE_L_Y;
    ArmState_Set.Pos.z = ARMPOS_4LEVELORE_L_Z;

    ArmState_Set.Angle.arm_yaw = ARMPOS_4LEVELORE_L_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_4LEVELORE_L_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_4LEVELORE_L_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂右四级矿状态
 */
void ArmPosState_4levelOreR_Init(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(Ore_State, Write, (void *)AimOre4level_R_Mode);    //
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_4LEVELORE_R_X;
    ArmState_Set.Pos.y = ARMPOS_4LEVELORE_R_Y;
    ArmState_Set.Pos.z = ARMPOS_4LEVELORE_R_Z;

    ArmState_Set.Angle.arm_yaw = ARMPOS_4LEVELORE_R_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_4LEVELORE_R_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_4LEVELORE_R_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂银矿初始状态
 */
void ArmPosState_SilverInit(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(SliverOre_State, Write, (void *)SliverAim_Mode);   // 银矿对准状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_SILVERINIT_X;
    ArmState_Set.Pos.y = ARMPOS_SILVERINIT_Y;
    ArmState_Set.Pos.z = ARMPOS_SILVERINIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_SILVERINIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_SILVERINIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_SILVERINIT_ROLL;
    ArmState_Set.Boommove=-100.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂银矿吸取状态
 */
void ArmPosState_SilverGrip(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(SliverOre_State, Write, (void *)SliverGrip_Mode); // 银矿对准状态

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_SILVERINIT_X;
    ArmState_Set.Pos.y = ARMPOS_SILVERINIT_Y;
    ArmState_Set.Pos.z = ARMPOS_SILVERINIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_SILVERINIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_SILVERINIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_SILVERINIT_ROLL;
    ArmState_Set.Boommove=-100.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂银矿上拔状态
 */
void ArmPosState_SilverPullup(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(SliverOre_State, Write, (void *)SliverPullup_Mode); // 银矿上拔状态

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_SILVERPULLUP_X;
    ArmState_Set.Pos.y = ARMPOS_SILVERPULLUP_Y;
    ArmState_Set.Pos.z = ARMPOS_SILVERPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_SILVERPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_SILVERPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_SILVERPULLUP_ROLL;
    ArmState_Set.Boommove=-100.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂正金矿初始状态
 */
void ArmPosState_FrontGoldInit(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(GoldOre_State, Write, (void *)FrontGoldAim_Mode);  // 金矿对准状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FRONTGOLDINIT_X;
    ArmState_Set.Pos.y = ARMPOS_FRONTGOLDINIT_Y;
    ArmState_Set.Pos.z = ARMPOS_FRONTGOLDINIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FRONTGOLDINIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FRONTGOLDINIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_FRONTGOLDINIT_ROLL;
    ArmState_Set.Boommove=-200.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂左侧金矿初始状态
 */
void ArmPosState_LeftSideGoldInit(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(GoldOre_State, Write, (void *)Side_LeftGoldAim_Mode);   // 金矿对准状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_SIDEGOLDINIT_X;
    ArmState_Set.Pos.y = ARMPOS_SIDEGOLDINIT_Y;
    ArmState_Set.Pos.z = ARMPOS_SIDEGOLDINIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_SIDEGOLDINIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_SIDEGOLDINIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_SIDEGOLDINIT_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}
/**
 * @brief 机械臂右侧金矿初始状态
 */
void ArmPosState_RightSideGoldInit(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(GoldOre_State, Write, (void *)Side_RightGoldAim_Mode);   // 金矿对准状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = R_ARMPOS_SIDEGOLDINIT_X;
    ArmState_Set.Pos.y = R_ARMPOS_SIDEGOLDINIT_Y;
    ArmState_Set.Pos.z = R_ARMPOS_SIDEGOLDINIT_Z;
    ArmState_Set.Angle.arm_yaw = R_ARMPOS_SIDEGOLDINIT_YAW;
    ArmState_Set.Angle.arm_pitch = R_ARMPOS_SIDEGOLDINIT_PITCH;
    ArmState_Set.Angle.arm_roll = R_ARMPOS_SIDEGOLDINIT_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}


/**
 * @brief 机械臂左侧金矿外拔状态
 */
void ArmPosState_SideGoldPullup(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(GoldOre_State, Write, (void *)GoldPullOut_Mode); // 金矿上拔状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_SIDE_GOLDPULLUP_X;
    ArmState_Set.Pos.y = ARMPOS_SIDE_GOLDPULLUP_Y;
    ArmState_Set.Pos.z = ARMPOS_SIDE_GOLDPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_SIDE_GOLDPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_SIDE_GOLDPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_SIDE_GOLDPULLUP_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}
/**
 * @brief 机械臂右侧金矿外拔状态
 */
void ArmPosState_RightSideGoldPullup(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(GoldOre_State, Write, (void *)GoldPullOut_Mode); // 金矿上拔状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = R_ARMPOS_SIDE_GOLDPULLUP_X;
    ArmState_Set.Pos.y = R_ARMPOS_SIDE_GOLDPULLUP_Y;
    ArmState_Set.Pos.z = R_ARMPOS_SIDE_GOLDPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = R_ARMPOS_SIDE_GOLDPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = R_ARMPOS_SIDE_GOLDPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = R_ARMPOS_SIDE_GOLDPULLUP_ROLL;

    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂正金矿上拔状态
 */
void ArmPosState_FrontGoldPullup(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(GoldOre_State, Write, (void *)GoldPullOut_Mode); // 金矿上拔状态

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FRONT_GOLDPULLUP_X;
    ArmState_Set.Pos.y = ARMPOS_FRONT_GOLDPULLUP_Y;
    ArmState_Set.Pos.z = ARMPOS_FRONT_GOLDPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FRONT_GOLDPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FRONT_GOLDPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_FRONT_GOLDPULLUP_ROLL;
   ArmState_Set.Boommove = -200.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂地上矿图传初始状态
 */
void ArmPosState_floorImageInit(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(FloorOre_State, Write, (void *)FloorOreReady_Mode); // 地上矿图传准备状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FLOORIMAGEINIT_X;
    ArmState_Set.Pos.y = ARMPOS_FLOORIMAGEINIT_Y;
    ArmState_Set.Pos.z = ARMPOS_FLOORIMAGEINIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FLOORIMAGEINIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FLOORIMAGEINIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_FLOORIMAGEINIT_ROLL;
    ArmState_Set.Boommove=-100;
    ArmPosStateSet_ABS(&ArmState_Set);
}
/**
 * @brief 机械臂地上矿图传初始状态拓展
 */
void ArmPosState_floorImageInit_Expand(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(FloorOre_State, Write, (void *)FloorOreReady_Mode); // 地上矿图传准备状态
    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FLOORPULLUP_X;
    ArmState_Set.Pos.y = ARMPOS_FLOORPULLUP_Y;
    ArmState_Set.Pos.z = ARMPOS_FLOORPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FLOORPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FLOORPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = 1.57f;
    ArmState_Set.Boommove=-100;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂地上矿初始状态
 */
void ArmPosState_floorInit(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(FloorOre_State, Write, (void *)FloorOreReady_Mode); // 地上矿对准状态

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FLOORINIT_X;
    ArmState_Set.Pos.y = ARMPOS_FLOORINIT_Y;
    ArmState_Set.Pos.z = ARMPOS_FLOORINIT_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FLOORINIT_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FLOORINIT_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_FLOORIMAGEINIT_ROLL;
    ArmState_Set.Boommove=-100.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂转矿上抬状态
 */
void ArmPosState_floorPullup(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(FloorOre_State, Write, (void *)NoFloorOre_Mode); // 地上矿上拔模式

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FLOORPULLUP_X;
    ArmState_Set.Pos.y = ARMPOS_FLOORPULLUP_Y;
    ArmState_Set.Pos.z = ARMPOS_FLOORPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FLOORPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FLOORPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_FLOORPULLUP_ROLL;
    ArmState_Set.Boommove=-100.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}
/**
 * @brief 机械臂转矿上抬状态并且roll转90°
 */
void ArmPosState_floorPullup_Expand(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(FloorOre_State, Write, (void *)NoFloorOre_Mode); // 地上矿上拔模式

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FLOORPULLUP_X;
    ArmState_Set.Pos.y = ARMPOS_FLOORPULLUP_Y;
    ArmState_Set.Pos.z = ARMPOS_FLOORPULLUP_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FLOORPULLUP_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FLOORPULLUP_PITCH;
    ArmState_Set.Angle.arm_roll = 3.14f;
    ArmState_Set.Boommove=-100.0f;
    ArmPosStateSet_ABS(&ArmState_Set);
}

/**
 * @brief 机械臂转矿下放状态
 */
void ArmPosState_floorLaydown(void)
{
    ArmPos_Remote_s ArmState_Set;

    Robo_Control(FloorOre_State, Write, (void *)FloorOreLaydown_Mode); // 地上矿对准状态

    Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
    Robo_Control(ArmPosInit_State, Write, (void *)NoArmPosInit);    // 机械臂非初始状态

    ArmState_Set.Pos.x = ARMPOS_FLOORLAYDOWN_X;
    ArmState_Set.Pos.y = ARMPOS_FLOORLAYDOWN_Y;
    ArmState_Set.Pos.z = ARMPOS_FLOORLAYDOWN_Z;
    ArmState_Set.Angle.arm_yaw = ARMPOS_FLOORLAYDOWN_YAW;
    ArmState_Set.Angle.arm_pitch = ARMPOS_FLOORLAYDOWN_PITCH;
    ArmState_Set.Angle.arm_roll = ARMPOS_FLOORLAYDOWN_ROLL;
		
    ArmPosStateSet_ABS(&ArmState_Set);
}


/**********************************机器人速度增益设定*******************************/
/**
 * @brief 正常状态
 */
void RoboState_NormalGain(void)
{
    Robo_Control(Speed_State, Write, (void *)NormalSpeed_Mode); // 正常状态
    Robo_SpeedGain_Set(ROBO_SPEEDGAIN_NORM);
}

/**
 * @brief 加速状态
 */
void RoboState_Acceleration(void)
{
    Robo_Control(Speed_State, Write, (void *)Acceleration_Mode); // 加速状态
    Robo_SpeedGain_Set(ROBO_SPEEDGAIN_ACCE);
}

/**
 * @brief 减速状态
 */
void RoboState_Deceleration(void)
{

    Robo_Control(Speed_State, Write, (void *)Deceleration_Mode); // 正常状态
    Robo_SpeedGain_Set(ROBO_SPEEDGAIN_DCCE);
}

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
                             ImageMotor_s *ImageMotor_Set_ADD,
                             float GimbalYaw_in,
                             float SpeedGain,
                             ArmPos_Remote_s *A_out,
                             ImageMotor_s *ImageMotor_Set,
                             ChassisMotion_t *C_out,
                             float *GimbalYaw_out)
{
    A_out->Pos.x = SpeedGain * A_in->Pos.x;
    A_out->Pos.y = SpeedGain * A_in->Pos.y;
    A_out->Pos.z = SpeedGain * A_in->Pos.z;
    A_out->Boommove = SpeedGain * A_in->Boommove;

    A_out->Angle.arm_yaw = SpeedGain * A_in->Angle.arm_yaw;
    A_out->Angle.arm_pitch = SpeedGain * A_in->Angle.arm_pitch;
    A_out->Angle.arm_roll = SpeedGain * A_in->Angle.arm_roll;

    ImageMotor_Set->ImageHigh = SpeedGain * ImageMotor_Set_ADD->ImageHigh;
    ImageMotor_Set->ImagePitch = SpeedGain * ImageMotor_Set_ADD->ImagePitch;   

    C_out->vel.x = SpeedGain * C_in->vel.x;
    C_out->vel.y = SpeedGain * C_in->vel.y;
    C_out->angvel = SpeedGain * C_in->angvel;

    *GimbalYaw_out = SpeedGain * GimbalYaw_in;
}

/********************************滤波相关********************************/
/**
 * @brief 平滑滤波器清零设置
 * @param type
 */
void SmoothFilter_Restart(SmoothFilter_Restart_e type)
{
    switch (type)
    {
    case ChassisSmoothFilter:
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_X), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw), 0);
        break;
    case BoomSmoothFilter:
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_X), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Y), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Z), 0);
        break;
    case ForearmSmoothFilter:
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Move), 0);
        break;
    case AllSmoothFilter:
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_X), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Y), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Z), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Move), 0);

        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll), 0);

        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(ImagePitch_Ctrl), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(ImageYaw_Ctrl), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(ImageHigh_Ctrl), 0);                

        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_X), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y), 0);
        Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw), 0);
        break;

    default:
        break;
    }
}

/**
 * @brief 初始化控制过程中需要的部分平滑滤波器
 */
static int RoboCtrl_Filter_Init(void)
{
    Smooth_Init(&ArmX_Filter, 0, 300,0,0,0);
    Smooth_Init(&ArmY_Filter, 0, 300,0,0,0);
    Smooth_Init(&ArmZ_Filter, 0, 300,0,0,0);
    Smooth_Init(&ArmMove_Filter, 0, 300,0,0,0);

    Smooth_Init(&ArmYaw_Filter, 0, 300,0,0,0);
    Smooth_Init(&ArmPitch_Filter, 0, 300,0,0,0);
    Smooth_Init(&ArmRoll_Filter, 0, 300,0,0,0);

    Smooth_Init(&ChassisX_Filter, 0, 150,0,0,0);
    Smooth_Init(&ChassisY_Filter, 0, 150,0,0,0);
    Smooth_Init(&ChassisYaw_Filter, 0, 150,0,0,0);

    Smooth_Init(&ImagePitch_Filter, 0, 150,0,0,0);
    Smooth_Init(&ImageYaw_Filter, 0, 150,0,0,0);
    Smooth_Init(&ImageHigh_Filter, 0, 150,0,0,0);
    return 0;
}
INIT_APP_EXPORT(RoboCtrl_Filter_Init);

/*********************************模拟量控制数据********************************/
/**
 * @brief 对机器人的控制模式进行配置
 * @author mylj
 */
void RoboCtrl_State_Judge(void)
{
  if(Remote_Tanslate==1)
    {
    rt_uint8_t UI_Reset;(void)UI_Reset;
    if (RC_data.Remote_Data.s1 == 1)
    {
        // S1 拨杆在上
        switch (RC_data.Remote_Data.s2)
        {
        case 3: // S2 拨杆在中
                // 代表进入客户端模式
            if ((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) != Client_Mode)
            {
                Robo_Control(RoboCtrl_State, Write, (void *)Client_Mode);

                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零

                Robo_Control(RoboCtrl_RC_State, Write, (void *)RoboNoCtrl_Mode); // 无控制模式
//                Ore_Drawing();

                SoundDisplay_AddSound(Client_EQ); // 提示音
            }
            break;
        case 1: // S2 拨杆在上
                // 代表进入单独遥控器模式
            if ((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) != Remote_Mode)
            {
                Robo_Control(RoboCtrl_State, Write, (void *)Remote_Mode);

                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零

                Robo_Control(RoboCtrl_RC_State, Write, (void *)RoboNoCtrl_Mode); // 无控制模式
//							  SuckerState_Set(chu_beng_close);
//							  SuckerState_Set(main_fa_close);
                SoundDisplay_AddSound(Remote_EQ); // 提示音
            }
            break;
        case 2: // S2 拨杆在下
            // 代表进入无控制模式
            // 单片机重启
             if (RC_data.Remote_Data.ch0 < CPU_RESER_THRESHOLD1 &&
                 RC_data.Remote_Data.ch1 < CPU_RESER_THRESHOLD1 &&
                 RC_data.Remote_Data.ch2 > CPU_RESER_THRESHOLD2 &&
                 RC_data.Remote_Data.ch3 < CPU_RESER_THRESHOLD1)
             {
                 CPU_DelayReset(CPURESET_DALEYTIME);
             }

            // if ((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) != NoCtrl_Mode)
            // {
            //     Robo_Control(RoboCtrl_State, Write, (void *)NoCtrl_Mode);
            //     Robo_Control(RoboCtrl_RC_State, Write, (void *)RoboNoCtrl_Mode); // 无控制模式

            //     SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
            //     SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
            //     SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零

            //     Robo_Control(Push_State, Write, (void *)NoPush_Mode); // 推矿关闭
            //     Ore_Drawing();                                        // 关闭气泵

            //     UIReset_State_Data_Read(&UI_Reset);
            //     if (UI_Reset == (rt_uint8_t)UIReset_Mode1)
            //         UIReset_State_Data_Write((rt_uint8_t)UIReset_Mode2);
            //     else
            //         UIReset_State_Data_Write((rt_uint8_t)UIReset_Mode1);

            //     SoundDisplay_AddSound(EmptyCtrl_EQ); // 提示音
            // }
//								SuckerState_Set(main_beng_close);
//							  SuckerState_Set(left_fa_close);
            break;
        default:
            break;
        }
    }
    else if (RC_data.Remote_Data.s1 == 3)
    {
        // S1 拨杆在中
        switch (RC_data.Remote_Data.s2)
        {
        case 1: // S2 拨杆在上
                // 代表进入大机械控制模式
            if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
                ((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) != ArmPos_Mode))
            {
                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
							  
                Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式

                Robo_Control(RoboCtrl_RC_State, Write, (void *)ArmPos_Mode); // 机械臂位置控制模式
							   SuckerState_Set(main_beng_close);
//							   SuckerState_Set(left_close);
//							   SuckerState_Set(right_close);
							buzzer_set(0);
                SoundDisplay_AddSound(Boom_EQ);                              // 提示音
            }
            break;
        case 2: // S2 拨杆在下
                // 代表进入小机械控制模式
            if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
                ((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) != ArmAngle_Mode))
            {
                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
						   SuckerState_Set(main_beng_open);
//							   SuckerState_Set(left_open);
//							SuckerState_Set(right_open);
							buzzer_set(1);
                Robo_Control(ImageFollow_State, Write, (void *)Following_Mode); // 图传跟随模式
                Robo_Control(RoboCtrl_RC_State, Write, (void *)ArmAngle_Mode);  // 整机械臂控制模式
                SoundDisplay_AddSound(Forearm_EQ);                              // 提示音
            }
            break;
        case 3: // S2 拨杆在中
                // 代表进入无控制模式
            if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
                 (uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) != RoboNoCtrl_Mode)
            {
                SmoothFilter_Restart(ChassisSmoothFilter);                       // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);                          // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter);                       // 切换数据源时将滤波器清零
                Robo_Control(RoboCtrl_RC_State, Write, (void *)RoboNoCtrl_Mode); // 无控制模式

                SoundDisplay_AddSound(EmptyCtrl_EQ); // 提示音
            }
            break;
        default:
            break;
        }
    }
    else if (RC_data.Remote_Data.s1 == 2)
    {
        // S1 拨杆在下
        switch (RC_data.Remote_Data.s2)
        {
        case 1: // S2 拨杆在上
                // 代表进入底盘模式
            if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
                    ((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) != ChassisNormal_Mode) ||
                ((uint32_t)(Robo_Control(AirPump_State, Read, NULL)) != AirPumpState_Close))
//            if (((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) != ChassisNormal_Mode) ||
//                ((uint32_t)(Robo_Control(AirPump_State, Read, NULL)) != AirPumpState_Close))
            {
                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零

                Robo_Control(RoboCtrl_RC_State, Write, (void *)ChassisNormal_Mode); // 底盘菜单
                Robo_Control(ImageFollow_State, Write, (void *)Notfollowing_Mode);  // 图传不跟随模式
                // ArmPosState_Init();                                                 // 机械臂复位状态
							//   SuckerState_Set(main_beng_open);
							//   SuckerState_Set(main_fa_open);
                SoundDisplay_AddSound(Chassis_EQ); // 提示音
            }
            break;
        case 2: // S2 拨杆在下
                // 代表进入吸盘模式
            if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
                (((uint32_t)(Robo_Control(AirPump_State, Read, NULL)) != AirPumpState_Open) ||
                 ((uint32_t)(Robo_Control(Push_State, Read, NULL)) != PushReady_Mode)))
            {
                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零

              //  Ore_Grabbing();
              //  Robo_Control(Push_State, Write, (void *)PushReady_Mode); // 推矿准备模式

                SoundDisplay_AddSound(Sucker_EQ); // 提示音
            }
            break;
        case 3: // S2 拨杆在中
                // 代表进入无控制模式
//             if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
//                 ((uint32_t)(Robo_Control(Push_State, Read, NULL)) != NoPush_Mode))
//             {
//                 SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
//                 SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
//                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零

//                 Robo_Control(Push_State, Write, (void *)NoPush_Mode); // 推矿关闭

//                 SoundDisplay_AddSound(EmptyCtrl_EQ); // 提示音
//             }
            if (((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) == Remote_Mode) &&
                ((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) != ArmCtrl_Mode))    
                {
                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
                Robo_Control(RoboCtrl_RC_State, Write, (void *)ArmCtrl_Mode);  // 自定义控制器

                }        
            break;
        default:
            break;
        }
    }
    }
    else 
    {
     if ((uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL)) != Client_Mode)
            {
                Robo_Control(RoboCtrl_State, Write, (void *)Client_Mode);
                SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
                SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
                SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
                Robo_Control(RoboCtrl_RC_State, Write, (void *)RoboNoCtrl_Mode); // 无控制模式
                SoundDisplay_AddSound(Client_EQ); // 提示音
            }    
    }
}
float boom_move_remote=-0.0006;
float debug_yaokongqi=0;
CustCtrler_Data_s CustCtrler_Data_temp;
CustCtrler_Data_s CustCtrler_Data_temp_Last;
/**
 * @brief 设定平滑滤波设定数据
 * @author mylj
 */
void SmootFilterhData_Set(void)
{
    uint32_t CtrlState;
    uint32_t ExpandCtrlState;

    CtrlState = (uint32_t)(Robo_Control(RoboCtrl_State, Read, NULL));
    ExpandCtrlState = (uint32_t)(Robo_Control(ExpandCtrl_State, Read, NULL));

    if (CtrlState == Client_Mode)
    {
        switch ((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)))
        {

        case ChassisNormal_Mode:// 正常底盘模式（键盘控制）
            SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
            SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw),
                              GAIN_CHASSIS_KEY_ROTATE * RC_data.Mouse_Data.x_speed);
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_X),
                              -GAIN_CHASSIS_KEY_MOVE * (RC_data.Key_Data.W - RC_data.Key_Data.S));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y),
                              -GAIN_CHASSIS_KEY_MOVE * (RC_data.Key_Data.A - RC_data.Key_Data.D));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(ImagePitch_Ctrl),
                              -GAIN_FOREARM_REMOTE * (RC_data.Mouse_Data.y_speed));
            break;

        case ChassisFineTuning_Mode:// 微调底盘模式（鼠标控制）
            SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
            SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零

            if (ExpandCtrlState == EnExpand_Mode)//控制底盘yaw和图传pitch
            {
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw),
                                  GAIN_CHASSIS_MOUSE_ROTATE_Y * RC_data.Mouse_Data.x_speed);
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(ImagePitch_Ctrl),
                                  GAIN_FOREARM_REMOTE * (RC_data.Mouse_Data.y_speed));

                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_X), 0);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y), 0);
            }
            else//控制底盘xy
            {
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_X),
                                  GAIN_CHASSIS_MOUSE_MOVE_Y * RC_data.Mouse_Data.y_speed);
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y),
                                  GAIN_CHASSIS_MOUSE_MOVE_X * (RC_data.Mouse_Data.x_speed));
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw), 0);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(ImagePitch_Ctrl), 0);
            }
            break;
        case ArmAngle_Mode:  // 机械臂角度控制模式
            SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
            SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零

            if (ExpandCtrlState == EnExpand_Mode)//控制小臂ROLL姿态
            {
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll),
                                  GAIN_FOREARM_MOUSE_MOVE_ROLL * RC_data.Mouse_Data.x_speed);
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(ImageHigh_Ctrl),
                                  GAIN_FOREARM_MOUSE_MOVE_PITCH * RC_data.Mouse_Data.y_speed*10.0f);								
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw), 0);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch), 0);
            }
            else //控制小臂pitch和yaw姿态
            {
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw),
                                  -GAIN_FOREARM_MOUSE_MOVE_YAW * RC_data.Mouse_Data.x_speed);
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch),
                                  -GAIN_FOREARM_MOUSE_MOVE_PITCH * RC_data.Mouse_Data.y_speed);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll), 0);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(ImageHigh_Ctrl), 0);							
            }
            break;
        case ArmPos_Mode:// 机械臂末端坐标点控制模式
            SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
            if (ExpandCtrlState == EnExpand_Mode)//控制z和滑台
            {
				Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Move),
                  -GAIN_BOOM_MOUSE_MOVE_Z * RC_data.Mouse_Data.x_speed*350.0f);
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Z),
                                  -GAIN_BOOM_MOUSE_MOVE_Z * RC_data.Mouse_Data.y_speed);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_X), 0);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Y), 0);

            }
            else //控制x和y
            {
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_X),
                                  -GAIN_BOOM_MOUSE_MOVE_X * RC_data.Mouse_Data.y_speed);
                Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Y),
                                  GAIN_BOOM_MOUSE_MOVE_Y * RC_data.Mouse_Data.x_speed);
                Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Z), 0);
								Smooth_SetData_Restart((ExactSmth_CTRL_S *)Get_Filter(Arm_Move), 0);
            }
            break;
       case ArmCtrl_Mode:                             // 自定义控制器
//            SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零、
//            CustCtrler_Data_temp_Last = CustCtrler_Data_temp;
//            CustCtrler_Data_Read(&CustCtrler_Data_temp); // 取数据
//            CustCtrl_Data_Solve(&CustCtrler_Data_temp,&CustCtrler_Data_temp_Last);   // 数据处理
//            
//            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_X),
//                              GAIN_CUSTCTRLER_X * CustCtrler_Data_temp.pos.x);
//            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Y),
//                              GAIN_CUSTCTRLER_Y * CustCtrler_Data_temp.pos.y);
//            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Z),
//                              GAIN_CUSTCTRLER_Z * CustCtrler_Data_temp.pos.z);

//            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw),
//                               CustCtrler_Data_temp.angle.yaw);
//            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch),
//                               CustCtrler_Data_temp.angle.pitch);
//            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll),
//                              CustCtrler_Data_temp.angle.roll);
				Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Move),
                  -GAIN_BOOM_MOUSE_MOVE_Z * RC_data.Mouse_Data.x_speed*350.0f);
			Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll),
							GAIN_FOREARM_MOUSE_MOVE_ROLL * RC_data.Mouse_Data.y_speed);
			Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_X),
                              -GAIN_CHASSIS_KEY_MOVE * (RC_data.Key_Data.W - RC_data.Key_Data.S));
      Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y),
                              -GAIN_CHASSIS_KEY_MOVE * (RC_data.Key_Data.A - RC_data.Key_Data.D));
           break;
        }
    }
    else if (CtrlState == Remote_Mode)
    {
        switch ((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)))
        {

        case ChassisNormal_Mode:
            SmoothFilter_Restart(BoomSmoothFilter); // 切换数据源时将滤波器清零
			SmoothFilter_Restart(ForearmSmoothFilter); // 切换数据源时将滤波器清零
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw),
                              GAIN_CHASSIS_REMOTE_ROTATE_Y * (RC_data.Remote_Data.ch0 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_Y),
                              GAIN_CHASSIS_REMOTE * (RC_data.Remote_Data.ch2 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Chassis_X),
                              -GAIN_CHASSIS_REMOTE * (RC_data.Remote_Data.ch3 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(ImagePitch_Ctrl),
                              -GAIN_FOREARM_REMOTE * (RC_data.Remote_Data.ch1 - 1024));
            break;
        case ArmAngle_Mode:
            SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
            SmoothFilter_Restart(BoomSmoothFilter);    // 切换数据源时将滤波器清零
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw),
                              -GAIN_FOREARM_REMOTE * (RC_data.Remote_Data.ch2 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch),
                              GAIN_FOREARM_REMOTE * (RC_data.Remote_Data.ch3 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll),
                              GAIN_FOREARM_REMOTE * (RC_data.Remote_Data.ch0 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Move),
                              boom_move_remote * (RC_data.Remote_Data.ch1 - 1024));
            break;
        case ArmPos_Mode:
            SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Y),
                              GAIN_BOOM_REMOTE * (RC_data.Remote_Data.ch2 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_X),
                              GAIN_BOOM_REMOTE * (RC_data.Remote_Data.ch3 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Z),
                              GAIN_BOOM_REMOTE * (RC_data.Remote_Data.ch1 - 1024));
            Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(ImageHigh_Ctrl),
                              boom_move_remote/4 * (RC_data.Remote_Data.ch0 - 1024));    
            break;
//		    case ArmCtrl_Mode:                             // 自定义控制器
//             SmoothFilter_Restart(ChassisSmoothFilter); // 切换数据源时将滤波器清零、
//             CustCtrler_Data_temp_Last = CustCtrler_Data_temp;
//             CustCtrler_Data_Read(&CustCtrler_Data_temp); // 取数据
//             CustCtrl_Data_Solve(&CustCtrler_Data_temp,&CustCtrler_Data_temp_Last);   // 数据处理
//            
//            
//             Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_X),
//                               GAIN_CUSTCTRLER_X * CustCtrler_Data_temp.pos.x);
//             Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Y),
//                               GAIN_CUSTCTRLER_Y * CustCtrler_Data_temp.pos.y);
//             Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Z),
//                               GAIN_CUSTCTRLER_Z * CustCtrler_Data_temp.pos.z);

//             Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw),
//                                CustCtrler_Data_temp.angle.yaw);
//             Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch),
//                                CustCtrler_Data_temp.angle.pitch);
//             Smooth_SetDataABS((ExactSmth_CTRL_S *)Get_Filter(Arm_Roll),
//                               CustCtrler_Data_temp.angle.roll);
//            break;
        default:
            break;
        }
    }
}

/**
 * @brief 获取平滑滤波后的数据
 * @author mylj
 */
void SmootFilterhData_Get(ArmPos_Remote_s *ArmState_Set,
                          VectorXY_Str *ChassisSpeXY_Set,
                          ImageMotor_s *ImageMotor_Set,
                          float *GimbalYaw_Set,
                          CustCtrler_Data_s *CustCtrlerState_Set)
{
    Smooth_GetDataABS(&ArmState_Set->Pos.x, (ExactSmth_CTRL_S *)Get_Filter(Arm_X));
    Smooth_GetDataABS(&ArmState_Set->Pos.y, (ExactSmth_CTRL_S *)Get_Filter(Arm_Y));
    Smooth_GetDataABS(&ArmState_Set->Pos.z, (ExactSmth_CTRL_S *)Get_Filter(Arm_Z));
    Smooth_GetDataABS(&ArmState_Set->Boommove, (ExactSmth_CTRL_S *)Get_Filter(Arm_Move));

    Smooth_GetDataABS(&ArmState_Set->Angle.arm_yaw, (ExactSmth_CTRL_S *)Get_Filter(Arm_Yaw));
    Smooth_GetDataABS(&ArmState_Set->Angle.arm_pitch, (ExactSmth_CTRL_S *)Get_Filter(Arm_Pitch));
    Smooth_GetDataABS(&ArmState_Set->Angle.arm_roll, (ExactSmth_CTRL_S *)Get_Filter(Arm_Roll));

    Smooth_GetDataABS(&ImageMotor_Set->ImageHigh,(ExactSmth_CTRL_S *)Get_Filter(ImageHigh_Ctrl));
    Smooth_GetDataABS(&ImageMotor_Set->ImagePitch,(ExactSmth_CTRL_S *)Get_Filter(ImagePitch_Ctrl));

    Smooth_GetDataABS(&ChassisSpeXY_Set->x, (ExactSmth_CTRL_S *)Get_Filter(Chassis_X));
    Smooth_GetDataABS(&ChassisSpeXY_Set->y, (ExactSmth_CTRL_S *)Get_Filter(Chassis_Y));
    Smooth_GetDataABS(GimbalYaw_Set, (ExactSmth_CTRL_S *)Get_Filter(Chassis_Yaw));
}

/************************************特殊功能************************************/

/**
 * @brief 计算坐标移动时，BoomYaw的旋转引起到ForearmYaw的旋转量
 * @param ArmPosSet_in
 * @param ForearmVector
 * @param BoomYaw                 用于对死点的处理，无特殊意义
 * @param BoomYawLatch
 * @param out
 */
static void ForearmYawOutformBoom(VectorXYZ_Str *ArmPosSet_in,
                                  VectorXYZ_Str *ForearmVector_latch,
                                  float BoomYaw, float BoomYawLatch,
                                  float ForearmYawlatch,
                                  float *out)
{
    float BoomYaw_Set; // 新坐标下，BoomYaw的期望角度
    float BoomYaw_Add; // 新坐标下，BoomYaw的变化角度
    VectorXYZ_Str BoomPosSet_temp;
    // 计算增量后的，大臂末端坐标
    Vector3D_Subb(ArmPosSet_in, ForearmVector_latch, &BoomPosSet_temp);
    // 计算增量后的，大臂Yaw角度值
    BoomPos2BoomYawangle(&BoomPosSet_temp, BoomYaw, &BoomYaw_Set);
    // 计算BoomYaw变化量（相对于推矿动作一开始）
    BoomYaw_Add = BoomYaw_Set - BoomYawLatch;
    // 计算当前ForearmYaw角度值
    *out = -BoomYaw_Add + ForearmYawlatch;
}

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
                   float *ForearmYaw_out)
{
    ForearmMotor_s ForearmAngle_temp;(void)ForearmAngle_temp;

    // 赋值
    ForearmAngle_temp.ForearmYaw = ArmState_in->Angle.arm_yaw;
    ForearmAngle_temp.ForearmPitch = ArmState_in->Angle.arm_pitch;
    ForearmAngle_temp.ForearmRoll = ArmState_in->Angle.arm_roll;

    // 锁存的机械臂臂Yaw的角度
    *BoomYaw_out = BoomYaw_in;
    *ForearmYaw_out = ArmState_in->Angle.arm_yaw;

    // 记存当前小臂矢量值
	//有待补充
}

static float ForearmYawSet = 0; // 仅用于下方函数
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
                                ArmPos_Remote_s *DeltaPos_out)
{
    VectorXYZ_Str ArmPos_temp;
    // 计算位置增量
    Vector3D_SetSize(ForearmVector_latch2, SpeedSet * dt, &DeltaPos_out->Pos);
    // 计算增量之后，机械臂末端坐标
    Vector3D_Add(ArmPos_Set, &DeltaPos_out->Pos, &ArmPos_temp);

    ForearmYawOutformBoom(&ArmPos_temp,
                          ForearmVector_latch1,
                          BoomYaw, BoomYawlatch,
                          ForearmYawlatch,
                          &ForearmYawSet);

    ForearmYawSet_ABS(&ForearmYawSet);
    // 其他增量值为0
    DeltaPos_out->Angle.arm_yaw = 0;
    DeltaPos_out->Angle.arm_pitch = 0;
    DeltaPos_out->Angle.arm_roll = 0;
    DeltaPos_out->Boommove = 0;
}

/**
 * @brief 坐标系转换——由小臂坐标系转换到机器人的坐标系
 * @param in
 * @param pitch   推荐使用图传pitch
 * @param ForearmYaw
 * @param out
 */
void XYZ_Forearm2Robo(VectorXYZ_Str *in,
                      float pitch, float ForearmYaw,
                      VectorXYZ_Str *out)
{
    float yaw_temp; // 需要转的yaw轴角度

    VectorXYZ_Str XYZ_Pitch_temp1; // 坐标沿着x旋转后的临时储存

    yaw_temp = ForearmYaw;
    in->z = in->z;
    Vector3D_Rotate_x(in, pitch + PI / 2.0f, &XYZ_Pitch_temp1);
    Vector3D_Rotate_z(&XYZ_Pitch_temp1, yaw_temp, out);
}

/**
 * @brief  与遥控器连接情况
 * @return 1：成功连接状态  0：连接失败状态
 */
int RemoteConnectState_Get(void)
{
    static rt_tick_t freshtick_old;

    if (RC_data.ValidData_FreshTick - freshtick_old > REMOTECONNECT_TIMEINTERVAL)
    {
        // 连接超过1s未更新数据，则判断遥控制器未与机器人连接
        freshtick_old = RC_data.ValidData_FreshTick;
        return 0;
    }
    else
    {
        freshtick_old = RC_data.ValidData_FreshTick;
        return 1;
    }
}
