/**
 * @file app_RoboCtrl.c
 * @brief 机械人控制线程
 * @author mylj
 * @version 1.0
 * @date 2022-12-30
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "app_RoboCtrl.h"
#include "drv_utils.h"
#include "drv_thread.h"
#include "drv_motor_Locked.h"
#include "drv_SetPlanning3D.h"
#include "func_Dataserver.h"
//#include "func_encoderCail.h"
#include "func_MusicPlayer.h"
#include "func_ArmMotor_Ctrl.h"
#include "func_ImageMotor_Ctrl.h"
#include "func_ChassisMotor_Ctrl.h"
#include "mod_Can_data.h"
#include "mod_RoboStateCtrl.h"
#include "app_monitor.h"
#include "drv_RemoteCtrl_data.h"
#include "func_Gpio_Ctrl.h"
#include "func_sd_master.h"

#define shujuyuan_normal 1
static struct rt_semaphore ArmCtrl_sem;      // 用于控制的信号量
static struct rt_semaphore ArmMotorCtrl_sem; // 用于控制电机线程的信号量
// 图传云台
ImageState_Data_s ImageState_Data = {
    .AngleNowFilter.ImageYaw = 0,
    .AngleNowFilter.ImagePitch = 0,
    .AngleHope.ImageYaw = 0,
    .AngleHope.ImagePitch = 0,
    .SpeedNowFilter.ImageYaw = 0,
    .SpeedNowFilter.ImagePitch = 0,
};

BoomState_Data_s BoomState_Data = {
    // 2024初始位置
    .AngleNowFilter.BoomYaw = 0,
    .AngleNowFilter.BoomPitch1 = 0,
    .AngleNowFilter.BoomPitch2 = 0,
    .AngleNowFilter.BoomMove = 0,

    .AngleHope.BoomYaw = 90.0f,
    .AngleHope.BoomPitch1 = 2.268f,
    .AngleHope.BoomPitch2 = -0.226f,
    .AngleHope.BoomMove = 0,

    .SpeedNowFilter.BoomYaw = 0,
    .SpeedNowFilter.BoomPitch1 = 0,
    .SpeedNowFilter.BoomPitch2 = 0,
    .SpeedNowFilter.BoomMove = 0,
}; // 大机械臂数据
ForearmMotor_s Forearm_AngleNow_old;


ForearmState_Data_s ForearmState_Data =
    {
        .AngleHope.ForearmYaw = PI / 2.0f,
        .AngleHope.ForearmPitch = 0.0f,
        .AngleHope.ForearmRoll = 0.0f,

        .SpeedNowFilter.ForearmYaw = 0,
        .SpeedNowFilter.ForearmPitch = 0,
        .SpeedNowFilter.ForearmRoll = 0,
}; // 小机械臂数据
Gold_Data_s Arm_Angle_Gold_n;
Gold_Data_s Arm_Angle_Gold_n1;
ArmPosState_s ArmPosState_Data = {
    .ArmPos_SetOld.x = 0,
    .ArmPos_SetOld.y = 0,
    .ArmPos_SetOld.z = 0,

    .ArmPos_SetPlanOld.x = 0,
    .ArmPos_SetPlanOld.y = 0,
    .ArmPos_SetPlanOld.z = 0,

}; // 机械臂位置速度姿态

SetPlanning3D_Str SetPlanning3D_Struct; // 三维设定值规划
SetPlanning3D_Str SetPlanning3D_Struct1; // 三维设定值规划

Chassis_t ChassisState_Data = {
    .filter_now.speed[0] = 0,
    .filter_now.speed[1] = 0,
    .filter_now.speed[2] = 0,
    .filter_now.speed[3] = 0,
};

Arm_Limit_s Arm_Limit_Data;            // 机械臂超限信息
static Image_Limit_s Image_Limit_Data; // 图传超限信息
static DataValid_s flag_DataValid;     // 通信有效性标志位
int SlowStart_finishflag=0;       // 机械臂缓启动完成标志位

// 编码器
float BoomYawEncoder_radnow;
float BoomYawEncoderCali_radnow;
float BoomYawEncoderLock_radnow; // 校正之后，锁定后编码器的数据
// static EncoderCali_State_e EncoderCali_State;      // 编码器标定情况

rt_uint8_t ImageGimbalAlionState = 0; // 图传云台对位状态

float GimbalYaw_SpeedNow;           // 云台转速
float ImageHighEncoder_radnow;      // 图传编码器High的数值
float ImageHighEncoder_radlast = 0; // 上一次图传编码器High的数值//图传pitch，yaw，high值就是云台pitch，yaw，high
float ImageHighEncoderLock_radnow;  // 图传编码器High的锁定值
float ImageMotorHighInit_rad;       // 图传电机High的初始位置
float lvboxishu = 0.965f;
Forearm_Mid_Data_s Forearm_Midin_Data = 
{
.AngleNow.IfConvert = 0,
.SpeedNow.IfConvert = 0,
.AngleSetPlan.IfConvert = 0,
	};//作为标签数据
Boom_Mid_Data_s Boom_Midin_Data = 
{
  .AngleNow.IfConvert = 0,
  .AngleSetPlan.IfConvert=0,
  .SpeedNow.IfConvert = 0
	};//作为标签数据
arm_input jiaodu_init;

static void ArmCtrl_1ms_Handler(void *parameter)
{
		while(rt_sem_trytake(&ArmCtrl_sem)==RT_EOK)
			;
    rt_sem_release(&ArmCtrl_sem);
}
static void ArmMotorCtrl_2ms_Handler(void *parameter)
{
	while(rt_sem_trytake(&ArmMotorCtrl_sem)==RT_EOK)
			;
    rt_sem_release(&ArmMotorCtrl_sem);
}
// static VectorXYZ_Str ArmPosState_Set_temp; // 机械臂末端坐标设定值
VectorXYZ_Str Position_Old;
VectorXYZ_Str Position_Now;
arm_input Angle_Pass1;
arm_output Pos_Pass1;//规划之后的位姿
arm_output Spe_Pass1;
arm_input Now_dianji_space1;
arm_output Now_space1;//当前的速度
arm_output Now_Zitai;
arm_input Angle_End1;//规划角度
arm_input Angle_End2;//规划角度
arm_input Arm_Angular_velocity1;
float YKB1[5][5];//第二次补偿角速度的雅可比矩阵
float YKB_Inverse1[5][5];//第二次补偿角速度的雅可比矩阵的逆
arm_matrix_instance_f32 YKB_dev1, YKB_inverse_dev1;
arm_matrix_instance_f32 Arm_Angular_velocity_dev1,Spe_Pass_dev1;
int timeflagggg=0;
rt_tick_t mid;
float real_frequency;
arm_input debugyong;
arm_output debugyong1;
int debugtime=0;
arm_input out_put_angle_to_look;
arm_input Angle_set_Data;
arm_output Position_Data;
uint8_t zhengxian_flag=0;
rt_base_t pin_number_front;
rt_base_t pin_number_back;
int Dianping_Pe8 = 0;
uint8_t time1 = 0;
rt_tick_t zhengxian_tick1;
rt_tick_t zhengxian_tick2;
float now_time = 0;
int out_put_time=0;
rt_base_t the1;
rt_base_t the2;
rt_base_t the3;
rt_base_t the4;
rt_base_t the5;
int time11 = 0;
float gold_data1;
float gold_data2;
int IF_Compute;
int IF_Compute1;
int jiesuan_again=0;
float suofangbi=20.0f;
int nijiesuanwuxiao_time =0;
int data_obvious = 0;
float x_delta;
float y_delta;
float z_delta;
float smaller_data=0.05f;

/**
 * @brief 机械臂控制线程
 * @param parameter
 */
static void ArmCtrl_Thread(void *parameter)
{
  rt_tick_t tick0=rt_tick_get();
	
  CREAT_ID(id);
  ADDTOMONITOR_ID("ArmMotorCtrl_Thread", 2000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
	arm_mat_init_f32(&YKB_dev1, 5, 5, (float32_t *)&YKB1);
  arm_mat_init_f32(&YKB_inverse_dev1, 5, 5, (float32_t *)&YKB_Inverse1);
  arm_mat_init_f32(&Arm_Angular_velocity_dev1, 5, 1, (float32_t *)&Arm_Angular_velocity1);
  arm_mat_init_f32(&Spe_Pass_dev1, 5, 1, (float32_t *)&Spe_Pass1);	
  SWDG_START(id);
  while (1)
  {
		timeflagggg++;
		if((timeflagggg%100)==0)
		{
			mid=tick0;
			tick0=rt_tick_get();
			real_frequency = 1/((tick0-mid)/100.0f)*1000.0f;
		}//用来检测当前实时计算的频率
    // 阻塞等待接收信号量
    rt_sem_take(&ArmCtrl_sem, RT_WAITING_FOREVER);
    SWDG_FEED(id);
    }
}
float f_pitch=0;
float f_roll=0;
float f_yaw=90;
float b_p=0;
float b_p2=0;
float b_y,b_move;
float canshu1;
VectorXYZ_Str Postion_Old;
VectorXYZ_Str Postion_Now;
float Delta_X;
float Delta_Y;
float Delta_Z;
float the_r = 0.618f;
uint8_t arm_motor_target;
uint8_t arm_pos_target;
arm_output Spe_Pass2;
arm_input Arm_Angular_velocity2;
arm_input Angle_Now2;
float YKB2[5][5];//第二次补偿角速度的雅可比矩阵
float YKB_Inverse2[5][5];//第二次补偿角速度的雅可比矩阵的逆
arm_matrix_instance_f32 YKB_dev2, YKB_inverse_dev2;
arm_matrix_instance_f32 Arm_Angular_velocity_dev2,Spe_Pass_dev2;
static char moveing_flag=0;
float lower_data = 0;
float the_theta = 0;
float xandmove;
float boompitch2_last;
float radio2;
CustCtrler_Data_s CustCtrler_Data_Hope;
CustCtrler_Data_s CustCtrler_Data_LHope;
float jscope_x;
float jscope_y;
float jscope_z;
float jscope_tanx;
float jscope_tany;
float jscope_tanz;
float image_nofollowing_set=1.25f;
rt_uint8_t ImageFollow_Mode;              // 图传跟随模式
/**
 * @brief 机械臂电机控制线程
 * @param parameter
 */
static void ArmMotorCtrl_Thread(void *parameter)
{
	uint8_t timeflag=0;(void)timeflag;
	rt_tick_t out0;(void)out0;
	rt_tick_t out1;(void)out1;
  arm_mat_init_f32(&YKB_dev2, 5, 5, (float32_t *)&YKB2);
  arm_mat_init_f32(&YKB_inverse_dev2, 5, 5, (float32_t *)&YKB_Inverse2);
  arm_mat_init_f32(&Arm_Angular_velocity_dev2, 5, 1, (float32_t *)&Arm_Angular_velocity2);
  arm_mat_init_f32(&Spe_Pass_dev2, 5, 1, (float32_t *)&Spe_Pass2);	
    ChassisMotor_t ChassisStateData_now_temp2; // 当前值的中间量(速度修正的输出)
    ChassisMotor_t ChassisStateData_set_temp;  // 当前值的中间量(单位换算的输出)

    CREAT_ID(id);
    ADDTOMONITOR_ID("ArmMotorCtrl_Thread", 2000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
//	  out0=rt_tick_get();

    while (1)
    {
        // 阻塞等待接收信号量
        rt_sem_take(&ArmMotorCtrl_sem, RT_WAITING_FOREVER);
        SWDG_FEED(id);
			
        /*通信判断*/
        flag_DataValid.Arm_IfValid = ArmMotor_DataValid_If();
        flag_DataValid.Chassis_IfValid = ChassisMotor_DataValid_If();
        flag_DataValid.Image_IfValid = ImageMotor_DataValid_If();
        /*数据读取*/
        ArmMotor_NowAngle_Read(&BoomState_Data.AngleNow,
                               &ForearmState_Data.AngleNow);    
        ArmMotor_NowSpeed_Read(&BoomState_Data.SpeedNow,
                               &ForearmState_Data.SpeedNow);
        Chassis_NowSpeed_Read(&ChassisState_Data.now);
        Chassis_SetSpeed_Read(&ChassisState_Data.set);
        ImageMotor_NowAngle_Read(&ImageState_Data.AngleNow);
        ImageMotor_NowSpeed_Read(&ImageState_Data.SpeedNow);
        ImageMotor_AngleHope_Read(&ImageState_Data.AngleHope);
				ImageState_Data.AngleHope.ImageHigh = -100.0f + ImageState_Data.AngleHope.ImageHigh;
			  ArmPosState_Data.ArmPos_SetOld.x =ArmPosState_Data.ArmPos_Set.x;
			  ArmPosState_Data.ArmPos_SetOld.y =ArmPosState_Data.ArmPos_Set.y;
			  ArmPosState_Data.ArmPos_SetOld.z =ArmPosState_Data.ArmPos_Set.z;
        ArmMotor_SetPos_Read(&ArmPosState_Data.ArmPos_Set,
                              &Arm_Angle_Gold_n.AngleHope,
                              &BoomState_Data.AngleHope);
        CustCtrler_Data_Read(&CustCtrler_Data_Hope); // 取自定义控制器数据
        CustCtrl_Data_Solve(&CustCtrler_Data_Hope,&CustCtrler_Data_LHope);   // 数据处理

        /*滑台与x轴联动*/
        
//				xandmove=ArmPosState_Data.ArmPos_Set.x;
//				 ArmPosState_Data.ArmPos_Set.x+=0.535f;
//         if(0.55f<ArmPosState_Data.ArmPos_Set.x&&ArmPosState_Data.ArmPos_Set.x<1.17f)
//         {
//          BoomState_Data.AngleHope.BoomMove=0.5f*(ArmPosState_Data.ArmPos_Set.x-0.55f)/0.31f*(-250.0f);
//          ArmPosState_Data.ArmPos_Set.x=0.5f*(ArmPosState_Data.ArmPos_Set.x-0.55f)+0.55f;
//         }
//         else if (ArmPosState_Data.ArmPos_Set.x<=0.55f)
//         {
//          BoomState_Data.AngleHope.BoomMove=0.0f;
//          ArmPosState_Data.ArmPos_Set.x= ArmPosState_Data.ArmPos_Set.x;
//         }
//        else
//        {
//          BoomState_Data.AngleHope.BoomMove=-250.0f;
//          ArmPosState_Data.ArmPos_Set.x=ArmPosState_Data.ArmPos_Set.x-0.31f;
//        }
        /*机械臂控制*/
        if (flag_DataValid.Arm_IfValid)
        {
            /*堵转判断及对位*/
            ArmPitchAlign();
            /*将机械臂电机速度转换成弧度/s*/
            ForearmEncoderSpeTorpm(&ForearmState_Data.SpeedNow,&ForearmState_Data.SpeedNow);
            BoomEncoderSpeTorpm(&BoomState_Data.SpeedNow,&BoomState_Data.SpeedNow);
			/*机械臂编码器值转成角度，附加传动比的转换*/
			BoomEncoder_angle(&BoomState_Data.AngleNow, &BoomState_Data.AngleNow,&boompitch2_last);
			ForearmEncoder_angle(&ForearmState_Data.AngleNow,&ForearmState_Data.AngleNow);
            //机械臂角度转化为弧度
            Arm_MotorAngle_Rad(&BoomState_Data.AngleNow, &BoomState_Data.AngleNow,
                                &ForearmState_Data.AngleNow,&ForearmState_Data.AngleNow);
			/*对机械臂的当前的电机角度和速度进行滤波*/
		    BoomMotDataFilter(&BoomState_Data);
		    ForearmMotDataFilter(&ForearmState_Data);

    //     if((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) == ArmCtrl_Mode)
    //     {

    //         CustCtr_To_Angle(&CustCtrler_Data_Hope,&ArmPosState_Data.ArmPos_Set,&Arm_Angle_Gold_n.AngleHope);
	// 	    jscope_x = CustCtrler_Data_Hope.pos.x*20.0f;
	// 	    jscope_y = CustCtrler_Data_Hope.pos.y*20.0f;
	// 	    jscope_z = CustCtrler_Data_Hope.pos.z*20.0f;
	// 	    jscope_tanx = ArmPosState_Data.ArmPos_Set.x*20.0f;
	// 	    jscope_tany = ArmPosState_Data.ArmPos_Set.y*20.0f;
	// 	    jscope_tanz = ArmPosState_Data.ArmPos_Set.z*20.0f;
    //    }
			ArmPosState_Data.ArmPos_Set.y+=0;
			ArmPosState_Data.ArmPos_Set.z+=0.0f;
			ArmPosState_Data.ArmPos_Set.x+=0.37f;		
			BoomState_Data.AngleSetPlan.BoomMove=BoomState_Data.AngleHope.BoomMove;
			Three_Postion_Now(&ArmPosState_Data.BoomPos_Now,&BoomState_Data.AngleNowFilter);
      Three_Postion(&ArmPosState_Data.ArmPos_Set,&ArmPosState_Data.BoomPos_Now,&BoomState_Data.AngleSetPlan);

	    if((uint32_t)(Robo_Control(RoboCtrl_RC_State, Read, NULL)) == ArmCtrl_Mode)//自定义控制器模式小机械臂设定值
        {
		        Little_CustCtr_To_Angle(&CustCtrler_Data_Hope,&BoomState_Data.AngleSetPlan,&Arm_Angle_Gold_n.AngleHope);
						Three_Postion_Now(&ArmPosState_Data.BoomPos_Now,&BoomState_Data.AngleNowFilter);
						Little_CustCtr_Pos_Stock(&ArmPosState_Data.BoomPos_Now,&Arm_Angle_Gold_n.AngleHope,&BoomState_Data.AngleSetPlan);
         }
			ForearmState_Data.AngleSetPlan.ForearmYaw=Arm_Angle_Gold_n.AngleHope.arm_yaw+1.57f;
			f_roll=Arm_Angle_Gold_n.AngleHope.arm_roll;
			f_pitch=Arm_Angle_Gold_n.AngleHope.arm_pitch;
			ForearmState_Data.AngleSetPlan.ForearmPitch=2*f_roll-f_pitch;
			ForearmState_Data.AngleSetPlan.ForearmRoll=2*f_roll+f_pitch;
            /*前馈补偿包括重力矩和角速度*/
						
            ArmComp_Slow(750, &SlowStart_finishflag,
                         &BoomState_Data,
                         &ForearmState_Data);
            /*电机控制*/
            switch (SlowStart_finishflag)
            {
				case 0:
                BoomMotor_Ctrl(BoomPitch1, &BoomState_Data);
                BoomMotor_Ctrl(BoomPitch2, &BoomState_Data);
				ForearmMotor_Ctrl(ForearmPitch, &ForearmState_Data);
                ForearmMotor_Ctrl(ForearmRoll, &ForearmState_Data); 
				break;
            case 1:
                SlowStart_finishflag = 2;
								break;
            case 2:
                // 缓补偿启动完成
                // 开始闭环控制
//debug时候用的
//                ForearmState_Data.AngleSetPlan.ForearmYaw=f_yaw;
//                BoomState_Data.AngleSetPlan.BoomPitch1=b_p;
//                ForearmState_Data.AngleSetPlan.ForearmRoll=-0.0f;
//                ForearmState_Data.AngleSetPlan.ForearmPitch=0.0f;		
//					      BoomState_Data.AngleSetPlan.BoomMove=b_p;
//                BoomState_Data.AngleSetPlan.BoomYaw=b_y;
//								ForearmState_Data.AngleSetPlan.ForearmRoll-=1.7;
//                ForearmState_Data.AngleSetPlan.ForearmPitch+=1.7;
				    Control_Limit(&BoomState_Data.AngleSetPlan,&ForearmState_Data.AngleSetPlan,&Arm_Limit_Data);
            BoomState_Data.AngleHope.BoomYaw = BoomState_Data.AngleSetPlan.BoomYaw;
						BoomState_Data.AngleHope.BoomPitch1 = BoomState_Data.AngleSetPlan.BoomPitch1;
						BoomState_Data.AngleHope.BoomPitch2 = BoomState_Data.AngleSetPlan.BoomPitch2;
						BoomArmSetPlanning(&BoomState_Data,&BoomState_Data.AngleSetPlan,&BoomState_Data.SpeedFeedforward);
                BoomMotor_Ctrl(BoomYaw, &BoomState_Data);
                BoomMotor_Ctrl(BoomPitch1, &BoomState_Data);
                BoomMotor_Ctrl(BoomPitch2, &BoomState_Data);
                BoomMotor_Ctrl(BoomMove, &BoomState_Data);
                ForearmMotor_Ctrl(ForearmYaw, &ForearmState_Data);
                ForearmMotor_Ctrl(ForearmPitch, &ForearmState_Data);
                ForearmMotor_Ctrl(ForearmRoll, &ForearmState_Data);         
                break;
            default:
                break;
            }
            /*电机电流设定值计算*/
						ForearmState_Data.Compensation.ForearmPitch = cosf(f_pitch);
						ForearmState_Data.Compensation.ForearmRoll = cosf(f_pitch);
            ArmMotorinput_Calculate(&BoomState_Data.Compensation,
                                    &ForearmState_Data.Compensation,
                                    &BoomState_Data.MotorCtrl_Out,
                                    &ForearmState_Data.MotorCtrl_Out);   						
				}
        else
        {
            Send_Slave2_Init(); // 通信再次初始化
        }
        /*图传部分*/
        if (flag_DataValid.Image_IfValid)
        {
            /*堵转判断及对位*/
            ImageGimbalAlign();
            /*将机械臂电机速度转换成弧度/s*/
            ImageEncoderSpeTorpm(&ImageState_Data.SpeedNow,
                                 &ImageState_Data.SpeedNow);
            /*编码器数据处理*/
            ImageEncoder_angle(&ImageState_Data.AngleNow,
                              &ImageState_Data.AngleNow);
//			/*yaw 电机数据跨圈处理*/
//			Image_Yaw_Cross_Circle(&ImageState_Data.AngleNow,
//                      &ImageState_Data.AngleLast);
            /*将机械臂电机角度转换成弧度制*/
            Image_angle2rad(&ImageState_Data.AngleNow,
                            &ImageState_Data.AngleNow);
            /*零点校准*/
            Image_ZeroAdjustment(&ImageState_Data.AngleNow,
                                 &ImageState_Data.AngleNow);
            /*对机械臂电机角度值和速度值进行滤波*/
            ImageMotDataFilter(&ImageState_Data);
            // 图传超限
            if (ImageMachinelimit_If(&ImageState_Data.AngleHope,
                                     &Image_Limit_Data))
            {
                Imageoutlimit(&ImageState_Data.AngleHope,&ImageState_Data.AngleHope_Old);
            }
            else
            {
                ImageNolimit(&ImageState_Data.AngleHope,&ImageState_Data.AngleHope_Old);
            }
                    /*图传电机跟随*/
        ImageFollow_Mode = (uint32_t)Robo_Control(ImageFollow_State, Read, NULL);
        switch (ImageFollow_Mode)
        {
        case Following_Mode:
						ImageState_Data.AngleHope.ImageYaw = -ForearmState_Data.AngleSetPlan.ForearmYaw*0.6+2.042f;
            break;
        case Notfollowing_Mode:
            ImageState_Data.AngleHope.ImageYaw =image_nofollowing_set;
            break;
        default:
            break;
        }
            //  电机角度设定值规划
            ImageSetPlanning(&ImageState_Data,
                             &ImageState_Data.AngleSetPlan,
                             &ImageState_Data.SpeedFeedforward);
            // 闭环控制
            ImageMotor_Ctrl(ImageYaw, &ImageState_Data);
            ImageMotor_Ctrl(ImagePitch, &ImageState_Data);
            ImageMotor_Ctrl(ImageHigh, &ImageState_Data);
            ImageMotorinput_Calculate(ROBOCTRL_TIMER_PIRIOD * 500,
                                      DISTURBANCE_AMPLITUDE,
                                      &ImageState_Data.MotorCtrl_Out);
        }
        else
        {
            Send_Slave1_Init();
        }
        /*底盘部分*/
        if (flag_DataValid.Chassis_IfValid)
        {
            // 限制设定速度
            Wheels_Speed_Limit(RATED_SPEED, &ChassisState_Data.set,
                               &ChassisState_Data.set);
            // 修正速度
            Wheel_NowSpeed_Revise(&ChassisState_Data.now,
                                  &ChassisStateData_now_temp2);
            // 速度滤波
            WheelsMot_SpeedFilter(&ChassisStateData_now_temp2,
                                  &ChassisState_Data.filter_now);
            // 设定值换算单位
            WheelsUnitCverStandard2Rpm(&ChassisState_Data.set,
                                       &ChassisStateData_set_temp);
            // PID计算
            WheelsMotPID_Cal(&ChassisState_Data.filter_now,
                             &ChassisStateData_set_temp,
                             &ChassisState_Data.out);
        }
        /*电机电流控制报文发送*/
        Send_Slave1_Handle(&ChassisState_Data.out,
                           &ImageState_Data.MotorCtrl_Out,&BoomState_Data.MotorCtrl_Out);
        Send_Slave2_Handle(&BoomState_Data.MotorCtrl_Out,
                           &ForearmState_Data.MotorCtrl_Out);
        /*写入数据服务器*/
        ArmMotor_RadFilter_Write(&BoomState_Data.AngleNowFilter,
                                 &ForearmState_Data.AngleNowFilter);
        //Arm_IfLimit_Data_Write((uint8_t *)&Arm_Limit_Data);


    }
}

//PE8引脚电平中断回调函数
void Table_Stop(void *args)
{
	if(moveing_flag==0)
	{

    Move_flag=1;
    BoomState_Data.AngleNowFilter.BoomMove=BoomState_Data.AngleNow.BoomMove=0;
    BoomState_Data.AngleSetPlan.BoomMove=-150;
		moveing_flag++;
	}
	else
		moveing_flag=1;
}


static char ArmCtrl_Thread_stack[THREAD_STACK_ArmCtrl];
static char ArmMotorCtrl_Thread_stack[THREAD_STACK_ArmMotorCtrl];
arm_output zitai_init;
/**
 * @brief 机械臂控制线程初始化
 * @return rt_err_t
 */
rt_err_t ArmCtrl_Thread_Init(void)
{

	  Calculate_Struct_Init();
    rt_err_t res = RT_EOK;
    static struct rt_thread ArmCtrl; // 控制的第一个线程
    static struct rt_thread Arm_MotorCtrl;
    Arminput_Setvalue(&jiaodu_init,PI/2.0f,PI/3.0f,PI/3.0f,0,0);


		//读取PE8引脚电压,后方引脚
		pin_number_front = GET_PIN(E, 8);
		/* 传感器引脚为输入模式 */
    rt_pin_mode(pin_number_front, PIN_MODE_INPUT);
    /* 绑定中断*/
    rt_pin_attach_irq(pin_number_front, PIN_IRQ_MODE_FALLING, Table_Stop, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(pin_number_front, PIN_IRQ_ENABLE);	
	


	  //机械臂开始姿态初始化
    /*三维设定值规划初始化*/
    PlanSettings3D_Init(&SetPlanning3D_Struct, POS_ERRORMAX,
                        POS_TOLERANCE, SPE_TOLERANCE,
                        SPEEDMAX_R, SPEEDMAX_T,
                        ACCLMAX_R, ACCLMAX_T,
                        SETPLANNING_PERIOD);
    /*初始化信号量*/
    res = rt_sem_init(&ArmCtrl_sem,
                      "ArmCtrl_sem", 0,
                      RT_IPC_FLAG_FIFO);
    res = rt_sem_init(&ArmMotorCtrl_sem,
                      "ArmMotorCtrl_sem", 0,
                      RT_IPC_FLAG_FIFO);
//    /*创建机械臂控制线程*/
    rt_thread_init(&ArmCtrl,
                   "ArmCtrl",
                   ArmCtrl_Thread,
                   RT_NULL,
                   &ArmCtrl_Thread_stack[0],
                   sizeof(ArmCtrl_Thread_stack),
                   THREAD_PRIO_ArmCtrl, THREAD_TICK_ArmCtrl);
    rt_thread_startup(&ArmCtrl);

    /*创建机械臂电机控制线程*/
    rt_thread_init(&Arm_MotorCtrl,
                   "Arm_motorCtrl",
                   ArmMotorCtrl_Thread,
                   RT_NULL,
                   &ArmMotorCtrl_Thread_stack[0],
                   sizeof(ArmMotorCtrl_Thread_stack),
                   THREAD_PRIO_ArmMotorCtrl, THREAD_TICK_ArmMotorCtrl);
    rt_thread_startup(&Arm_MotorCtrl);

    /*创建线程定时器*/
    rt_timer_t timer = rt_timer_create("ArmCtrl_1ms_timer",
                                       ArmCtrl_1ms_Handler, // 回调函数
                                       RT_NULL,
                                       ROBOCTRL_TIMER_PIRIOD, // 定时时间4ms（ms）
                                       RT_TIMER_FLAG_PERIODIC);
    res = rt_timer_start(timer); // 启动定时器
    if (res != RT_EOK)
        return res; // 开启失败
    rt_timer_t timer_2 = rt_timer_create("ArmMotorCtrl_2ms_timer",
                                         ArmMotorCtrl_2ms_Handler, // 回调函数
                                         RT_NULL,
                                         ROBOMOTOR_TIMER_PIRIOD, // 定时时间2ms（ms）
                                         RT_TIMER_FLAG_PERIODIC);
    res = rt_timer_start(timer_2); // 启动定时器
    if (res != RT_EOK)
        return res; // 开启失败
    return RT_EOK;
}
