/**
 * @file drv_Arm_Solve.c
 * @brief 机械臂姿态解算及扭矩补偿
 * @brief 根据小机械臂的姿态解算大机械臂的姿态
 * @brief 根据当前姿态解算补偿扭矩
 * @brief 特定位置、特定功能
 * @author mylj
 * @version 2.0
 * @date 2023-03-03
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */

#include "drv_Arm_Solve.h"
#include "drv_utils.h"
#include "drv_Vector.h"
#include "string.h"
#include "arm_math.h"
#include "func_Gpio_Ctrl.h"
// #include "func_Dataserver.h"
#define sin arm_sin_f32
#define cos arm_cos_f32
#define arm_atan2_f32    atan2
#define pi PI
#define sqrt sqrtf
float a=0.195f;
float b=0.260f;
float c=0.2f;
float d=0.068f;
float e=0.1f;
float timex = 0;
arm_matrix_instance_f32 test_dev, test1_dev, test_negetive_dev;
arm_output testx, test1, test_negetive;

arm_matrix_instance_f32 test_input_dev, test_delta_dev, temp_dev;
arm_input test_input, test_input1, test_delta, temp;

arm_matrix_instance_f32 J_test_dev, J_inverse_dev;
float J_test[5][5], J_inverse[5][5];

/**
 * @brief 限幅判断函数（只用于判断，不修改值）
 * @param in
 * @param max
 * @param min
 * @return int  1：超限 0：没有超限
 */
static int LimitIf(float in, float max, float min)
{
    if (in > max || in < min)
        return 1;
    else
        return 0;
}

int LimitIfIn(float *in,float a1_now,float a2_set)
{
	float min;
	float max;
	uint8_t flag1 = 3;
	if(a1_now<=a2_set)
	{
	min = a1_now;
	max = a2_set;
	flag1 = 1;
	}else
	{
		min = a2_set;
		max = a1_now;
		flag1 =2;
	}
	if((*in>max||*in<min)&&(flag1==1))
	{
		*in = min;
		return 1;
	}
	if((*in>max||*in<min)&&(flag1==2))
	{
		*in = max;
		return 1;
	}
	return 0;

}
/*******************************静态力矩补偿解算*******************************/
float Boom_Pitch1_Currency = 1.0f;
float debug_bpbuchang_1,debug_bpbuchang_2,debug_bpbuchang_3;
/**
 * @brief 大机械臂pitch1轴静态扭矩补偿
 * @brief 输入为大机械臂pitch1当前的角度值
 * @brief 输出为大机械臂pitch1轴静态不动时的补偿扭矩对应的电流设定值
 */
static void BoomPitch1_StaticM2(float rad,float th2, float *out) // 2024重力矩补偿
{
    float BoomPitch1_StaticM2; // 电机的补偿扭矩
    BoomPitch1_StaticM2 = fabsf((float)(((0.5f * Motor_P1P2_Glink )  * Motor_P2_P1_L)
                                    -Motor_Y0P1_Glink *g0_link)* arm_sin_f32(rad+pi/4)+Boom_Pitch1_Currency*arm_cos_f32(pi/4-rad+th2-0.6f));
    BoomPitch1_StaticM2 = (float)(1.45f * BoomPitch1_M2_Ratio * BoomPitch1_StaticM2)+5.0f; // 2.1
    *out = BoomPitch1_StaticM2;
}


/**
 * @brief 大机械臂pitch2轴静态扭矩补偿
 * @brief 输入为大机械臂pitch1,pitch2当前的角度值
 * @brief 输出为大机械臂pitch2轴静态不动时的补偿扭矩对应的电流设定值
 */
static void BoomPitch2_StaticM3(float th1,float th2, float *out)
{
    float BoomPitch2_StaticM3; // 电机的补偿扭矩
   if(th2>0.6f)
	 {
    BoomPitch2_StaticM3 = fabsf((0.5f * Motor_P2Y1_Glink * Motor_P2_Y1_L + Motor_P2_Y1_L * moduan) * arm_cos_f32(th1+0.78f-th2-0.6f));
    BoomPitch2_StaticM3 = (1.35f * BoomPitch2_M3_Ratio * BoomPitch2_StaticM3);
	 }
	 else
	 {
		 BoomPitch2_StaticM3 = fabsf((0.5f * Motor_P2Y1_Glink * Motor_P2_Y1_L + Motor_P2_Y1_L * moduan) * arm_cos_f32(th1+0.78f-th2-0.6f));
     BoomPitch2_StaticM3 = (1.35f * BoomPitch2_M3_Ratio * BoomPitch2_StaticM3)+7.0f;
	 }
    *out = BoomPitch2_StaticM3;
}

/**
 * @brief 小机械臂pitch轴静态扭矩补偿
 * @param 输入为小机械臂的pitch角度（相对于地面）
 * @param 输出为小机械臂pitch轴静态不动时的补偿扭矩对应的电流设定值
 */
static void ForearmPitch_StaticM5(float rad_th1,float rad_th2,float rad_th4, float *out)
{
    float ForearmPitch_StaticM5;
		rad_th4 =rad_th1+rad_th2-rad_th4;
    ForearmPitch_StaticM5 = 0.5f*Pitch3_Last_L* Motor_P3Last_Glink * arm_cos_f32(rad_th4);
    ForearmPitch_StaticM5 = (ForearmPitch_M5_Ratio * ForearmPitch_StaticM5);
    *out = ForearmPitch_StaticM5;
}

static float time_now = 0; // 记录当前缓启动的时间
/**
 * @brief  补偿缓启动
 * @param  dt                控制周期 单位：ms
 * @param  period            缓启动的时长 单位：ms
 * @param  out               输出范围：0~1
 * @return int              1:表示缓启动已经完成
 */
static int Comp_SlowStart(int dt, int period, float *out)
{
    if (time_now == period)
    {
        *out = 1;
        return 1;
    }
    else
    {
        time_now += dt;
        *out = (float)time_now / (float)period;
        return 0;
    }
}
static float time_now1 = 0; // 记录当前缓启动的时间
/**
 * @brief  补偿缓启�?
 * @param  dt                控制周期 单位：ms
 * @param  period            缓启动的时长 单位：ms
 * @param  out               输出范围�?~1
 * @return int              1:表示缓启动已经完�?
 */
int Comp_SlowStart1(int dt, int period, float *out)
{
    if (time_now == period)
    {
        *out = 1;
        return 1;
    }
    else
    {
        time_now += dt;
        *out = (float)time_now / (float)period;
        return 0;
    }
}

/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 * @param   period            缓启动的时长 单位：ms
 * @return  int
 */
float ratio1=1.7f;
//flaot ratio2=1.45f
int compensation_time_flag = 0;
rt_tick_t tick0;
rt_tick_t tick1;
void ArmComp_Slow(int period,int *SlowStart_flag,
                 BoomState_Data_s *Boom_Slow,
                 ForearmState_Data_s *Forearm_Slow)
{
    float ratio;

	 BoomMotor_s boom_comptemp;
    ForearmMotor_s forearm_comptemp;
    if(*SlowStart_flag==0)
    {
	  tick1=rt_tick_get();
        boom_comptemp.BoomPitch1 = 0;
        boom_comptemp.BoomPitch2 = 0;
        forearm_comptemp.ForearmPitch = 0;
    
        BoomPitch1_StaticM2(Boom_Slow->AngleNowFilter.BoomPitch2,Boom_Slow->AngleNowFilter.BoomPitch2, &boom_comptemp.BoomPitch1);
        BoomPitch2_StaticM3(Boom_Slow->AngleNowFilter.BoomPitch2,Boom_Slow->AngleNowFilter.BoomPitch2, &boom_comptemp.BoomPitch2);
//        ForearmPitch_StaticM5(boom_anglenow->BoomPitch1,boom_anglenow->BoomPitch2,
//                                                            forearm_anglenow->ForearmPitch, &forearm_compout->ForearmPitch);
    
        *SlowStart_flag = Comp_SlowStart(1, period, &ratio);
    
        Boom_Slow->Compensation.BoomPitch1 = ratio* boom_comptemp.BoomPitch1;
        Boom_Slow->Compensation.BoomPitch2 = ratio* boom_comptemp.BoomPitch2;
		Boom_Slow->AngleSetPlan.BoomPitch1 = ratio*0.78f;
		Boom_Slow->AngleSetPlan.BoomPitch2 = 0.78f+ratio*0.88f;
		Forearm_Slow->SpeedHope.ForearmPitch = ratio*4.0f;
		Forearm_Slow->SpeedHope.ForearmRoll = -ratio*4.0f;
//        forearm_compout->ForearmPitch = ratio * forearm_comptemp.ForearmPitch;

    }
    else
    {
        BoomPitch1_StaticM2(Boom_Slow->AngleNowFilter.BoomPitch2,Boom_Slow->AngleNowFilter.BoomPitch2,&Boom_Slow->Compensation.BoomPitch1);
        BoomPitch2_StaticM3(Boom_Slow->AngleNowFilter.BoomPitch2,Boom_Slow->AngleNowFilter.BoomPitch2,&Boom_Slow->Compensation.BoomPitch1);
//        ForearmPitch_StaticM5(boom_anglenow->BoomPitch1,boom_anglenow->BoomPitch2,
//                                                            forearm_anglenow->ForearmPitch, &forearm_compout->ForearmPitch); 
    }
}




/*********************************机械臂相关解算**********************************/

/**
 * @brief 求前三个机械臂到达的位置
 * 
 */
float debug_look1;
float debug_look2;
float debug_look3;
   float th1_1;
    float th1_2;
void Three_Postion(VectorXYZ_Str *Pos_Hope,VectorXYZ_Str *Pos_Now,BoomMotor_s *Boom_Data)
{
    float b_distance;
    float c_distance;
 
    b_distance = Pos_Hope->x*Pos_Hope->x+Pos_Hope->y*Pos_Hope->y+(Pos_Hope->z-a)*(Pos_Hope->z-a);
    c_distance = Pos_Hope->x*Pos_Hope->x+Pos_Hope->y*Pos_Hope->y+Pos_Hope->z*Pos_Hope->z;
    debug_look1=(b*b+c*c-b_distance)/(2*b*c);
    if(debug_look1>1)
        Boom_Data->BoomPitch2 = 0;
    else if(debug_look1<-1)
         Boom_Data->BoomPitch2 = pi;
    else
        Boom_Data->BoomPitch2=acosf((b*b+c*c-b_distance)/(2*b*c));
    debug_look2 = c*sinf(Boom_Data->BoomPitch2)/sqrtf(b_distance);
    if(debug_look2<-1)
        th1_1=-pi/2;
    else if(debug_look2>1)
        th1_1 = pi/2;
    else
        th1_1 = asinf(c*sinf(Boom_Data->BoomPitch2)/sqrtf(b_distance));
    debug_look3 = (a*a+b_distance-c_distance)/(2*a*sqrtf(b_distance));
    if(debug_look3>1)
        th1_2 = 0;
    else if(debug_look3<-1)
        th1_2 = pi;
    else
        th1_2 = acosf((a*a+b_distance-c_distance)/(2*a*sqrtf(b_distance)));

    Boom_Data->BoomYaw = atanf(Pos_Hope->y/Pos_Hope->x);

			Boom_Data->BoomPitch1=th1_1+th1_2;
			Boom_Data->BoomPitch2 = pi - Boom_Data->BoomPitch2+35.0f/180.0f*pi;

//    Pos_Now->x = (b*sinf(Boom_Data->BoomPitch1)+c*sinf(pi-Boom_Data->BoomPitch1+Boom_Data->BoomPitch2-35.0f/180.0f*pi))*cosf(Boom_Data->BoomYaw);
//    Pos_Now->y = (b*sinf(Boom_Data->BoomPitch1)+c*sinf(pi-Boom_Data->BoomPitch1+Boom_Data->BoomPitch2-35.0f/180.0f*pi))*sinf(Boom_Data->BoomYaw);
//    Pos_Now->z = a-b*cosf(Boom_Data->BoomPitch1)+c*cosf(pi-Boom_Data->BoomPitch1+Boom_Data->BoomPitch2-35.0f/180.0f*pi);
    Boom_Data->BoomPitch1 =Boom_Data->BoomPitch1-pi/4;
    Boom_Data->BoomYaw =-(Boom_Data->BoomYaw+pi/2);
}

/**
 * @brief 求前三个机械臂当下位置
 * 
 */
void Three_Postion_Now(VectorXYZ_Str *Pos_Now,BoomMotor_s *Boom_Data)
{
    float b_distance;
    float c_distance;
    float th1_1;
    float th1_2;
    Pos_Now->x = (b*sinf(Boom_Data->BoomPitch1+pi/4)+c*sinf(pi-Boom_Data->BoomPitch1-pi/4+Boom_Data->BoomPitch2-35.0f/180.0f*pi))*cosf(Boom_Data->BoomYaw-pi/2);
    Pos_Now->y = (b*sinf(Boom_Data->BoomPitch1+pi/4)+c*sinf(pi-Boom_Data->BoomPitch1-pi/4+Boom_Data->BoomPitch2-35.0f/180.0f*pi))*sinf(Boom_Data->BoomYaw-pi/2);
    Pos_Now->z = a-b*cosf(Boom_Data->BoomPitch1+pi/4)+c*cosf(pi-Boom_Data->BoomPitch1-pi/4+Boom_Data->BoomPitch2-35.0f/180.0f*pi);
}


/**
* @brief 实际角度转化为算法角度
 */
void True_To_Compute(BoomMotor_s *Boom_Data,ForearmMotor_s *Forearm_Data,arm_input *output)
{
	output->arm_begin.th0 = Boom_Data->BoomYaw-PI/2;
	output->arm_begin.th1 = 125.0f/180.0f*PI-Boom_Data->BoomPitch1;
	output->arm_begin.th2 = Boom_Data->BoomPitch2-35.0f/180.0f*PI;
	output->arm_begin.th3 = Forearm_Data->ForearmYaw-85.0f/180.0f*PI;
	output->arm_begin.th4 = (Forearm_Data->ForearmRoll-Forearm_Data->ForearmPitch)/2.0f;
}
/**
* @brief 算法角度转化为实际角度
 */
void Compute_To_True(BoomMotor_s *Boom_Data,ForearmMotor_s *Forearm_Data,arm_input *output,float roll)
{
	Boom_Data->BoomYaw = output->arm_begin.th0*180.0f/PI+90.0f;
	Boom_Data->BoomPitch1 = 125.0f-output->arm_begin.th1*180.0f/PI;
	Boom_Data->BoomPitch2 = output->arm_begin.th2*180.0f/PI;
	Forearm_Data->ForearmYaw = output->arm_begin.th3*180.0f/PI+85.0f;
	Forearm_Data->ForearmPitch = 2.0f*roll-output->arm_begin.th4;
	Forearm_Data->ForearmRoll = 2.0f*roll+output->arm_begin.th4;
}

/**
 * @brief 根据小机械臂Yaw轴的顶部解算当前Boom电机Yaw角度值
 * @param BoomPos
 * @param LastYaw
 * @param Angle_Out
 */
void BoomPos2BoomYawangle(VectorXYZ_Str *BoomPos,
                          float LastYaw,
                          float *Angle_Out)
{
    float BoomPos_XOY_mod;
    VectorXY_Str BoomPos_XOY;
    VectorRT_Str BoomPos_XOY_RT;

    BoomPos_XOY.x = BoomPos->x;
    BoomPos_XOY.y = BoomPos->y;
    // 规划输出位置在XOY面上的投影
    Vector2D_Mod(&BoomPos_XOY, &BoomPos_XOY_mod);
    // if (BoomPos_XOY.y >= 0)
    {
        if (Float_ZeroIf(BoomPos_XOY_mod, 0.03f) == 0)
        {
            // 投影非0
            Vector2D_ToRT(&BoomPos_XOY, &BoomPos_XOY_RT);
            *Angle_Out = BoomPos_XOY_RT.t;
        }
        else
        {
            *Angle_Out = LastYaw;
        }
    }

}



/**
*@brief 小机械臂pitch和roll算法角度转化成实际角度
*@brief 锥齿轮数据转化 两个电机同向同速控制pitch，反向同速控制roll
*/
void Forearm_Fact_Angle(Forearm_Mid_Data_s *ForearmMath,ForearmMotor_s *ForearmFact)
{
	// 转化的时候首先判断标签是否为1，标签为1，则进行转化，并且只能转化一次，否则不进行转化
	// ForearmFact->ForearmYaw=(ForearmMath->AngleSetPlan.Data.ForearmYaw)*180.0f/PI+85.0f;
	// float pitch= ForearmMath->AngleSetPlan.Data.ForearmPitch*180.0f/PI;//算法角度PITCH为负
	// float roll= ForearmMath->AngleSetPlan.Data.ForearmRoll*180.0f/PI;
// if(ForearmMath->AngleSetPlan.IfConvert==1)
// {
    // ForearmFact->ForearmPitch = (2*roll - pitch);
    // ForearmFact->ForearmRoll = (pitch+2*roll);
		// ForearmMath->AngleSetPlan.IfConvert=0;
// }
    ForearmFact->ForearmYaw = ForearmMath->AngleSetPlan.Data.ForearmYaw + 1.4635f;
    float pitch = ForearmMath->AngleSetPlan.Data.ForearmPitch;
    float roll = ForearmMath->AngleSetPlan.Data.ForearmRoll;
    if(ForearmMath->AngleSetPlan.IfConvert==1)
    {
        ForearmFact->ForearmPitch = (2*roll - pitch);
        ForearmFact->ForearmRoll = (pitch+2*roll);
	    ForearmMath->AngleSetPlan.IfConvert= 0;       
    }

}

/**
*@brief 控制限幅
*/
void Control_Limit(BoomMotor_s *Boom_Data,ForearmMotor_s *Forearm_Data,Arm_Limit_s *Arm_Limit_data)
{
    Arm_Limit_data->BoomYaw_Iflimit = utils_truncate_number(&Boom_Data->BoomYaw,
                                             -BoomYaw_LimitMax , -BoomYaw_LimitMin);
    
    Arm_Limit_data->BoomPitch1_Iflimit = utils_truncate_number(&Boom_Data->BoomPitch1,
                                                 BoomPitch1_LimitMin, BoomPitch1_LimitMax);
    Arm_Limit_data->BoomPitch2_Iflimit = utils_truncate_number(&Boom_Data->BoomPitch2,
                                                 BoomPitch2_LimitMin, BoomPitch2_LimitMax);
    Arm_Limit_data->BoomMove_Iflimit = utils_truncate_number(&Boom_Data->BoomMove,
                                                BoomMove_LimitMin, BoomMove_LimitMax);

    Arm_Limit_data->ForearmYaw_Iflimit = utils_truncate_number(&Forearm_Data->ForearmYaw,
                                                 FOREARMYAW_LIMITMIN, FOREARMYAW_LIMITMAX);
//    Arm_Limit_data->ForearmPitch_Iflimit = utils_truncate_number(&Forearm_Data->ForearmPitch,
//                                                   FOREARMPITCH_LIMITMIN, FOREARMPITCH_LIMITMAX);
//    Arm_Limit_data->ForearmRoll_Iflimit = utils_truncate_number(&Forearm_Data->ForearmRoll,
//                                                  FOREARMROLL_LIMITMIN, FOREARMROLL_LIMITMAX);

}

/**
*@brief 逆解算角度控制限幅
*/
static int Control_Limit_Th(arm_input *intput_th)
{
    int th0,th1,th2,th3,th4;
    th0 = utils_truncate_number(&intput_th->arm_begin.th0,
             -80.0f/180.0f*PI,80.0f/180.0f*PI);
 
    th1 = utils_truncate_number(&intput_th->arm_begin.th1,
             45.0f/180.0f*PI,125.0f/180.0f*PI);
 
    th2 = utils_truncate_number(&intput_th->arm_begin.th2,
             -35.0f/180.0f*PI,110.0f/180.0f*PI);
 
    th3 = utils_truncate_number(&intput_th->arm_begin.th3,
             -90.0f/180.0f*PI,90.0f/180.0f*PI);
 
    th4 = utils_truncate_number(&intput_th->arm_begin.th4,
             -120.0f/180.0f*PI,75.0f/180.0f*PI);

if(th0==th1==th2==th3==th4==0)
{
    return 1;
}
else return 0;
}
/**
*@brief 逆解算角度控制限幅
*/
static int Control_Limit_Th1(arm_input *intput_th)
{
    int th0,th1,th2,th3,th4;
    th0 = utils_truncate_number(&intput_th->arm_begin.th0,
             -80.0f/180.0f*PI,80.0f/180.0f*PI);
 
    th1 = utils_truncate_number(&intput_th->arm_begin.th1,
             45.0f/180.0f*PI,125.0f/180.0f*PI);
 
    th2 = utils_truncate_number(&intput_th->arm_begin.th2,
             0,110.0f/180.0f*PI);
 
    th3 = utils_truncate_number(&intput_th->arm_begin.th3,
             -90.0f/180.0f*PI,90.0f/180.0f*PI);
 
    th4 = utils_truncate_number(&intput_th->arm_begin.th4,
             -120.0f/180.0f*PI,75.0f/180.0f*PI);

if(th0==th1==th2==th3==th4==0)
{
    return 1;
}
else return 0;
}
/**
*@brief 大机械臂弧度转化为角度贴上标签
*/
void Boom_Fact_Angle(Boom_Mid_Data_s *BoomMath,BoomMotor_s *BoomFact)
{
    if(BoomMath->AngleSetPlan.IfConvert==1)
    {
        BoomFact->BoomYaw = BoomMath->AngleSetPlan.Data.BoomYaw+PI/2;
        BoomFact->BoomPitch1 = 125.0f/180.0f*PI-BoomMath->AngleSetPlan.Data.BoomPitch1;
        BoomFact->BoomPitch2 = BoomMath->AngleSetPlan.Data.BoomPitch2+35.0f/180.0f*PI;
        BoomMath->AngleSetPlan.IfConvert=0;
    }
}

/**
*@brief 小机械臂pitch和roll目标角度转化成实际值
*@brief 锥齿轮数据转化 两个电机同向同速控制pitch，反向同速控制roll
*/
void Forearm_Old_Angle(ForearmMotor_s *Forearmangle_New,ForearmMotor_s *Forearmangle_Old)
{
    if(Forearmangle_New->ForearmPitch==0)
        Forearmangle_New->ForearmPitch=Forearmangle_Old->ForearmPitch;
    else
        Forearmangle_Old->ForearmPitch=Forearmangle_New->ForearmPitch;
    if(Forearmangle_New->ForearmRoll==0)
        Forearmangle_New->ForearmRoll=Forearmangle_Old->ForearmRoll;
    else
        Forearmangle_Old->ForearmRoll=Forearmangle_New->ForearmRoll;        
}
/**
*@brief 小机械臂pitch和rollshiji角度转化成suanfa
*@brief 锥齿轮数据转化 两个电机同向同速控制pitch，反向同速控制roll
*/
void Forearm_Suanfa_Angle(ForearmMotor_s *ForearmMath,ForearmMotor_s *ForearmFact)
{
    ForearmFact->ForearmPitch = (ForearmMath->ForearmRoll - ForearmMath->ForearmPitch)/2;
    ForearmFact->ForearmRoll = (ForearmMath->ForearmPitch + ForearmMath->ForearmRoll)/2;
}

/**
 * @brief   机械臂电机力矩补偿
 * @param   angle_now
 * @param   comp_out
 * @param   period            缓启动的时长 单位：ms
 * @return  int
 */

/*********************************机械臂相关解算**********************************/
/**
 * @brief 机械臂电机限幅判断
 * @param arm_get 解算得到的电机的角度
 */
int Arm_Limit_Determine(arm_input *arm_get)
{
    if (fabsf(arm_get->arm_begin.th0) <= (PI / 2) && fabsf(arm_get->arm_begin.th1) <= (PI / 2) && fabsf(arm_get->arm_begin.th2) <= (PI / 2) && fabsf(arm_get->arm_begin.th3) <= (PI / 2) && fabsf(arm_get->arm_begin.th4) <= (PI / 2) )
    {
        return 1;
    }
    else
        return 0;
}
arm_input angle_look;
/**
* @brief 判断机械臂是否缓启动完成
* @param arm_set 设定的角度(角度制度)
* @param Boom_state 大机械臂的当前角度
* @param ForearmMotor_s 小机械臂的当前角度
 */
int Arm_Start_Determine(arm_input *arm_set,BoomMotor_s *Boom_state,ForearmMotor_s *Forearm_state)
{ 
	angle_look.arm_begin.th0=fabsf(arm_set->arm_begin.th0-Boom_state->BoomYaw);
	angle_look.arm_begin.th1=fabsf(arm_set->arm_begin.th1-Boom_state->BoomPitch1);
	angle_look.arm_begin.th2=fabsf(arm_set->arm_begin.th2-Boom_state->BoomPitch2);
	angle_look.arm_begin.th3=fabsf(arm_set->arm_begin.th3-Forearm_state->ForearmYaw);
	angle_look.arm_begin.th4=fabsf(arm_set->arm_begin.th4-Forearm_state->ForearmPitch);
    if ( angle_look.arm_begin.th0<= 2.0f 
			  && angle_look.arm_begin.th1 <= 2.5f 
			  && angle_look.arm_begin.th2 <= 2.5f 
		    && angle_look.arm_begin.th3 <= 2.5f 
		    && angle_look.arm_begin.th4 <= 2.5f )
    {
        return 1;
    }
    else
		{
        return 0;
		}
}
/**
* @brief 为arm_input结构体赋值
 */
void Arminput_Setvalue(arm_input *arm_set,float a,float b,float c,float d,float e)
{
	arm_set->arm_begin.th0 = a;
	arm_set->arm_begin.th1 = b;
	arm_set->arm_begin.th2 = c;
	arm_set->arm_begin.th3 = d;
	arm_set->arm_begin.th4 = e;
}
/**
* @brief 初始化设定SETPLAN
* @param arm_set 设定的角度(角度制度)
* @param Boom_state 大机械臂的当前角度
* @param ForearmMotor_s 小机械臂的当前角度
 */
void Arm_Start_SetPlan(arm_input *arm_set,BoomMotor_s *Boom_state,ForearmMotor_s *Forearm_state)
{
  Boom_state->BoomYaw = arm_set->arm_begin.th0;
	Boom_state->BoomPitch1 = arm_set->arm_begin.th1;
	Boom_state->BoomPitch2= arm_set->arm_begin.th2;
	Forearm_state->ForearmYaw = arm_set->arm_begin.th3;
	Forearm_state->ForearmRoll = arm_set->arm_begin.th4;
}

/**
 * @brief   正运动学解算，输入是弧度制，输出是三个轴的位置和pitch,yaw
 * @param   input   五个电机的角度
 * @return  output
 */
arm_output forward2(arm_input *input)
{
#define th0 input->arm_begin.th0
#define th1 input->arm_begin.th1
#define th2 input->arm_begin.th2
#define th3 input->arm_begin.th3
#define th4 input->arm_begin.th4
    arm_output output;
    float x_tmp;
    float b_x_tmp;
    float c_x_tmp;
    float d_x_tmp;
    float e_x_tmp;
    float f_x_tmp;
    float g_x_tmp;
    float h_x_tmp;
    float i_x_tmp;
    float j_x_tmp;
    float x_temp_1;
    float d_x_tmp_1;
    x_tmp = cos(th0);
    x_temp_1 = cos(th0);
    b_x_tmp = sin(th1);
    c_x_tmp = sin(th2);
    d_x_tmp = cos(th2);
    d_x_tmp_1 = cos(th2) * sin(th1);
    e_x_tmp = cos(th1);
    f_x_tmp = sin(th0);
    g_x_tmp = sin(th3);
    h_x_tmp = cos(th3);
    i_x_tmp = cos(th4);
    j_x_tmp = sin(th4);
    output.arm_end.x = ((c * (x_tmp * e_x_tmp * c_x_tmp + x_tmp * d_x_tmp * b_x_tmp) - e * (i_x_tmp * (f_x_tmp * g_x_tmp - h_x_tmp * (x_temp_1 * e_x_tmp * sin(th2) + x_temp_1 * d_x_tmp_1)) - j_x_tmp * (x_tmp * b_x_tmp * c_x_tmp - x_temp_1 * e_x_tmp * d_x_tmp))) - d * (sin(th0) * sin(th3) - h_x_tmp * (x_temp_1 * e_x_tmp * sin(th2) + x_temp_1 * d_x_tmp_1))) + b * x_tmp * b_x_tmp;
    output.arm_end.y = ((c * (e_x_tmp * f_x_tmp * c_x_tmp + d_x_tmp * f_x_tmp * b_x_tmp) + e * (i_x_tmp * (x_tmp * g_x_tmp + h_x_tmp * (e_x_tmp * sin(th0) * sin(th2) + sin(th0) * d_x_tmp_1)) + j_x_tmp * (f_x_tmp * b_x_tmp * c_x_tmp - e_x_tmp * d_x_tmp * f_x_tmp))) + d * (x_temp_1 * sin(th3) + h_x_tmp * (e_x_tmp * sin(th0) * sin(th2) + sin(th0) * d_x_tmp_1))) + b * f_x_tmp * b_x_tmp;
    x_tmp = th1 + th2;
    f_x_tmp = cos(x_tmp);
    output.arm_end.z = (((((a + c * f_x_tmp) + b * e_x_tmp) + e * f_x_tmp * cos(th3 + th4) / 2.0f) + d * f_x_tmp * h_x_tmp) + e * sin(x_tmp) * j_x_tmp) + e * cos(th3 - th4) * f_x_tmp / 2.0f;
    f_x_tmp = i_x_tmp * (sin(th0) * sin(th3) - h_x_tmp * (x_temp_1 * e_x_tmp * sin(th2) + x_temp_1 * d_x_tmp_1)) - j_x_tmp * (x_temp_1 * sin(th1) * sin(th2) - x_temp_1 * e_x_tmp * cos(th2));
    g_x_tmp = i_x_tmp * (x_temp_1 * sin(th3) + h_x_tmp * (e_x_tmp * sin(th0) * sin(th2) + sin(th0) * d_x_tmp_1)) + j_x_tmp * (sin(th0) * sin(th1) * sin(th2) - e_x_tmp * cos(th2) * sin(th0));
    x_tmp = e * e;
    output.arm_end.pitch=arm_atan2_f32(e * (j_x_tmp * (e_x_tmp * c_x_tmp + d_x_tmp * b_x_tmp) + h_x_tmp * i_x_tmp * (e_x_tmp * cos(th2) - b_x_tmp * c_x_tmp)),
                  sqrt(x_tmp * (f_x_tmp * f_x_tmp) + x_tmp * (g_x_tmp * g_x_tmp)));
    output.arm_end.yaw=arm_atan2_f32(g_x_tmp, -f_x_tmp);
    return output;
#undef th0
#undef th1
#undef th2
#undef th3
#undef th4
}

/**
 * @brief 辅助雅可比矩阵的函数
 * @param u0
 * @param u1
 * @return float
 */
static float rt_powd_snf(float u0, float u1)
{
    float y;
    float d;
    float d1;
    if (isnan(u0) || isnan(u1))
    {
        y = FP_NAN;
    }
    else
    {
        d = fabs(u0);
        d1 = fabs(u1);
        if (isinf(u1))
        {
            if (d == 1.0f)
            {
                y = 1.0f;
            }
            else if (d > 1.0f)
            {
                if (u1 > 0.0f)
                {
                    y = FP_INFINITE;
                }
                else
                {
                    y = 0.0f;
                }
            }
            else if (u1 > 0.0f)
            {
                y = 0.0f;
            }
            else
            {
                y = FP_INFINITE;
            }
        }
        else if (d1 == 0.0f)
        {
            y = 1.0;
        }
        else if (d1 == 1.0f)
        {
            if (u1 > 0.0f)
            {
                y = u0;
            }
            else
            {
                y = 1.0f / u0;
            }
        }
        else if (u1 == 2.0f)
        {
            y = u0 * u0;
        }
        else if ((u1 == 0.5f) && (u0 >= 0.0f))
        {
            y = sqrt(u0);
        }
        else if ((u0 < 0.0f) && (u1 > floor(u1)))
        {
            y = FP_NAN;
        }
        else
        {
            y = pow(u0, u1);
        }
    }

    return y;
}

	

// 获取机械臂正运动学的雅可比矩阵
/**
 * @brief 根据当前的位姿求解当前的雅可比矩阵求解各个关节电机角度
 * @param J 雅可比矩阵
 * @param input 输入的各个关节电机角度
 */
void get_J(float J[5][5], arm_input *input)
{
#define th0 input->arm_begin.th0
#define th1 input->arm_begin.th1
#define th2 input->arm_begin.th2
#define th3 input->arm_begin.th3
#define th4 input->arm_begin.th4
    float J_1_1_tmp_tmp_tmp;
    float b_J_1_1_tmp_tmp_tmp;
    float c_J_1_1_tmp_tmp_tmp;
    float d_J_1_1_tmp_tmp_tmp;
    float e_J_1_1_tmp_tmp_tmp;
    float f_J_1_1_tmp_tmp_tmp;
    float J_1_1_tmp_tmp;
    float g_J_1_1_tmp_tmp_tmp;
    float b_J_1_1_tmp_tmp;
    float h_J_1_1_tmp_tmp_tmp;
    float i_J_1_1_tmp_tmp_tmp;
    float j_J_1_1_tmp_tmp_tmp;
    float c_J_1_1_tmp_tmp;
    float d_J_1_1_tmp_tmp;
    float J_1_2_tmp_tmp;
    float J_1_2_tmp;
    float J_1_5_tmp;
    float J_2_1_tmp;
    float J_2_2_tmp;
    float J_3_2_tmp_tmp;
    float J_3_2_tmp;
    float J_4_2_tmp_tmp_tmp;
    float J_4_2_tmp_tmp;
    float J_4_2_tmp;
    float b_J_4_2_tmp;
    float c_J_4_2_tmp;
    float b_J_4_2_tmp_tmp;
    float d_J_4_2_tmp;
    float e_J_4_2_tmp;
    float f_J_4_2_tmp;
    float g_J_4_2_tmp;
    float J_4_3_tmp;
    float q_j_1_1_temp;
    float q_y_1_temp;
    float q_a_1_temp;
    float q_b_1_temp;
    float q_c_1_temp;
    float q_d_1_temp;
    J_1_1_tmp_tmp_tmp = sin(th0);
    b_J_1_1_tmp_tmp_tmp = sin(th1);
    c_J_1_1_tmp_tmp_tmp = sin(th2);
    d_J_1_1_tmp_tmp_tmp = cos(th1);
    e_J_1_1_tmp_tmp_tmp = cos(th2);
    f_J_1_1_tmp_tmp_tmp = cos(th0);
    J_1_1_tmp_tmp = sin(th4);
    g_J_1_1_tmp_tmp_tmp = cos(th3);
    b_J_1_1_tmp_tmp = cos(th4);
    h_J_1_1_tmp_tmp_tmp = sin(th3);
    i_J_1_1_tmp_tmp_tmp = d_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp_tmp *
                          c_J_1_1_tmp_tmp_tmp;
    j_J_1_1_tmp_tmp_tmp = e_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp_tmp *
                          b_J_1_1_tmp_tmp_tmp;
    J_1_2_tmp_tmp = f_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp;
    c_J_1_1_tmp_tmp = i_J_1_1_tmp_tmp_tmp + j_J_1_1_tmp_tmp_tmp;
    d_J_1_1_tmp_tmp = f_J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp +
                      g_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp;
    q_j_1_1_temp = J_1_2_tmp_tmp * c_J_1_1_tmp_tmp_tmp;
    q_y_1_temp = f_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp;
    q_a_1_temp = J_1_2_tmp_tmp * e_J_1_1_tmp_tmp_tmp;
    q_b_1_temp = J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp;
    q_d_1_temp = d_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp;
    J_2_2_tmp = g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp;
    J[0][0] = ((-c * c_J_1_1_tmp_tmp - e * (b_J_1_1_tmp_tmp * d_J_1_1_tmp_tmp +
                                            J_1_1_tmp_tmp * (J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp *
                                                                 c_J_1_1_tmp_tmp_tmp -
                                                             d_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp *
                                                                 J_1_1_tmp_tmp_tmp))) -
               d * d_J_1_1_tmp_tmp) -
              b * J_1_1_tmp_tmp_tmp *
                  b_J_1_1_tmp_tmp_tmp;
    J_1_2_tmp = f_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp -
                J_1_2_tmp_tmp * e_J_1_1_tmp_tmp_tmp;
    J[0][1] = ((e * (J_1_1_tmp_tmp * (J_1_2_tmp_tmp * c_J_1_1_tmp_tmp_tmp +
                                      f_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp) -
                     J_2_2_tmp * (f_J_1_1_tmp_tmp_tmp *
                                      b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp -
                                  J_1_2_tmp_tmp *
                                      e_J_1_1_tmp_tmp_tmp)) -
                c * J_1_2_tmp) +
               b * f_J_1_1_tmp_tmp_tmp *
                   d_J_1_1_tmp_tmp_tmp) -
              d * g_J_1_1_tmp_tmp_tmp * J_1_2_tmp;
    J[0][2] = (e * (J_1_1_tmp_tmp * (q_j_1_1_temp + q_y_1_temp) - g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * (f_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp - q_a_1_temp)) - c * (f_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp *
                                                                                                                                                                                                     c_J_1_1_tmp_tmp_tmp -
                                                                                                                                                                                                 q_a_1_temp)) -
              d * g_J_1_1_tmp_tmp_tmp * (f_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp - q_a_1_temp);
    J[0][3] = -(d + e * b_J_1_1_tmp_tmp) * ((g_J_1_1_tmp_tmp_tmp *
                                                 J_1_1_tmp_tmp_tmp +
                                             q_j_1_1_temp * h_J_1_1_tmp_tmp_tmp) +
                                            q_y_1_temp * h_J_1_1_tmp_tmp_tmp);
    J_1_5_tmp = q_j_1_1_temp + q_y_1_temp;
    J[0][4] = e * (J_1_1_tmp_tmp * (J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp -
                                    g_J_1_1_tmp_tmp_tmp * J_1_5_tmp) +
                   b_J_1_1_tmp_tmp * J_1_2_tmp);
    J_2_1_tmp = J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp - g_J_1_1_tmp_tmp_tmp * (q_j_1_1_temp +
                                                                                 q_y_1_temp);
    J[1][0] = ((c * J_1_5_tmp - e * (b_J_1_1_tmp_tmp * J_2_1_tmp - J_1_1_tmp_tmp *
                                                                       J_1_2_tmp)) -
               d * J_2_1_tmp) +
              b * f_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp;
    J_1_2_tmp = q_b_1_temp - q_d_1_temp * J_1_1_tmp_tmp_tmp;
    J[1][1] = ((e * (J_1_1_tmp_tmp * c_J_1_1_tmp_tmp - J_2_2_tmp * J_1_2_tmp) - c *
                                                                                    J_1_2_tmp) -
               d * g_J_1_1_tmp_tmp_tmp * J_1_2_tmp) +
              b * d_J_1_1_tmp_tmp_tmp *
                  J_1_1_tmp_tmp_tmp;
    J[1][2] = (e * (J_1_1_tmp_tmp * (d_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp + e_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp) - J_2_2_tmp * (q_b_1_temp - q_d_1_temp * J_1_1_tmp_tmp_tmp)) - c * (q_b_1_temp - q_d_1_temp * J_1_1_tmp_tmp_tmp)) - d * g_J_1_1_tmp_tmp_tmp * (sin(th0) * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp - q_d_1_temp * J_1_1_tmp_tmp_tmp);
    J[1][3] = -(d + e * b_J_1_1_tmp_tmp) * ((i_J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp -
                                             f_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) +
                                            j_J_1_1_tmp_tmp_tmp *
                                                h_J_1_1_tmp_tmp_tmp);
    J[1][4] = -e * (J_1_1_tmp_tmp * d_J_1_1_tmp_tmp - b_J_1_1_tmp_tmp * J_1_2_tmp);
    J[2][0] = 0.0;
    J_3_2_tmp_tmp = th1 + th2;
    J_3_2_tmp = sin(J_3_2_tmp_tmp);
    J_3_2_tmp_tmp = cos(J_3_2_tmp_tmp);
    J[2][1] = ((((e * J_3_2_tmp_tmp * J_1_1_tmp_tmp - b * b_J_1_1_tmp_tmp_tmp) - e *
                                                                                     cos(th3 + th4) * J_3_2_tmp / 2.0f) -
                d * J_3_2_tmp *
                    g_J_1_1_tmp_tmp_tmp) -
               c * J_3_2_tmp) -
              e * cos(th3 - th4) *
                  J_3_2_tmp / 2.0f;
    J[2][2] = ((e * cos(th1 + th2) * J_1_1_tmp_tmp - d * sin(th1 + th2) * g_J_1_1_tmp_tmp_tmp) - c *
                                                                                                     sin(th1 + th2)) -
              e * J_3_2_tmp * g_J_1_1_tmp_tmp_tmp *
                  b_J_1_1_tmp_tmp;
    J[2][3] = -J_3_2_tmp_tmp * h_J_1_1_tmp_tmp_tmp * (d + e * b_J_1_1_tmp_tmp);
    J[2][4] = e * (J_3_2_tmp * b_J_1_1_tmp_tmp - J_3_2_tmp_tmp *
                                                     g_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp);
    J[3][0] = 0.0;
    d_J_1_1_tmp_tmp = ((b_J_1_1_tmp_tmp * J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp + J_3_2_tmp_tmp * f_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) -
                       J_1_2_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp *
                           c_J_1_1_tmp_tmp_tmp) -
                      f_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp *
                          g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp;
    J_2_1_tmp = d_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp;
    c_J_1_1_tmp_tmp = e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp;
    i_J_1_1_tmp_tmp_tmp = ((f_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp *
                                h_J_1_1_tmp_tmp_tmp -
                            J_3_2_tmp_tmp * J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                           J_2_1_tmp * J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) +
                          c_J_1_1_tmp_tmp *
                              J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp;
    J_4_2_tmp_tmp_tmp = 2.0f * d_J_1_1_tmp_tmp_tmp;
    J_4_2_tmp_tmp = J_4_2_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp;
    j_J_1_1_tmp_tmp_tmp = e * e;
    J_1_1_tmp_tmp_tmp = rt_powd_snf(e, 3.0f);
    f_J_1_1_tmp_tmp_tmp = q_d_1_temp;
    J_4_2_tmp = e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp;
    b_J_4_2_tmp = d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp;
    c_J_4_2_tmp = b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp;
    b_J_4_2_tmp_tmp = 2.0f * b_J_4_2_tmp;
    J_1_2_tmp = b_J_4_2_tmp_tmp * J_4_2_tmp;
    J_1_5_tmp = g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp;
    J_1_2_tmp_tmp = j_J_1_1_tmp_tmp_tmp * (d_J_1_1_tmp_tmp * d_J_1_1_tmp_tmp) +
                    j_J_1_1_tmp_tmp_tmp * (i_J_1_1_tmp_tmp_tmp * i_J_1_1_tmp_tmp_tmp);
    d_J_4_2_tmp = rt_powd_snf(J_1_2_tmp_tmp, 1.5f);
    e_J_4_2_tmp = b_J_4_2_tmp * J_1_5_tmp * c_J_4_2_tmp;
    f_J_4_2_tmp = J_4_2_tmp * J_1_5_tmp * c_J_4_2_tmp;
    g_J_4_2_tmp = J_4_2_tmp_tmp * J_1_5_tmp * c_J_4_2_tmp * b_J_1_1_tmp_tmp_tmp *
                  c_J_1_1_tmp_tmp_tmp;
    J[3][1] = -(J_1_1_tmp_tmp_tmp * (((b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp - f_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + J_2_1_tmp * c_J_1_1_tmp_tmp_tmp) + c_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp) *
                (((((((((((((((((J_1_2_tmp - J_4_2_tmp) - b_J_4_2_tmp) +
                               b_J_4_2_tmp * c_J_4_2_tmp) +
                              J_4_2_tmp * c_J_4_2_tmp) -
                             J_1_5_tmp *
                                 c_J_4_2_tmp) -
                            J_1_2_tmp * c_J_4_2_tmp) +
                           e_J_4_2_tmp) +
                          f_J_4_2_tmp) -
                         J_1_2_tmp * J_1_5_tmp * c_J_4_2_tmp) -
                        J_4_2_tmp_tmp *
                            b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) +
                       J_4_2_tmp_tmp *
                           c_J_4_2_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) +
                      J_4_2_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp *
                          b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                     2.0f *
                         e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp *
                         c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                    g_J_4_2_tmp) -
                   4.0f *
                       d_J_1_1_tmp_tmp_tmp * J_4_2_tmp * g_J_1_1_tmp_tmp_tmp *
                       b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) -
                  4.0f *
                      b_J_4_2_tmp * e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp *
                      b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                 1.0f)) /
              d_J_4_2_tmp;
    q_c_1_temp = J_2_2_tmp * J_1_1_tmp_tmp;
    J_4_3_tmp = 2.0f * q_d_1_temp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp;
    J_1_2_tmp = 2.0f * d_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * q_c_1_temp;
    J_1_5_tmp = 2.0f * e_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * q_c_1_temp;
    J[3][2] = -(rt_powd_snf(e, 3.0f) * (((b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp - d_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + d_J_1_1_tmp_tmp_tmp * J_2_2_tmp * c_J_1_1_tmp_tmp_tmp) + e_J_1_1_tmp_tmp_tmp * J_2_2_tmp * b_J_1_1_tmp_tmp_tmp) * (((((((((((((((((2.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) - e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) - d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) + d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) + e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) - g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) - 2.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) + d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) + e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) - 2.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) - J_4_3_tmp) + J_4_2_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) + J_1_2_tmp) + J_1_5_tmp) + J_4_2_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) - 4.0f * d_J_1_1_tmp_tmp_tmp * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) - 4.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + 1.0f)) / rt_powd_snf(j_J_1_1_tmp_tmp_tmp * (d_J_1_1_tmp_tmp * d_J_1_1_tmp_tmp) +
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        j_J_1_1_tmp_tmp_tmp * (i_J_1_1_tmp_tmp_tmp * i_J_1_1_tmp_tmp_tmp),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    1.5f);
    c_J_1_1_tmp_tmp = c_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp;
    i_J_1_1_tmp_tmp_tmp = b_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp;
    J_1_2_tmp = ((((((((((((((((2.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) *
                                    (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) -
                                e_J_1_1_tmp_tmp_tmp *
                                    e_J_1_1_tmp_tmp_tmp) -
                               d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) +
                              d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) +
                             e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp *
                                 (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) -
                            g_J_1_1_tmp_tmp_tmp *
                                g_J_1_1_tmp_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) -
                           2.0f *
                               (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) +
                          d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp *
                              (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) *
                              (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) +
                         e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp *
                             (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) *
                             (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) -
                        2.0f *
                            (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) *
                            (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) *
                            (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) *
                            (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp)) -
                       J_4_3_tmp) +
                      J_4_2_tmp_tmp * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) *
                          b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) +
                     J_1_2_tmp) +
                    J_1_5_tmp) +
                   J_4_2_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) *
                       b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) -
                  4.0f * d_J_1_1_tmp_tmp_tmp *
                      (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) *
                      g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp *
                      J_1_1_tmp_tmp) -
                 4.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp *
                     b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                1.0f;
    J[3][3] = -(e * cos(th1 + th2) * b_J_1_1_tmp_tmp * h_J_1_1_tmp_tmp_tmp / sqrt(J_1_2_tmp_tmp) - J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp *
                                                                                                       h_J_1_1_tmp_tmp_tmp * (J_3_2_tmp * J_1_1_tmp_tmp + cos(th1 + th2) * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp) * (((((((d_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp * J_1_1_tmp_tmp - d_J_1_1_tmp_tmp_tmp * J_4_2_tmp * b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) - b_J_4_2_tmp * e_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) - J_2_2_tmp) + e_J_1_1_tmp_tmp_tmp * i_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + b_J_4_2_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp) + J_4_2_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * i_J_1_1_tmp_tmp_tmp) + J_4_2_tmp_tmp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) / d_J_4_2_tmp) *
              J_1_2_tmp;
    J[3][4] = J_1_1_tmp_tmp_tmp * (((d_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp + e_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp) - f_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) * J_1_2_tmp / d_J_4_2_tmp;
    J[4][0] = 1.0f;
    J_1_2_tmp = 2.0f * d_J_1_1_tmp_tmp_tmp * J_2_2_tmp * b_J_1_1_tmp_tmp_tmp;
    J_1_5_tmp = 2.0f * e_J_1_1_tmp_tmp_tmp * J_2_2_tmp;
    J_2_1_tmp = J_1_1_tmp_tmp * J_1_1_tmp_tmp;
    J_2_1_tmp = (((((((((c_J_4_2_tmp * (h_J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp) + b_J_4_2_tmp * J_4_2_tmp * J_2_1_tmp) +
                        i_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp * J_2_1_tmp) +
                       e_J_4_2_tmp * c_J_1_1_tmp_tmp) +
                      f_J_4_2_tmp *
                          i_J_1_1_tmp_tmp_tmp) -
                     J_4_3_tmp * J_2_1_tmp) +
                    g_J_4_2_tmp) -
                   J_4_2_tmp_tmp_tmp * J_4_2_tmp * g_J_1_1_tmp_tmp_tmp *
                       b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) -
                  b_J_4_2_tmp_tmp * e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp *
                      b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                 J_1_2_tmp * c_J_1_1_tmp_tmp * J_1_1_tmp_tmp) +
                J_1_5_tmp *
                    i_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp;
    J[4][1] = -(b_J_1_1_tmp_tmp * h_J_1_1_tmp_tmp_tmp * (((d_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp + e_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + q_d_1_temp * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp) - J_2_2_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp)) /
              J_2_1_tmp;
    J[4][2] = -(b_J_1_1_tmp_tmp * h_J_1_1_tmp_tmp_tmp * (((d_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp + e_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + q_d_1_temp * J_2_2_tmp) - g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp)) / ((((((((((b_J_1_1_tmp_tmp *
                                                                                                                                                                                                                                                                                                                  b_J_1_1_tmp_tmp * (h_J_1_1_tmp_tmp_tmp * h_J_1_1_tmp_tmp_tmp) +
                                                                                                                                                                                                                                                                                                              d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) * (J_1_1_tmp_tmp * J_1_1_tmp_tmp)) +
                                                                                                                                                                                                                                                                                                             b_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * (c_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) * (J_1_1_tmp_tmp * J_1_1_tmp_tmp)) +
                                                                                                                                                                                                                                                                                                            d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) * (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) *
                                                                                                                                                                                                                                                                                                                (c_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp)) +
                                                                                                                                                                                                                                                                                                           e_J_1_1_tmp_tmp_tmp *
                                                                                                                                                                                                                                                                                                               e_J_1_1_tmp_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) *
                                                                                                                                                                                                                                                                                                               (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) * (b_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp)) -
                                                                                                                                                                                                                                                                                                          J_4_3_tmp * (J_1_1_tmp_tmp * J_1_1_tmp_tmp)) +
                                                                                                                                                                                                                                                                                                         J_4_2_tmp_tmp * (g_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp) *
                                                                                                                                                                                                                                                                                                             (b_J_1_1_tmp_tmp * b_J_1_1_tmp_tmp) * b_J_1_1_tmp_tmp_tmp *
                                                                                                                                                                                                                                                                                                             c_J_1_1_tmp_tmp_tmp) -
                                                                                                                                                                                                                                                                                                        2.0f * d_J_1_1_tmp_tmp_tmp * (e_J_1_1_tmp_tmp_tmp * e_J_1_1_tmp_tmp_tmp) * g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp *
                                                                                                                                                                                                                                                                                                            b_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) -
                                                                                                                                                                                                                                                                                                       2.0f * (d_J_1_1_tmp_tmp_tmp * d_J_1_1_tmp_tmp_tmp) * e_J_1_1_tmp_tmp_tmp * g_J_1_1_tmp_tmp_tmp *
                                                                                                                                                                                                                                                                                                           b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) +
                                                                                                                                                                                                                                                                                                      J_1_2_tmp *
                                                                                                                                                                                                                                                                                                          (c_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp) * J_1_1_tmp_tmp) +
                                                                                                                                                                                                                                                                                                     J_1_5_tmp *
                                                                                                                                                                                                                                                                                                         (b_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp) * c_J_1_1_tmp_tmp_tmp *
                                                                                                                                                                                                                                                                                                         J_1_1_tmp_tmp);
    J[4][3] = b_J_1_1_tmp_tmp * (((d_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp * c_J_1_1_tmp_tmp_tmp + e_J_1_1_tmp_tmp_tmp * cos(th4) * b_J_1_1_tmp_tmp_tmp) - q_d_1_temp * g_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) + g_J_1_1_tmp_tmp_tmp * b_J_1_1_tmp_tmp_tmp * c_J_1_1_tmp_tmp_tmp * J_1_1_tmp_tmp) / J_2_1_tmp;
    J[4][4] = J_3_2_tmp_tmp * h_J_1_1_tmp_tmp_tmp / J_2_1_tmp;
#undef th0
#undef th1
#undef th2
#undef th3
#undef th4
}

/**
 * @brief 用于解算的矩阵结构体初始化
 *
 */
void Calculate_Struct_Init(void)
{
    // 初始化矩阵结构体
    arm_mat_init_f32(&J_test_dev, 5, 5, (float32_t *)&J_test);
    arm_mat_init_f32(&J_inverse_dev, 5, 5, (float32_t *)&J_inverse);

    arm_mat_init_f32(&test_input_dev, 5, 1, (float32_t *)&test_input);
    arm_mat_init_f32(&test_delta_dev, 5, 1, (float32_t *)&test_delta);
    arm_mat_init_f32(&temp_dev, 5, 1, (float32_t *)&temp);

    arm_mat_init_f32(&test_dev, 5, 1, (float32_t *)&testx);
    arm_mat_init_f32(&test1_dev, 5, 1, (float32_t *)&test1);
    arm_mat_init_f32(&test_negetive_dev, 5, 1, (float32_t *)&test_negetive);
}
arm_output Forward_Determine;
int mid_Contorl=0;
arm_input test_last;
arm_input the_first;
float data_to_look[10];
float last_x;
float last_y;
float last_z;
float now_x;
float now_y;
float now_z;
int fixed_set = 0;
int time_flag=0;
float alpha = 0.7;
int Shake_Timeflag = 0;
rt_tick_t tick0;
rt_tick_t tick1;
float chaoxian_fluency;
float time_fluency;
float data_forward_error=0.1f;
/**
 * @brief 根据输入位姿和输出位姿进行逆解算输出角度
 * @param th1 输入的角度结构体
 * @param pos_hop 目标机械臂位姿
 */
int Angle_Inverse(arm_input th1, arm_output pos_hop,arm_input *out,float *th2_now)
{
	  last_x = now_x;
		last_y = now_y;
		last_z = now_z;
	  now_x = pos_hop.arm_end.x;
	  now_y = pos_hop.arm_end.y;
	  now_z = pos_hop.arm_end.z;
	  if(fabsf(now_x-last_x)<=0.01f)
		// 	&&fabsf(now_y-last_y)<=0.005f
		//   &&fabsf(now_z-last_z)<=0.005f)
		{
			fixed_set = 0;
		}else
		{
			fixed_set = 0;
		}
	  the_first = th1;
	  uint8_t flag1=0;
	  uint8_t flag2=0;
	  uint8_t flag3=0;
    int timeflag = 20;
    test_input = th1;
    test1 = pos_hop;              // 获取目标位姿
    testx = forward2(&test_input); // 获取当前位姿
	  tick0=rt_tick_get();
    while (timeflag--)            // 实际编程的时候这里应该有次数限制
    {
        // 获取当前位姿态的x,y,z,pitch和yaw
        testx = forward2(&test_input);
        // 获取目标位姿和目标位姿的差值
        arm_mat_sub_f32(&test1_dev, &test_dev, &test_negetive_dev);
        // 检验是否满足退出运算条件
        if ((fabsf(test_negetive.arm_end.x) + fabsf(test_negetive.arm_end.y) + fabsf(test_negetive.arm_end.z)) < 0.05f && (fabsf(test_negetive.arm_end.pitch) + fabsf(test_negetive.arm_end.yaw)) < 2.0f / 180.0f * pi)
            break;
        // 获取当前位姿下的雅可比矩阵
        get_J(J_test, &test_input);
        // 对当前雅可比矩阵求逆
        arm_mat_inverse_f32(&J_test_dev, &J_inverse_dev);
        // 将雅可比矩阵的逆左乘以差值矩阵
        arm_mat_mult_f32(&J_inverse_dev, &test_negetive_dev, &test_delta_dev);
        // 将计算出来的偏移量乘以alpha加给当前的位姿，因为这个矩阵是求逆求出来的，所以alpha最好为1，alpha大可以提升收敛速度，也可能会导致发散
        test_input.arm_begin.th0 += alpha * test_delta.arm_begin.th0;
        test_input.arm_begin.th1 += alpha * test_delta.arm_begin.th1;
        test_input.arm_begin.th2 += alpha * test_delta.arm_begin.th2;
        test_input.arm_begin.th3 += alpha * test_delta.arm_begin.th3;
        test_input.arm_begin.th4 += alpha * test_delta.arm_begin.th4;
        // 编程建议：1.这边增加偏移量的时候要考虑限位，如果超过限位，就让他等于限位的值，若果小于限位可以继续减
        // 2.如果频繁出现不收敛的现象，要考虑在计算的时候实时检测如果发散了就减少alpha再算
        // 3.计算出结果也要看一下是不是发散了
        //对得到的角度进行机械限幅和正解算再次判断
		//返回1代表未超出限幅
		if(pos_hop.arm_end.z>0.4f)
		{
         mid_Contorl=Control_Limit_Th(&test_input);
		}else
		{
		mid_Contorl=Control_Limit_Th1(&test_input);
		}
    }
	//机械限幅 超限0 没超限1
   if(mid_Contorl==1)
	 {
	 flag1=1;
	 }
	 else
	 {
		 chaoxian_fluency++;
	 	 tick1=rt_tick_get(); 
	 }
	 if(test_input.arm_begin.th4>2.0f)
	 {
		test_input.arm_begin.th4 = 2.0f;
	 }
	 if(timeflag<=1)
	 {
		 time_fluency++;
	 }
	//正解算判断是否超出限幅
	Forward_Determine = forward2(&test_input);
	if(fabsf(Forward_Determine.arm_end.x-test1.arm_end.x)<data_forward_error
		&&fabsf(Forward_Determine.arm_end.y-test1.arm_end.y)<data_forward_error
		&&fabsf(Forward_Determine.arm_end.z-test1.arm_end.z)<data_forward_error
		&&fabsf(Forward_Determine.arm_end.pitch-test1.arm_end.pitch)<data_forward_error
		&&fabsf(Forward_Determine.arm_end.yaw-test1.arm_end.yaw)<data_forward_error)
	{
	    flag2=1;		
	}else
	{
	    flag2=0;
	}
	if(flag1==1&&flag2==1)
	{
		if(test_input.arm_begin.th4<=(120.0f/180.0f*PI))
		{
		  *out = test_input;
			return 1;
		}
	}
	else
	{
			return 0;
	}
	if(fixed_set==1)
       {
        if(fabsf(test_input.arm_begin.th2-*th2_now)>0.05f)
            test_input.arm_begin.th2=*th2_now;
       }
	
}
/**
 * @brief 转化机械臂角度数据到arm_input结构体中
 *
 */
void Arm_Angle_Storage(Boom_Mid_Data_s *Boom_out,
                       Forearm_Mid_Data_s *Forearm_out, arm_input *mid)
{
    mid->arm_begin.th0 = Boom_out->AngleNow.Data.BoomYaw;
    mid->arm_begin.th1 = Boom_out->AngleNow.Data.BoomPitch1;
    mid->arm_begin.th2 = Boom_out->AngleNow.Data.BoomPitch2;
    mid->arm_begin.th3 = Forearm_out->AngleNow.Data.ForearmYaw;
    mid->arm_begin.th4 = Forearm_out->AngleNow.Data.ForearmPitch;
}
/**
 * @brief 转化arm_input结构体数据到机械臂角度数据中
 *
 */
void Angle_Arm_Storage(Boom_Mid_Data_s *Boom_out,
                       Forearm_Mid_Data_s *Forearm_out, arm_input *mid)
{
    Boom_out->AngleSetPlan.Data.BoomYaw = mid->arm_begin.th0;
    Boom_out->AngleSetPlan.Data.BoomPitch1 = mid->arm_begin.th1;
    Boom_out->AngleSetPlan.Data.BoomPitch2 = mid->arm_begin.th2;
	  Boom_out->AngleSetPlan.IfConvert = 1;
    Forearm_out->AngleSetPlan.Data.ForearmYaw = mid->arm_begin.th3;
    Forearm_out->AngleSetPlan.Data.ForearmPitch = mid->arm_begin.th4;
	  Forearm_out->AngleSetPlan.IfConvert = 1;
}


/**
 * @brief 转化机械臂设定位置到arm_output中
 *
 */
void Arm_Pos_Storage(VectorXYZ_Str *Pos_out,
                     ForearmMotor_s *Forearm_out, arm_output *mid)
{
    mid->arm_end.x = Pos_out->x;
    mid->arm_end.y = Pos_out->y;
    mid->arm_end.z = Pos_out->z;
    mid->arm_end.pitch = Forearm_out->ForearmPitch;
    mid->arm_end.yaw = Forearm_out->ForearmYaw;
}
/**
 * @brief 存储三维设定值规划速度到arm_output结构体中
 *
 */
void Set_Plan_To_Input(VectorXYZ_Str *Pos_out,Arm_Angle_Gold *Angle_Out,arm_output *str)
{
	str->arm_end.x = Pos_out->x;
  str->arm_end.y = Pos_out->y;
	str->arm_end.z = Pos_out->z;
	str->arm_end.pitch = Angle_Out->arm_pitch;
	str->arm_end.yaw = Angle_Out->arm_yaw;
}
/**
 * @brief 将解算得到的当前位置存入当前位置的结构体中
 *
 */
void Solve_Pos_To_Now(arm_output *solve,VectorXYZ_Str *data)
{
	data->x = solve->arm_end.x;
	data->y = solve->arm_end.y;
	data->z = solve->arm_end.z;
}
/**
* @brief 存储当前的关节电机速度
 *
 */
void Storage_Speed_Now(ForearmMotor_s *forearm,BoomMotor_s *boom,arm_input *input1)
{
	input1->arm_begin.th0 =  boom->BoomYaw;
  input1->arm_begin.th1 =  boom->BoomPitch1;
	input1->arm_begin.th2 =  boom->BoomPitch2;
	input1->arm_begin.th3 =  forearm->ForearmYaw;
	input1->arm_begin.th4 =  forearm->ForearmPitch;

}

