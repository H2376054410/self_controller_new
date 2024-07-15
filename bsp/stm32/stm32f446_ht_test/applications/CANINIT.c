#include "CANINIT.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "CONTROL.h"

static struct rt_semaphore canrx_sem;			/* 用于接收消息的信号量 */
static rt_device_t can_dev;					/* CAN 设备句柄 */
static rt_thread_t can_trans = RT_NULL;			// can接收线程的
struct rt_can_msg msg = {0}; /* CAN 消息 */ // 这个是自己设定的一个链表名字
#define THREAD_PRIORITY 20
#define THREAD_STACK_SIZE 1024
#define THREAD_TIMESLICE 10
#define CAN_DEV_NAME1 "can1" /* CAN 设备名称 */
struct rt_semaphore rx_time;			/*用于定时器的信号量*/
static struct rt_timer timer1;				/*定时器1*/
rec_data rec_data_s;
can_msg Boom_Left,Boom_Right,Boom_Yaw;
Motor_t Boomleft_Motor,Boomright_Motor,Boomyaw_Motor;
BoomMotor_s BoomCompension;
void ArmMotorinput_Calculate(BoomMotor_s *Boom_in,
                             BoomMotor_s *Boom_out)
{
	Boom_out->BoomLeft = Boomleft_Motor.spe.out+Boom_in->BoomLeft;
	Boom_out->BoomRight = Boomright_Motor.spe.out+Boom_in->BoomRight;
	Boom_out->BoomYaw = Boomyaw_Motor.spe.out+Boom_in->BoomYaw;
}
void UpliftEncoderSpeTorpm(BoomMotor_s *Spe_encoder,
                          BoomMotor_s *Speed_rad)
{
    Speed_rad->BoomLeft=PI*(Spe_encoder->BoomLeft/V_DM4310_ENCODERLEN*V_DM4310_MAX)/30.0f ;	
    Speed_rad->BoomRight =PI* (Spe_encoder->BoomRight/V_DM4310_ENCODERLEN*V_DM4310_MAX)/30.0f;
    Speed_rad->BoomYaw =PI* (Spe_encoder->BoomYaw/V_DM4310_ENCODERLEN*V_DM4310_MAX)/30.0f;
}
uint8_t last_data_get=0;
/**
 * @brief 图传电机从编码器值转换到实际角度
 * @param UpliftAngle_in
 * @param Imagerad_out
 */
 void UpliftEncoder_angle(BoomMotor_s *UpliftAngle_in,
                     BoomMotor_s *UpliftAngle_out,BoomMotor_s *UpliftAngle_last)
{
    UpliftAngle_out->BoomLeft =UpliftAngle_in->BoomLeft*360.0f/A4310_ENCODERLEN;
    UpliftAngle_out->BoomYaw =UpliftAngle_in->BoomYaw*360.0f/A4310_ENCODERLEN;
    UpliftAngle_out->BoomRight =UpliftAngle_in->BoomRight*360.0f/A4310_ENCODERLEN;
    if(last_data_get=1)
    {
    if(fabsf(UpliftAngle_last->BoomLeft-UpliftAngle_out->BoomLeft)>330.0f)
    {
        if(UpliftAngle_last->BoomLeft>100.0f) UpliftAngle_out->BoomLeft = UpliftAngle_out->BoomLeft+360.0f;
        else UpliftAngle_out->BoomLeft = UpliftAngle_out->BoomLeft-360.0f;
    }
    if(fabsf(UpliftAngle_last->BoomRight-UpliftAngle_out->BoomRight)>330.0f)
    {
        if(UpliftAngle_last->BoomRight>100.0f) UpliftAngle_out->BoomRight = UpliftAngle_out->BoomRight+360.0f;
        else UpliftAngle_out->BoomRight = UpliftAngle_out->BoomRight-360.0f;        
    }
    if(fabsf(UpliftAngle_last->BoomYaw-UpliftAngle_out->BoomYaw)>330.0f)
    {
        if(UpliftAngle_last->BoomYaw>100.0f) UpliftAngle_out->BoomYaw = UpliftAngle_out->BoomYaw+360.0f;
        else UpliftAngle_out->BoomYaw = UpliftAngle_out->BoomYaw-360.0f;        
    }
    }
    else
    {
        last_data_get++;
    }
    UpliftAngle_last->BoomLeft = UpliftAngle_out->BoomLeft;
    UpliftAngle_last->BoomRight = UpliftAngle_out->BoomRight;    
    UpliftAngle_last->BoomYaw = UpliftAngle_out->BoomYaw;
}
/**
 * @brief 电机位置信息由角度转向弧度
 * @param UpliftAngle_in
 * @param Upliftrad_out
 */
void Uplift_angle2rad(BoomMotor_s *UpliftAngle_in,
                     BoomMotor_s *Upliftrad_out)
{
    Upliftrad_out->BoomLeft = DEG2RAD_f(UpliftAngle_in->BoomLeft);
	    Upliftrad_out->BoomYaw = DEG2RAD_f(UpliftAngle_in->BoomYaw);
    Upliftrad_out->BoomRight = DEG2RAD_f(UpliftAngle_in->BoomRight);	
}
/**
 * @brief 对Boom电机角度值及速度值进行滤波
 * @param BoomStateData
 */
void BoomMotDataFilter(BoomState_Data_s *BoomStateData)
{

    /*BoomYaw*/
    BoomStateData->AngleNowFilter.BoomLeft =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomLeft,
                      BoomStateData->AngleNow.BoomLeft, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomLeft =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomLeft,
                      BoomStateData->SpeedNow.BoomLeft, 0.8f);

    /*BoomPitch1*/
    BoomStateData->AngleNowFilter.BoomRight =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomRight,
                      BoomStateData->AngleNow.BoomRight, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomRight =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomRight,
                      BoomStateData->SpeedNow.BoomRight, 0.8f);

    /*BoomPitch2*/
    BoomStateData->AngleNowFilter.BoomYaw =
        UTILS_LP_FAST(BoomStateData->AngleNowFilter.BoomYaw,
                      BoomStateData->AngleNow.BoomYaw, 0.8f); // 滞后滤波
    BoomStateData->SpeedNowFilter.BoomYaw =
        UTILS_LP_FAST(BoomStateData->SpeedNowFilter.BoomYaw,
                      BoomStateData->SpeedNow.BoomYaw, 0.8f);

}


/**
 * @brief：电机转速环pid输出的计算
 * @param [Motor_t*]	Motor:需要速度环计算的电机的结构体
 * @param [float]	SpeedNow:转速实际值
 * @return: [float] PID计算结果
 * @author：ych
 */
float Motor_SpeedPIDCalculate(Motor_t *Motor, float SpeedNow)
{
    float Error;
    Error = Motor->spe.set - SpeedNow;
    if (Motor->dji.reverse_flag)
        PID_Calculate(&Motor->spe, -Error); // 完成PID计算
    else
        PID_Calculate(&Motor->spe, Error); // 完成PID计算
    return Motor->spe.out;
}
/**
 * @brief：电机转速设定值绝对式修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的转速设定值
 * @author：ych
 */
void Motor_Write_SetSpeed_ABS(Motor_t *Motor, float Set)
{
    Motor->spe.set = Set;
}
/**
 * @brief：电机转动量计算（跨圈处理）
 * @param float Angle1: 要转到的角度对应的数值
 * @param float Angle2: 转动起点数值
 * @param float Round_Len: 电机整圈对应的数值
 * @return float: 输出从Angle2到Angle1需要的最短路径的距离长度, 范围 负半圈~正半圈，正好半圈时取正半圈
 * @author：ych
 */
float Motor_Get_DeltaAngle(float Angle1, float Angle2, float Round_Len)
{
    float DAngle;// HalfRound;
		float HalfRound;
    DAngle = Angle1 - Angle2;
    HalfRound = Round_Len / 2;
    if (DAngle >= 0)
    {
        if (DAngle > HalfRound)
            DAngle -= Round_Len;
    }
    else
    {
        if (DAngle <= -HalfRound)
            DAngle += Round_Len;
    }
    return DAngle;
}
/**
 * @brief：电机角度设定值读取
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 读取角度设定值
 * @author：ych
 */
float Motor_Read_SetAngle(Motor_t *Motor)
{
    return Motor->ang.set;
}
/**
 * @brief：电机角度环pid输出的计算，不会修改速度环设定值
 * @brief：调用过程中需要保证同一个电机的设定值和传入的实际值的单位相同
 * @param [Motor_t*]	Motor:需要角度环计算的电机的结构体
 * @param [float]	AngleNow:角度实际值
 * @return: [float] PID计算结果
 * @author：ych
 */
float Motor_AnglePIDCalculate(Motor_t *Motor, float AngleNow)
{
    float Error;

    if (Motor->dji.Round_Len != 0)
    {                                                     // 电机结构体初始化正常，可以进行跨圈处理计算，获得PID需要的Error
        if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL) // 计算偏差量Error
            Error = Motor_Get_DeltaAngle(Motor->ang.set, AngleNow, Motor->dji.Round_Len);
        else
        { // 如果电机使用的是可跨圈角度环模式，则不进行跨圈计算
            Error = Motor_Read_SetAngle(Motor) - AngleNow;
        }
    }
    else
        // 电机结构体没有设定闭环整圈长度 参数，可能没有调用电机结构体初始化函数。
        while (1)
            continue;
    PID_Calculate(&Motor->ang, Error); // 完成PID计算
    return Motor->ang.out;
}
/**
 * @brief：电机角度设定值绝对式修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的角度设定值
 * @author：ych
 */
void Motor_Write_SetAngle_ABS(Motor_t *Motor, float Set)
{
    float SetCal = Set;
    if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL) // 如果是不跨圈的闭环模式
    {
        if (SetCal > Motor->dji.Set_MAX) // 设定值不可跨圈
        {
            do
            {
                SetCal -= Motor->dji.Round_Len;
            } while (SetCal > Motor->dji.Set_MAX);
        }
        else if (SetCal <= Motor->dji.Set_MIN) // 设定值不可跨圈
        {
            do
            {
                SetCal += Motor->dji.Round_Len;
            } while (SetCal <= Motor->dji.Set_MIN);
        }
    }
    Motor->ang.set = SetCal;
}
/**
 * @brief 大机械臂电机闭环控制
 * @param ArmMotor
 * @param BoomStateData
 */
void BoomMotor_Ctrl(ArmMotor_e ArmMotor,
                    BoomState_Data_s *BoomStateData)
{
    switch (ArmMotor)
    {
    case BoomLeft:
        Motor_Write_SetAngle_ABS(&Boomleft_Motor, BoomStateData->AngleSetPlan.BoomLeft); // 设定值
        Motor_AnglePIDCalculate(&Boomleft_Motor, BoomStateData->AngleNowFilter.BoomLeft);
        Motor_Write_SetSpeed_ABS(&Boomleft_Motor,
                                 Boomleft_Motor.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomLeft);
        Motor_SpeedPIDCalculate(&Boomleft_Motor,
                                BoomStateData->SpeedNowFilter.BoomLeft);
        break;
    case BoomRight:
        Motor_Write_SetAngle_ABS(&Boomright_Motor, BoomStateData->AngleSetPlan.BoomRight); // 设定值
        Motor_AnglePIDCalculate(&Boomright_Motor,
                                BoomStateData->AngleNowFilter.BoomRight);
        // Motor_Write_SetSpeed_ABS(&BoomPitch1Motor_Str,
        //                          BoomPitch1Motor_Str.ang.out);
        Motor_Write_SetSpeed_ABS(&Boomright_Motor,
                                 Boomright_Motor.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomRight);
        Motor_SpeedPIDCalculate(&Boomright_Motor,
                                BoomStateData->SpeedNowFilter.BoomRight);
        break;
    case BoomYaw:
        Motor_Write_SetAngle_ABS(&Boomyaw_Motor, BoomStateData->AngleSetPlan.BoomYaw); // 设定值
        Motor_AnglePIDCalculate(&Boomyaw_Motor, BoomStateData->AngleNowFilter.BoomYaw);
        // Motor_Write_SetSpeed_ABS(&BoomPitch2Motor_Str,
        //                          BoomPitch2Motor_Str.ang.out);
        Motor_Write_SetSpeed_ABS(&Boomyaw_Motor,
                                 Boomyaw_Motor.ang.out +
                                     BoomStateData->SpeedFeedforward.BoomYaw);
        Motor_SpeedPIDCalculate(&Boomyaw_Motor,
                                BoomStateData->SpeedNowFilter.BoomYaw);
    default:
        break;
    }
}
/**
 * @brief：返回转速数据 不同控制模式下返回的数据不同
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 按照初始化时设置的电机控制模式返回对应的的转速数据
 * @author：ych
 */
float Motor_Read_NowSpeed(Motor_t *Motor)
{
    if (Motor->dji.Angle_CtrlMode == ANGLE_CTRL_ABS)
    {
        return Motor->dji.speed;
    }
    else
    {
        return Motor->dji.speed / Motor->dji.ratio; // 返回输出轴转速数据
    }
}
/**
 * @brief：电机角度读取
 * @brief：ABS控制模式下，返回编码器的原始数据
 * @brief：EXTRA控制模式下，返回电机减速箱输出轴的角度，以上电位置为零点，0~360°
 * @brief：FULL控制模式下，返回电机减速箱输出轴上电后转动过的总角度，以上电位置为零点，单位°
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 返回当前电机控制模式下闭环所需的角度数值
 * @author：ych
 */
float Motor_Read_NowAngle(Motor_t *Motor)
{
    switch (Motor->dji.Angle_CtrlMode)
    {
    case ANGLE_CTRL_ABS:
        return Motor->dji.angle;
    case ANGLE_CTRL_EXTRA:
        return Motor->dji.extra_angle;
    case ANGLE_CTRL_FULL:
        return Motor->dji.extra_angle + 360.0f * Motor->dji.loop;
    default:
        return 0;
    }
}

/**
 * @brief  对反馈角度进行换算为0-8191
 * @param  motor：电机数据结构体
 */
static void motor_angle_adjust(DjiMotor_t *motor)
{
    float angletemp = 0;
    rt_int32_t LEN;

    if (motor->oldangle_state == RT_ERROR)
    { // 第一次
        motor->oldangle_state = RT_EOK;
        motor->old_angle = motor->angle; // 第一次记录上次值为当前值
        motor->extra_angle = 0;
        motor->Data_Valid = 1;
        motor->loop = 0; // motor初始化前CAN就会有通信，所以初始化后的第一次CAN通信时程序才运行到这里，此时需要清空loop的数值
    }
    else
    {
        LEN = motor->Encoder_LEN;
        angletemp = Motor_Get_DeltaAngle(motor->angle, motor->old_angle, LEN) / motor->ratio / LEN * 360.0f + motor->extra_angle; // 角度积分
        if (angletemp > 360.f)
        { // 积分大于一圈
            motor->extra_angle = angletemp - 360.f;
            motor->loop += 1;
        }
        else if (angletemp < 0)
        { // 积分小于0
            motor->extra_angle = angletemp + 360.f;
            motor->loop -= 1;
        }
        else
        {                                   // 正常
            motor->extra_angle = angletemp; // 更新积分值
        }
        motor->old_angle = motor->angle;
    }
}
/**
 * @brief  读取can中的达妙电机数据
 * @param  rxmsg：反馈报文数据
 * @param  motor：电机数据结构
 * @retval None
 */
void motor_readmsg_DM(rt_uint8_t rxmsg[], DjiMotor_t *motor)
{
		motor->ErrorID = rxmsg[0];
    if (motor->reverse_flag)
    {
        motor->speed = -(rt_uint16_t)((rxmsg[3] << 4) | (rxmsg[4] >> 4)) + (1 << 11);
        motor->angle = motor->Encoder_LEN - (rt_uint16_t)((rxmsg[1] << 8) | rxmsg[2]); // 转子角度
        motor->current = -(rt_uint16_t)(((rxmsg[4] & 0x0F) << 8) | rxmsg[5]) + (1 << 11);
    }
    else
    {
        motor->speed = (rt_uint16_t)((rxmsg[3] << 4) | (rxmsg[4] >> 4)) - (1 << 11);
        motor->angle = (rt_uint16_t)((rxmsg[1] << 8) | rxmsg[2]); // 转子角度
        motor->current = (rt_uint16_t)(((rxmsg[4] & 0x0F) << 8) | rxmsg[5])- (1 << 11);
    }
    motor->angle = motor->angle - (1 << 15);
//		if((motor->angle - motor->lastangle > 2000 || motor->angle - motor->lastangle < -2000) && motor->lastangle != 0) 
//		{
//				motor->angle = motor->lastangle;
//		}
		motor->lastangle = motor->angle;
    motor->temperature = rxmsg[7]; // 内部线圈温度

    motor_angle_adjust(motor);
    motor->FreshTick = rt_tick_get();
}


/**
 * @brief 达妙电机报文接收处理函数
 */
static void DM_MotorCan_Receive(int id,Motor_t *Motor,
                                rt_uint8_t rxmsg[])
{
       motor_readmsg_DM(rxmsg, &Motor->dji);
	switch(id)
	{
		case BOOM_LEFTID:
	     rec_data_s.AngleNowBOOM_LEFT = Motor_Read_NowAngle(Motor);
	     rec_data_s.SpeedNowBOOM_LEFT = Motor_Read_NowSpeed(Motor);
			break;
		case BOOM_RIGHTID:
	     rec_data_s.AngleNowBOOM_RIGHT = Motor_Read_NowAngle(Motor);
	     rec_data_s.SpeedNowBOOM_RIGHT = Motor_Read_NowSpeed(Motor);
			break;
		case BOOM_YAWID:
	     rec_data_s.AngleNowBOOM_YAW = Motor_Read_NowAngle(Motor);
	     rec_data_s.SpeedNowBOOM_YAW = Motor_Read_NowSpeed(Motor);
			break;
		default:
			break;
	}
}
/**
 * @brief 达妙电机使能函数//每次上电都需要使能
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID         电机ID （1~4）
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
static void DM_MotorCan_Init(can_msg *Msg)
{
    Msg->data[0] = 0xFF;
    Msg->data[1] = 0xFF;
    Msg->data[2] = 0xFF;
    Msg->data[3] = 0xFF;
    Msg->data[4] = 0xFF;
    Msg->data[5] = 0xFF;
    Msg->data[6] = 0xFF;
    Msg->data[7] = 0xFC;
}
/**
 * @brief 发送can报文初始化
 * @param Msg       can报文
 * @param CanID     标识符
 * @param CanNum    选择can的编号 (0：CAN1	1：CAN2)
 */
static void Can_SendInit(can_msg *Msg,
                            rt_int32_t CanID)
{
    rt_int8_t i;

    Msg->id = CanID;
    Msg->ide = 0;
    Msg->rtr = 0;
    Msg->len = 8;

    for (i = 0; i < 8; i++)
        Msg->data[i] = 0;
}
/* can接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{

	/* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
	rt_sem_release(&canrx_sem);
	return RT_EOK;
}


int16_t id_ceshi;
int16_t receive_size;
static void can_rx_thread(void *parameter)
{
	can_msg rxmsg;
	rt_err_t res;
	/* 设置接收回调函数 */
	rt_device_set_rx_indicate(can_dev, can_rx_call);
	(void)res;

	while (1)
	{
		/* 阻塞等待接收信号量 */
		rt_sem_take(&canrx_sem, RT_WAITING_FOREVER);
	  rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
	        switch (rxmsg.id)
        {
        case BOOM_LEFTID: // 大机械臂Yaw轴电机
//					  Boom_Left = rxmsg;
            DM_MotorCan_Receive(BOOM_LEFTID,&Boomleft_Motor,
                                rxmsg.data);
            break;
        case BOOM_RIGHTID: // 大机械臂Pitch1轴电机
//						 Boom_Right = rxmsg;
            DM_MotorCan_Receive(BOOM_RIGHTID,&Boomright_Motor,
                                rxmsg.data);
            break;
        case BOOM_YAWID: // 小机械臂Pitch轴电机
//						 Boom_Yaw = rxmsg;
            DM_MotorCan_Receive(BOOM_YAWID,&Boomyaw_Motor,
                                rxmsg.data);
            break;
        default:
            break;
        }

	}
}


static void thread1_entry(void *parameter) // can发送线程
{
	rt_size_t size;
	while (1)
	{

	msg.id = 0x01;			/* ID  */
	msg.ide = RT_CAN_STDID; /* 标准格式 */
	msg.rtr = RT_CAN_DTR;	/* 数据帧 */
	msg.len = 8;			/* 数据长度为 2 */
	/* 待发送的pwm数据 */
		
//*(float*)(&msg.data[0])= 0.1;
//msg.data[7]= 255;
		msg.data[0]=0x20;
		msg.data[1]=0x00;
		msg.data[2]=0x00;
		msg.data[3]=0x40;
		msg.data[4]=0x08;
		msg.data[5]=0x00;
		msg.data[6]=0x10;
		msg.data[7]=0x08;

		rt_sem_take(&rx_time, RT_WAITING_FOREVER);
//		size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
//		if (size == 0)
//		{
//			rt_kprintf("can dev write data failed!\n");
//		}
	}
}

static void timeout1(void *parameter) // 定时器回调函数
{
	/*释放信号量*/
	rt_sem_release(&rx_time);
}
/**
 * @brief can报文发送初始化函数
 * @brief 机械臂和抬升末端部分电机报文发送
 * @param in
 */
rt_uint8_t init_debug = 0;
void Send_Slave2_Init(void)
{


    DM_MotorCan_Init(&Boom_Left);
    DM_MotorCan_Init(&Boom_Right);
    DM_MotorCan_Init(&Boom_Yaw);


    rt_device_write(can_dev, 0, &Boom_Left, sizeof(Boom_Left));
    rt_thread_mdelay(2);
    rt_device_write(can_dev, 0, &Boom_Right, sizeof(Boom_Right));
    rt_thread_mdelay(2);
    rt_device_write(can_dev, 0, &Boom_Yaw, sizeof(Boom_Yaw));
    rt_thread_mdelay(2);
		
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
 * @brief 达妙电机单报文写入函数
 * @param [SPICAN_MsgOnTransfer_t *]  Msg              can报文
 * @param [rt_int8_t]                 Motor_ID         电机ID （1~4）
 * @param [rt_int8_t]                 CtrlSetData      电机设定值
 */
rt_uint16_t data_tran;
static void DM_MotorCan_Write(can_msg *Msg,
	
                              float CtrlSetData)
{

    data_tran = float_to_uint(CtrlSetData, -1000, 1000.0, 12);
    Msg->data[0] = 0x00;
    Msg->data[1] = 0x00;
    Msg->data[2] = 0x00;
    Msg->data[3] = 0x00;
    Msg->data[4] = 0x00;
    Msg->data[5] = 0x00;
    Msg->data[6] = (data_tran >> 8) | ((0x00 & 0x0F) << 4);
    Msg->data[7] = data_tran;
}
void can_save_handle(BoomMotor_s *Boom_in)
{
		rt_size_t size1;
	  rt_size_t size2;
		rt_size_t size3;
    DM_MotorCan_Write(&Boom_Left,
                       Boom_in->BoomLeft);
    DM_MotorCan_Write(&Boom_Right,
                       Boom_in->BoomRight);
    DM_MotorCan_Write(&Boom_Yaw,
                       Boom_in->BoomYaw);
	  size1 = rt_device_write(can_dev, 0, &Boom_Left, sizeof(Boom_Left));
		size2 = rt_device_write(can_dev, 0, &Boom_Right, sizeof(Boom_Right));
		size3 = rt_device_write(can_dev, 0, &Boom_Yaw, sizeof(Boom_Yaw));
}
/**
* @brief：电机结构体初始化
* @param [Motor_t*]	Motor:需要修改的电机的结构体
* @param [rt_uint32_t]	ID:电机的CANID
* @param [float]	ratio:	电机的减速比，用于计算电机转动圈数
                            填写电机输出轴转动一圈时，编码器数据转过的圈数，以完整的M3508为例，应填写19.0f
* @param [float]	Encoder_Len:电机转动一圈对应多少编码器角度单位，例如GM6020采用8192线编码器，则应填写8192
* @param [rt_int32_t]	Set_Max:设定值的最大值
* @param [rt_int32_t]	Set_Min:设定值的最小值
                                如果使用编码器，则填写编码器能达到的数据范围 如0-8192（不是8191）
                                如果闭环时将使用外部角度数据，则填写外部角度数据能达到的数据范围 如0~360 / -180~180
                                闭环时PID输入的角度的范围必须与设定值范围和方向一致
* @param MotorReverse: 是否需要将电机数据反向, 需要反向写入 1
* @author：ych
*/
void motor_init_DM(Motor_t *motor, rt_uint32_t ID, float ratio, Angle_CtrlMode_E ModeSet, rt_int32_t Encoder_Len, rt_int32_t Set_Max, rt_int32_t Set_Min, int MotorReverse)
{
    if (Set_Max < Set_Min)
        while (1)
            continue; // 最大值不应小于最小值
    if ((MotorReverse != 1) && (MotorReverse != 0))
        while (1)
            continue; // 输入参数检查
    motor->dji.motorID = ID;
    motor->dji.ratio = ratio;
    motor->dji.oldangle_state = RT_ERROR;
    motor->dji.Round_Len = Set_Max - Set_Min; // 设定编码器一圈对应的变化量
    motor->dji.Angle_CtrlMode = ModeSet;      // 设定电机闭环控制模式
    motor->dji.Data_Valid = 0;                // 刚初始化完成，数据还不可用，需要等待下一次CAN通信接收完成
    motor->dji.Encoder_LEN_DM = Encoder_Len;
		motor->dji.Encoder_LEN = Encoder_Len;			// 代码中使用没有dm的标志位
    motor->dji.reverse_flag = MotorReverse;
    motor->dji.Motor_Read_Msg = motor_readmsg_DM; // 填入默认的回调函数
    if (ModeSet != ANGLE_CTRL_FULL)
    { // 如果选用是不能跨圈的角度闭环模式
        // 记录设定值的允许范围
        motor->dji.Set_MIN = Set_Min;
        motor->dji.Set_MAX = Set_Max;
    }
}
void motor_init(void)
{
motor_init_DM(&Boomleft_Motor, 1, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_ABS,
                  A4310_ENCODERLEN,
                  360, 0, 1);
motor_init_DM(&Boomright_Motor, 2, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  A4310_ENCODERLEN,
                  180, -180, 1);
motor_init_DM(&Boomyaw_Motor, 3, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  A4310_ENCODERLEN,
                  180, -180, 1);
    pid_init(&Boomleft_Motor.ang, 2, 0.1, 0, 1, 4, -4);
    pid_init(&Boomleft_Motor.spe, 2, 0, 0, 0, 20, -20);
    pid_init(&Boomright_Motor.ang, 5, 0.1, 0, 1, 4, -4);
    pid_init(&Boomright_Motor.spe, 20, 0, 0, 0, 20, -20);
    pid_init(&Boomyaw_Motor.ang, 5, 0.1, 0, 1, 4, -4);
    pid_init(&Boomyaw_Motor.spe, 20, 0, 0, 0, 100, -100);

}
void can_init(void)
{
	
	/*初始化达妙电机的Msg*/
	  Can_SendInit(&Boom_Left,1);
		Can_SendInit(&Boom_Right,2);
		Can_SendInit(&Boom_Yaw,3);

		rt_err_t res;
		rt_thread_t thread;
		/* 查找设备 */
	can_dev = rt_device_find(CAN_DEV_NAME1);
		if (!can_dev)
	{
		rt_kprintf("find %s failed!\n", CAN_DEV_NAME1);
	}
	/* 初始化 CAN 接收信号量 */
	rt_sem_init(&canrx_sem, "canrx_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&rx_time, "rx_time", 0, RT_IPC_FLAG_FIFO);
	/* 以中断接收及发送方式打开 CAN 设备 */
	res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
	/* 创建数据接收线程 */
	thread = rt_thread_create("can_rx", can_rx_thread, RT_NULL, 1024, 25, 5);
	if (thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}	else
	{
		rt_kprintf("create can_rx thread failed!\n");
	}

	can_trans = rt_thread_create("cantrans",
							thread1_entry, RT_NULL,
							1024,
							20, 1);
	/* 如 果 获 得 线 程 控 制 块， 启 动 这 个 线 程 */
	if (can_trans != RT_NULL)
		rt_thread_startup(can_trans);

	/* 初始化定时器 */
	rt_timer_init(&timer1, "timer1",	   /* 定时器名字是 timer1 */
				  timeout1,				   /* 超时时回调的处理函数 */
				  RT_NULL,				   /* 超时函数的入口参数 */
				  1,					   /* 定时长度，以 OS Tick 为单位，即 10 个 OS Tick */
				  RT_TIMER_FLAG_PERIODIC); /* 周期性定时器 */

	rt_timer_start(&timer1);


}

