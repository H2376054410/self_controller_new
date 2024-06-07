#include "drv_motor.h"

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
 * @brief  读取can中的3508或6020电机数据
 * @param  rxmsg：反馈报文数据
 * @param  motor：电机数据结构
 * @retval None
 */
void motor_readmsg(rt_uint8_t rxmsg[], DjiMotor_t *motor)
{
    if (motor->reverse_flag)
    {
        motor->speed = -(rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]);
        motor->angle = motor->Encoder_LEN - (rt_int16_t)((rxmsg[0] << 8) + rxmsg[1]); // 转子角度
        motor->current = -(rt_int16_t)(rxmsg[4] << 8 | rxmsg[5]);
    }
    else
    {
        motor->speed = (rt_int16_t)(rxmsg[2] << 8 | rxmsg[3]);
        motor->angle = (rt_int16_t)((rxmsg[0] << 8) + rxmsg[1]); // 转子角度
        motor->current = (rt_int16_t)(rxmsg[4] << 8 | rxmsg[5]);
    }
    motor->temperature = rxmsg[6];

    motor_angle_adjust(motor);
    motor->FreshTick = rt_tick_get();
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
 * @brief  电机电流发送
 * @param  dev：can设备
 * @param  setcurn:设定电流
 * @retval RT_EOK or RT_ERROR(成功，失败或其他报错)
 */
rt_size_t motor_current_send(rt_device_t dev,
                             SendID_e ID,
                             rt_int16_t setcur1,
                             rt_int16_t setcur2,
                             rt_int16_t setcur3,
                             rt_int16_t setcur4)
{
    struct rt_can_msg txmsg;

    txmsg.id = ID;
    txmsg.ide = RT_CAN_STDID;
    txmsg.rtr = RT_CAN_DTR;
    txmsg.len = 8;
    txmsg.data[0] = (rt_uint8_t)(setcur1 >> 8);
    txmsg.data[1] = (rt_uint8_t)(setcur1);
    txmsg.data[2] = (rt_uint8_t)(setcur2 >> 8);
    txmsg.data[3] = (rt_uint8_t)(setcur2);
    txmsg.data[4] = (rt_uint8_t)(setcur3 >> 8);
    txmsg.data[5] = (rt_uint8_t)(setcur3);
    txmsg.data[6] = (rt_uint8_t)(setcur4 >> 8);
    txmsg.data[7] = (rt_uint8_t)(setcur4);

    return rt_device_write(dev, 0, &txmsg, sizeof(txmsg));
}

/**
 * @brief  达妙电机电流发送
 * @param  dev：can设备
 * @param  setcurn:设定电流
 * @retval RT_EOK or RT_ERROR(成功，失败或其他报错)
 */
rt_size_t motor_current_send_DM(rt_device_t dev,
                                SendID_e ID,
                                rt_int16_t torque)
{
    struct rt_can_msg txmsg;

    txmsg.id = ID;
    txmsg.ide = RT_CAN_STDID;
    txmsg.rtr = RT_CAN_DTR;
    txmsg.len = 8;
    txmsg.data[0] = 0x00;
    txmsg.data[1] = 0x00;
    txmsg.data[2] = 0x00;
    txmsg.data[3] = 0x00;
    txmsg.data[4] = 0x00;
    txmsg.data[5] = 0x00;
    txmsg.data[6] = (rt_uint8_t)((torque >> 8) & 0x0F);
    txmsg.data[7] = (rt_uint8_t)((torque) & 0xFF);

    return rt_device_write(dev, 0, &txmsg, sizeof(txmsg));
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
 * @brief：带前馈的电机角度环pid输出的计算，不会修改速度环设定值
 * @brief：调用过程中需要保证同一个电机的设定值和传入的实际值的单位相同
 * @param [Motor_t*]	Motor:需要角度环计算的电机的结构体
 * @param [float]	AngleNow:角度实际值
 * @param [uint8_t]	ANDPID_Period_ms:角度闭环的计算周期, 单位ms
 * @return: [float] PID计算结果
 * @author：dhc
 */
float Motor_FrontAnglePIDCalculate(Motor_t *Motor, float AngleNow, float FEEDFORWARD_RATE_set, uint8_t ANDPID_Period_ms)
{

    float Error;
    float Gyro_compen;
    float SetAng_Error;

    // 计算偏差量Error
    if (Motor->dji.Round_Len != 0)
    { // 电机结构体初始化正常，可以进行跨圈处理计算，获得PID需要的Error
        if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL)
        {
            Error = Motor_Get_DeltaAngle(Motor->ang.set, AngleNow, Motor->dji.Round_Len);
        }
        else
        { // 如果电机使用的是可跨圈角度环模式，则不进行跨圈计算
            Error = Motor_Read_SetAngle(Motor) - Motor_Read_NowAngle(Motor);
        }
    }
    else
        // 电机结构体没有设定闭环整圈长度参数，可能没有调用电机结构体初始化函数。
        while (1)
            continue;

    // 计算角速度补偿值（前馈）
		SetAng_Error = Motor_Get_DeltaAngle(Motor->ang.set, Motor->ang.last_set, 180);
    Gyro_compen = SetAng_Error / ANDPID_Period_ms * 1000 * FEEDFORWARD_RATE_set; // 计算角度环前馈控制的角速度补偿量		
    Motor->ang.last_set = Motor->ang.set;                                        // 更新历史设定值					
				
    PID_Calculate(&Motor->ang, Error); // 完成PID计算
    return Motor->ang.out + Gyro_compen;	
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
 * @brief：电机速度设定值读取
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 读取速度设定值
 * @author：ych
 */
float Motor_Read_SetSpeed(Motor_t *Motor)
{
    return Motor->spe.set;
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
 * @brief：返回编码器的原始数据
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 返回编码器的原始数据
 * @author：ych
 */
float Motor_Read_NowEncoder(Motor_t *Motor)
{
    return Motor->dji.angle;
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
 * @brief：返回当前输出轴实际扭矩
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float]
 * @author：ych
 */
float Motor_Read_NowTorque(Motor_t *Motor)
{
    if (Motor->dji.Angle_CtrlMode == ANGLE_CTRL_ABS)
    {
        return Motor->dji.current;
    }
    else
    {
        return Motor->dji.current * Motor->dji.ratio;
    }
}

/**
 * @brief：返回速度pid计算结果
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 返回pid计算结果
 * @author：ych
 */
float Motor_Read_OutSpeed(Motor_t *Motor)
{
    return Motor->spe.out;
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
 * @brief：电机角度设定值增量式修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的角度设定值
 * @author：ych
 */
void Motor_Write_SetAngle_ADD(Motor_t *Motor, float SetAdd)
{
    float SetCal;
    SetCal = Motor->ang.set + SetAdd;
    if (Motor->dji.Angle_CtrlMode != ANGLE_CTRL_FULL)
    { // 如果是不跨圈的闭环模式
        if (SetCal >= Motor->dji.Set_MAX)
        { // 设定值不可跨圈
            do
            {
                SetCal -= Motor->dji.Round_Len;
            } while (SetCal >= Motor->dji.Set_MAX);
        }
        else if (SetCal < Motor->dji.Set_MIN)
        { // 设定值不可跨圈
            do
            {
                SetCal += Motor->dji.Round_Len;
            } while (SetCal < Motor->dji.Set_MIN);
        }
    }
    Motor->ang.set = SetCal;
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
 * @brief：电机转速设定值增量式修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的转速设定值
 * @author：ych
 */
void Motor_Write_SetSpeed_ADD(Motor_t *Motor, float SetADD)
{
    Motor->spe.set += SetADD;
}

/**
 * @brief：使用角度环输出作为电机转速设定值
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @author：ych
 */
void Motor_Write_SetSpeed_FromAnglePID(Motor_t *Motor)
{
    Motor_Write_SetSpeed_ABS(Motor, Motor->ang.out);
}

/**
 * @brief：屏蔽转速PID中的I
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @author：ych
 */
void Motor_Write_SpeedPID_Idat_Fix(Motor_t *Motor)
{
    Motor->spe.I_Dis = 1;
}

/**
 * @brief：恢复转速PID中的I
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @author：ych
 */
void Motor_Write_SpeedPID_Idat_Recover(Motor_t *Motor)
{
    Motor->spe.I_Dis = 0;
}

/**
 * @brief：电机读取通信信息的回调函数修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [void (*Read_Msg)(rt_uint8_t rxmsg[], DjiMotor_t *motor)] 需要修改的电机读取数据的函数
 * @author：wdl
 */
void Motor_Write_Read_Msg(Motor_t *Motor, void (*Read_Msg)(rt_uint8_t rxmsg[], DjiMotor_t *motor))
{
    Motor->dji.Motor_Read_Msg = Read_Msg;
    Motor->dji.Data_Valid = 0;
    Motor->dji.oldangle_state = RT_ERROR; // 这个修改放后面是保证不会出现oldangle_state=RT_EOK但是Data_Valid=0的情况
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
void motor_init(Motor_t *motor, rt_uint32_t ID, float ratio, Angle_CtrlMode_E ModeSet, rt_int32_t Encoder_Len, rt_int32_t Set_Max, rt_int32_t Set_Min, int MotorReverse)
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
    motor->dji.Encoder_LEN = Encoder_Len;
    motor->dji.reverse_flag = MotorReverse;
    motor->dji.Motor_Read_Msg = motor_readmsg; // 填入默认的回调函数
    if (ModeSet != ANGLE_CTRL_FULL)
    { // 如果选用是不能跨圈的角度闭环模式
        // 记录设定值的允许范围
        motor->dji.Set_MIN = Set_Min;
        motor->dji.Set_MAX = Set_Max;
    }
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
