#ifndef __DRV_MOTOR_H
#define __DRV_MOTOR_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "pid.h"

/*电机消息发送ID（STDID）*/
typedef enum
{
    STDID_Pantilt = 0x1FF,
    STDID_launch = 0x200,

    STDID_ANGLE_WHEEL = 0x1FF, // 6020 0x1FF
    STDID_RUN_WHEEL = 0x200,   // 3508 0x200

} SendID_e;

typedef enum
{
    // 若切换模式，则需要重新调参数
    ANGLE_CTRL_ABS,   // 按照编码器数值进行控制，设定值不能跨圈，减速比为1的云台电机推荐使用此设置，也可使用extra模式
    ANGLE_CTRL_EXTRA, // 按照减速电机的电机输出轴转动情况进行控制，设定值不能跨圈， 单位°
    ANGLE_CTRL_FULL   // 按照减速电机的电机输出轴转动情况进行控制，设定值可以跨圈， 单位°
} Angle_CtrlMode_E;

typedef struct DjiMotor
{
    rt_uint32_t motorID; /*电机反馈报文ID*/

    rt_int32_t angle;     /*编码器原始角度 0~8191*/
		rt_int32_t lastangle;
    rt_int32_t old_angle; /*上一次编码器数值*/
    rt_int32_t Encoder_LEN;
	  float Encoder_LEN_DM;
    rt_err_t oldangle_state; /* 记录当前历史数据是否有效，首次使用时历史数据无效 */
    rt_int32_t speed;
    rt_int32_t current;
    rt_uint8_t temperature;

		rt_uint8_t ErrorID;

    rt_uint8_t reverse_flag; // 电机数据是否需要反向, 需要反向为 1
    float extra_angle;       // 记录的圈数之外多出来的角度 单位 °
    rt_int32_t loop;         /*一共所转圈数*/
    float ratio;             /* 填写电机输出轴转动一圈时，编码器数据转过的圈数，以完整的M3508为例，应填写19.0f */

    Angle_CtrlMode_E Angle_CtrlMode; // 记录角度控制模式，如果有减速箱则为EXTRA模式，无减速箱则用ABS模式
    rt_int32_t Round_Len;            // 角度环计算时的整圈长度
    rt_int32_t Set_MAX;              // 角度环计算时的角度数值的最大值
    rt_int32_t Set_MIN;              // 角度环计算时的角度数值的最小值
    rt_int32_t Data_Valid;           /* 是否在初始化电机结构体后接收到新报文 */
    void (*Motor_Read_Msg)(rt_uint8_t rxmsg[], struct DjiMotor *motor);
    rt_int32_t FreshTick; // 记录最后一次数据刷新时刻，用于判断电机是否离线
} DjiMotor_t;

typedef struct __Motor_t
{
    DjiMotor_t dji;
    pid_t ang;
    pid_t spe;

} Motor_t;
/**
 * @brief  读取can中的电机数据
 * @param  rxmsg：反馈报文数据
 * @param  motor：电机数据结构
 * @retval None
 */
extern void motor_readmsg(rt_uint8_t rxmsg[], DjiMotor_t *motor);

/**
 * @brief  读取达妙电机数据
 * @param  rxmsg：反馈报文数据
 * @param  motor：电机数据结构
 * @retval None
 */
extern void motor_readmsg_DM(rt_uint8_t rxmsg[], DjiMotor_t *motor);
/**
 * @brief  读取go电机数据，都是未经处理的原始数据
 * @param  rxmsg：反馈报文数据
 * @param  motor：电机数据结构
 * @retval None
 */
extern void go_motor_readmsg(rt_uint8_t rxmsg[], DjiMotor_t *motor);

/**
 * @brief  读取轮毂电机数据（只有力矩Nm和转速rpm圈每分钟，两个float，一前一后）
 * @param  rxmsg：反馈报文数据
 * @param  motor：电机数据结构
 * @retval None
 */
extern void hub_motor_readmsg(rt_uint8_t rxmsg[], DjiMotor_t *motor);

/**
 * @brief  电机电流发送
 * @param  dev：can设备
 * @param  setcurn:设定电流
 * @retval RT_EOK or RT_ERROR(成功，失败或其他报错)
 */
extern rt_size_t motor_current_send(rt_device_t dev,
                                    SendID_e ID,
                                    rt_int16_t setcur1,
                                    rt_int16_t setcur2,
                                    rt_int16_t setcur3,
                                    rt_int16_t setcur4);

/**
 * @brief  达妙电机电流发送
 * @param  dev：can设备
 * @param  setcurn:设定电流
 * @retval RT_EOK or RT_ERROR(成功，失败或其他报错)
 */
extern rt_size_t motor_current_send_DM(rt_device_t dev,SendID_e ID,rt_int16_t torque);

/**
 * @brief：电机转动量计算（跨圈处理）
 * @param float Angle1: 要转到的角度对应的数值
 * @param float Angle2: 转动起点数值
 * @param float Round_Len: 电机整圈对应的数值
 * @return float: 输出从Angle2到Angle1需要的最短路径的距离长度, 范围 负半圈~正半圈，正好半圈时取正半圈
 * @author：ych
 */
extern float Motor_Get_DeltaAngle(float Angle1, float Angle2, float Round_Len);

/**
 * @brief：电机角度环pid输出的计算，此函数 *不会* 直接修改速度环设定值
 * @brief：如果需要修改转速换设定值，则可调用其它接口，如 Motor_Write_SetSpeed_FromAnglePID 或 Motor_Write_SetSpeed_ABS
 * @param [Motor_t*]	Motor:需要进行角度环计算的电机的结构体
 * @param [float]	AngleNow:角度实际值
 * @return: [float] PID计算结果
 * @author：ych
 */
extern float Motor_AnglePIDCalculate(Motor_t *Motor, float AngleNow);

/**
 * @brief：带前馈的电机角度环pid输出的计算，不会修改速度环设定值
 * @brief：调用过程中需要保证同一个电机的设定值和传入的实际值的单位相同
 * @param [Motor_t*]	Motor:需要角度环计算的电机的结构体
 * @param [float]	AngleNow:角度实际值
 * @param [uint8_t]	ANDPID_Period_ms:角度闭环的计算周期, 单位ms
 * @return: [float] PID计算结果
 * @author：zzj
 */
extern float Motor_FrontAnglePIDCalculate(Motor_t *Motor, float AngleNow, float FEEDFORWARD_RATE_set, uint8_t ANDPID_Period_ms);

/**
 * @brief：电机转速环pid输出的计算
 * @param [Motor_t*]	Motor:需要速度环计算的电机的结构体
 * @param [float]	SpeedNow:转速实际值
 * @return: [float] PID计算结果
 * @author：ych
 */
extern float Motor_SpeedPIDCalculate(Motor_t *Motor, float SpeedNow);

/**
 * @brief：电机角度设定值读取
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 角度设定值
 * @author：ych
 */
extern float Motor_Read_SetAngle(Motor_t *Motor);

/**
 * @brief：电机速度设定值读取
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 速度设定值
 * @author：ych
 */
extern float Motor_Read_SetSpeed(Motor_t *Motor);

/**
 * @brief：返回编码器的原始数据
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 返回编码器的原始数据
 * @author：ych
 */
extern float Motor_Read_NowEncoder(Motor_t *Motor);

/**
 * @brief：返回转速数据 不同控制模式下返回的数据不同
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 按照初始化时设置的电机控制模式返回对应的的转速数据
 * @author：ych
 */
extern float Motor_Read_NowSpeed(Motor_t *Motor);

/**
 * @brief：按照电机的角度控制模式，获取相应的电机当前实际角度数据
 * @brief：ABS控制模式下，返回编码器的原始数据
 * @brief：EXTRA控制模式下，返回电机减速箱输出轴的角度，以上电位置为零点，0~360°
 * @brief：FULL控制模式下，返回电机减速箱输出轴上电后转动过的总角度，以上电位置为零点，单位°
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 返回当前电机控制模式下闭环所需的角度数值
 * @author：ych
 */
extern float Motor_Read_NowAngle(Motor_t *Motor);

/**
 * @brief：返回当前实际扭矩
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 按照初始化时设置的电机控制模式返回对应的的转速数据
 * @author：ych
 */
extern float Motor_Read_NowTorque(Motor_t *Motor);

/**
 * @brief：返回速度pid计算结果
 * @param [Motor_t*]	Motor:需要读取的电机的结构体
 * @return [float] 返回pid计算结果
 * @author：ych
 */
extern float Motor_Read_OutSpeed(Motor_t *Motor);

/**
 * @brief：电机角度设定值绝对式修改
 * @brief：函数会根据电机角度闭环工作模式选择性进行设定值的跨圈处理
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的角度设定值
 * @author：ych
 */
extern void Motor_Write_SetAngle_ABS(Motor_t *Motor, float Set);

/**
 * @brief：电机角度设定值增量式修改
 * @brief：不能跨圈的工作模式下，此函数会自动进行跨圈处理
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的角度设定值
 * @author：ych
 */
extern void Motor_Write_SetAngle_ADD(Motor_t *Motor, float SetAdd);

/**
 * @brief：电机转速设定值绝对式修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的转速设定值
 * @author：ych
 */
extern void Motor_Write_SetSpeed_ABS(Motor_t *Motor, float Set);

/**
 * @brief：电机转速设定值增量式修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [float]	Set:新的转速设定值
 * @author：ych
 */
extern void Motor_Write_SetSpeed_ADD(Motor_t *Motor, float SetADD);

/**
 * @brief：使用角度环输出作为电机转速设定值
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @author：ych
 */
extern void Motor_Write_SetSpeed_FromAnglePID(Motor_t *Motor);

/**
 * @brief：屏蔽转速PID中的I
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @author：ych
 */
extern void Motor_Write_SpeedPID_Idat_Fix(Motor_t *Motor);

/**
 * @brief：恢复转速PID中的I
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @author：ych
 */
extern void Motor_Write_SpeedPID_Idat_Recover(Motor_t *Motor);

/**
 * @brief：电机读取通信信息的回调函数修改
 * @param [Motor_t*]	Motor:需要修改的电机的结构体
 * @param [void (*Read_Msg)(rt_uint8_t rxmsg[], DjiMotor_t *motor)] 需要修改的电机读取数据的函数
 * @author：wdl
 */
extern void Motor_Write_Read_Msg(Motor_t *Motor, void (*Read_Msg)(rt_uint8_t rxmsg[], DjiMotor_t *motor));

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
extern void motor_init(Motor_t *motor, rt_uint32_t ID, float ratio, Angle_CtrlMode_E ModeSet, rt_int32_t Encoder_Len, rt_int32_t Set_Max, rt_int32_t Set_Min, int MotorReverse);


extern void motor_init_DM(Motor_t *motor, rt_uint32_t ID, float ratio, Angle_CtrlMode_E ModeSet, rt_int32_t Encoder_Len, rt_int32_t Set_Max, rt_int32_t Set_Min, int MotorReverse);
#endif
