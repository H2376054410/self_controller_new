/**
 * @file drv_encoder.c
 * @brief 编码器数据处理文件
 * @author mylj
 * @version 1.0
 * @date 2023-07-20
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */
#include "drv_encoder.h"
#include "drv_utils.h"

/**
 * @brief：转轴转动量计算（跨圈处理）
 * @param float rad1: 要转到的角度对应的数值
 * @param float rad2: 转动起点数值
 * @return float: 输出从rad2到rad1需要的最短路径的距离长度, 范围 负半圈~正半圈，正好半圈时取正半圈
 */
static float Encoder_Get_DeltaAngle(float rad1, float rad2)
{
    float DAngle, HalfRound;
    DAngle = rad1 - rad2;
    HalfRound = ENCODERRAD_LEN / 2;
    if (DAngle >= 0)
    {
        if (DAngle > HalfRound)
            DAngle -= ENCODERRAD_LEN;
    }
    else
    {
        if (DAngle <= -HalfRound)
            DAngle += ENCODERRAD_LEN;
    }
    return DAngle;
}

/**
 * @brief 编码器原始值转弧度
 * @param in
 * @return out
 */
static float Encoder_angleorigin2rad(float in)
{
    return (in / ENCODERORIGIN_LEN * ENCODERRAD_LEN);
}

/**
 * @brief  对反馈角度进行换算（弧度）
 * @param  in
 */
static void Encoder_angle_adjust(Encoder_s *in)
{
    float radtemp = 0;
    if (in->olddata_state == RT_ERROR)
    { // 第一次
        in->olddata_state = RT_EOK;
        in->rad_now = Encoder_angleorigin2rad(in->angle_origin);
        in->rad_old = in->rad_now;
        in->Data_Valid = valid;
        in->loop = 0; // motor初始化前CAN就会有通信，所以初始化后的第一次CAN通信时程序才运行到这里，此时需要清空loop的数值
    }
    else
    {
        in->rad_now = Encoder_angleorigin2rad(in->angle_origin);
        radtemp = Encoder_Get_DeltaAngle(in->rad_now, in->rad_old) + in->rad_extra; // 角度积分
        if (radtemp > ENCODERRAD_LEN)
        { // 积分大于一圈
            in->rad_extra = radtemp - ENCODERRAD_LEN;
            in->loop += 1;
        }
        else if (radtemp < 0)
        { // 积分小于0
            in->rad_extra = radtemp + ENCODERRAD_LEN;
            in->loop -= 1;
        }
        else
        {                            // 正常
            in->rad_extra = radtemp; // 更新积分值
        }
        in->rad_old = in->rad_now;
    }
}

/**
 * @brief 读can中的编码器数据
 * @param rxmsg
 * @param encoder
 */
void Encoder_readmsg(rt_uint8_t rxmsg[],
                     Encoder_s *encoder)
{
    rt_uint8_t num;
    num = 2 * (encoder->encoderID - 1);
    if (encoder->reverse_flag)
    {
        encoder->angle_origin = encoder->Encoder_LEN -
                                (rt_uint16_t)(rxmsg[num + 1] << 8 | rxmsg[num]);
    }
    else
    {
        encoder->angle_origin = (rt_uint16_t)(rxmsg[num + 1] << 8 | rxmsg[num]);
    }

    Encoder_angle_adjust(encoder);
    encoder->FreshTick = rt_tick_get();
}

/**
 * @brief  读取编码器原始数据
 * @param  encoder
 * @return float
 */
float Encoder_AngleOrigin_Read(Encoder_s *encoder)
{
    return encoder->angle_origin;
}

/**
 * @brief  读取编码器弧度值
 * @param  encoder
 * @return float
 */
float Encoder_Radnow_Read(Encoder_s *encoder)
{
    switch (encoder->AngleMode)
    {
    case AngleMode_ABS:
        return encoder->rad_now;
    case AngleMode_FULL:
        return encoder->rad_extra + ENCODERRAD_LEN * encoder->loop;
    default:
        return 0;
    }
}

/**
 * @brief 编码器数据初始化
 * @param encoder
 * @param encoderID         //用于处理报文（1~4）
 * @param Encoder_LEN
 * @param AngleMode
 * @param encoder_reverse
 */
void EncoderData_Init(Encoder_s *encoder,
                      rt_uint8_t encoderID,
                      rt_uint16_t Encoder_LEN,
                      anglemode_e AngleMode,
                      reverse_flag_e encoder_reverse)
{
    // 保护处理
    if (Encoder_LEN == (rt_uint16_t)DEFAULT_VALUE)
    {
        encoder->Encoder_LEN = ENCODERORIGIN_LEN; // 默认值
    }
    else
    {
        encoder->Encoder_LEN = Encoder_LEN;
    }

    if (1 <= encoderID && encoderID <= 4)
    {
        encoder->encoderID = encoderID;
    }
    else
    {
        while (1)
            ;
    }

    encoder->loop = 0;
    encoder->olddata_state = RT_ERROR;
    encoder->rad_extra = 0;
    encoder->rad_old = 0;
    encoder->AngleMode = AngleMode;
    encoder->Data_Valid = invalid;
    encoder->reverse_flag = encoder_reverse;
}

/*******************************跨圈处理函数****************************/

/**
 * @brief 跨圈数据初始化
 * @param data
 * @param Len
 */
void CrossCircleData_Init(CrossCircleData_s *data,
                          float Len)
{
    data->loop = 0;
    data->value_last = 0;
    data->value_extra = 0;
    data->Circle_Len = Len;
}

/**
 * @brief 普通跨圈处理函数
 * @param data
 * @param value_now
 */
void CrossCircle_ProcessingNormal(CrossCircleData_s *data,
                                  float value_now)
{
    float DAngle, HalfRound;
    DAngle = value_now - data->value_last;
    HalfRound = data->Circle_Len / 2;

    if (DAngle >= 0)
    {
        if (DAngle > HalfRound)
        {
            DAngle -= data->Circle_Len;
            data->loop--;
        }
    }
    else
    {
        if (DAngle <= -HalfRound)
        {
            DAngle += data->Circle_Len;
            data->loop++;
        }
    }
    data->value_last = value_now;
}

/**
 * @brief 特殊跨圈处理
 * @param data
 * @param value_now
 */
void CrossCircle_ProcessingSpecial(CrossCircleData_s *data,
                                   float value_now)
{
    float DAngle, HalfRound;
    DAngle = value_now - data->value_last;
    HalfRound = data->Circle_Len / 2;
    if (DAngle >= 0)
    {
        if (DAngle > HalfRound)
            DAngle -= data->Circle_Len;
    }
    else
    {
        if (DAngle <= -HalfRound)
            DAngle += data->Circle_Len;
    }
    DAngle += data->value_extra; // 角度积分
    if (DAngle > data->Circle_Len)
    { // 积分大于一圈
        data->value_extra = DAngle - data->Circle_Len;
        data->loop++;
    }
    else if (DAngle < 0)
    { // 积分小于0
        data->value_extra = DAngle + data->Circle_Len;
        data->loop--;
    }
    else
    {                               // 正常
        data->value_extra = DAngle; // 更新积分值
    }
    data->value_last = value_now;
    data->crossdata = data->value_extra + data->Circle_Len * data->loop;
}

/**
 * @brief 跨圈数据读取
 * @param data
 */
float CrossCircle_Data_Read(CrossCircleData_s *data)
{
    return data->crossdata;
}

/**
 * @brief 一般跨圈处理，超圈直接倍增(减)一圈
 * @param in
 * @param len
 * @param min
 * @param out
 */
void CrossCircle_Normal(float in, float len,
                        float min, float *out)
{
    if (in > len + min)
    {
        *out = in - len;
    }
    else if (in < min)
    {
        *out = in + len;
    }
    else
    {
        *out = in;
    }
}
