#ifndef __DRV_ENCODER_H__
#define __DRV_ENCODER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define ENCODERRAD_LEN (2 * PI)
#define ENCODERORIGIN_LEN (4096.0f)

#define DEFAULT_VALUE 0

typedef enum
{
    forward = 0,
    reverse,
} reverse_flag_e;

typedef enum
{
    invalid = 0,
    valid,
} encoderdata_validity_e;

typedef enum
{
    AngleMode_ABS = 0, // 直出按照编码器原始数值直接进行弧度换算，不跨圈处理
    AngleMode_FULL,    // 对编码器数值进行跨圈处理，输出跨圈后的弧度
} anglemode_e;

typedef struct
{
    /*编码器基本设置*/
    rt_uint8_t encoderID;        // 编码器反馈报文ID
    rt_uint16_t Encoder_LEN;     // 编码器长度  默认4096
    reverse_flag_e reverse_flag; // 数据反向
    rt_int32_t FreshTick;        // 记录最后一次数据刷新时刻，用于判断编码器是否离线
    anglemode_e AngleMode;       // 角度模式

    /*接收原始数据*/
    rt_int32_t angle_origin; // 编码器原始角度值

    /*编码器换算值*/
    float rad_now;   // 编码器弧度值
    float rad_old;   // 上一次编码器弧度值
    float rad_extra; // 记录的圈数之外多出来的角度 单位 rad
    float loop;      // 上电之后，一共所转圈数

    /*数据保护处理*/
    rt_err_t olddata_state; // 记录当前历史数据是否有效，首次使用时历史数据无效
    encoderdata_validity_e Data_Valid;
} Encoder_s;

/**
 * @brief 读can中的编码器数据
 * @param rxmsg
 * @param encoder
 */
void Encoder_readmsg(rt_uint8_t rxmsg[],
                     Encoder_s *encoder);

/**
 * @brief  读取编码器原始数据
 * @return float
 */
float Encoder_AngleOrigin_Read(Encoder_s *encoder);

/**
 * @brief  读取编码器弧度值
 * @param  encoder
 * @return float
 */
float Encoder_Radnow_Read(Encoder_s *encoder);

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
                      reverse_flag_e encoder_reverse);

/**********************跨圈处理*******************/
typedef struct
{
    rt_int16_t loop;   // 圈数
    float value_last;  // 上一次的值
    float Circle_Len;  // 一圈的长度
    float value_extra; // 跨圈
    float crossdata;   // 跨圈数据
} CrossCircleData_s;   // 跨圈数据结构体

/**
 * @brief 跨圈数据初始化
 * @param data
 * @param Len
 */
void CrossCircleData_Init(CrossCircleData_s *data,
                          float Len);

/**
 * @brief 跨圈处理函数
 * @param data
 * @param value_now
 */
void CrossCircle_ProcessingNormal(CrossCircleData_s *data,
                            float value_now);

/**
 * @brief 特殊跨圈处理
 * @param data
 * @param value_now
 */
void CrossCircle_ProcessingSpecial(CrossCircleData_s *data,
                                   float value_now);

/**
 * @brief 跨圈数据读取
 * @param data
 */
float CrossCircle_Data_Read(CrossCircleData_s *data);

/**
 * @brief 一般跨圈处理，超圈直接倍增(减)一圈
 * @param in
 * @param len
 * @param min
 * @param out
 */
void CrossCircle_Normal(float in, float len,
                        float min, float *out);

#endif /*__DRV_ENCODER_H__*/
