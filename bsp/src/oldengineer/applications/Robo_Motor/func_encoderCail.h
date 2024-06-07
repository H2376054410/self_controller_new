#ifndef __FUNC_ENCODERCAIL_H__
#define __FUNC_ENCODERCAIL_H__

#include "drv_flash.h"
#include <board.h>

#define ENCODERDATA_NUM 72
#define ENCODERCIRCLE_LEN 360

#define ENCODER_CALI_DATA_ADDR ((uint32_t)(0x08070000))
#define ENCODER_CALI_FLAG_ADDR ((uint32_t)(ENCODER_CALI_DATA_ADDR + 4 * ENCODERDATA_NUM))

typedef enum
{
    Cali_Error = 0,
    Cali_Start,
    Cali_Recording,    // 正在获取数据
    Cali_RecordFinish, // 获取数据完毕
    Cali_WriteStart,   // 开始写入flash
    Cali_WriteFinish,  // 写入flash完毕
    Cali_OK,
} EncoderCali_State_e;

typedef enum
{
    NoReCail = 0,
    ReCali,
} EncoderReCali_flag_e;

/**
 * @brief 获得编码器校正数据
 * @param value_doubt   //角度值
 * @param out_rad           //弧度值
 */
void EncoderCorrectData_Get(float value_doubt,
                            float *out_rad);

/**
 * @brief 编码器数据校准，记录校正数据
 * @param Value_true 准确角度值
 * @param Value_doubt 不准确角度值
 */
void EncoderCailData_Record(rt_int16_t Value_true,
                            float Value_doubt);

/**
 * @brief  从Flash中读取数据
 * @brief  若无数据或需要重测，则会自动重测，完成后函数返回
 * @return int
 */
int Load_EncoderCaliData(void);

#endif
