#include "drv_gyroscope_data.h"
#include "drv_utils.h"

/**
 * @brief 陀螺仪角度角速度数据，原始值转弧度值
 * @param GyroStateData
 */
void Gyro_angle2rad(Gyroscope_data_s *GyroStateData)
{
    GyroStateData->rad.ang = DEG2RAD_f(GyroStateData->origin.ang / DATA_AMPLIFICATION_RATIO) -
                             GyroStateData->offset;
    GyroStateData->rad.spe = DEG2RAD_f(GyroStateData->origin.spe / DATA_AMPLIFICATION_RATIO);
}

/**
 * @brief 陀螺仪零点偏移设定
 * @param GyroStateData
 * @param offset
 */
void Gyro_offset_set(Gyroscope_data_s *GyroStateData,
                     float offset)
{
    GyroStateData->offset = offset;
}
