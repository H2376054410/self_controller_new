#ifndef __DRV_GYROSCOPE_DATA_H__
#define __DRV_GYROSCOPE_DATA_H__

#define DATA_AMPLIFICATION_RATIO 100

typedef struct
{
    float ang;
    float spe;
} MotionData_s;

typedef enum
{
    Gyrodata_invalid = 0,
    Gyrodata_valid,
} Gyrodata_validity_e;

typedef struct
{
    float offset;                   // 校正零点
    Gyrodata_validity_e Data_Valid; // 数据有效性

    MotionData_s origin;    // 原始数据
    MotionData_s rad;       // 弧度值
} Gyroscope_data_s;

/**
 * @brief 陀螺仪角度角速度数据，原始值转弧度值
 * @param GyroStateData
 */
void Gyro_angle2rad(Gyroscope_data_s *GyroStateData);

/**
 * @brief 陀螺仪零点偏移设定
 * @param GyroStateData
 * @param offset
 */
void Gyro_offset_set(Gyroscope_data_s *GyroStateData,
                     float offset);

#endif /*__DRV_GYROSCOPE_DATA_H__*/
