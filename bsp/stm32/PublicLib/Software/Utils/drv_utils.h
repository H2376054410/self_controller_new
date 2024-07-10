/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef UTILS_H_
#define UTILS_H_

#include "rtconfig.h"
#include <stdbool.h>
#include <stdint.h>
#if defined BSP_UTILS_USING_ARM_MATH
#include <arm_math.h>
#define matrix arm_matrix_instance_f32
#else
#include <math.h>
#endif

#ifndef PI
#define PI (3.141592653589793f)
#endif

void utils_step_towards(float *value, float goal, float step);
float utils_calc_ratio(float low, float high, float val);
void utils_norm_angle(float *angle);
void utils_norm_angle_rad(float *angle);
void utils_norm_circle_number(float *number, float min, float len);
int utils_truncate_number(float *number, float min, float max);
int utils_truncate_number_int(int *number, int min, int max);
int utils_truncate_number_abs(float *number, float max);
float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max);
float utils_clamp(float preferred, float min, float max);
void utils_deadband(float *value, float tres, float max);
float utils_angle_difference(float angle1, float angle2);
float utils_angle_difference_rad(float angle1, float angle2);
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num);
int utils_min_2_int(int a, int b);
int utils_max_2_int(int a, int b);
float utils_max_of_4(float a, float b, float c, float d);
float utils_middle_of_3(float a, float b, float c);
int utils_middle_of_3_int(int a, int b, int c);
float utils_min_abs(float va, float vb);
float utils_max_abs(float va, float vb);
void utils_write_bit(char *FlagChar, unsigned char Num, char Flag);
char utils_read_bit(char Flags_IN, unsigned char Num);
float utils_circle_number_difference(float number1, float number2, float len);
void utils_point_rotate(float *Axis1, float *Axis2, float rotation);
float utils_circle_number_LowPass_Filter(float value, float sample, float filter_constant, float min, float len);
bool utils_get_least_significant_setbit_pos(uint32_t value, int *pos);
int utils_get_matrix_mutiply(void *matrixA, void *matrixB, void *Out, int rowA, int colA, int rowB, int colB);
void utils_get_matrix_rotate_along_axis(char RotateAxis, float RotateAngle, float (*RotateMatrix)[3]);
void utils_get_3d_rotate_matrix(float RotateDirX, float RotateDirY, float RotateDirZ, float RotateAngle, float (*RotateMatrix)[3]);
float utils_Circle_Shortest_Dis(float start_val, float tar_val, float low_lim, float high_lim);
#if defined BSP_UTILS_USING_ARM_MATH
void matrix_init(matrix *S, uint16_t nRows, uint16_t nCols, float32_t *pData);
matrix *matrix_add(matrix *A, matrix *B, matrix *C);
matrix *matrix_sub(matrix *A, matrix *B, matrix *C);
matrix *matrix_multiply(matrix *A, matrix *B, matrix *C);
matrix *matrix_inverse(matrix *A, matrix *A_1);
matrix *matrix_trans(matrix *A, matrix *A_T);
void matrix_data_fresh(matrix *A, uint16_t Row, uint16_t Col, float32_t contant);
matrix *matrix_tri_mutiply_5(matrix *A, matrix *B, matrix *C, matrix *answer);

void utils_modulus_get(const float32_t *pSrc, uint32_t blockSize, float32_t *result);
int utils_vectorangle_get(const float32_t *pSrcA, const float32_t *pSrcB, uint32_t blockSize, float32_t *result);
#endif
// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN_F(x) (((x) < 0.f) ? -1 : 1)

// Squared
#define SQUARE(x) ((x) * (x))

// nan and infinity check for floats
#define UTILS_IS_INF_F(x) ((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))
#define UTILS_IS_NAN(x) isnan(x)
#define UTILS_NAN_ZERO_F(x) (x = UTILS_IS_NAN(x) ? 0.f : x)

// Handy conversions for radians/degrees and RPM/radians-per-second
#define DEG2RAD_f(deg) ((deg) * (float)(PI / 180.0f))
#define RAD2DEG_f(rad) ((rad) * (float)(180.0f / PI))
#define RPM2RADPS_f(rpm) ((rpm) * (float)((2.0f * PI) / 60.0f))
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0f / (2.0f * PI)))

#define SETBIT(x, y) ((x) |= (1 << y))  // 将X的第Y位置1 标号从0开始
#define CLRBIT(x, y) ((x) &= ~(1 << y)) // 将X的第Y位清0 标号从0开始
#define READBIT(x, y) ((x) & (1 << y))  // 读取某一位 标号从0开始

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant) (value - ((filter_constant) * ((value) - (sample))))
#define UTILS_HP_FAST(value, sample, sample_last, filter_constant) (filter_constant * (sample - sample_last + value))
/**
 * @brief 浮点数判断相等
 * @brief 相等返回1，不相等返回0
 */
#define FLOAT_ISEQUAL(A, B) (fabsf(A - B) < 0.0001f ? 1 : 0)

#endif /* UTILS_H_ */
