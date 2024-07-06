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

#include "drv_utils.h"

/**
 * @brief *value向goal的方向移动step，如果超过了就直接等于目标值。
 *
 * @param value
 * @param goal
 * @param step
 */
void utils_step_towards(float *value, float goal, float step)
{
    if (*value < goal)
    {
        if ((*value + step) < goal)
        {
            *value += step;
        }
        else
        {
            *value = goal;
        }
    }
    else if (*value > goal)
    {
        if ((*value - step) > goal)
        {
            *value -= step;
        }
        else
        {
            *value = goal;
        }
    }
}
float utils_calc_ratio(float low, float high, float val)
{
    return (val - low) / (high - low);
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float *angle)
{
    *angle = fmodf(*angle, 360.0f);

    if (*angle < 0.0f)
    {
        *angle += 360.0f;
    }
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad(float *angle)
{
    while (*angle < -PI)
    {
        *angle += 2.0f * PI;
    }

    while (*angle > PI)
    {
        *angle -= 2.0f * PI;
    }
}
/** @brief 将number限制在最大最小值内，如果限制了就返回1  */

int utils_truncate_number(float *number, float min, float max)
{
    int did_trunc = 0;

    if (*number > max)
    {
        *number = max;
        did_trunc = 1;
    }
    else if (*number < min)
    {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}

int utils_truncate_number_int(int *number, int min, int max)
{
    int did_trunc = 0;

    if (*number > max)
    {
        *number = max;
        did_trunc = 1;
    }
    else if (*number < min)
    {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}
/**
 * @brief 判断number对应地址的数据的绝对值是否超过max
 * 		  超过了就把number限制，并返回1
 *
 * @param number
 * @param max
 * @return int
 */
int utils_truncate_number_abs(float *number, float max)
{
    int did_trunc = 0;

    if (*number > max)
    {
        *number = max;
        did_trunc = 1;
    }
    else if (*number < -max)
    {
        *number = -max;
        did_trunc = 1;
    }

    return did_trunc;
}

float utils_map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Limit a value between an upper and lower bound, and when this value exceeds the range of minimum and maximum values,
 * choose a value between the minimum and maximum values to use.
 * @param preferred
 * The preferred value
 * @param min
 * The lower bound
 * @param max
 * The upper bound
 * @return
 * The clamped value
 */
float utils_clamp(float preferred, float min, float max)
{
    if ((min < preferred) && (preferred < max))
    {
        return preferred;
    }
    else if (preferred >= max)
    {
        return max;
    }
    else if (preferred <= min)
    {
        return min;
    }
    return 0.f;
}

/**
 * Truncate absolute values less than tres to zero. The value
 * tres will be mapped to 0 and the value max to max.
 */
void utils_deadband(float *value, float tres, float max)
{
    if (fabsf(*value) < tres)
    {
        *value = 0.0f;
    }
    else
    {
        float k = max / (max - tres);
        if (*value > 0.0f)
        {
            *value = k * *value + max * (1.0f - k);
        }
        else
        {
            *value = -(k * -*value + max * (1.0f - k));
        }
    }
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2)
{
    //	utils_norm_angle(&angle1);
    //	utils_norm_angle(&angle2);
    //
    //	if (fabsf(angle1 - angle2) > 180.0) {
    //		if (angle1 < angle2) {
    //			angle1 += 360.0;
    //		} else {
    //			angle2 += 360.0;
    //		}
    //	}
    //
    //	return angle1 - angle2;

    // Faster in most cases
    float difference = angle1 - angle2;
    while (difference < -180.0f)
        difference += 2.0f * 180.0f;
    while (difference > 180.0f)
        difference -= 2.0f * 180.0f;
    return difference;
}

/**
 * @brief 找到循环圈内的两个点的最短距离 表示从start_val到tar_val的最短差值即存在正负之分
 * 例如：当为电机角度时，若电机转动时，其角度增大认为其逆时针转动
 * 则当返回正值表示电机需要逆时针转动以使其角度由start_val到tar_val
 * @param {float} start_val 起始角度 以low_lim-high_lim范围表示
 * @param {float} tar_val 目标角度 以low_lim-high_lim范围表示
 * @param {float} low_lim 循环里的下限 必须低于high_lim
 * @param {float} high_lim 循环里的上限 必须高于low_lim
 * @return 最短距离 当为0时可能是参数逻辑错误 返回值的范围为(-mid,mid] mid为半循环
 */
float utils_Circle_Shortest_Dis(float start_val, float tar_val, float low_lim, float high_lim)
{
    float diff = tar_val - start_val;
    float Circle_distance = high_lim - low_lim;
    float mid_val = (Circle_distance) / 2;

    if ((low_lim > high_lim) || (start_val > high_lim) || (start_val < low_lim))
        return 0;
    if ((tar_val > high_lim) || (tar_val < low_lim))
        return 0;

    else
    {
        if (diff >= mid_val)
            return (diff - Circle_distance);
        else if (diff < (-mid_val))
            return (diff + Circle_distance);
        else
        {
            if (FLOAT_ISEQUAL(diff, -mid_val))
                return -diff;
            return diff;
        }
    }
}
/**
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param angle1
 * The first angle in radians
 * @param angle2
 * The second angle in radians
 * @return
 * The difference between the angles in radians
 */
float utils_angle_difference_rad(float angle1, float angle2)
{
    float difference = angle1 - angle2;
    while (difference < -PI)
        difference += 2.0f * PI;
    while (difference > PI)
        difference -= 2.0f * PI;
    return difference;
}

/**
 * Get the min value of two integer values
 * @param a
 * First value
 * @param b
 * Second value
 * @return
 * The min value
 */
int utils_min_2_int(int a, int b)
{
    return (a < b) ? a : b;
}

/**
 * Get the max value of two integer values
 * @param a
 * First value
 * @param b
 * Second value
 * @return
 * The max value
 */
int utils_max_2_int(int a, int b)
{
    return (a > b) ? a : b;
}

/**
 * Get the max value of four values
 * @param a
 * First value
 * @param b
 * Second value
 * @param c
 * Third value
 * @param d
 * Fourth value
 * @return
 * The max value
 */
float utils_max_of_4(float a, float b, float c, float d)
{
    float max = a;

    max = (max > b) ? max : b;
    max = (max > c) ? max : c;
    max = (max > d) ? max : d;

    return max;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
float utils_middle_of_3(float a, float b, float c)
{
    float middle;

    if ((a <= b) && (a <= c))
    {
        middle = (b <= c) ? b : c;
    }
    else if ((b <= a) && (b <= c))
    {
        middle = (a <= c) ? a : c;
    }
    else
    {
        middle = (a <= b) ? a : b;
    }
    return middle;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
int utils_middle_of_3_int(int a, int b, int c)
{
    int middle;

    if ((a <= b) && (a <= c))
    {
        middle = (b <= c) ? b : c;
    }
    else if ((b <= a) && (b <= c))
    {
        middle = (a <= c) ? a : c;
    }
    else
    {
        middle = (a <= b) ? a : b;
    }
    return middle;
}

/**
 * Truncate the magnitude of a vector.
 * 向量的缩放，x为横向，y为纵向，max为合成向量最大的模，这个函数是判断合成是否超过了最大值，超过了就成比例放缩
 *
 * @param x
 * The first component.
 *
 * @param y
 * The second component.
 *
 * @param max
 * The maximum magnitude.
 *
 * @return
 * True if saturation happened, false otherwise
 */
bool utils_saturate_vector_2d(float *x, float *y, float max)
{
    bool retval = false;
    float mag = sqrtf(SQUARE(*x) + SQUARE(*y));
    max = fabsf(max);

    if (mag < 1e-10f)
    {
        mag = 1e-10f;
    }

    if (mag > max)
    {
        const float f = max / mag;
        *x *= f;
        *y *= f;
        retval = true;
    }

    return retval;
}

/**
 * Calculate the values with the lowest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the lowest magnitude.
 */
float utils_min_abs(float va, float vb)
{
    float res;
    if (fabsf(va) < fabsf(vb))
    {
        res = va;
    }
    else
    {
        res = vb;
    }

    return res;
}

/**
 * Calculate the values with the highest magnitude.
 *
 * @param va
 * The first value.
 *
 * @param vb
 * The second value.
 *
 * @return
 * The value with the highest magnitude.
 */
float utils_max_abs(float va, float vb)
{
    float res;
    if (fabsf(va) > fabsf(vb))
    {
        res = va;
    }
    else
    {
        res = vb;
    }

    return res;
}

/**
 * Write the flag to the specific bit of a number.
 * @param FlagChar
 * The numebr to be writed in.
 * @param Number
 * The bit specified to writed in.
 * @param Flag
 * The number to write into the bit.
 */
void utils_write_bit(char *FlagChar, unsigned char Num, char Flag)
{
    if (Num > 7)
    {
        return;
    }
    else
    {
        if (Flag)
        {
            *FlagChar = *FlagChar | (1 << Num);
        }
        else
        {
            *FlagChar = *FlagChar & (~(1 << Num));
        }
    }
}

/**
 * Read the specific bit of a number.
 * @param Flags_IN
 * The numebr to read.
 * @param Num
 * The bit specified to read.
 * @return
 * The value on the specfied bit.
 */
char utils_read_bit(char Flags_IN, unsigned char Num)
{
    if (Num > 7)
    {
        return 0;
    }
    else
    {
        if (Flags_IN & (1 << Num))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
}

/**
 * Make sure that min <= number < max
 *
 * @param number
 * The number to normalize.
 * @param min
 * The min number in the circle.
 * @param len
 * The length of the circle.
 */
void utils_norm_circle_number(float *number, float min, float len)
{
    *number = fmodf(*number - min, len) + min;

    while (*number < min)
    {
        *number += len;
    }
}

/**
 * Get the difference between two numbers. Will always be between -len/2 and +len/2.
 * @param number1
 * The first number
 * @param number2
 * The second number.
 * @param len
 * The length of the circle.
 * @return
 * The difference between the numbers
 */
float utils_circle_number_difference(float number1, float number2, float len)
{
    float difference = number1 - number2;
    while (difference < -len / 2)
        difference += len;
    while (difference > len / 2)
        difference -= len;
    return difference;
}

// 坐标点二维旋转函数，传参：横轴(向右)，纵轴(向上)，从纵轴正方向转向横轴正方向(顺时针)的弧度值
void utils_point_rotate(float *Axis1, float *Axis2, float rotation)
{
    float cosdat, sindat;
    float x, y;
    // 计算
    cosdat = cosf(rotation);
    sindat = sinf(rotation);
    x = *Axis1 * cosdat + *Axis2 * sindat;
    y = *Axis2 * cosdat - *Axis1 * sindat;
    *Axis1 = x;
    *Axis2 = y;
}

/**
 * A low pass filter for circled number.
 * @param value
 * The filtered value.
 * @param sample
 * Next sample.
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 * @param min
 * The min number in the circle.
 * @param len
 * The length of the circle.
 * @return
 * The output number of the filter.
 */
float utils_circle_number_LowPass_Filter(float value, float sample, float filter_constant, float min, float len)
{
    float out;
    if (value - sample > len / 2)
        out = UTILS_LP_FAST(value, sample + len, filter_constant);
    else if (sample - value > len / 2)
        out = UTILS_LP_FAST(value, sample - len, filter_constant);
    else
        out = UTILS_LP_FAST(value, sample, filter_constant);
    utils_norm_circle_number(&out, min, len);
    return out;
}

/**
 * Get where the first "1" bit appears from the left side.
 * @param value
 * The number to be search.
 * @param pos
 * Where the result will be stored.
 * @return
 * Whether the "1" bit is found.
 */
bool utils_get_least_significant_setbit_pos(uint32_t value, int *pos)
{
    if (!pos || !value)
        return false;
    uint8_t Pos = 0;
    while (!(value & (1 << Pos)))
    {
        if (Pos == 31)
            break;
        else
            ++Pos;
    }
    if (value & (1 << Pos))
    {
        *pos = Pos;
        return true;
    }
    else
        return false;
}

/**
 * @brief Calculate grades for two matrices Out = AB.
 * @author fwlh
 * @param  matrixA          The matrix A to be calculated, A[1][2] is the element of the third column of the second row
 * @param  matrixB          The matrix B to be calculated
 * @param  Out              Calculate the result matrix
 * @param  rowA             The number of rows in matrix A
 * @param  colA             The number of columns of matrix A
 * @param  rowB             The number of rows in matrix B
 * @param  colB             The number of columns of matrix B
 * @return int              Calculation error is to return -1, else 0
 */
int utils_get_matrix_mutiply(void *matrixA, void *matrixB, void *Out, int rowA, int colA, int rowB, int colB)
{
    if (!matrixA || !matrixB || !Out)
        return -1;
    if (colA != rowB)
        return -1;
    float(*pA)[colA] = (float(*)[colA])matrixA;
    float(*pB)[colB] = (float(*)[colB])matrixB;
    float(*pOut)[colB] = (float(*)[colB])Out;
    for (int i = 0; i < rowA; ++i)
    {
        for (int j = 0; j < colB; ++j)
        {
            pOut[i][j] = 0;
            for (int k = 0; k < colA; ++k)
                pOut[i][j] += pA[i][k] * pB[k][j];
        }
    }
    return 0;
}

/**
 * @brief Gets the 3-d rotation matrix that rotates around the specified axis.
 * @author fwlh
 * @param  RotateAxis       Character that describe the axis of rotation.'x','y''z' or 'X''Y''Z'
 * @param  RotateAngle      Rotation angle(rad)
 * @param  RotateMatrix     The rotation matrix
 */
void utils_get_matrix_rotate_along_axis(char RotateAxis, float RotateAngle, float (*RotateMatrix)[3])
{
    if (!RotateMatrix)
        return;
    switch (RotateAxis)
    {
    case 'x':
    case 'X':
        RotateMatrix[0][0] = 1.f;
        RotateMatrix[0][1] = 0.f;
        RotateMatrix[0][2] = 0.f;
        RotateMatrix[1][0] = 0.f;
        RotateMatrix[1][1] = cosf(RotateAngle);
        RotateMatrix[1][2] = -sinf(RotateAngle);
        RotateMatrix[2][0] = 0.f;
        RotateMatrix[2][1] = sinf(RotateAngle);
        RotateMatrix[2][2] = cosf(RotateAngle);
        return;
    case 'y':
    case 'Y':
        RotateMatrix[0][0] = cosf(RotateAngle);
        RotateMatrix[0][1] = 0.f;
        RotateMatrix[0][2] = -sinf(RotateAngle);
        RotateMatrix[1][0] = 0.f;
        RotateMatrix[1][1] = 1.f;
        RotateMatrix[1][2] = 0.f;
        RotateMatrix[2][0] = sinf(RotateAngle);
        RotateMatrix[2][1] = 0.f;
        RotateMatrix[2][2] = cosf(RotateAngle);
        return;
    case 'z':
    case 'Z':
        RotateMatrix[0][0] = cosf(RotateAngle);
        RotateMatrix[0][1] = -sinf(RotateAngle);
        RotateMatrix[0][2] = 0.f;
        RotateMatrix[1][0] = sinf(RotateAngle);
        RotateMatrix[1][1] = cosf(RotateAngle);
        RotateMatrix[1][2] = 0.f;
        RotateMatrix[2][0] = 0.f;
        RotateMatrix[2][2] = 0.f;
        RotateMatrix[2][1] = 1.f;
        return;
    default:
        return;
    }
}

/**
 * @brief Obtains a rotation matrix centered at the origin that rotates in any three-dimensional direction.
 * @author fwlh
 * @param  RotateDirX       The X component of the rotation direction vector
 * @param  RotateDirY       The Y component of the rotation direction vector
 * @param  RotateDirZ       The Z component of the rotation direction vector
 * @param  RotateAngle      Rotation angle(rad)
 * @param  RotateMatrix     The rotation matrix
 */
void utils_get_3d_rotate_matrix(float RotateDirX, float RotateDirY, float RotateDirZ, float RotateAngle, float (*RotateMatrix)[3])
{
    if (!RotateMatrix)
        return;
    RotateMatrix[0][0] = RotateDirX * RotateDirX * (1 - cosf(RotateAngle)) + cosf(RotateAngle);
    RotateMatrix[0][1] = RotateDirX * RotateDirY * (1 - cosf(RotateAngle)) + RotateDirZ * sinf(RotateAngle);
    RotateMatrix[0][2] = RotateDirX * RotateDirY * (1 - cosf(RotateAngle)) + RotateDirZ * sinf(RotateAngle);
    RotateMatrix[1][0] = RotateDirX * RotateDirY * (1 - cosf(RotateAngle)) - RotateDirZ * sinf(RotateAngle);
    RotateMatrix[1][1] = RotateDirY * RotateDirY * (1 - cosf(RotateAngle)) + cosf(RotateAngle);
    RotateMatrix[1][2] = RotateDirY * RotateDirZ * (1 - cosf(RotateAngle)) + RotateDirX * sinf(RotateAngle);
    RotateMatrix[2][0] = RotateDirX * RotateDirZ * (1 - cosf(RotateAngle)) + RotateDirY * sinf(RotateAngle);
    RotateMatrix[2][1] = RotateDirY * RotateDirZ * (1 - cosf(RotateAngle)) - RotateDirX * sinf(RotateAngle);
    RotateMatrix[2][2] = RotateDirZ * RotateDirZ * (1 - cosf(RotateAngle)) + cos(RotateAngle);
}

#if defined BSP_UTILS_USING_ARM_MATH

/**
 * @brief 矩阵初始化
 * @param S 矩阵结构体 nRows 矩阵行数  nCols 矩阵列数  pData 存储矩阵元素的二维数组
 */
void matrix_init(matrix *S, uint16_t nRows, uint16_t nCols, float32_t *pData)
{
    arm_mat_init_f32(S, nRows, nCols, pData);
}

/**
 * @brief 矩阵相加
 * @param A,B为两个加数矩阵，C为和矩阵
 */
matrix *matrix_add(matrix *A, matrix *B, matrix *C)
{
    arm_mat_add_f32(A, B, C);

    return C;
}

/**
 * @brief 矩阵相除
 * @param A/B，即A*inv(B),C为结果
 */
matrix *matrix_sub(matrix *A, matrix *B, matrix *C)
{
    arm_mat_sub_f32(A, B, C);

    return C;
}

/**
 * @brief 矩阵相乘
 * @param A*B，C为结果
 */
matrix *matrix_multiply(matrix *A, matrix *B, matrix *C)
{
    arm_mat_mult_f32(A, B, C);

    return C;
}

/**
 * @brief 矩阵求逆
 * @param A 原矩阵，A_1为逆矩阵
 */
matrix *matrix_inverse(matrix *A, matrix *A_1)
{
    arm_mat_inverse_f32(A, A_1);

    return A_1;
}

/**
 * @brief 矩阵转置
 * @param A 为原矩阵，A_T为转置矩阵
 */
matrix *matrix_trans(matrix *A, matrix *A_T)
{
    arm_mat_trans_f32(A, A_T);

    return A_T;
}

/**
 * @brief 矩阵元素填充
 * @param A 为矩阵，Row、Col为元素的行、列，contant为填充内容
 */
void matrix_data_fresh(matrix *A, uint16_t Row, uint16_t Col, float32_t contant)
{
    float32_t **a;
    a = &(A->pData);
    if (Row <= A->numRows && Col <= A->numCols)
        a[Row - 1][Col - 1] = contant;
    else
        return;
}

/**
 * @brief 5*5矩阵连乘（不支持其他形式的矩阵）
 * @param A*B*C，answer为结果
 */
matrix *matrix_tri_mutiply_5(matrix *A, matrix *B, matrix *C, matrix *answer)
{
    matrix temp;
    float32_t temp_data[5][5];
    matrix_init(&temp, 5, 5, temp_data[0]);

    matrix_multiply(A, B, &temp);
    matrix_multiply(&temp, C, answer);

    return answer;
}
/**
 * @brief 求取矢量的模长
 * @param[in]     pSrc        向量的首地址
 * @param         blockSize   每个向量中的样本数
 * @param[out]    result      模长地址
 */
void utils_modulus_get(const float32_t *pSrc,
                       uint32_t blockSize,
                       float32_t *result)
{
    arm_dot_prod_f32(pSrc, pSrc, blockSize, result);
    if (ARM_MATH_SUCCESS != arm_sqrt_f32(*result, result))
    {
        while (1)
            continue;
    }
}

/**
 * @brief 求取两个矢量的夹角
 * @brief 求取两个矢量的夹角
 * @param pSrcA             A向量的首地址
 * @param pSrcB             B向量的首地址
 * @param blockSize         每个向量中的样本数
 * @param result            存储夹角的地址
 */
int utils_vectorangle_get(const float32_t *pSrcA,
                          const float32_t *pSrcB,
                          uint32_t blockSize,
                          float32_t *result)
{
    float modulus_A;
    float modulus_B;
    float Value_Temp;

    utils_modulus_get(pSrcA, blockSize, &modulus_A);        // A向量模长
    utils_modulus_get(pSrcB, blockSize, &modulus_B);        // B向量模长
    arm_dot_prod_f32(pSrcA, pSrcB, blockSize, &Value_Temp); // 向量点乘

    if (FLOAT_ISEQUAL(0, modulus_A) && (FLOAT_ISEQUAL(0, modulus_B)))
    {
        // 保证分母不是零
        Value_Temp = Value_Temp / (modulus_A * modulus_B);
        Value_Temp = acosf(Value_Temp); // 求夹角
        *result = Value_Temp;
        return 1;
    }
    else
    {
        *result = 0;
        return 0;
    }
}

#endif
