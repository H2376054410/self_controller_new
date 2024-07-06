#ifndef __DRV_VECTOR_H__
#define __DRV_VECTOR_H__

#define ASSERT_FULL (1)

typedef struct
{
    float x;
    float y;
    float z;
} VectorXYZ_Str;

typedef struct
{
    float x;
    float y;
} VectorXY_Str;

typedef struct
{
    float pitch;
    float yaw;
    float mod;
} VectorPYM_Str;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} EulerAngle_Str;

typedef struct
{
    float r;
    float t;
} VectorRT_Str;

/**
 * @brief  二维矢量比较
 * @param  v1/2 矢量1/2
 * @return 当v1 == v2时,返回1;否则返回0
 **/
#define VECTOR2_CMP(v1, v2) (v1.x == v2.x && v1.y == v2.y)

/**
 * @brief 判断该浮点数是否为0
 * @param Value             需要判断的浮点数
 * @param Tolerance         容差
 * @return                  0则是零，1则非零
 */
int Float_ZeroIf(float Value, float Tolerance);

/**
 * @brief 浮点数在阈值范围内为0
 * @param Value             需要判断的浮点数
 * @param Tolerance         容差
 */
void Float_ZeroIs(float *Value, float Tolerance);

/**
 * @brief 三维矢量二分法
 */
void Vector3D_Dichotomy(VectorXYZ_Str *in1,
                        VectorXYZ_Str *in2,
                        VectorXYZ_Str *out);

/**
 * @brief 求解反余弦pro
 * @brief 避免因浮点数精度损失而造成：被取反余弦的值的绝对值略大于1
 * @param value 被取反余弦的值
 * @return float
 */
float acosf_pro(float value);

/**
 * @brief 求解反正弦pro
 * @brief 避免因浮点数精度损失而造成：被取反正弦的值的绝对值略大于1
 * @param value 被取反正弦的值
 * @return float
 */
float asinf_pro(float value);

/**
 * @brief 三维向量初始化
 */

void Vector3D_Init(VectorXYZ_Str *vector);

/**
 * @brief 二维向量初始化
 */
void Vector2D_Init(VectorXY_Str *vector);

/**
 * @brief 三维向量赋值
 */
void Vector3D_Assign(float x, float y,
                     float z, VectorXYZ_Str *out);

/**
 * @brief 二维向量赋值
 */
void Vector2D_Assign(float x, float y,
                     VectorXYZ_Str *out);

/**
 * @brief 三维向量传递值
 */
void Vector3D_Transmit(VectorXYZ_Str *in, VectorXYZ_Str *out);

/**
 * @brief 二维向量传递值
 */
void Vector2D_Transmit(VectorXY_Str *in, VectorXY_Str *out);

/**
 * @brief 三维向量数乘
 * @param k 数乘的倍数
 */
void Vector3D_Scale(VectorXYZ_Str *in, float k, VectorXYZ_Str *out);

/**
 * @brief 二维向量数乘
 * @param k 数乘的倍数
 */
void Vector2D_Scale(VectorXY_Str *in, float k, VectorXY_Str *out);

/**
 * @brief 三维矢量叉乘
 */
void Vector3D_CrossProd(VectorXYZ_Str *in1,
                        VectorXYZ_Str *in2,
                        VectorXYZ_Str *out);

/**
 * @brief 二维矢量叉乘
 */
void Vector2D_CrossProd(VectorXY_Str *in1,
                        VectorXY_Str *in2,
                        float *out);

/**
 * @brief 通过二维矢量获得三维矢量
 * @param in
 * @param gain
 * @param out
 */
void Vector_2To3_Get(VectorXY_Str *in, float gain, VectorXYZ_Str *out);

/**
 * @brief 通过三维矢量获得二维矢量
 * @param in
 * @param gain
 * @param out
 */
void Vector_3To2_Get(VectorXYZ_Str *in, float gain, VectorXY_Str *out);

/**
 * @brief 三维中，求已知向量方向上指定距离处的点对应的向量
 * @brief 方向向量 in 不能传入0向量
 * @param in         已知方向向量
 * @param L          有向距离
 * @param out        所求向量
 */
void Vector3D_SetSize(VectorXYZ_Str *in, float L, VectorXYZ_Str *out);

/**
 * @brief 二维中，求已知向量方向上指定距离处的点对应的向量
 * @brief 方向向量Str2不能传入0向量
 * @param in          已知方向向量
 * @param L           有向距离
 * @param out         所求向量
 */
void Vector2D_SetSize(VectorXY_Str *in, float L, VectorXY_Str *out);

/**
 * @brief 求两个三维矢量的夹角
 * @brief 两个向量都不能为零向量
 */
void Vector3D_DAngle(VectorXYZ_Str *in1, VectorXYZ_Str *in2, float *out);

/**
 * @brief 求两个二维矢量的夹角
 * @brief 两个向量都不能为零向量
 */
void Vector2D_DAngle(VectorXY_Str *in1, VectorXY_Str *in2, float *out);

/**
 * @brief 三维矢量加法
 */
void Vector3D_Add(VectorXYZ_Str *in1, VectorXYZ_Str *in2, VectorXYZ_Str *out);

/**
 * @brief 二维矢量加法
 */
void Vector2D_Add(VectorXY_Str *in1, VectorXY_Str *in2, VectorXY_Str *out);

/**
 * @brief 三维矢量减法
 * @brief in1-in2
 */
void Vector3D_Subb(VectorXYZ_Str *in1, VectorXYZ_Str *in2, VectorXYZ_Str *out);

/**
 * @brief 二维矢量减法
 * @brief in1-in2
 */
void Vector2D_Subb(VectorXY_Str *in1, VectorXY_Str *in2, VectorXY_Str *out);

/**
 * @brief 三维矢量求模长
 */
void Vector3D_Mod(VectorXYZ_Str *in, float *mod);

/**
 * @brief 二维矢量求模长
 */
void Vector2D_Mod(VectorXY_Str *in, float *mod);

/**
 * @brief 由直角坐标系转球坐标系
 * @brief 矢量不能为0
 */
void Vector3D_ToPYM(VectorXYZ_Str *in, VectorPYM_Str *out);

/**
 * @brief 由直角坐标系转极坐标
 * @brief 矢量不能为0
 */
void Vector2D_ToRT(VectorXY_Str *in, VectorRT_Str *out);

/**
 * @brief 由球坐标系转直角坐标系
 * @brief 矢量不能为0
 */
void Vector3D_ToXYZ(VectorPYM_Str *in, VectorXYZ_Str *out);

/**
 * @brief 由极坐标系转直角坐标
 * @brief 矢量不能为0
 */
void Vector2D_ToXY(VectorRT_Str *in, VectorXY_Str *out);

/**
 * @brief 坐标点二维旋转函数
 * @brief 将坐标点逆时针旋转一定角度
 *
 * @param rotation      旋转的弧度值
 */
void Vector2D_Rotate(VectorXY_Str *in, float rotation, VectorXY_Str *out);

/*
 * 坐标使用右手坐标系，角度逆时针旋转为正。
 * 绕X轴旋转角度为 俯仰角 即Pitch
 * 绕Y轴旋转角度为 偏航角 即Yaw
 * 绕Z轴旋转角度为 翻滚角 即Roll
 */

/**
 * @brief 绕x轴旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_x(VectorXYZ_Str *in, float rotation, VectorXYZ_Str *out);

/**
 * @brief 绕y轴旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_y(VectorXYZ_Str *in, float rotation, VectorXYZ_Str *out);

/**
 * @brief 绕z轴旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_z(VectorXYZ_Str *in, float rotation, VectorXYZ_Str *out);

/**
 * @brief 三维全旋转
 * @brief 先绕x，再绕y，最后绕z旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_xyz(VectorXYZ_Str *in,
                         EulerAngle_Str *rotation,
                         VectorXYZ_Str *out);

/**
 * @brief 二维基底分解
 * @brief 两个基底不能共线
 * @brief 所输入向量不能是零向量
 * @brief 注意基底模长
 */
void Vector2D_Resolve(VectorXY_Str *in, VectorXY_Str *Base1,
                      VectorXY_Str *Base2, VectorXY_Str *out);

#endif /*#__DRV_VECTOR_H__*/
