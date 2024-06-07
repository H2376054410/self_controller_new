#include "drv_Vector.h"
#include "drv_utils.h"

/**
 * @brief 判断该浮点数是否为0
 * @param Value             需要判断的浮点数
 * @param Tolerance         容差
 * @return                  1则是零，0则非零
 */
int Float_ZeroIf(float Value, float Tolerance)
{
    if ((Value > -Tolerance) && (Value < Tolerance))
        return 1;
    else
        return 0;
}

/**
 * @brief 浮点数在阈值范围内为0
 * @param Value             需要判断的浮点数
 * @param Tolerance         容差
 */
void Float_ZeroIs(float *Value, float Tolerance)
{
    if ((*Value > -Tolerance) && (*Value < Tolerance))
    {
        *Value = 0;
    }
}

/**
 * @brief 三维矢量二分法
 */
void Vector3D_Dichotomy(VectorXYZ_Str *in1,
                        VectorXYZ_Str *in2,
                        VectorXYZ_Str *out)
{
    out->x = (in1->x + in2->x) / 2.0f;
    out->y = (in1->y + in2->y) / 2.0f;
    out->z = (in1->z + in2->z) / 2.0f;
}

/**
 * @brief 求解反余弦pro
 * @brief 避免因浮点数精度损失而造成：被取反余弦的值的绝对值略大于1
 * @param value 被取反余弦的值
 * @return float
 */
float acosf_pro(float value)
{
    if (value > 1.0f)
    {
        value = 1.0f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }
    return acosf(value);
}

/**
 * @brief 求解反正弦pro
 * @brief 避免因浮点数精度损失而造成：被取反正弦的值的绝对值略大于1
 * @param value 被取反正弦的值
 * @return float
 */
float asinf_pro(float value)
{
    if (value > 1.0f)
    {
        value = 1.0f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }
    return asinf(value);
}

/**
 * @brief 三维向量初始化
 */
void Vector3D_Init(VectorXYZ_Str *vector)
{
    vector->x = 0;
    vector->y = 0;
    vector->z = 0;
}

/**
 * @brief 二维向量初始化
 */
void Vector2D_Init(VectorXY_Str *vector)
{
    vector->x = 0;
    vector->y = 0;
}

/**
 * @brief 三维向量赋值
 */
void Vector3D_Assign(float x, float y,
                     float z, VectorXYZ_Str *out)
{
    out->x = x;
    out->y = y;
    out->z = z;
}

/**
 * @brief 二维向量赋值
 */
void Vector2D_Assign(float x, float y,
                     VectorXYZ_Str *out)
{
    out->x = x;
    out->y = y;
}

/**
 * @brief 三维向量传递值
 */
void Vector3D_Transmit(VectorXYZ_Str *in, VectorXYZ_Str *out)
{
    arm_copy_f32((float32_t *)in, (float32_t *)out, 3);
}

/**
 * @brief 二维向量传递值
 */
void Vector2D_Transmit(VectorXY_Str *in, VectorXY_Str *out)
{
    arm_copy_f32((float32_t *)in, (float32_t *)out, 2);
}

/**
 * @brief 三维向量数乘
 * @param k 数乘的倍数
 */
void Vector3D_Scale(VectorXYZ_Str *in, float k, VectorXYZ_Str *out)
{
    arm_scale_f32((float32_t *)in, k, (float32_t *)out, 3);
}

/**
 * @brief 二维向量数乘
 * @param k 数乘的倍数
 */
void Vector2D_Scale(VectorXY_Str *in, float k, VectorXY_Str *out)
{
    arm_scale_f32((float32_t *)in, k, (float32_t *)out, 2);
}

/**
 * @brief 三维矢量叉乘
 */
void Vector3D_CrossProd(VectorXYZ_Str *in1,
                        VectorXYZ_Str *in2,
                        VectorXYZ_Str *out)
{

    out->x = in1->y * in2->z - in2->y * in1->z;
    out->y = in2->x * in1->z - in1->x * in2->z;
    out->z = in1->x * in2->y - in2->x * in1->y;
}

/**
 * @brief 二维矢量叉乘
 */
void Vector2D_CrossProd(VectorXY_Str *in1,
                        VectorXY_Str *in2,
                        float *out)
{
    *out = in1->x * in2->y - in2->x * in1->y;
}

/**
 * @brief 三维中，求已知向量方向上指定距离处的点对应的向量
 * @brief 方向向量in不能传入0向量
 * @param in         已知方向向量
 * @param L          有向距离
 * @param out        所求向量
 */
void Vector3D_SetSize(VectorXYZ_Str *in, float L, VectorXYZ_Str *out)
{
    float mod;
    float register Value_Temp;
    utils_modulus_get((float32_t *)in, 3, &mod);
    Value_Temp = L / mod;
    out->x = Value_Temp * in->x;
    out->y = Value_Temp * in->y;
    out->z = Value_Temp * in->z;
}

/**
 * @brief 通过二维矢量获得三维矢量
 * @param in
 * @param gain
 * @param out
 */
void Vector_2To3_Get(VectorXY_Str *in, float gain, VectorXYZ_Str *out)
{
    out->x = in->x * gain;
    out->y = in->y * gain;
    out->z = 0;
}

/**
 * @brief 通过三维矢量获得二维矢量
 * @param in
 * @param gain
 * @param out
 */
void Vector_3To2_Get(VectorXYZ_Str *in, float gain, VectorXY_Str *out)
{
    out->x = in->x * gain;
    out->y = in->y * gain;
}

/**
 * @brief 二维中，求已知向量方向上指定距离处的点对应的向量
 * @brief 方向向量in不能传入0向量
 * @param in          已知方向向量
 * @param L           有向距离
 * @param out         所求向量
 */
void Vector2D_SetSize(VectorXY_Str *in, float L, VectorXY_Str *out)
{
    float mod;
    float register Value_Temp;
    utils_modulus_get((float32_t *)in, 2, &mod);
    Value_Temp = L / mod;
    out->x = Value_Temp * in->x;
    out->y = Value_Temp * in->y;
}

/**
 * @brief 求两个三维矢量的夹角
 * @brief 两个向量都不能为零向量
 */
void Vector3D_DAngle(VectorXYZ_Str *in1, VectorXYZ_Str *in2, float *out)
{
    float mod1;
    float mod2;
    float Dot12;
    static float Value_Temp; // 点乘/模长之积

    utils_modulus_get((float32_t *)in1, 3, &mod1);                   // 向量1模长
    utils_modulus_get((float32_t *)in2, 3, &mod2);                   // 向量2模长
    arm_dot_prod_f32((float32_t *)in1, (float32_t *)in2, 3, &Dot12); // 向量点乘

    Value_Temp = Dot12 / (mod1 * mod2);

    *out = acosf_pro(Value_Temp);
}

/**
 * @brief 求两个二维矢量的夹角
 * @brief 两个向量都不能为零向量
 */
void Vector2D_DAngle(VectorXY_Str *in1, VectorXY_Str *in2, float *out)
{
    float mod1;
    float mod2;
    float Dot12;

    utils_modulus_get((float32_t *)in1, 2, &mod1);                   // 向量1模长
    utils_modulus_get((float32_t *)in2, 2, &mod2);                   // 向量2模长
    arm_dot_prod_f32((float32_t *)in1, (float32_t *)in2, 2, &Dot12); // 向量点乘

    *out = acosf_pro(Dot12 / (mod1 * mod2));
}

/**
 * @brief 三维矢量加法
 */
void Vector3D_Add(VectorXYZ_Str *in1, VectorXYZ_Str *in2, VectorXYZ_Str *out)
{
    out->x = in1->x + in2->x;
    out->y = in1->y + in2->y;
    out->z = in1->z + in2->z;
}

/**
 * @brief 二维矢量加法
 */
void Vector2D_Add(VectorXY_Str *in1, VectorXY_Str *in2, VectorXY_Str *out)
{
    out->x = in1->x + in2->x;
    out->y = in1->y + in2->y;
}

/**
 * @brief 三维矢量减法
 * @brief in1-in2
 */
void Vector3D_Subb(VectorXYZ_Str *in1, VectorXYZ_Str *in2, VectorXYZ_Str *out)
{
    out->x = in1->x - in2->x;
    out->y = in1->y - in2->y;
    out->z = in1->z - in2->z;
}

/**
 * @brief 二维矢量减法
 * @brief in1-in2
 */
void Vector2D_Subb(VectorXY_Str *in1, VectorXY_Str *in2, VectorXY_Str *out)
{
    out->x = in1->x - in2->x;
    out->y = in1->y - in2->y;
}

/**
 * @brief 三维矢量求模长
 */
void Vector3D_Mod(VectorXYZ_Str *in, float *mod)
{
    utils_modulus_get((float32_t *)in, 3, (float32_t *)mod);
}

/**
 * @brief 二维矢量求模长
 */
void Vector2D_Mod(VectorXY_Str *in, float *mod)
{
    utils_modulus_get((float32_t *)in, 2, (float32_t *)mod);
}

/**
 * @brief 由直角坐标系转球坐标系
 * @brief 矢量不能为0
 */
void Vector3D_ToPYM(VectorXYZ_Str *in, VectorPYM_Str *out)
{
    float XY_mod;

    out->yaw = atan2(in->y, in->x);
    utils_modulus_get((float32_t *)in, 2, &XY_mod);
    out->pitch = atan2(in->z, XY_mod);
    utils_modulus_get((float32_t *)in, 3, &out->mod);
}

/**
 * @brief 由直角坐标系转极坐标
 * @brief 矢量不能为0
 */
void Vector2D_ToRT(VectorXY_Str *in, VectorRT_Str *out)
{
    out->t = atan2(in->y, in->x);
    utils_modulus_get((float32_t *)in, 2, &out->r);
}

/**
 * @brief 由球坐标系转直角坐标系
 * @brief 矢量不能为0
 */
void Vector3D_ToXYZ(VectorPYM_Str *in, VectorXYZ_Str *out)
{
    float XY_mod;
    float psin;
    float pcos;
    float ysin;
    float ycos;

    arm_sin_cos_f32(RAD2DEG_f(in->pitch), &psin, &pcos);
    XY_mod = in->mod * pcos;
    out->z = in->mod * psin;
    arm_sin_cos_f32(RAD2DEG_f(in->yaw), &ysin, &ycos);
    out->x = XY_mod * ycos;
    out->y = XY_mod * ysin;
}

/**
 * @brief 由极坐标系转直角坐标
 * @brief 矢量不能为0
 */
void Vector2D_ToXY(VectorRT_Str *in, VectorXY_Str *out)
{
    float sin;
    float cos;

    arm_sin_cos_f32(RAD2DEG_f(in->t), &sin, &cos);
    out->x = in->r * cos;
    out->y = in->r * sin;
}

/**
 * @brief 坐标点二维旋转函数
 * @brief 将坐标点逆时针旋转一定角度
 * @param rotation      旋转的弧度值
 */
void Vector2D_Rotate(VectorXY_Str *in, float rotation, VectorXY_Str *out)
{
    float cos, sin;
    float out_x_temp;

    arm_sin_cos_f32(RAD2DEG_f(rotation), &sin, &cos);

    out_x_temp = in->x * cos - in->y * sin;
    out->y = in->y * cos + in->x * sin;
    out->x = out_x_temp;
}

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
void Vector3D_Rotate_x(VectorXYZ_Str *in, float rotation, VectorXYZ_Str *out)
{
    float cos, sin;
    float out_y_temp;
    arm_sin_cos_f32(RAD2DEG_f(rotation), &sin, &cos);

    out->x = in->x;
    out_y_temp = in->y * cos - in->z * sin;
    out->z = in->y * sin + in->z * cos;
    out->y = out_y_temp;
}

/**
 * @brief 绕y轴旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_y(VectorXYZ_Str *in, float rotation, VectorXYZ_Str *out)
{
    float cos, sin;
    float out_x_temp;
    arm_sin_cos_f32(RAD2DEG_f(rotation), &sin, &cos);

    out_x_temp = in->x * cos + in->z * sin;
    out->y = in->y;
    out->z = -in->x * sin + in->z * cos;
    out->x = out_x_temp;
}

/**
 * @brief 绕z轴旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_z(VectorXYZ_Str *in, float rotation, VectorXYZ_Str *out)
{
    float cos, sin;
    float out_x_temp;
    arm_sin_cos_f32(RAD2DEG_f(rotation), &sin, &cos);

    out_x_temp = in->x * cos - in->y * sin;
    out->y = in->x * sin + in->y * cos;
    out->z = in->z;
    out->x = out_x_temp;
}

/**
 * @brief 三维全旋转
 * @brief 先绕x，再绕y，最后绕z旋转
 * @param in
 * @param rotation
 * @param out
 */
void Vector3D_Rotate_xyz(VectorXYZ_Str *in,
                         EulerAngle_Str *rotation,
                         VectorXYZ_Str *out)
{
    VectorXYZ_Str out_temp_x;
    VectorXYZ_Str out_temp_y;

    Vector3D_Rotate_x(in, rotation->pitch, &out_temp_x);
    Vector3D_Rotate_y(&out_temp_x, rotation->roll, &out_temp_y);
    Vector3D_Rotate_z(&out_temp_y, rotation->yaw, out);
}

/**
 * @brief 二维基底分解
 * @brief 两个基底不能共线
 * @brief 所输入向量不能是零向量
 * @brief 注意基底模长
 */
void Vector2D_Resolve(VectorXY_Str *in, VectorXY_Str *Base1,
                      VectorXY_Str *Base2, VectorXY_Str *out)
{
#define x1 (Base1->x)
#define y1 (Base1->y)

#define x2 (Base2->x)
#define y2 (Base2->y)

#define x0 (in->x)
#define y0 (in->y)

    VectorXY_Str Base1_abs;

    arm_abs_f32((float32_t *)Base1, (float32_t *)&Base1_abs, 2);

    if (Base1_abs.x > Base1_abs.y)
    { // 说明x1非零
        out->y = (y0 - (x0 / x1 * y1)) / (y2 - (x2 / x1 * y1));
        out->x = (x0 - out->y * x2) / x1;
    }
    else
    {
        out->y = (x0 - (y0 / y1 * x1)) / (x2 - (y2 / y1 * x1));
        out->x = (y0 - out->y * y2) / y1;
    }

#undef x1
#undef y1

#undef x2
#undef y2

#undef x0
#undef y0
}
