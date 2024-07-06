#include "drv_Chassis_Solve.h"
#include "drv_utils.h"

/**
 * @brief  获得矢量大小和转向角
 * @param  value 待处理矢量
 **/
static void Vector3D_mod_theta_Get(ChassisVector_t *value)
{
    Vector3D_Mod(&value->Vector, &value->mod);
    value->xy_theta = atan2f(-value->Vector.x, value->Vector.y) / PI * 180.0f;
}

/**
 * @brief  通过大小和转向角获得矢量,角度单位为度
 * @param  temp 待处理矢量
 * @return None
 **/
static void Vector3D_From_mod_theta_Get(ChassisVector_t *temp)
{
    temp->Vector.y = temp->mod * cosf(temp->xy_theta / 180.0f * PI);
    temp->Vector.x = -temp->mod * sinf(temp->xy_theta / 180.0f * PI);
}

/**
 * @brief 矢量坐标转换模块 右手坐标系 y轴为正前方
 * @param {VectorXY_Str} speed 速度矢量,不改变单位；推荐单位：mm/s
 * @param {float} theta 两个坐标系的夹角 输入单位：弧度，范围(-PI~PI]
 * @param {uint8_t} isReverse 当存在云台时和云台角度匹配即可
 * @param chassis_speed_out
 */
static void MotModule_Transform(VectorXY_Str speed,
                                float theta,
                                uint8_t isReverse,
                                VectorXY_Str *chassis_speed_out)
{
    VectorXY_Str gimSpeed = speed;
    // 输入保护
    if (theta > PI || theta <= -PI)
    {
        while (1)
            ;
    }
    if (isReverse)
    {
        chassis_speed_out->x = gimSpeed.x * arm_cos_f32(theta) - gimSpeed.y * arm_sin_f32(theta); // x=x'cosθ-y'sinθ
        chassis_speed_out->y = gimSpeed.x * arm_sin_f32(theta) + gimSpeed.y * arm_cos_f32(theta); // y=y'cosθ+x'sinθ
    }
    else
    {
        chassis_speed_out->x = gimSpeed.x * arm_cos_f32(theta) + gimSpeed.y * arm_sin_f32(theta);  // x'=ysinθ+xcosθ
        chassis_speed_out->y = -gimSpeed.x * arm_sin_f32(theta) + gimSpeed.y * arm_cos_f32(theta); // y'=ycosθ-xsinθ
    }
}

/**
 * @brief   获取所在函数被调用的时间间隔
 * @param   record_p    静态的或者全局的变量指针
 * @param   period      当前被调用到上一次被调用的时间间隔，单位ms，float类型
 */
static void RT_TICK_PROBE(Tick_probe_t *record_p, float *period)
{
    Tick_probe_t *__record = record_p;
    if (__record->if_init == RT_TRUE)
    {
        rt_tick_t now_tick = rt_tick_get();
        *period = (float)(now_tick - __record->last_tick) / (float)RT_TICK_PER_SECOND * 1000.0f; /*时间间隔，单位ms*/
        __record->last_tick = now_tick;
    }
    else /*第一次执行到,给last_tick赋初值*/
    {
        __record->last_tick = rt_tick_get();
        __record->if_init = RT_TRUE;
        *period = 0;
    }
}

/**
 * @brief   输入幅度限制模块1
 * @param   VxyW   速度矢量和角速度矢量
 * @return  限制后Mot_base_t数据
 */
static ChassisMotion_t MotModule_Clamp_Base(ChassisMotion_t VxyW)
{
    ChassisMotion_t temp =
        {
            .vel.y = utils_clamp(VxyW.vel.y, -M_MAX_YSPEED, M_MAX_YSPEED),
            .vel.x = utils_clamp(VxyW.vel.x, -M_MAX_XSPEED, M_MAX_XSPEED),
            .angvel = utils_clamp(VxyW.angvel, -M_MAX_ROTATE_AC, M_MAX_ROTATE_AC)};
    return temp;
}

/**
 * @brief   输入幅度限制模块2
 * @param   pos   位置坐标
 * @return  限制后VectorXY_Str数据
 */
static VectorXY_Str MotModule_Clamp_Pos(VectorXY_Str pos)
{
    VectorXY_Str temp =
        {
            .y = utils_clamp(pos.y, -M_MAX_POSITION_Y, M_MAX_POSITION_Y),
            .x = utils_clamp(pos.x, -M_MAX_POSITION_X, M_MAX_POSITION_X)};
    return temp;
}

/**
 * @brief   偏心运动模块（将绕某点旋转转化为底盘中心的速度矢量）
 * @param   pos_in      旋转轴坐标，单位 m。在二维的底盘坐标系上，正前方为 +y轴，右手边为 +x轴。
 * @param   angvel      自旋速度大小，单位 rad/s，正方向俯视图逆时针。
 * @param   spe_out     中心的速度矢量,单位：m/s
 */
static void MotModule_Eccentric(VectorXY_Str *pos_in,
                                float angvel,
                                VectorXY_Str *spe_out) // VectorXY_Str postion,float angvel)
{
    ChassisVector_t vector_axis1 = {0};
    ChassisVector_t vector_axis2 = {0};
    /*角速度矢量 ω*/
    ChassisVector_t vector_w = {.Vector.x = 0,
                                .Vector.y = 0,
                                .Vector.z = angvel,
                                .mod = 0,
                                .xy_theta = 0};

    /*底盘中心相对于以旋转点为原点的xy坐标系的位置矢量，单位：m*/
    Vector_2To3_Get(pos_in, -1, &vector_axis1.Vector);

    /*υ = ω x r，计算底盘中心的速度矢量，单位 rad*m/s*/
    Vector3D_CrossProd(&vector_w.Vector,
                       &vector_axis1.Vector,
                       &vector_axis2.Vector);

    /*单位转换:rad°*m/s -> m/s*/
    Vector_3To2_Get(&vector_axis2.Vector, PI / 180.0f, spe_out);
}

/**
 * @brief   公转运动模块（将绕某点旋转转化为底盘中心随时间自旋的速度矢量）
 * @param   pos   旋转轴坐标，单位 mm。在二维的底盘坐标系上，正前方为 +y轴，右手边为 +x轴。
 * @param   angvel  公转角速度大小，单位 0.1°/s，正方向俯视图逆时针。
 * @param   record   由调用MotModule_Revolve函数的上下文提供一个静态或者全局的变量,record为该变量的指针
 * @return  中心随时间旋转的速度矢量,单位：mm/s
 */
static void MotModule_Revolve(VectorXY_Str *pos, float angvel,
                              ChassisRecord_t *record,
                              VectorXY_Str *out)
{
    /*如果旋转点和公转速度发生改变时，重置*/
    if (!VECTOR2_CMP(record->last_pos, (*pos)) || record->last_angvel != angvel)
    {
        record->incre_theta = 0;
        record->last_pos = *pos;
        record->last_angvel = angvel;
    }

    /*计算切向的速度矢量*/
    VectorXY_Str vector_center_2D;
    ChassisVector_t vector_center_Chas;
    MotModule_Eccentric(&vector_center_2D, angvel, &vector_center_2D);
    Vector_2To3_Get(&vector_center_2D, 1, &vector_center_Chas.Vector);
    /*求模长和偏向角*/
    Vector3D_mod_theta_Get(&vector_center_Chas);

    /* ∆θ = ω * ∆t ,计算速度矢量的角度增量，单位°*/
    float period;
    RT_TICK_PROBE(&record->tick, &period); // 单位ms

    record->incre_theta += period / 1000.0f * angvel * 0.1f; // 公转角速度，单位°/s
    utils_norm_circle_number(&record->incre_theta, -180.0f, 360.f);

    /*计算旋转后的速度矢量x,y*/
    vector_center_Chas.xy_theta += record->incre_theta;
    Vector3D_From_mod_theta_Get(&vector_center_Chas);

    /*转化为二维矢量*/
    Vector_3To2_Get(&vector_center_Chas.Vector, 1, out);
}

/**
 * @brief 偏心小陀螺运动
 * @param {Ctrl_data_t} *ctrlMsg 控制数据 【底盘xy速度矢量，单位mm/s 底盘自转角速度大小，单位0.1°/s】
 * @param {Vector2_t} *pos 偏心坐标点，单位(mm,mm)
 * @param yaw_angle          跟随角时yaw的角度,单位 rad，范围[-PI,PI)
 * @param pos                偏心坐标点，单位(m,m)
 * @param folIsReverse       跟随时候，坐标系转化时角度是否取反
 */
void MotPack_Offset_SmallTop(ChassisMotion_t *vxyW_input,
                             VectorXY_Str *pos,
                             float yaw_angle,
                             uint8_t folIsReverse,
                             ChassisMotion_t *vxyW_output)
{
    VectorXY_Str spe;
    /*输入限制*/
    utils_clamp(pos->x, -M_MAX_POSITION_X, M_MAX_POSITION_X);
    utils_clamp(pos->y, -M_MAX_POSITION_Y, M_MAX_POSITION_Y);

    /*将云台的速度矢量转化到底盘坐标系上*/
    MotModule_Transform(vxyW_input->vel,
                        yaw_angle,
                        folIsReverse,
                        &vxyW_input->vel);

    /* 叠加偏心运动 */
    MotModule_Eccentric(pos, vxyW_input->angvel, &spe);

    /*输出限制*/
    vxyW_output->vel.x = utils_clamp(spe.x, -M_MAX_XSPEED, M_MAX_XSPEED);
    vxyW_output->vel.y = utils_clamp(spe.y, -M_MAX_YSPEED, M_MAX_YSPEED);
    vxyW_output->angvel = utils_clamp(vxyW_input->angvel, -M_MAX_POSITION_X, M_MAX_POSITION_X);
}

/**
 * @brief 单位换算，遥控器数值转成国际单位
 * @param VxyW_in
 * @param VxyW_out
 */
void UnitCverRemote2standard(ChassisMotion_t *VxyW_in,
                             ChassisMotion_t *VxyW_out)
{
    // 660(遥控器满值)对应速度5m/s
    // 660(遥控器满值)对应角速度30rad/s
    VxyW_out->vel.x = VxyW_in->vel.x * 0.003f;
    VxyW_out->vel.y = VxyW_in->vel.y * 0.003f;
    VxyW_out->angvel = VxyW_in->angvel * 0.5f;
}

/**
 * @brief    只有底盘运动
 * @param    VxyW.vel   底盘xy速度矢量，单位m/s
 * @param    VxyW.angvel   底盘自转角速度大小，单位rad°/s
 * @return   输出到Resolve函数的VxyW
 */
void MotPack_Only_Chass(ChassisMotion_t *VxyW_in,
                        ChassisMotion_t *VxyW_out)
{
    *VxyW_out = MotModule_Clamp_Base(*VxyW_in);
}

/**
 * @brief    偏心的只有底盘运动
 * @param    VxyW.vel        底盘xy速度矢量，单位m/s
 * @param    VxyW.angvel     底盘自转角速度大小，单位rad/s
 * @param    pos             偏心坐标点，单位(m,m)
 * @param    out             输出到Resolve函数的VxyW
 */
void MotPack_Offset_OnlyChass(ChassisMotion_t *VxyW,
                              VectorXY_Str *pos,
                              ChassisMotion_t *out)
{
    /*输入限制*/
    *VxyW = MotModule_Clamp_Base(*VxyW);
    *pos = MotModule_Clamp_Pos(*pos);

    /*计算切向速度*/
    VectorXY_Str V_offset;
    MotModule_Eccentric(pos, VxyW->angvel, &V_offset);

    /*矢量叠加*/
    Vector2D_Add(&VxyW->vel, &V_offset, &out->vel);
}

/**
 * @brief    绕点旋转运动
 * @param    Pxy 偏心坐标点，单位(m,m)
 * @param    dot_angvel 公转角速度大小，单位rad/s
 * @return   输出到Resolve函数的VxyW
 */
void MotPack_Spin_Dot(VectorXY_Str *pos, float dot_angvel, ChassisMotion_t *out)
{
    static ChassisRecord_t storage;

    *pos = MotModule_Clamp_Pos(*pos);
    dot_angvel = utils_clamp(dot_angvel, -M_MAX_REVOLVE_AC, M_MAX_REVOLVE_AC);

    VectorXY_Str velocity;
    MotModule_Revolve(pos, dot_angvel, &storage, &velocity);

    Vector2D_Transmit(&velocity, &out->vel);
}

/**
 * @brief   麦轮/全向底盘运动解算,并输出到wheel层
 * @param   chs.vel     底盘xy方向速度大小,单位m/s
 * @param   chs.angvel  自转角速度,单位rad/s,底盘逆时针旋转为正
 * @return  None
 * @author  lfp
 */
void MecanOmni_Resolve(ChassisMotion_t *chs, float *output)
{
    // 麦轮运动解算
#ifdef MECANUM_WHEEL
    //*MY_PI/1800.0为单位转换系数，*(VEHICLE_WIDTH + VEHICLE_LONG)/2.0f将角速度转化为轮毂的线速度
    const float coefficient = PI / 180.0f * (float)(VEHICLE_WIDTH + VEHICLE_LONG) / 2.0f;

    // v1~4单位rad/s
    output[0] = chs->vel.y - chs->vel.x + chs->angvel * coefficient;
    output[1] = chs->vel.y + chs->vel.x - chs->angvel * coefficient;
    output[2] = chs->vel.y - chs->vel.x - chs->angvel * coefficient;
    output[3] = chs->vel.y + chs->vel.x + chs->angvel * coefficient;
#endif

    // 全向轮运动解算
#ifdef OMNI_WHEEL
    //*Lx_PI/1800.0为单位转换系数，*(VEHICLE_DIAMETER/2.0)将角速度转化为线速度
    const float coefficient = Lx_PI / 1800.0f * (float)(VEHICLE_DIAMETER / 2.0f);

    // v1~4单位rad/s
    output[0] = chs->vel.y + chs->angvel * coefficient;
    output[1] = chs->vel.x - chs->angvel * coefficient;
    output[2] = chs->vel.y - chs->angvel * coefficient;
    output[3] = chs->vel.x + chs->angvel * coefficient;
#endif
}
