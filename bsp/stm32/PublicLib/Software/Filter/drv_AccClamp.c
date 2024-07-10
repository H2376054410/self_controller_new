#include "drv_AccClamp.h"

#include "arm_math.h"
#include "drv_utils.h"

// static struct rt_semaphore AccClamp_sem; // 读写访问保护锁

/*------------------------------------XXX----------------------------------------------------------*/
/*
输入要平滑的设定值
1.假设当前的设定值不变
则可以选择当前的增长模式 默认为直线增长速度则每次增长为v*Δt，这时速度必须合理，一般来讲是比较快的或者说从一直到设定值过程当中的动态设定值数目较多
或者为曲线模式即动态改变速度即可实现曲线
2.当上一次设定值还在动态变化时此时目标设定值变化时则 定一个最小可以接受的阶跃量Δr 小于Δr则直接赋予上一次的目标设定值增长速度不变
大于Δr则先暂时加快速度到上一次设定值，之后恢复正常
*/

rt_inline float GetDeSpeed(Smooth_t *Smooth_target)
{
    float now = 0;
    if (Smooth_target->info.add_dir == Smooth_ADD_UP)
        now = Smooth_target->ctrl.IncreaseSpeed;
    else
        now = Smooth_target->ctrl.DecreaseSpeed;
		return now;
}

static void Smooth_Input(Smooth_t *Smooth_target, float set)
{
    Smooth_target->value.in = set;

    if (set >= Smooth_target->value.out_dyn)
        Smooth_target->info.add_dir = Smooth_ADD_UP;
    else
        Smooth_target->info.add_dir = Smooth_ADD_DOWN;
}

static void Smooth_Process(Smooth_t *Smooth_target)
{
    float nowSpeed;

    /*当不是第一次平滑 并且需要平滑时时*/
    if ((Smooth_target->info.Is_smoothing== RT_TRUE) && (Smooth_target->value.in != Smooth_target->value.out_dyn))
    {
        // 正常处理
        switch (Smooth_target->ctrl.mode)
        {
        case SM_NORMAL:
        {
            nowSpeed = GetDeSpeed(Smooth_target);
            break;
        }
        case SM_MIRROR:
        {
            if (Smooth_target->value.out_dyn < 0)
            {
                if (Smooth_target->info.add_dir == Smooth_ADD_UP)
                    nowSpeed = Smooth_target->ctrl.DecreaseSpeed;
                else
                {
                    nowSpeed = Smooth_target->ctrl.IncreaseSpeed;
                }
            }
            else
            {
                nowSpeed = GetDeSpeed(Smooth_target);
                break;
            }          
        }
        default:
            nowSpeed = 0;
            break;
        }
        // 距离上次时间差值
        uint32_t DeltaTick = rt_tick_get() - Smooth_target->info.LastOutTime;
        Smooth_target->value.out_dyn += Smooth_target->info.add_dir * nowSpeed * DeltaTick;
    }
}

static void Smooth_Output(Smooth_t *Smooth_target)
{
    /*当不是第一次平滑 并且需要平滑时*/
    if ((Smooth_target->info.Is_smoothing == RT_TRUE) && (Smooth_target->value.in != Smooth_target->value.out_dyn))
    {
        // 本次输出限幅
        if ((Smooth_target->value.in - Smooth_target->value.out_dyn) * Smooth_target->info.add_dir < 0)
            Smooth_target->value.out_dyn = Smooth_target->value.in;
    }
    else
        Smooth_target->info.Is_smoothing = RT_TRUE;

}

/**
 * @brief 平滑处理
 * @param {Smooth_t} *Smooth_target 平滑处理结构体
 * @param {float} *set 需要平滑的值，该函数执行后set自动增减
 * @return {*}NONE
 */
void Smooth_Acc_Process(Smooth_t *Smooth_target, float *const set)
{
    Smooth_Input(Smooth_target, *set);
    Smooth_Process(Smooth_target);
    Smooth_Output(Smooth_target);

    // 刷新输出时间
    Smooth_target->info.LastOutTime = rt_tick_get();

    *set = Smooth_target->value.out_dyn;
}

/**
 * @brief 修改平滑加速度
 * @param {Smooth_t} *Smooth_target 平滑结构体
 * @param {float} add_speed 增长速度  单位为：设定值数值/rt_tick 为绝对值,必须>0
 * @param {float} down_speed 减小速度  单位为：设定值数值/rt_tick 为绝对值,必须>0
 * @return {rt_err_t} 当add_speed或者down_speed为0时错误
 */
rt_err_t Smooth_Acc_Modify_Speed(Smooth_t *Smooth_target, float add_speed, float down_speed)
{
    if (add_speed == 0 || down_speed == 0)
        return RT_ERROR;
    /*输入正负限制*/
    if (add_speed < 0)
        Smooth_target->ctrl.IncreaseSpeed = -add_speed;
    else
        Smooth_target->ctrl.IncreaseSpeed = add_speed;
    if (down_speed < 0)
        Smooth_target->ctrl.DecreaseSpeed = -down_speed;
    else
        Smooth_target->ctrl.IncreaseSpeed = down_speed;
    return RT_EOK;
}

/**
 * @brief 修改平滑模式
 * @param {Smooth_t} *Smooth_target 平滑结构体
 * @param {float} step 修改的模式
 */
void Smooth_Acc_Modify_Mode(Smooth_t *Smooth_target, Smooth_mode_e mode)
{
    if (mode <SM_MODE_ALL)
        Smooth_target->ctrl.mode = mode;
}

/**
 * @brief 默认初始化一些配置值
 * @param {Smooth_t} *Smooth_target
 */
void Smooth_Acc_DeInit(Smooth_t *Smooth_target)
{
    Smooth_target->value.in = 0;
    Smooth_target->value.out_dyn = 0;

    // (1000.f / RT_TICK_PER_SECOND)代表一个tick为多少ms    常用1tick为1ms
    Smooth_target->ctrl.IncreaseSpeed = 5;
    Smooth_target->ctrl.DecreaseSpeed = 5;
    Smooth_target->ctrl.mode = (Smooth_mode_e)0;

    Smooth_target->info.LastOutTime = 0;
    Smooth_target->info.Is_smoothing = RT_FALSE;
    Smooth_target->info.add_dir = Smooth_ADD_UP; // 默认增大

}

/**
 * @brief 初始化平滑结构体
 * @param {Smooth_t} *Smooth_target 平滑结构体
 * @param {float} add_speed  增长速度  单位为：设定值数值/rt_tick 为绝对值,必须>0
 * @param {float} down_speed  减小速度  单位为：设定值数值/rt_tick 为绝对值,必须>0
 * @param {float} step 一次最多可增加/减小的数值 为绝对值 必须大于0
 * @param {Smooth_mode_e} mode 修改的模式
 * @return {*}返回1代表错误
 */
rt_err_t Smooth_Acc_Init(Smooth_t *Smooth_target, float add_speed,
                         float down_speed,Smooth_mode_e mode)
{
    rt_err_t xreturn = RT_EOK;

    Smooth_target->value.in = 0;
    Smooth_target->value.out_dyn = 0;

    Smooth_target->info.add_dir = Smooth_ADD_UP; // 默认增大
    Smooth_target->info.Is_smoothing = RT_FALSE;
    Smooth_target->info.LastOutTime=0;

    xreturn = Smooth_Acc_Modify_Speed(Smooth_target, add_speed, down_speed);

    Smooth_target->ctrl.DecreaseSpeed = down_speed;
    Smooth_target->ctrl.IncreaseSpeed = add_speed;
    Smooth_target->ctrl.mode=mode;

    return xreturn;
}

/**
 * @brief 获得加速或者加速的速度
 * @param {Smooth_t} *Smooth_target
 * @param {Smooth_dir_e} type 加速或者加速的速度
 * @return {*}返回0代表错误 其他正确
 */
float Smooth_Acc_GetDataABS(Smooth_t *Smooth_target, int8_t type)
{
    if (type == Smooth_ADD_DOWN)
        return Smooth_target->ctrl.DecreaseSpeed;
    else if (type == Smooth_ADD_UP)
        return Smooth_target->ctrl.IncreaseSpeed;
    else
        return 0;
}

/**
 * @brief  重置当前值，
 *         当smooth[kind].value的实际值与上一次最终设定值不符合，
 *         重新刷新上一次最终值。
 * @note   应用于外部强行切断了对某个值的平滑，同时改变了该值。此时切回来需要同步该值结构体的某些值
 * @param  value 数值
 */
void Smooth_Acc_Refresh_Nowvalue(Smooth_t *Smooth_target, float value)
{
    Smooth_target->value.in = value;
    Smooth_target->value.out_dyn = value;
    Smooth_target->info.Is_smoothing = RT_FALSE; // 重新开始平滑
}

/**
 * @brief  暂停平滑
 * @param {Smooth_t} *Smooth_target
 * @param {float} *set
 */
void Smooth_Acc_Suspend(Smooth_t *Smooth_target, float *const set)
{
    *set = Smooth_target->value.in;
}

/**
 * @brief 读取平滑的输出值
 * @param {Smooth_t} *Smooth_target
 * @return {*}返回输出值
 */
float Smooth_Acc_ReadOut(const Smooth_t *Smooth_target)
{
    if (Smooth_target->info.Is_smoothing == RT_TRUE)
        return Smooth_target->value.out_dyn;
    else
        return Smooth_target->value.in;
}
