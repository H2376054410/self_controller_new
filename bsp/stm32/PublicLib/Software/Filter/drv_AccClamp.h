#ifndef __DRV_ACCCLAMP_H_
#define __DRV_ACCCLAMP_H_

#include <board.h>

#define ADDMultiple (2)

#define Smooth_ADD_UP 1
#define Smooth_ADD_DOWN -1
/*平滑变化方向*/

/*平滑模式*/
typedef enum
{
    SM_NORMAL, // 正常模式：值变减小时，使用SMOOTH_ADD_UP，增大时，使用SMOOTH_ADD_DOWN。
    SM_MIRROR, // 镜像模式：当值小于0时（大于0时和正常模式一样），值减小时，用的加速度大小为SMOOTH_ADD_UP，增大时，使用SMOOTH_ADD_DOWN。

    SM_MODE_ALL,

} Smooth_mode_e;

/*平滑数值结构体*/
typedef struct
{
    float in;      // 当前的输入的设定值
    float out_dyn; // 随时间改变的平滑速度，会以恒定加速度增加或者减少

} Smooth_value_t;

typedef struct
{
    int8_t add_dir;         // 平滑变大还是变小
    rt_tick_t LastOutTime;  // 上一次输出时间 单位rt_tick
    rt_bool_t Is_smoothing; // 是否正在平滑 用于判断是否第一次运行
} Smooth_info_t;

typedef struct
{
    float IncreaseSpeed; // 增长速度  单位为：设定值数值/rt_tick 为绝对值
    float DecreaseSpeed; // 减小速度  单位为：设定值数值/rt_tick 为绝对值
    Smooth_mode_e mode;  // 平滑方式
} Smooth_ctrl_t;

/*平滑结构体*/
typedef struct
{
    Smooth_value_t value;
    Smooth_info_t info;
    Smooth_ctrl_t ctrl;

} Smooth_t;

/**
 * @brief 平滑处理
 * @param {Smooth_t} *Smooth_target 平滑处理结构体
 * @param {float} *set 需要平滑的值，该函数执行后set自动增减
 * @return {*}NONE
 */
void Smooth_Acc_Process(Smooth_t *Smooth_target, float *const set);

/**
 * @brief 修改平滑加速度
 * @param {Smooth_t} *Smooth_target 平滑结构体
 * @param {float} add_speed 增长速度  单位为：设定值数值/rt_tick 为绝对值,必须>0
 * @param {float} down_speed 减小速度  单位为：设定值数值/rt_tick 为绝对值,必须>0
 * @return {rt_err_t} 当add_speed或者down_speed为0时错误
 */
rt_err_t Smooth_Acc_Modify_Speed(Smooth_t *Smooth_target, float add_speed, float down_speed);

/**
 * @brief 修改平滑模式
 * @param {Smooth_t} *Smooth_target 平滑结构体
 * @param {float} step 修改的模式
 */
void Smooth_Acc_Modify_Mode(Smooth_t *Smooth_target, Smooth_mode_e mode);

/**
 * @brief 默认初始化一些配置值
 * @param {Smooth_t} *Smooth_target
 */
void Smooth_Acc_DeInit(Smooth_t *Smooth_target);

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
                         float down_speed, Smooth_mode_e mode);

/**
 * @brief 获得加速或者加速的速度
 * @param {Smooth_t} *Smooth_target
 * @param {Smooth_dir_e} type 加速或者加速的速度
 * @return {*}返回0代表错误 其他正确
 */
float Smooth_Acc_GetDataABS(Smooth_t *Smooth_target, int8_t type);

/**
 * @brief  重置当前值，
 *         当smooth[kind].value的实际值与上一次最终设定值不符合，
 *         重新刷新上一次最终值。
 * @note   应用于外部强行切断了对某个值的平滑，同时改变了该值。此时切回来需要同步该值结构体的某些值
 * @param  value 数值
 * @return None
 */
void Smooth_Acc_Refresh_Nowvalue(Smooth_t *Smooth_target, float value);

/**
 * @brief  暂停平滑
 * @param {Smooth_t} *Smooth_target
 * @param {float} *set
 */
void Smooth_Acc_Suspend(Smooth_t *Smooth_target, float *const set);

/**
 * @brief 读取平滑的输出值
 * @param {Smooth_t} *Smooth_target
 * @return {*}返回输出值
 */
float Smooth_Acc_ReadOut(const Smooth_t *Smooth_target);
#endif
