/**
 * @file drv_PLLFilter.c
 * @brief 本文件用于实现锁相环滤波器
 * @author fwlh
 * @version 1.0
 * @date 2022-07-23
 *
 * @copyright Copyright (c) 2022  哈尔滨工业大学(威海)HERO战队
 */

// TODO: 注意本模块还未经过充分测试, 使用时注意提早测试数据

#include "drv_PLLFilter.h"
#include "drv_utils.h"

/**
 * @brief 初始化锁相环滤波器控制块
 * @author fwlh
 * @param  PLLFilter_Ctrl   待初始化的锁相环滤波器结构体
 * @param  Kp               锁相环滤波器的比例系数
 * @param  Ki               锁相环滤波器的积分系数
 * @param  StartPos         锁相环滤波器的初始位置
 */
void PLLFilter_Init(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float Kp, float Ki, float StartPos)
{
    PLLFilter_Ctrl->Settings.Kp = UTILS_NAN_ZERO_F(Kp);
    PLLFilter_Ctrl->Settings.Ki = UTILS_NAN_ZERO_F(Ki);
    PLLFilter_Ctrl->Err_Integral = 0;
    PLLFilter_Ctrl->Pos_Sense = UTILS_NAN_ZERO_F(StartPos);
    PLLFilter_Ctrl->PosErr = 0.f;
    PLLFilter_Ctrl->OutData.Spe = 0.f;
    PLLFilter_Ctrl->OutData.Pos = UTILS_NAN_ZERO_F(StartPos);
    PLLFilter_Ctrl->Settings.CrossCircle_Flag = 0;
    PLLFilter_Ctrl->Last_FreshTick = 0;
}

/**
 * @brief 进行锁相环滤波器的跨圈设置
 * @author fwlh
 * @param  PLLFilter_Ctrl   待配置的锁相环滤波器结构体
 * @param  CircleMax        跨圈计算时的圈内最大数值
 * @param  CircleMin        跨圈计算时的圈内最小数值
 */
void PLLFilter_CrossCircle_Set(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float CircleMax, float CircleMin)
{
    PLLFilter_Ctrl->Settings.CrossCircle_Flag = 1;
    PLLFilter_Ctrl->Settings.CircleMax = UTILS_NAN_ZERO_F(CircleMax);
    PLLFilter_Ctrl->Settings.CircleMin = UTILS_NAN_ZERO_F(CircleMin);
}

/**
 * @brief 锁相环滤波器的计算
 * @author fwlh
 * @param  PLLFilter_Ctrl   待计算滤波结果的锁相环滤波器控制块
 * @param  NowPos           当前待滤波的数据
 */
void PLLFilter_Calc(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float NowPos)
{
    rt_tick_t NowTick = rt_tick_get();
    if (PLLFilter_Ctrl->Settings.CrossCircle_Flag)
    {
        // 输入新的数据
        UTILS_NAN_ZERO_F(NowPos);
        // 跨圈处理
        utils_norm_circle_number(&NowPos, PLLFilter_Ctrl->Settings.CircleMin, PLLFilter_Ctrl->Settings.CircleMax - PLLFilter_Ctrl->Settings.CircleMin);
        PLLFilter_Ctrl->Pos_Sense = NowPos;
        // 计算滤波器估计误差
        PLLFilter_Ctrl->PosErr = utils_circle_number_difference(PLLFilter_Ctrl->Pos_Sense, PLLFilter_Ctrl->OutData.Pos, PLLFilter_Ctrl->Settings.CircleMin, PLLFilter_Ctrl->Settings.CircleMax - PLLFilter_Ctrl->Settings.CircleMin);
    }
    else
    {
        PLLFilter_Ctrl->Pos_Sense = UTILS_NAN_ZERO_F(NowPos);
        PLLFilter_Ctrl->PosErr = PLLFilter_Ctrl->Pos_Sense - PLLFilter_Ctrl->OutData.Pos;
    }
    // 结算误差积分
    PLLFilter_Ctrl->Err_Integral += (PLLFilter_Ctrl->Settings.Ki * PLLFilter_Ctrl->PosErr);
    // 通过 PI 控制器计算输出速度数据
    PLLFilter_Ctrl->OutData.Spe = PLLFilter_Ctrl->Settings.Kp * PLLFilter_Ctrl->PosErr + PLLFilter_Ctrl->Err_Integral;

    float TempPos;
    if (!PLLFilter_Ctrl->Last_FreshTick)
        // 速度数据积分得到位置数据
        TempPos = PLLFilter_Ctrl->OutData.Pos + PLLFilter_Ctrl->OutData.Spe * (NowTick - PLLFilter_Ctrl->Last_FreshTick) / 1000;
    else
        // 首次计算数据直接用于重置滤波器
        TempPos = PLLFilter_Ctrl->Pos_Sense;
    // 检查跨圈处理
    if (PLLFilter_Ctrl->Settings.CrossCircle_Flag)
    {
        utils_norm_circle_number(&TempPos, PLLFilter_Ctrl->Settings.CircleMin, PLLFilter_Ctrl->Settings.CircleMax - PLLFilter_Ctrl->Settings.CircleMin);
        PLLFilter_Ctrl->OutData.Pos = TempPos;
    }
    else
        PLLFilter_Ctrl->OutData.Pos = TempPos;
    // 刷新写入时间
    PLLFilter_Ctrl->Last_FreshTick = NowTick;
}

/**
 * @brief 重置锁相环滤波器
 * @author fwlh
 * @param  PLLFilter_Ctrl   需要重置的锁相环滤波器控制块
 * @param  NowPos           重置以后的当前位置
 */
void PLLFilter_Restart(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float NowPos)
{
    PLLFilter_Ctrl->Last_FreshTick = rt_tick_get();
    PLLFilter_Ctrl->Err_Integral = 0.f;
    PLLFilter_Ctrl->OutData.Pos = UTILS_NAN_ZERO_F(NowPos);
    PLLFilter_Ctrl->OutData.Spe = 0.f;
    PLLFilter_Ctrl->Pos_Sense = UTILS_NAN_ZERO_F(NowPos);
    PLLFilter_Ctrl->PosErr = 0.f;
}
