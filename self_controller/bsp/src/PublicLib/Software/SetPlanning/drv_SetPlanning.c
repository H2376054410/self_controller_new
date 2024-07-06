#include "drv_SetPlanning.h"

#include "drv_utils.h"

void SetPlanning_OldCal(SetPlanning_Str *Str)
{
    // 一些中间变量
    float Accl_Cal;

    // 记录计算时刻，仅供Debug
    Str->CalTick = rt_tick_get();

    // 对输出的位置和速度按照实际状态进行限幅
    if (Str->Output.pos - Str->Input.Now.pos > Str->Settings.POS_Error_Max)
    {
        Str->Output.pos = Str->Input.Now.pos + Str->Settings.POS_Error_Max;
        // Str->Output.spe = 0.f;
    }
    else if (Str->Output.pos - Str->Input.Now.pos < -Str->Settings.POS_Error_Max)
    {
        Str->Output.pos = Str->Input.Now.pos - Str->Settings.POS_Error_Max;
        // Str->Output.spe = 0.f;
    }

    Str->Temp.DeltaSpe = Str->Input.Set.spe - Str->Output.spe;
    Str->Temp.DeltaSpe_2 = Str->Temp.DeltaSpe * Str->Temp.DeltaSpe;
    Str->Temp.DeltaPos = Str->Input.Set.pos - Str->Output.pos;
    Str->Temp.DeltaPos_Temp = Str->Temp.DeltaSpe_2 / (2 * Str->Settings.Accl_Max * 0.95f); // 以最大加速度将速度减为零所用的位移（减速位移）

    if (Str->Temp.DeltaPos * Str->Temp.DeltaSpe < 0) // 当前位移增量和速度增量的方向相反
    {
        if (Str->Temp.DeltaPos > 0)
        {
            if (Str->Temp.DeltaPos * 0.9f > Str->Temp.DeltaPos_Temp)
            {
                Accl_Cal = Str->Settings.Accl_Max; // 当前位置和减速位移还有一定距离,则依然加速
            }
            else
            {
                Accl_Cal = -Str->Settings.Accl_Max; // 到该减速的位置了，则减速
            }
        }
        else
        {
            if (-Str->Temp.DeltaPos * 0.9f > Str->Temp.DeltaPos_Temp) // 原理同上
            {
                Accl_Cal = -Str->Settings.Accl_Max;
            }
            else
            {
                Accl_Cal = Str->Settings.Accl_Max;
            }
        }
    }
    else
    {
        if (Str->Temp.DeltaPos > 0)
        {
            Accl_Cal = Str->Settings.Accl_Max;
        }
        else
        {
            Accl_Cal = -Str->Settings.Accl_Max;
        }
    }
    Str->Temp.Accl_Now = Accl_Cal;
    Str->Temp.OutputTemp.spe = Str->Temp.Accl_Now * Str->Settings.dt + Str->Output.spe; // 计算规划后的速度
    /*对规划后的速度进行限幅*/
    if (Str->Temp.OutputTemp.spe > Str->Settings.Speed_Max)
    {
        Str->Temp.OutputTemp.spe = Str->Settings.Speed_Max;
    }
    else if (Str->Temp.OutputTemp.spe < -Str->Settings.Speed_Max)
    {
        Str->Temp.OutputTemp.spe = -Str->Settings.Speed_Max;
    }

    if ((fabsf(Str->Temp.DeltaPos) > fabsf(Str->Output.spe * Str->Settings.dt)) ||
        (fabsf(Str->Temp.DeltaSpe) > 2 * Str->Settings.Accl_Max * Str->Settings.dt))
    {
        // 速度增量或位置增量都大于从静止状态加速一个周期的距离和速度时，正常设定值规划的输出
        Str->Output.spe = Str->Temp.OutputTemp.spe;
        Str->Output.pos += Str->Output.spe * Str->Settings.dt;
    }
    else
    {
        // 剩余增量很小，可以忽略时，则不进行设定值规划，直出设定值
        Str->Output.pos = Str->Input.Set.pos;
        Str->Output.spe = Str->Input.Set.spe;
    }
}

// 执行设定值规划计算
void SetPlanning_Cal(SetPlanning_Str *Str)
{
    // 一些中间变量
    float Accl_Cal;

    // 记录计算时刻，仅供Debug
    Str->CalTick = rt_tick_get();

    // 对输出的位置和速度按照实际状态进行限幅
    if (Str->Output.pos - Str->Input.Now.pos > Str->Settings.POS_Error_Max)
    {
        Str->Output.pos = Str->Input.Now.pos + Str->Settings.POS_Error_Max;
        // Str->Output.spe = 0.f;
    }
    else if (Str->Output.pos - Str->Input.Now.pos < -Str->Settings.POS_Error_Max)
    {
        Str->Output.pos = Str->Input.Now.pos - Str->Settings.POS_Error_Max;
        // Str->Output.spe = 0.f;
    }

    Str->Temp.DeltaSpe = Str->Input.Set.spe - Str->Output.spe;
    Str->Temp.DeltaSpe_2 = fabsf(Str->Input.Set.spe * Str->Input.Set.spe - Str->Output.spe * Str->Output.spe);
    Str->Temp.DeltaPos = Str->Input.Set.pos - Str->Output.pos;
    Str->Temp.DeltaPos_Temp = Str->Temp.DeltaSpe_2 / (2 * Str->Settings.Accl_Max * 0.95f); // 以最大加速度将速度减为零所用的位移（减速位移）

    if (Str->Temp.DeltaPos * Str->Temp.DeltaSpe < 0) // 当前位移增量和速度增量的方向相反
    {
        if (Str->Temp.DeltaPos > 0)
        {
            if (Str->Temp.DeltaPos * 0.9f > Str->Temp.DeltaPos_Temp)
            {
                Accl_Cal = Str->Settings.Accl_Max; // 当前位置和减速位移还有一定距离,则依然加速
            }
            else
            {
                Accl_Cal = -Str->Settings.Accl_Max; // 到该减速的位置了，则减速
            }
        }
        else
        {
            if (-Str->Temp.DeltaPos * 0.9f > Str->Temp.DeltaPos_Temp) // 原理同上
            {
                Accl_Cal = -Str->Settings.Accl_Max;
            }
            else
            {
                Accl_Cal = Str->Settings.Accl_Max;
            }
        }
    }
    else
    {
        if (Str->Temp.DeltaPos > 0)
        {
            Accl_Cal = Str->Settings.Accl_Max;
        }
        else
        {
            Accl_Cal = -Str->Settings.Accl_Max;
        }
    }
    Str->Temp.Accl_Now = Accl_Cal;
    Str->Temp.OutputTemp.spe = Str->Temp.Accl_Now * Str->Settings.dt + Str->Output.spe; // 计算规划后的速度
    /*对规划后的速度进行限幅*/
    if (Str->Temp.OutputTemp.spe > Str->Settings.Speed_Max)
    {
        Str->Temp.OutputTemp.spe = Str->Settings.Speed_Max;
    }
    else if (Str->Temp.OutputTemp.spe < -Str->Settings.Speed_Max)
    {
        Str->Temp.OutputTemp.spe = -Str->Settings.Speed_Max;
    }

    if ((fabsf(Str->Temp.DeltaPos) > fabsf(Str->Temp.OutputTemp.spe * Str->Settings.dt)) &&
        (fabsf(Str->Temp.DeltaSpe) > Str->Settings.Accl_Max * Str->Settings.dt))
    {
        // 速度增量或位置增量都大于从静止状态加速一个周期的距离和速度时，正常设定值规划的输出
        Str->Output.spe = Str->Temp.OutputTemp.spe;
        Str->Output.pos += Str->Output.spe * Str->Settings.dt;
    }
    else
    {
        // 剩余增量很小，可以忽略时，则不进行设定值规划，直出设定值
        Str->Output.pos = Str->Input.Set.pos;
        Str->Output.spe = Str->Input.Set.spe;
    }
}

// 直接修改规划结果
void SetPlanning_SetOutput(SetPlanning_Str *Str, float PosSet)
{
    Str->Output.pos = PosSet;
}

void Planning_updata(float Now_pos, float Set_spe, float Set_pos, float speed_max, SetPlanning_Str *Str)
{
    static float Last_current = 0;
    Str->Settings.Speed_Max = speed_max;
    Str->Input.Now.pos = Now_pos; // SCAP: pos 对应的就现在的电流
    Str->Input.Now.spe = (Now_pos - Last_current) / Str->Settings.dt;
    Str->Input.Set.pos = Set_pos;
    Str->Input.Set.spe = Set_spe;

    Last_current = Str->Input.Now.spe;
}

/**
 * @brief 一维设定值规划
 * @param Str
 * @param Accl_Max
 * @param dt
 * @param POS_Error_Max
 * @param Speed_Max
 */
void SetPlanning_Init(SetPlanning_Str *Str, SetPlanSettings_Str *Settings)
{
    // 初始化结构体内容
    Str->CalTick = 0;
    Str->Input.Now.pos = 0;
    Str->Input.Now.spe = 0;
    Str->Input.Set.pos = 0;
    Str->Input.Set.spe = 0;
    Str->Output.pos = 0;
    Str->Output.spe = 0; // 输出的设定值速度作为速度环的前馈，而不是直接输入
    Str->Settings.Accl_Max = Settings->Accl_Max;
    Str->Settings.dt = Settings->dt;
    Str->Settings.POS_Error_Max = Settings->POS_Error_Max;
    Str->Settings.Speed_Max = Settings->Speed_Max;
}

/**
 * @brief 设定值规划设定数据初始化
 * @param Settings          设定数据结构体
 * @param Speed_Max         调整过程最大速度
 * @param POS_Error_Max     输出设定值对应的最大Error
 * @param Accl_Max          最大加速度
 * @param Period            计算周期 单位s
 */
void SetPlanSettings_Init(SetPlanSettings_Str *Settings,
                          float Speed_Max,
                          float POS_Error_Max,
                          float Accl_Max,
                          float Period)
{
    Settings->Speed_Max = Speed_Max;
    Settings->POS_Error_Max = POS_Error_Max;
    Settings->Accl_Max = Accl_Max;
    Settings->dt = Period;
}
