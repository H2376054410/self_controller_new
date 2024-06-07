#include "Cpu_reset.h"
#include "mod_Can_data.h"
#include "mod_RoboStateCtrl.h"
/**
 * @brief 单片机复位
 * @param status
 * @return rt_err_t
 */
rt_err_t monitorErrHandle(rt_bool_t status)
{
    ArmPosState_Init(); // 机械臂复位状态
    rt_thread_mdelay(500);

    Send_Slave1_Init();
    Send_Slave2_Init();

    rt_thread_mdelay(100);
    rt_hw_cpu_reset();

    return RT_EOK;
}

/**
 * @brief 单片机复位
 */
void CPU_Reset(void)
{
    ArmPosState_Init(); // 机械臂复位状态
    rt_thread_mdelay(500);

    Send_Slave1_Init();
    Send_Slave2_Init();

    rt_thread_mdelay(100);
    rt_hw_cpu_reset();
}

/**
 * @brief 单片机延时复位
 * @param time
 */
void CPU_DelayReset(rt_tick_t time)
{
    static rt_tick_t CPUReset_Tick;
    static rt_uint8_t CPUReset_Tick_flag;
    switch (CPUReset_Tick_flag)
    {
    case 0:
        CPUReset_Tick = rt_tick_get();
        CPUReset_Tick_flag = 1;
        break;
    case 1:
        // 单片机延时复位
        if ((rt_tick_get() - CPUReset_Tick) > time)
        {
            CPUReset_Tick_flag = 0;
            CPU_Reset();
        }
        break;
    default:
        CPUReset_Tick = 0;
        CPUReset_Tick_flag = 0;
        break;
    }
}
