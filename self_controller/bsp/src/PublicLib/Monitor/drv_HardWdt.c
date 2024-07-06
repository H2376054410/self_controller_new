#include "drv_HardWdt.h"

#ifdef BSP_USING_WDG //不加当宏没使能时，且scons后，工程内不存在看门狗文件，会编译报错

#include "drv_iwdg.h"
#include <stdbool.h>

bool iwdg_inited = false;

/**
 * @brief    硬件看门狗初始化
 * @param    无
 * @return   初始化成功or失败(初始化失败可能是开启硬件看门狗功能后没有scons工程)
 * @author   Lvfp
 */
rt_err_t Hwdt_Init(void)
{
    if (iwdg_inited)
        return RT_EOK;
    // 初始化硬件看门狗, 超时时间 1/(32kHz/32*4) = 4ms
    MX_IWDG_Init();
    iwdg_inited = true;

    return RT_EOK;
}

/**
 * @brief    硬件看门狗喂狗
 * @param    无
 * @return   初始化成功or失败
 * @author   Lvfp
 */
void Hwdt_Feed(void)
{
    if (iwdg_inited)
        HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @brief 硬件看门狗缓慢喂狗
 * @author fwlh
 * @param  if_slow          传入真代表需要缓慢喂狗
 */
void Hwdt_Feed_Slowly(rt_bool_t if_slow)
{
    if (!iwdg_inited)
        return;
    Hwdt_Feed();
    // 低速喂狗时最多允许超时 1s
    if (if_slow)
    {
        hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
        hiwdg.Init.Reload = 1000;
        HAL_IWDG_Init(&hiwdg);
    }
    else
        MX_IWDG_Init();
    Hwdt_Feed();
}

#endif // BSP_USING_WDG
