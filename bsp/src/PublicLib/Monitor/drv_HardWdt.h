#ifndef __DRV_HARDWAREDOG_H__
#define __DRV_HARDWAREDOG_H__

#include <rtthread.h>

#ifdef BSP_USING_WDG

/**
* @brief    硬件看门狗初始化
* @param    无
* @return   初始化成功or失败
* @author   Lvfp
*/
rt_err_t Hwdt_Init(void);

/**
* @brief    硬件看门狗喂狗
* @param    无
* @return   初始化成功or失败
* @author   Lvfp
*/
void Hwdt_Feed(void);

/**
 * @brief 硬件看门狗缓慢喂狗
 * @author fwlh
 * @param  if_slow          传入真代表需要缓慢喂狗
 */
extern void Hwdt_Feed_Slowly(rt_bool_t if_slow);

#endif // BSP_USING_WDG
#endif
