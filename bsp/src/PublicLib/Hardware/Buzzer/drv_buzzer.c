/************************************** Copyright ******************************
 *                 (C) Copyright 2020,China, HITwh.
 *                            All Rights Reserved
 *
 *                     HITwh Excellent Robot Organization
 *                     https://github.com/HERO-ECG
 *                     https://gitee.com/HIT-718LC
 *
 * FileName   : drv_buzzer.c
 * Version    : v1.0
 * Author     : mqy
 * Date       : 2020-02-14
 * Description:
 ********************************************************************************/

#include "drv_buzzer.h"

#ifndef BSP_USING_PWM
#error "Please open the PWM driver for buzzer!"
#elif (!defined BUZZER_PWM_DEVICE_NAME) || (!defined BUZZER_PWM_CHANNEL)
#error "Please specify the specific PWM driver for buzzer!"
#else
//蜂鸣器PWM设备句柄与通道号
static struct rt_device_pwm *hummer = RT_NULL;

rt_uint8_t isReset;
/**
* @brief：该函数初始化核心板上的蜂鸣器
* @param [in] 无
* @return：	true：初始化成功
            false：初始化失败
* @author：	mqy
*/
int buzzer_init(void)
{
    //尝试查找设备，查找失败时返回
    hummer = (struct rt_device_pwm *)rt_device_find(BUZZER_PWM_DEVICE_NAME);
    if (hummer == RT_NULL)
    {
        return 0;
    }
    rt_pwm_set(hummer, BUZZER_PWM_CHANNEL, 1000000, 0);
    rt_pwm_enable(hummer, BUZZER_PWM_CHANNEL);

    return 1;
}
INIT_APP_EXPORT(buzzer_init);
#endif

/**
 * @brief：该函数设置蜂鸣器频率
 * @param frequency：声音频率，该值为0时停止发声
 * @param Power 0-1之间
 * @return：		无
 * @author：mqy
 */
void set_buzzer(rt_uint16_t frequency, float Power)
{
#if (defined BSP_USING_PWM) && (defined BUZZER_PWM_DEVICE_NAME) && (defined BUZZER_PWM_CHANNEL)
    //为0时返回
    if (!frequency)
    {
        rt_pwm_set(hummer, BUZZER_PWM_CHANNEL, 1000000, 0);
        return;
    }
    rt_uint32_t period = 1000000000 / frequency;
    if (Power > 1.f)
        Power = 1.f;
    else if (Power < 0.f)
    {
        Power = 0.f;
    }

    rt_pwm_set(hummer, BUZZER_PWM_CHANNEL, period, (int)(period / 2 * Power));
#endif
}
