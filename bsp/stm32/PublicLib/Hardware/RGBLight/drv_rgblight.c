/************************************** Copyright ******************************
 *                 (C) Copyright 2020,China, HITwh.
 *                            All Rights Reserved
 *
 *                     HITwh Excellent Robot Organization
 *                     https://github.com/HERO-ECG
 *                     https://gitee.com/HIT-718LC
 *
 * FileName   : drv_rgblight.c
 * Version    : v1.0
 * Author     : mqy
 * Date       : 2020-02-14
 * Description:
 ********************************************************************************/

#include "drv_rgblight.h"

// RGB颜色控制PWM设备句柄与通道号
struct rt_device_pwm *red_light = RT_NULL;
struct rt_device_pwm *green_light = RT_NULL;
struct rt_device_pwm *blue_light = RT_NULL;

#ifndef BSP_USING_PWM
#error "Please open the drivers for PWM device!"
#elif (!defined RLIGHT_PWM_DEVICE_NAME) || (!defined RLIGHT_PWM_CHANNEL) || \
       (!defined GLIGHT_PWM_DEVICE_NAME) || (!defined GLIGHT_PWM_CHANNEL) || \
       (!defined BLIGHT_PWM_DEVICE_NAME) || (!defined BLIGHT_PWM_CHANNEL)
#error "Please specify the specific PWM device for rgb light!"
#else

/**
* @brief：该函数初始化核心板上的RGB灯
* @param [in] 无
* @return：	true：初始化成功
            false：初始化失败
* @author：	mqy
*/
int RGB_init(void)
{
    //尝试查找设备，查找失败时返回
    red_light = (struct rt_device_pwm *)rt_device_find(RLIGHT_PWM_DEVICE_NAME);
    if (red_light == RT_NULL)
        return 0;

    green_light = (struct rt_device_pwm *)rt_device_find(GLIGHT_PWM_DEVICE_NAME);
    if (green_light == RT_NULL)
        return 0;

    blue_light = (struct rt_device_pwm *)rt_device_find(BLIGHT_PWM_DEVICE_NAME);
    if (blue_light == RT_NULL)
        return 0;

    rt_pwm_set(red_light, RLIGHT_PWM_CHANNEL, 2550000, 2550000);
    rt_pwm_set(green_light, GLIGHT_PWM_CHANNEL, 2550000, 2550000);
    rt_pwm_set(blue_light, BLIGHT_PWM_CHANNEL, 2550000, 2550000);

    rt_pwm_enable(red_light, RLIGHT_PWM_CHANNEL);
    rt_pwm_enable(green_light, GLIGHT_PWM_CHANNEL);
    rt_pwm_enable(blue_light, BLIGHT_PWM_CHANNEL);

    return 1;
}
INIT_APP_EXPORT(RGB_init);

#endif

/**
* @brief：该函数设置灯的亮度
* @param[in] red：红色亮度
            green：绿色亮度
            blue：蓝色亮度
*/
void set_RGB(float red, float green, float blue)
{
#if (defined BSP_USING_PWM) &&                                              \
        (defined RLIGHT_PWM_DEVICE_NAME) && (defined RLIGHT_PWM_CHANNEL) && \
        (defined GLIGHT_PWM_DEVICE_NAME) && (defined GLIGHT_PWM_CHANNEL) && \
    (defined BLIGHT_PWM_DEVICE_NAME) && (defined BLIGHT_PWM_CHANNEL)
    static float red_last, green_last, blue_last;

    red = (rt_uint32_t)(PWM_DEV_PERIOD * (1 - red));
    green = (rt_uint32_t)(PWM_DEV_PERIOD * (1 - green));
    blue = (rt_uint32_t)(PWM_DEV_PERIOD * (1 - blue));

    if ((red != red_last) || (green != green_last) || (blue != blue_last))
    {
        rt_pwm_set(red_light, RLIGHT_PWM_CHANNEL, PWM_DEV_PERIOD, (rt_uint32_t)red);
        rt_pwm_set(green_light, GLIGHT_PWM_CHANNEL, PWM_DEV_PERIOD, (rt_uint32_t)green);
        rt_pwm_set(blue_light, BLIGHT_PWM_CHANNEL, PWM_DEV_PERIOD, (rt_uint32_t)blue);
    }
    red_last = red;
    green_last = green;
    blue_last = blue;
#endif
}
