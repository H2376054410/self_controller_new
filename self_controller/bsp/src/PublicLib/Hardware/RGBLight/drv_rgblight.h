/************************************** Copyright ****************************** 
  *                 (C) Copyright 2020,mqy,China,HITwh.
  *                            All Rights Reserved
  *
  *                     HITwh Excellent Robot Organization
  *                     https://github.com/HERO-ECG
  *                     https://gitee.com/HIT-718LC
  *    
  * FileName   : drv_rgblight.h
  * Version    : v1.0
  * Author     : mqy
  * Date       : 2020-02-14
  * Description:
********************************************************************************/

#ifndef __DRV_RGBLIGHT_H__
#define __DRV_RGBLIGHT_H__

#include <rtthread.h>
#include "drv_common.h"

#define CORE_WHITE		255,255,255
#define CORE_BLACK		0,0,0
#define CORE_RED		255,0,0
#define CORE_BLUE		0,0,255
#define CORE_GREEN		0,255,0
#define CORE_YELLOW		255,255,0
#define CORE_PURPLE		128,0,128

//RGB颜色控制PWM设备句柄与通道号
extern struct rt_device_pwm* red_light;

extern struct rt_device_pwm* green_light;

extern struct rt_device_pwm* blue_light;

//#define RGB_FLASH_TIME  120
//#define RGB_RED_flash  set_RGB(CORE_RED);rt_thread_mdelay(RGB_FLASH_TIME);set_RGB(CORE_BLACK);
//#define RGB_BLUE_flash  set_RGB(CORE_BLUE);rt_thread_mdelay(RGB_FLASH_TIME);set_RGB(CORE_BLACK);
//#define RGB_WHITE_flash  set_RGB(CORE_WHITE);rt_thread_mdelay(RGB_FLASH_TIME);set_RGB(CORE_BLACK);
//#define RGB_YELLOW_flash  set_RGB(CORE_YELLOW);rt_thread_mdelay(RGB_FLASH_TIME);set_RGB(CORE_BLACK);
//#define RGB_GREEN_flash  set_RGB(CORE_GREEN);rt_thread_mdelay(RGB_FLASH_TIME);set_RGB(CORE_BLACK);
//#define RGB_PURPLE_flash  set_RGB(CORE_PURPLE);rt_thread_mdelay(RGB_FLASH_TIME);set_RGB(CORE_BLACK);

#define PWM_DEV_PERIOD 100000 /* PWM周期100us,10khz */
void set_RGB(float red, float green, float blue);
int RGB_init(void);
#endif
