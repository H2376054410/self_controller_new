/************************************** Copyright ****************************** 
  *                 (C) Copyright 2020,mqy,China,HITwh.
  *                            All Rights Reserved
  *
  *                     HITwh Excellent Robot Organization
  *                     https://github.com/HERO-ECG
  *                     https://gitee.com/HIT-718LC
  *    
  * FileName   : drv_buzzer.h
  * Version    : v1.0
  * Author     : mqy
  * Date       : 2020-02-14
  * Description:
********************************************************************************/

#ifndef __DRV_BUZZER_H__
#define __DRV_BUZZER_H__

#include "rtthread.h"
#include "drv_common.h"

/**
 * @brief：该函数设置蜂鸣器频率
 * @param frequency：声音频率，该值为0时停止发声
 * @param frequency：输出功率 0-1之间
 * @return：		无
 * @author：mqy
 */
extern void set_buzzer(rt_uint16_t frequency, float Power);

#endif
