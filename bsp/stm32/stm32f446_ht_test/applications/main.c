/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <pid.h>
#include <math.h>
#include <CANINIT.h>
#include <USARTINIT.h>

int main(void)
{


can_init();
uart_init();
motor_init_DM(&Boomleft_Motor, 0, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  A4310_ENCODERLEN,
                  180, -180, 0);
motor_init_DM(&Boomright_Motor, 0, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  A4310_ENCODERLEN,
                  180, -180, 0);
motor_init_DM(&Boomyaw_Motor, 0, // 控制th4角度电机
                  1,
                  ANGLE_CTRL_FULL,
                  A4310_ENCODERLEN,
                  180, -180, 0);
Send_Slave2_Init();//达妙电机初始化
	
	control_init();
}
