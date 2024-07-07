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
}
