/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include "mod_UartSlave.h"
uint32_t cpu_777 = 0;
int main(void)
{
    // 从机，启动！
    UartSlave_Init_and_Start();
	return 0;
}
