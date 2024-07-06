/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-06     thread-liu   first version
 */

#ifndef __DRV_FDCAN_H__
#define __DRV_FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>

struct stm32_fdcan
{
    struct rt_device dev;
    char *name;
    FDCAN_HandleTypeDef fdcan;
    FDCAN_FilterTypeDef filter;
    FDCAN_TxHeaderTypeDef tx_config;
    FDCAN_RxHeaderTypeDef rx_config;
    volatile rt_uint8_t fifo0;
    volatile rt_uint8_t fifo1;
};

#ifdef __cplusplus
}
#endif

#endif
