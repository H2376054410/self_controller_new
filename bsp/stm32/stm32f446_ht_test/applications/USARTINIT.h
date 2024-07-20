#ifndef __USARTINIT_H__
#define __USARTINIT_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_CRC.h"
#include "CONTROL.h"
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
void uart_init(void);
#endif
