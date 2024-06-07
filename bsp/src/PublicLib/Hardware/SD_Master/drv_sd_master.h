#ifndef __DRV_SD_MASTER_H__
#define __DRV_SD_MASTER_H__

#include <rtdef.h>

void sd_master_send_func(rt_uint8_t *buff, rt_uint16_t size);
void sd_master_drv_init(void);

#endif
