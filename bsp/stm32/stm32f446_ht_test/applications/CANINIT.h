#ifndef __CANINIT_H__
#define __CANINIT_H__

#include <rtthread.h>
#define CAN_DEV_NAME "can1" 



extern  struct rt_semaphore rx_time;
void can_init(void);
#endif
