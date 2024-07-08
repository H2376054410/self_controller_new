#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <rtthread.h>
typedef struct
{
    float BoomLeft;
    float BoomRight;
    float BoomYaw;
} BoomMotor_s;


void control_init(void);
#endif
