#ifndef __DRV_IIR_FILTER_H__
#define __DRV_IIR_FILTER_H__

#include <rtthread.h>

#define IIR_N 5
typedef struct
{
    float z_delay[IIR_N];
    float output;
} IIR_cutoff_init;

//初始化采样频率为1000Hz，截止频率为120左右IIR的滤波器
void IIR_cutoff_1000Hz_init(IIR_cutoff_init *IIR_filter, float origin);

// 计算采样频率为1000Hz，截止频率为120左右IIR的滤波器
float IIR_cutoff_1000Hz_cal(IIR_cutoff_init *IIR_filter, float input);

#endif
