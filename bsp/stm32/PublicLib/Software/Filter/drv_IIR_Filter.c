#include "drv_IIR_Filter.h"

const float a_12_cutoff[IIR_N] =
    {0.00298706122332176f,0.0119482448932871f,0.0179223673399306f,0.0119482448932871f,0.00298706122332176f};
const float b_12_cutoff[IIR_N] =
    {1.0f,- 2.57988975080055f,2.66517258703128f,- 1.27224251184340f,0.234752655185816f};

void IIR_cutoff_1000Hz_init(IIR_cutoff_init *IIR_filter, float origin)
{
    for (int i = 0; i < IIR_N; i++)
    {
        IIR_filter->z_delay[i] = origin;
    }
}

float IIR_cutoff_1000Hz_cal(IIR_cutoff_init *IIR_filter, float input)
{
    // 实现是直接二型
    IIR_filter->output = b_12_cutoff[0] * input + IIR_filter->z_delay[0];
    for (int i = 1; i < IIR_N; i++)
        IIR_filter->z_delay[i - 1] = b_12_cutoff[i] * input + -a_12_cutoff[i] * IIR_filter->output + IIR_filter->z_delay[i];
    return IIR_filter->output;
}
