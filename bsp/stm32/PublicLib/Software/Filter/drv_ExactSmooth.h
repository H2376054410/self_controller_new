#ifndef __DRV_EXACTSMOOTH_H__
#define __DRV_EXACTSMOOTH_H__

#include "rtthread.h"

typedef struct
{
    struct rt_semaphore sem; // 读写访问保护锁
    float NowSetData; // 当前输入的设定值

    float NowOutData; // 最近一次取到的输出值
    float NowOutData_Add; // 最近一次刷新输出值时的增量

    float AddTemp; // 存储每次刷新输入数据后重新计算出的积攒数据量

    rt_tick_t OutData_FreshTime; // 当前输出值的结算时刻
    rt_tick_t SetData_FreshTime; // 当前输出值的结算时刻
    float AddData_Speed; // 输出数据需要的增长速度

    float SetFresh_DataAdd; // 上一次更新设定值时遗留的输出增量
    char SetFresh_Flag; // 每次更新设定值并记录遗留增量之后置1，下一次读取后清零

    rt_int32_t Set_SmthPeriodMax; // 数据结清时间设置
    char LimFlag; // 记录是否需要进行设定值限幅
    char CircleFlag; // 记录是否需要跨圈处理
    float Lim_Len; // 若需要跨圈处理，则会计算单圈长度并记录于此
    float UpLim; // 数值上限
    float DownLim; // 数值下限
		
} ExactSmth_CTRL_S;

// 绝对式设定数据
extern void Smooth_SetDataABS(ExactSmth_CTRL_S *CTRL, float Set);
// 增量式设定数据
extern void Smooth_SetDataADD(ExactSmth_CTRL_S *CTRL, float Add);

// 获取平滑后的数据的绝对数值
extern void Smooth_GetDataABS(float *GetDataABS, ExactSmth_CTRL_S *CTRL);
// 获取平滑后的数据的变化速度
extern void Smooth_GetDataADD(float *GetDataADD, ExactSmth_CTRL_S *CTRL);

// 按照指定的数值重启平滑，一般用于切换数据源后无缝换回
extern void Smooth_SetData_Restart(ExactSmth_CTRL_S *CTRL, float SetNow);

// 初始化平滑控制块 初始化后默认没有限幅和跨圈处理 需要给定初始设定值和平滑时间
// 如有限幅/跨圈处理需要 则可以在初始化完成后调用 Smooth_SetDataFix 进行单独设定
extern void Smooth_Init(ExactSmth_CTRL_S *CTRL, float InitSet, int PeriodMax, char Lim, float Limup, float Limdown);

// 可选的跨圈处理功能：设定限幅/跨圈处理后，程序会按照设定的范围对参与平滑的数据进行限幅或跨圈处理
// 当CircleFlag==0时，仅对数据进行限幅处理，不会对多余部分进行跨圈处理
// 当CircleFlag==1时，若数据出现超限，会按照限幅对应的数据长度进行跨圈处理，溢出部分会进入下一圈
extern void Smooth_SetDataFix(ExactSmth_CTRL_S *CTRL, float UpLim, float DownLim, char CircleFlag);

#endif
