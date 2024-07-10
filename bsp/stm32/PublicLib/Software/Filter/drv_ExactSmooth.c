#include "drv_ExactSmooth.h"

static char Smooth_CTRL_InitCount = 0; // 记录已经初始化的控制块的个数，不能超过10个，否则信号量的名称无法分配

// 按照CTRL中的配置对输入的数据进行限幅或跨圈处理
static void DataLimFix(float *Data, ExactSmth_CTRL_S *CTRL)
{
    char CircleHandle_Count;
    float DataTemp = *Data;
    if (CTRL->CircleFlag)
    { // 需要进行跨圈处理
        CircleHandle_Count = 0;
        while (DataTemp > CTRL->UpLim)
        {
            CircleHandle_Count++;
            DataTemp -= CTRL->Lim_Len;
            if (CircleHandle_Count > 10)
            {
                CircleHandle_Count = 0;
                DataTemp = CTRL->UpLim; // 如果数据异常则将输出数据置为数据上限
                break;
            }
        }
        while (DataTemp <= CTRL->DownLim)
        {
            CircleHandle_Count++;
            DataTemp += CTRL->Lim_Len;
            if (CircleHandle_Count > 10)
            {
                CircleHandle_Count = 0;
                DataTemp = CTRL->DownLim; // 如果数据异常则将输出数据置为数据下限
                break;
            }
        }
    }
    else if (CTRL->LimFlag)
    { // 需要进行普通限幅
        if (DataTemp > CTRL->UpLim)
        {
            DataTemp = CTRL->UpLim;
        }
        else if (DataTemp < CTRL->DownLim)
        {
            DataTemp = CTRL->DownLim;
        }
    }
    *Data = DataTemp;
}

// 按照跨圈处理的设定情况对AddTemp进行更新
static void Fresh_DeltaData(ExactSmth_CTRL_S *CTRL)
{
    float DeltaDataTemp;

    DeltaDataTemp = CTRL->NowSetData - (CTRL->NowOutData + CTRL->SetFresh_Flag * CTRL->SetFresh_DataAdd);
    if (CTRL->CircleFlag)
    {
        // 需要跨圈处理
        if (DeltaDataTemp > CTRL->Lim_Len / 2)
        {
            DeltaDataTemp -= CTRL->Lim_Len;
        }
        else if (DeltaDataTemp <= -CTRL->Lim_Len / 2)
        {
            DeltaDataTemp += CTRL->Lim_Len;
        }
    }
    CTRL->AddTemp = DeltaDataTemp;
}

// 按照当前的rt_tick刷新一次输出数据
static void Smooth_FreshOutData(ExactSmth_CTRL_S *CTRL)
{
    rt_tick_t Tick_Now = rt_tick_get(); // 记录当前时刻
    rt_int32_t DeltaTick;
    rt_tick_t Smooth_EndTick;

    float OutData_Add_Temp, OutData_Old;

    Smooth_EndTick = CTRL->Set_SmthPeriodMax + CTRL->SetData_FreshTime;

    if (Tick_Now < Smooth_EndTick)
    { // 当前平滑过程还没结束
        DeltaTick = Tick_Now - CTRL->OutData_FreshTime;
        CTRL->OutData_FreshTime = Tick_Now;
    }
    else if (CTRL->OutData_FreshTime < Smooth_EndTick)
    { // 平滑时间已经结束，检查输出是否已经结算到平滑结束时间
        DeltaTick = Smooth_EndTick - CTRL->OutData_FreshTime;
        CTRL->OutData_FreshTime = Tick_Now;
    }
    else
    { // 平滑结束，无需处理，直接返回
        CTRL->OutData_FreshTime = Tick_Now;
        return;
    }

    OutData_Add_Temp = DeltaTick * CTRL->AddData_Speed; // 计算输出增量
    if (CTRL->SetFresh_Flag)
    {
        CTRL->SetFresh_Flag = 0;
        OutData_Add_Temp += CTRL->SetFresh_DataAdd;
        CTRL->SetFresh_DataAdd = 0; // 叠加上累积数据，清空累积数据
    }

    OutData_Old = CTRL->NowOutData;
    CTRL->NowOutData += OutData_Add_Temp; // 输出结算至现在

    DataLimFix(&CTRL->NowOutData, CTRL); // 输出数据限幅处理

    // 汇总计算本轮结算中的输出数据增量
    if (CTRL->CircleFlag)
    {
        // 如果开启了跨圈处理 增量应为输出跨圈处理之前的增量
        CTRL->NowOutData_Add = OutData_Add_Temp;
    }
    else
    {
        // 其它情况下 输出增量应为新输出值-旧输出值
        CTRL->NowOutData_Add = CTRL->NowOutData - OutData_Old;
    }
}

// 绝对式设定数据
void Smooth_SetDataABS(ExactSmth_CTRL_S *CTRL, float Set)
{
    rt_tick_t Tick_Now = rt_tick_get(); // 记录当前时刻
    rt_int32_t DeltaTick;
    rt_tick_t Smooth_EndTick;
    // 使用信号量进行读写保护
    if (rt_sem_take(&CTRL->sem, 20) == RT_EOK) // 挂起时间过长时直接放弃
    {                                          // 如果正常取到了信号量，则再取一次确保没有多余的信号量
        rt_sem_trytake(&CTRL->sem);
    }

    DataLimFix(&Set, CTRL); // 输入数据限幅
    CTRL->NowSetData = Set;

    Smooth_EndTick = CTRL->Set_SmthPeriodMax + CTRL->SetData_FreshTime;

    if (Tick_Now < Smooth_EndTick)
    { // 上一平滑过程还没结束
        DeltaTick = Tick_Now - CTRL->OutData_FreshTime;
    }
    else if (CTRL->OutData_FreshTime < Smooth_EndTick)
    { // 上一平滑已经结束，检查输出是否已经结算到平滑结束时间
        DeltaTick = Smooth_EndTick - CTRL->OutData_FreshTime;
    }
    else
    { // 平滑结束
        DeltaTick = 0;
    }
    CTRL->SetFresh_DataAdd += CTRL->AddData_Speed * DeltaTick; // 记录之前没有应用的增量数据
    CTRL->SetFresh_Flag = 1;

    CTRL->SetData_FreshTime = Tick_Now; // 记录数据输入时刻
    CTRL->OutData_FreshTime = Tick_Now; // 重置数据输出结算时刻

    Fresh_DeltaData(CTRL); // 计算AddTemp
    CTRL->AddData_Speed = CTRL->AddTemp / CTRL->Set_SmthPeriodMax;

    rt_sem_release(&CTRL->sem); // 访问完毕，重新释放信号量
}

// 增量式设定数据
void Smooth_SetDataADD(ExactSmth_CTRL_S *CTRL, float Add)
{
    float SetTemp;
    rt_tick_t Tick_Now = rt_tick_get(); // 记录当前时刻
    rt_int32_t DeltaTick;
    rt_tick_t Smooth_EndTick;
    // 使用信号量进行读写保护
    if (rt_sem_take(&CTRL->sem, 20) == RT_EOK) // 挂起时间过长时直接放弃
    {                                          // 如果正常取到了信号量，则再取一次确保没有多余的信号量
        rt_sem_trytake(&CTRL->sem);
    }

    SetTemp = CTRL->NowSetData + Add;

    DataLimFix(&SetTemp, CTRL); // 输入数据限幅
    CTRL->NowSetData = SetTemp;

    Smooth_EndTick = CTRL->Set_SmthPeriodMax + CTRL->SetData_FreshTime;

    if (Tick_Now < Smooth_EndTick)
    { // 上一平滑过程还没结束
        DeltaTick = Tick_Now - CTRL->OutData_FreshTime;
    }
    else if (CTRL->OutData_FreshTime < Smooth_EndTick)
    { // 上一平滑已经结束，检查输出是否已经结算到平滑结束时间
        DeltaTick = Smooth_EndTick - CTRL->OutData_FreshTime;
    }
    else
    { // 平滑结束
        DeltaTick = 0;
    }
    CTRL->SetFresh_DataAdd += CTRL->AddData_Speed * DeltaTick; // 记录之前没有应用的增量数据
    CTRL->SetFresh_Flag = 1;

    CTRL->SetData_FreshTime = Tick_Now; // 记录数据输入时刻
    CTRL->OutData_FreshTime = Tick_Now; // 重置数据输出结算时刻

    Fresh_DeltaData(CTRL); // 计算AddTemp
    CTRL->AddData_Speed = CTRL->AddTemp / CTRL->Set_SmthPeriodMax;

    rt_sem_release(&CTRL->sem); // 访问完毕，重新释放信号量
}

// 获取平滑后的数据的绝对数值
void Smooth_GetDataABS(float *GetDataABS, ExactSmth_CTRL_S *CTRL)
{
    // 使用信号量进行读写保护
    if (rt_sem_take(&CTRL->sem, 20) == RT_EOK) // 挂起时间过长时直接放弃
    {                                          // 如果正常取到了信号量，则再取一次确保没有多余的信号量
        rt_sem_trytake(&CTRL->sem);
    }
    // 刷新数据
    Smooth_FreshOutData(CTRL);
    // 输出数据
    *GetDataABS = CTRL->NowOutData;

    rt_sem_release(&CTRL->sem); // 访问完毕，重新释放信号量
}

// 获取平滑后的数据的变化增量
void Smooth_GetDataADD(float *GetDataADD, ExactSmth_CTRL_S *CTRL)
{
    // 使用信号量进行读写保护
    if (rt_sem_take(&CTRL->sem, 20) == RT_EOK) // 挂起时间过长时直接放弃
    {                                          // 如果正常取到了信号量，则再取一次确保没有多余的信号量
        rt_sem_trytake(&CTRL->sem);
    }
    // 刷新数据
    Smooth_FreshOutData(CTRL);
    // 输出数据
    *GetDataADD = CTRL->NowOutData_Add;

    rt_sem_release(&CTRL->sem); // 访问完毕，重新释放信号量
}

// 按照指定的数值重启平滑，一般用于切换数据源后无缝换回
void Smooth_SetData_Restart(ExactSmth_CTRL_S *CTRL, float SetNow)
{
    // 使用信号量进行读写保护
    if (rt_sem_take(&CTRL->sem, 20) == RT_EOK) // 挂起时间过长时直接放弃
    {                                          // 如果正常取到了信号量，则再取一次确保没有多余的信号量
        rt_sem_trytake(&CTRL->sem);
    }
    CTRL->OutData_FreshTime = rt_tick_get();
    CTRL->SetData_FreshTime = rt_tick_get();
    CTRL->NowSetData = SetNow;
    CTRL->NowOutData = SetNow;
    CTRL->NowOutData_Add = 0;
    CTRL->AddData_Speed = 0;
    CTRL->AddTemp = 0;
    CTRL->SetFresh_Flag = 0;
    CTRL->SetFresh_DataAdd = 0;
    rt_sem_release(&CTRL->sem); // 访问完毕，重新释放信号量
}

// 初始化平滑控制块 初始化后默认没有限幅和跨圈处理 需要给定初始设定值和平滑时间
// 如有限幅/跨圈处理需要 则可以在初始化完成后调用 Smooth_SetDataFix 进行单独设定
void Smooth_Init(ExactSmth_CTRL_S *CTRL, float InitSet, int PeriodMax, char Lim, float Limup, float Limdown)
{
    char Name[8] = {'S', 'm', '0', '0', 's', 'e', 'm', '\0'};
    if (Smooth_CTRL_InitCount <= 99)
    { // 按照初始化顺序分配信号量名称
        Name[3] += Smooth_CTRL_InitCount % 10;
        Name[2] += Smooth_CTRL_InitCount / 10;
        Smooth_CTRL_InitCount++;
    }
    else
    {
        while (1)
        {
            ; // Smooth控制块最多初始化100个，更多则无法分配信号量名称
        }
    }
    rt_sem_init(&CTRL->sem, (const char *)Name, 1, RT_IPC_FLAG_FIFO); // 初始化读写保护信号量

    CTRL->OutData_FreshTime = rt_tick_get();
    CTRL->SetData_FreshTime = rt_tick_get();
    CTRL->NowSetData = InitSet;
    CTRL->NowOutData = InitSet;
    CTRL->NowOutData_Add = 0;
    CTRL->AddData_Speed = 0;
    CTRL->AddTemp = 0;
    CTRL->SetFresh_Flag = 0;
    CTRL->SetFresh_DataAdd = 0;
    CTRL->Set_SmthPeriodMax = PeriodMax;
		if(Lim == 0)
		{
			CTRL->LimFlag = 0;
			CTRL->CircleFlag = 0;
			CTRL->UpLim = 0;
			CTRL->DownLim = 0;
			CTRL->Lim_Len = 0;
		}
    else if(Lim == 1)
		{
			CTRL->LimFlag = 1;
			CTRL->CircleFlag = 0;
			CTRL->UpLim = Limup;
			CTRL->DownLim = Limdown;
			CTRL->Lim_Len = 0;
		}
}

// 可选的跨圈处理功能：设定限幅/跨圈处理后，程序会按照设定的范围对参与平滑的数据进行限幅或跨圈处理
// 当CircleFlag==0时，仅对数据进行限幅处理，不会对多余部分进行跨圈处理
// 当CircleFlag==1时，若数据出现超限，会按照限幅对应的数据长度进行跨圈处理，溢出部分会进入下一圈
void Smooth_SetDataFix(ExactSmth_CTRL_S *CTRL, float UpLim, float DownLim, char CircleFlag)
{
    if (UpLim <= DownLim)
    {
        while (1)
        {
            ; // UpLim不能小于等于DownLim
        }
    }

    // Enable Limit of SetData and OutData
    // SetBit: LimFlag
    CTRL->LimFlag = 1;
    CTRL->UpLim = UpLim;
    CTRL->DownLim = DownLim;

    if (CircleFlag)
    {
        CTRL->CircleFlag = 1;
        CTRL->Lim_Len = UpLim - DownLim; // 计算出当前跨圈的圈长
    }
}
