#include "func_MusicPlayer.h"

#include "rtthread.h"
#include "drv_thread.h"
#include "drv_thread.h"
#include "drv_Queue.h"
#include "app_monitor.h"
// 触发式播放的音效队列
static int SingleSoundList[SOUND_QUEUE_MAX];
static LoopQueueCTRL_Type Queue; // 队列管理块

// 返回当前队列中等待播放的音效的个数
int SoundDisplay_GetCount(void)
{
    return Queue.Valid_Data;
}

// 向队列中添加一个需要播放的音效
void SoundDisplay_AddSound(PrompEQ_e SoundNum)
{
    // 检查队列是否已满，如果已满则不再写入
    if (Queue.Valid_Data == Queue.Len)
    {
        // 队列已满，放弃写入
        return;
    }
    else
    {
        // 向队列中添加一个需要播放的音效
        SingleSoundList[Queue_GetWriteNum(&Queue)] = SoundNum;
        return;
    }
}

// 音效播放线程
static void SoundDisplay_entry(void *Para)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("SoundDisplay_entry", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (1)
    {
        SWDG_FEED(id);
        if (Queue.Valid_Data == 0)
        {
            rt_thread_delay(10);
        }
        else
        {
            // 执行播放任务
            PromptEQ_Set((PrompEQ_e)SingleSoundList[Queue_Get_ReadEnd(&Queue)]);
            Queue_Delete_End(&Queue);
        }
    }
}

/**
 * @brief  任务创建
 */
static void SoundDisplay_start(void)
{
    rt_thread_t thread;
    thread = rt_thread_create("SD_trd", SoundDisplay_entry, RT_NULL,
                              THREAD_STACK_MUSICCTRL,
                              THREAD_PRIO_MUSICCTRL,
                              THREAD_TICK_MUSICCTRL);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
}

// 音效播放初始化
void SoundDisplay_Init()
{
    QueueCtrl_Init(&Queue, SOUND_QUEUE_MAX);
    SoundDisplay_AddSound(StartUp_EQ);

    SoundDisplay_start();
}
