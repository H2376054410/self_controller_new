#ifndef __FUNC_MUSICPLAYER_H__
#define __FUNC_MUSICPLAYER_H__

#include "drv_MusicLibrary.h"

// 播放队列最大值
#define SOUND_QUEUE_MAX 2

// 返回当前队列中等待播放的音效的个数
int SoundDisplay_GetCount(void);

// 向队列中添加一个需要播放的音效
void SoundDisplay_AddSound(PrompEQ_e SoundNum);

// 音效播放线程创建
void SoundDisplay_Init(void);
#endif
