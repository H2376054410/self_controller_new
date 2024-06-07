#ifndef __DRV_QUEUE_H__
#define __DRV_QUEUE_H__

//静态循环队列控制句柄
typedef struct LoopQueue
{
    int Len;        // 记录队列长度
    int Write_p;    // 记录写入指针
    int Valid_Data; // 记录有效数据个数
} LoopQueueCTRL_Type;

// 获取队列写入位置
extern int Queue_GetWriteNum(LoopQueueCTRL_Type *QueueCtrlSTR);
// 获取循环队列最新一次写入位置处，向前指定个数的数据位置
extern int Queue_Get_ReadOld(LoopQueueCTRL_Type *QueueCtrlSTR, int Old_Count);
// 获取循环队列最久远的数据的位置
extern int Queue_Get_ReadEnd(LoopQueueCTRL_Type *QueueCtrlSTR);
// 删除循环队列最久远的数据
extern void Queue_Delete_End(LoopQueueCTRL_Type *QueueCtrlSTR);
// 初始化队列控制块
extern void QueueCtrl_Init(LoopQueueCTRL_Type *QueueCtrlSTR, int QueueLen);

#endif
