#include "drv_Queue.h"

int Queue_GetWriteNum(LoopQueueCTRL_Type* QueueCtrlSTR)
{
    QueueCtrlSTR->Write_p++;
    QueueCtrlSTR->Valid_Data++;
    if (QueueCtrlSTR->Valid_Data > QueueCtrlSTR->Len)
    {
        QueueCtrlSTR->Valid_Data = QueueCtrlSTR->Len;
    }
    if (QueueCtrlSTR->Write_p >= QueueCtrlSTR->Len)
    {
        QueueCtrlSTR->Write_p = 0;
    }
    return QueueCtrlSTR->Write_p; //写入数据并修改写入指针
}


// 获取循环队列最久远的数据的位置
int Queue_Get_ReadEnd(LoopQueueCTRL_Type *QueueCtrlSTR)
{
    int Read_p;
    if (QueueCtrlSTR->Valid_Data > 0)
    {// 有数据
        Read_p = QueueCtrlSTR->Write_p - QueueCtrlSTR->Valid_Data + 1;
    }
    if (Read_p < 0)
    {
        Read_p += QueueCtrlSTR->Len;
    }
    return Read_p;
}

// 获取循环队列最新一次写入位置处，向前指定个数的数据位置
int Queue_Get_ReadOld(LoopQueueCTRL_Type *QueueCtrlSTR, int Old_Count)
{
    int Read_p;
    if (Old_Count > QueueCtrlSTR->Valid_Data - 1)
    { //数组不足则返回最旧的数据
        Read_p = Queue_Get_ReadEnd(QueueCtrlSTR);
    }
    else if (Old_Count < 0)
    { // 不能获取未来数据，返回最近的一组数据
        Read_p = QueueCtrlSTR->Write_p;
    }
    else
    { //数组充足，可以正常计算所需数据的存放位置
        Read_p = QueueCtrlSTR->Write_p - Old_Count;
        if (Read_p < 0)
        {
            Read_p += QueueCtrlSTR->Len;
        }
    }
    return Read_p;
}

// 删除循环队列最久远的数据
void Queue_Delete_End(LoopQueueCTRL_Type *QueueCtrlSTR)
{
    if (QueueCtrlSTR->Valid_Data>0)
    {
        QueueCtrlSTR->Valid_Data--;
    }
}

// 初始化队列控制块
void QueueCtrl_Init(LoopQueueCTRL_Type *QueueCtrlSTR,int QueueLen)
{
    QueueCtrlSTR->Valid_Data = 0;
    QueueCtrlSTR->Write_p = 0;
    QueueCtrlSTR->Len = QueueLen;
}
