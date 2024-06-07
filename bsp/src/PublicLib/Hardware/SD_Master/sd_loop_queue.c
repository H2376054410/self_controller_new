#include "sd_loop_queue.h"

/**
 * @brief 环形队列-初始化
 * 
 * @param lq    
 * @param size 
 * @param buff 
 */
void lq_init(LoopQueue_Ctrl_Type *lq, rt_uint32_t size, rt_uint8_t *buff)
{
    lq->size = size;
    lq->buff = buff;
    lq->head = 0;
    lq->tail = 0;
    lq->status = 1;
}

/**
 * @brief 环形队列-是否空
 * 
 * @param lq 
 * @return rt_uint8_t 空：1，非空：0
 */
rt_uint8_t lq_isEmpty(LoopQueue_Ctrl_Type *lq)
{
    if (lq->status == 1)
        return 1;
    else
        return 0;
}

/**
 * @brief 环形队列-是否满
 * 
 * @param lq 
 * @return rt_uint8_t 满：1，非满：0
 */
rt_uint8_t lq_isFull(LoopQueue_Ctrl_Type *lq)
{
    if (lq->status == 2)
        return 1;
    else
        return 0;
}

/**
 * @brief 环形队列-写入后队列满的判定
 *        这么做是因为数据结构特性（需要格外判断 head == tail 代表空/满）
 * 
 * @note  每次写操作后都要调用一次；要求调用函数对 0 做特判：0 保持原状，不调用
 * 
 * @param lq 
 */
static void lq_pushJudgeFull(LoopQueue_Ctrl_Type *lq)
{
    if (lq->head == lq->tail)
        lq->status = 2;
    else
        lq->status = 0;
}

/**
 * @brief 环形队列-读出后队列空的判定
 *        这么做是因为数据结构特性（需要格外判断 head == tail 代表空/满）
 * 
 * @note  每次读操作后都要调用一次；要求调用函数对 0 做特判：0 保持原状，不调用
 * 
 * @param lq 
 */
static void lq_popJudgeEmpty(LoopQueue_Ctrl_Type *lq)
{
    if (lq->head == lq->tail)
        lq->status = 1;
    else
        lq->status = 0;
}

/* 环形队列-队尾入队列 */
// 成功：1，失败：0
rt_uint8_t lq_push(LoopQueue_Ctrl_Type *lq, rt_uint8_t data)
{
    if (lq_isFull(lq))
        return 0;

    lq->buff[lq->tail] = data;
    lq->tail = (lq->tail + 1) % lq->size;

    lq_pushJudgeFull(lq);

    return 1;
}


/* @brief 环形队列-队头出队列 */
rt_uint8_t lq_pop(LoopQueue_Ctrl_Type *lq)
{
    rt_uint8_t out = 0;

    if (lq_isEmpty(lq) == 1)  // 队列为空
        return 0;
    
    out = lq->buff[lq->head];
    lq->head = (lq->head + 1) % lq->size;

    lq_popJudgeEmpty(lq);

    return out;
}

/**
 * @brief 环形队列-有效元素个数
 * 
 * @param lq 
 * @return rt_uint32_t 
 */
rt_uint32_t lq_size(LoopQueue_Ctrl_Type *lq)
{
    if (lq->head == lq->tail)
    {
        if (lq_isEmpty(lq))
            return 0;
        else if (lq_isFull(lq))
            return lq->size;
    }
    else if (lq->head > lq->tail)
        return lq->size - lq->head + lq->tail;

        return lq->tail - lq->head;
}

// 队列空空间
// return 队列空的空间大小
rt_uint32_t lq_esz(LoopQueue_Ctrl_Type *lq)
{
    return lq->size - lq_size(lq);
}

// 写入一组数据
// return 1：失败（队列空间不足） 0：成功
rt_err_t lq_pushString(LoopQueue_Ctrl_Type *lq, rt_uint8_t *buff, rt_uint32_t size)
{
    if (size == 0)
        return 0;

    if (lq_esz(lq) < size)
        return 1;

    if (lq->size >= lq->tail + size)
    {  // 未跨圈
        rt_memcpy(&lq->buff[lq->tail], buff, size);

        lq->tail = (lq->tail + size) % lq->size;
    }
    else
    {
        rt_memcpy(&lq->buff[lq->tail], buff, lq->size - lq->tail);
        rt_memcpy(&lq->buff[0], buff + lq->size - lq->tail, size + lq->tail - lq->size);

        lq->tail = size + lq->tail - lq->size;
    }

    lq_pushJudgeFull(lq);

    return 0;
}

// 读取并弹出一组数据
// return 1：失败（队列数据不足） 0：成功
rt_err_t lq_popString(LoopQueue_Ctrl_Type *lq, rt_uint8_t *buff, rt_uint32_t size)
{
    if (size == 0)
        return 0;

    if (lq_size(lq) < size)  // 确保有足够空间存储
        return 1;
    
    if (lq->size >= lq->head + size)
    {  // 未跨圈
        rt_memcpy(buff, &lq->buff[lq->head], size);

        lq->head = (lq->head + size) % lq->size;
    }
    else
    {
        rt_memcpy(buff, &lq->buff[lq->head], lq->size - lq->head);
        rt_memcpy(buff + lq->size - lq->head, &lq->buff[0], size + lq->head - lq->size);

        lq->head = size + lq->head - lq->size;
    }

    lq_popJudgeEmpty(lq);

    return 0;
}

// 只读不弹出
rt_err_t lq_getString(LoopQueue_Ctrl_Type *lq, rt_uint8_t *buff, rt_uint32_t size)
{
    if (lq_size(lq) < size)  // 确保有足够元素
        return 1;
    
    if (lq->size >= lq->head + size)
    {
        rt_memcpy(buff, &lq->buff[lq->head], size);
        
        // lq->head = (lq->head + size) % lq->size;
    }
    else
    {
        rt_memcpy(buff, &lq->buff[lq->head], lq->size - lq->head);
        rt_memcpy(buff + lq->size - lq->head, &lq->buff[0], size + lq->head - lq->size);

        // lq->head = size + lq->head - lq->size;
    }

    return 0;
}

// 只弹出不读
rt_err_t lq_discardString(LoopQueue_Ctrl_Type *lq, rt_uint32_t size)
{
    if (size == 0)
        return 0;

    if (lq_size(lq) < size)  // 确保有足够空间存储
        return 1;
    
    if (lq->size >= lq->head + size)
    {
        // rt_memcpy(buff, &lq->buff[lq->head], size);

        lq->head = (lq->head + size) % lq->size;
    }
    else
    {
        // rt_memcpy(buff, &lq->buff[lq->head], lq->size - lq->head);
        // rt_memcpy(buff + size + lq->head - lq->size, &lq->buff[0], size + lq->head - lq->size);

        lq->head = size + lq->head - lq->size;
    }

    lq_popJudgeEmpty(lq);

    return 0;
}
