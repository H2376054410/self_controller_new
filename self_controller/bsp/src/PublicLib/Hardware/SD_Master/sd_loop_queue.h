#ifndef __LOOP_QUEUE_H__
#define __LOOP_QUEUE_H__

#include <rtthread.h>

typedef struct
{
    rt_uint32_t size;
    rt_uint8_t *buff;
    rt_uint32_t head;
    rt_uint32_t tail;
    rt_uint32_t status;  // 0：非空非满  1：空  2：满
} LoopQueue_Ctrl_Type;

void lq_init(LoopQueue_Ctrl_Type *lq, rt_uint32_t size, rt_uint8_t *buff);
rt_uint8_t lq_isEmpty(LoopQueue_Ctrl_Type *lq);
rt_uint8_t lq_isFull(LoopQueue_Ctrl_Type *lq);
rt_uint8_t lq_push(LoopQueue_Ctrl_Type *lq, rt_uint8_t data);
rt_uint8_t lq_pop(LoopQueue_Ctrl_Type *lq);
rt_uint32_t lq_size(LoopQueue_Ctrl_Type *lq);
rt_uint32_t lq_esz(LoopQueue_Ctrl_Type *lq);
rt_err_t lq_pushString(LoopQueue_Ctrl_Type *lq, rt_uint8_t *buff, rt_uint32_t size);
rt_err_t lq_popString(LoopQueue_Ctrl_Type *lq, rt_uint8_t *buff, rt_uint32_t size);
rt_err_t lq_getString(LoopQueue_Ctrl_Type *lq, rt_uint8_t *buff, rt_uint32_t size);
rt_err_t lq_discardString(LoopQueue_Ctrl_Type *lq, rt_uint32_t size);

#endif
