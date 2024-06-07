#ifndef _LIST_D_H_
#define _LIST_D_H_

#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>

///***************************freertos*************************************************************************/

// 最大数据个数的类型可以更改
typedef uint8_t list_count_t;

// 一个链表最大节点
#define list_MAX_NUM 20

// 节点数据类型
typedef struct xLIST_ITEM
{
    list_count_t xItemValue;       /* 辅助值，用于帮助节点做顺序排列 */
    struct xLIST_ITEM *pxNext;     /* 指向链表下一个节点 */
    struct xLIST_ITEM *pxPrevious; /* 指向链表前一个节点 */
    void *pvOwner;                 /* 指向拥有该节点的内核对象， 即可以自定义数据类型挂载到链表上*/
    void *pvContainer;             /* 指向该节点所在的链表 */

} ListItem_t;

/* 精简节点数据类型重定义 */
typedef struct xMINI_LIST_ITEM
{
    list_count_t xItemValue;       /* 辅助值，用于帮助节点做升序排列 */
    struct xLIST_ITEM *pxNext;     /* 指向链表下一个节点 */
    struct xLIST_ITEM *pxPrevious; /* 指向链表前一个节点 */

} MiniListItem_t;

// 链表根节点数据类型
typedef struct xLIST
{
    list_count_t uxNumberOfItems; /* 链表节点计数器 */
    ListItem_t *pxIndex;          /* 链表节点索引指针 */
    MiniListItem_t xListEnd;      /* 链表最后一个节点 */

} List_t;

// 初始化链表节点
void vListInitialiseItem(ListItem_t *const pxItem);
// 初始化链表根节点
void vListInitialise(List_t *const pxList);
// 尾(头)插入链表
void vListInsertEnd(List_t *const pxList, ListItem_t *const pxNewListItem);
// 按照值大小升序插入链表
void vListInsert(List_t *const pxList, ListItem_t *const pxNewListItem);
// 移除链表中的节点
list_count_t uxListRemove(ListItem_t *const pxItemToRemove);

/***************************实用宏函数*******************************************************/
/* 初始化节点的拥有者 */
#define listSET_LIST_ITEM_OWNER(pxListItem, pxOwner) ((pxListItem)->pvOwner = (void *)(pxOwner))

/* 获取节点拥有者 */
#define listGET_LIST_ITEM_OWNER(pxListItem) ((pxListItem)->pvOwner)

/* 初始化节点排序辅助值 */
#define listSET_LIST_ITEM_VALUE(pxListItem, xValue) ((pxListItem)->xItemValue = (xValue))

/* 获取节点排序辅助值 */
#define listGET_LIST_ITEM_VALUE(pxListItem) ((pxListItem)->xItemValue)

/* 获取链表根节点的节点计数器的值 */
#define listGET_ITEM_VALUE_OF_HEAD_ENTRY(pxList) (((pxList)->xListEnd).pxNext->xItemValue)

/* 获取链表的入口节点 */
#define listGET_HEAD_ENTRY(pxList) (((pxList)->xListEnd).pxNext)

/* 获取节点的下一个节点 */
#define listGET_NEXT(pxListItem) ((pxListItem)->pxNext)

/* 获取链表的最后一个节点 */
#define listGET_END_MARKER(pxList) ((ListItem_t const *)(&((pxList)->xListEnd)))

/* 判断链表是否为空 */
#define listLIST_IS_EMPTY(pxList) ((list_count_t)((pxList)->uxNumberOfItems == (list_count_t)0))

/* 获取链表的节点数 */
#define listCURRENT_LIST_LENGTH(pxList) ((pxList)->uxNumberOfItems)

/* 获取链表第一个节点的 OWNER，即 TCB 为自定义数据类型 */
#define listGET_OWNER_OF_NEXT_ENTRY(pxTCB, pxlist)                                \
    {                                                                             \
        list *const pxConstlsit = (pxlist);                                       \
        (pxConstList)->pxIndex = (pxConstList)->pxIndex->pxNext;                  \
        if ((void *)(pxConstList)->pxIndex == (void *)&((pxConstList)->xListEnd)) \
            (pxConstList)->pxIndex = (pxConstList)->pxIndex->pxNext;              \
        (pxTCB) = (pxConstList)->pxIndex->pvOwner;                                \
    }

/***************************freertos*************************************************************************/
#endif
