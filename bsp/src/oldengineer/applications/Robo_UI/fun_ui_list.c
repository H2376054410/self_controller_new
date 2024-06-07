#include "fun_ui_list.h"
#include "string.h"

void UI_TCB_Init(ui_TCB *UI_TCB)
{
    UI_TCB->name = (UI_imageName_e)0;
    UI_TCB->list_node.pvContainer = NULL;
    UI_TCB->list_node.pxNext = NULL;
    UI_TCB->list_node.pvOwner = NULL;
    UI_TCB->list_node.pxPrevious = NULL;
    UI_TCB->list_node.xItemValue = 0;
    memset(&(UI_TCB->member), 0, sizeof(UI_TCB->member));
}

// 初始化链表根节点
void UI_List_Init(List_t *ui_list)
{
    vListInitialise(ui_list);
}

/**
 * @brief  将节点从链表根节点移除 因为节点本身记录其挂载的根节点所以无需根节点
 * @param {ui_TCB} *ui_TCB 自定义节点
 * @param {ListItem_t} *List_Item 将指定指针指向前一节点便于遍历--这里可以使用根节点的链表节点索引指针
 */
void UI_List_Delete(ui_TCB *ui_TCB, ListItem_t *List_Item)
{
    // 链表节点本身记录挂在哪个链表
    if (List_Item != NULL)
    {
        List_Item = List_Item->pxPrevious;
    }

    // 链表节点本身记录挂在哪个链表
    uxListRemove(&ui_TCB->list_node);
    rt_free(ui_TCB);
    ui_TCB = NULL;
}

// 判断当前根节点是否挂载节点
rt_bool_t UI_List_IsEmpty(List_t *head)
{
    if (listLIST_IS_EMPTY(head))
        return RT_TRUE;
    else
        return RT_FALSE;
}

// 销毁ui整条链表 list_head:链表头指针所在结构体
rt_bool_t UI_list_Destroy(List_t *list_head)
{
    ListItem_t *temp;

    // 得到要删除链表的第一个节点
    ListItem_t *pxListItem = listGET_HEAD_ENTRY(list_head);
    while (!listLIST_IS_EMPTY(list_head))
    {
        // 保存当前节点
        temp = pxListItem;

        // 从链表移除
        uxListRemove(pxListItem);

        // 得到下一个节点
        pxListItem = listGET_NEXT(temp);
        // 释放当前节点拥有者
        rt_free(temp->pvOwner);
    }
    if (listCURRENT_LIST_LENGTH(list_head) == 0)
        return RT_TRUE;
    else
        return RT_FALSE;
}

/**
 * @brief 添加ui绘制函数
 * @param {List_t} *head 链表头指针所在结构体
 * @param {func} UI绘制函数
 * @param {uint16_t} run_times 该UI绘制次数   n=-1:一直绘制   n>0:绘制n次
 * @param {int} period 运行周期,每循环n次绘制一次, 注:只绘制一次的图形要设成1,
 *                     不然重置UI的时候有概率无法显示
 * @param {uint32_t} name 自定义链表节点名字(纯数字) 便于拓展
 */
ui_TCB *UI_FuncListAdd(List_t *head, ui_basic_e (*func)(void *param),
                       int16_t run_times, int period,
                       UI_imageName_e name, uint8_t ISsendSeparatley)
{
    // 申请一个UI 自定义控制块
    ui_TCB *ui_tcb_new = (ui_TCB *)rt_malloc(sizeof(ui_TCB));
    UI_TCB_Init(ui_tcb_new);

    if (ui_tcb_new == NULL)
        return NULL;

    (ui_tcb_new->member).func.run_times = (int16_t)run_times;
    (ui_tcb_new->member).func.period = period;
    (ui_tcb_new->member).func.func = func;

    ui_tcb_new->list_node.xItemValue = name; // 链表节点的值即为图像的名字
    ui_tcb_new->name = name;                 // 图像的名字
    ui_tcb_new->sendSeparatley = ISsendSeparatley;

    // 记住当前UI_TCB
    listSET_LIST_ITEM_OWNER(&ui_tcb_new->list_node, ui_tcb_new);

    // 尾插链表当中
    vListInsertEnd(head, &ui_tcb_new->list_node);

    return ui_tcb_new;
}
