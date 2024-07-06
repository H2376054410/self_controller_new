#ifndef _FUN_UI_LIST_H_
#define _FUN_UI_LIST_H_
#include "list_d.h"
#include "fun_drawCustomUI.h"

typedef struct
{
    ui_basic_e (*func)(void *param); // 回调函数
    int16_t run_times;               // 总共运行次数
    uint16_t period;                 // 运行的循环次数 即每个几次循环运行一次
} ui_func_t;

typedef struct
{
    uint8_t id;
    float position;
    uint16_t hurt_time; // 攻击时间
    uint8_t hurt_type;
} ui_enemy_t;

// 共用体
typedef union
{
    ui_func_t func;   // 回调函数运行相关
    ui_enemy_t enemy; // 敌人信息相关
} ui_member_u;

typedef struct
{
    ListItem_t list_node;   // 双向链表
    UI_imageName_e name;    // 该TCB的名字 便于后期拓展
    ui_member_u member;     // 主要成员
    uint8_t sendSeparatley; // 是否需要单独发送
} ui_TCB;                   // 自定义结构体用于挂载到链表当中

// 初始化链表
void UI_List_Init(List_t *ui_list);

// 删除节点拥有者 entry:删除的节点
void UI_List_Delete(ui_TCB *ui_TCB, ListItem_t *List_Item);

// 判断链表是否为空
rt_bool_t UI_List_IsEmpty(List_t *head);

// 清空所有链表
rt_bool_t UI_list_Destroy(List_t *list_head);

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
                       UI_imageName_e name, uint8_t ISsendSeparatley);

void UI_TCB_Init(ui_TCB *UI_TCB);
#endif
