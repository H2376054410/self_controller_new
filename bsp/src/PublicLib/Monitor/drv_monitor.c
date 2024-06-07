#include "drv_monitor.h"
#include <rtthread.h>
#include <board.h>

// 该文件实现了对看门狗单向链表的创建和报警双向链表创建的函数

// 通过名字动态查找的相关变量
swdg_info_t swdg_info[SWDG_DEV_MAXNUM_S]; // 名字和ID
uint16_t swdg_Index = 0;                  // 当前已用ID数 按顺序自增

static rt_sem_t swdg_list_sem = RT_NULL; // 用来对操作链表上锁的信号量
// 报警线程双向链表头
static alarm_dev_t alarm_head;
// 取头指针
static alarm_dev_t *alarm_hp = &alarm_head;
static rt_sem_t alarm_sem = RT_NULL; // 用来对操作链表上锁的信号量
static uint8_t alarmIsInit;

void InitSwdgHandle(rt_err_t (*handle)(rt_bool_t))
{
    uint16_t index = 0;
    while (SWDG_INITED_FLAG == swdg_info[index].swdg_dev.flag_inited) // 需要判断该模块是否真的已经被初始化
    {
        swdg_info[index].swdg_dev.handle = handle; // 处理函数
        index++;
    }
}

/**
 * @brief  创建一个软件看门狗对象
 * @param  color  报警颜色
 * @param  if_alarm  是否启用报警功能
 * @param  time_threshold    报警时间，超过该时间不喂狗则会报警（单位ms）
 * @param  handle    异常处理函数指针
 * @param  init_flag 初始化用的标志位
 * @return RT_ERROR  初始化失败，RT_EOK  初始化成功
 */
rt_err_t Swdg_Create(Alarm_color_e color, rt_bool_t if_alarm,
                     rt_uint32_t time_threshold, rt_err_t (*handle)(rt_bool_t),
                     rt_uint8_t init_flag)
{
    // 判断信号量是不是存在
    if (!swdg_list_sem)
    {
        swdg_list_sem = rt_sem_create("MonitorSem", 1, RT_IPC_FLAG_FIFO);
        if (!swdg_list_sem)
            return RT_ERROR;
    }

    // 操作前第一步是上锁
    rt_sem_take(swdg_list_sem, RT_WAITING_FOREVER);
    while (RT_EOK == rt_sem_trytake(swdg_list_sem))
        continue;

    if (swdg_Index >= SWDG_DEV_MAXNUM_S)
        return RT_ERROR;

    // 赋值
    swdg_info[swdg_Index].if_start = RT_FALSE;
    swdg_info[swdg_Index].swdg_Id = swdg_Index;
    swdg_info[swdg_Index].isError = RT_FALSE;

    swdg_info[swdg_Index].swdg_dev.ID = swdg_Index;
    swdg_info[swdg_Index].swdg_dev.flag_inited = init_flag;
    swdg_info[swdg_Index].swdg_dev.color = color;
    swdg_info[swdg_Index].swdg_dev.if_alarm = if_alarm;
    swdg_info[swdg_Index].swdg_dev.time_threshold = time_threshold;
    swdg_info[swdg_Index].swdg_dev.time_deadline = rt_tick_get() + time_threshold;
    swdg_info[swdg_Index].swdg_dev.handle = handle;

    swdg_Index++;
    // 操作完成, 解锁
    rt_sem_release(swdg_list_sem);
    return RT_EOK;
}

/**
 * @brief initialize a list
 * @param list_node list to be initialized
 */
void Mlist_Init(alarm_dev_t *list_node)
{
    list_node->next = list_node;
    list_node->prev = list_node;
    list_node->swdg = RT_NULL;
    alarmIsInit = 1;
}

/***
 * @brief  insert a node after alarm_head
 * @param  insertSwdgDev   没有及时被喂狗的swdg指针
 * @return none
 * @author Lvfp
 ***/

void Mlist_Insert(swdg_info_t *insertSwdgDev)
{
    if (alarmIsInit)
    {
        // 判断信号量是不是存在
        if (!alarm_sem)
        {
            alarm_sem = rt_sem_create("AlarmSem", 1, RT_IPC_FLAG_FIFO);
            if (!alarm_sem)
            {
                RT_ASSERT(0);
            }
            else
                rt_sem_release(alarm_sem);
        }

        // 操作前第一步是上锁
        rt_sem_take(alarm_sem, RT_WAITING_FOREVER);

        // 申请空间，挂上指针
        alarm_dev_t *n = (alarm_dev_t *)rt_malloc(sizeof(alarm_dev_t));

        if (NULL == n)
            return;

        n->swdg = insertSwdgDev;

        // 尾插法

        // 新节点
        n->next = alarm_hp;       // 新节点的下一个节点为头节点
        n->prev = alarm_hp->prev; // 新节点的前一个节点为尾节点

        // 尾结点
        alarm_hp->prev->next = n; // 插入前的尾节点的下一个节点尾当前节点
        alarm_hp->prev = n;       // 尾结点为当前节点

        // 操作完以后解锁
        rt_sem_release(alarm_sem);
    }
}

/***
 * @brief  remove node from list and release memory block
 * @param  mID: 对应监视器的ID
 ***/
void Mlist_Remove(uint16_t mID)
{
    if (alarmIsInit)
    {
        // 判断信号量是不是存在
        if (!alarm_sem)
        {
            alarm_sem = rt_sem_create("AlarmSem", 1, RT_IPC_FLAG_FIFO);
            if (!alarm_sem)
                RT_ASSERT(0);
        }

        alarm_dev_t *n = RT_NULL;
        alarm_dev_t *alarm_dev_tem = alarm_hp->next;

        // 索引找到ID对应的报警链表的节点
        while (alarm_dev_tem != alarm_hp)
        {
            if (mID == alarm_dev_tem->swdg->swdg_Id)
            {
                n = alarm_dev_tem;
                break;
            }
            alarm_dev_tem = alarm_dev_tem->next;
        }

        if (n == RT_NULL)
            return; // 无效id

        // 操作前第一步是上锁
        rt_sem_take(alarm_sem, RT_WAITING_FOREVER);
        while (RT_EOK == rt_sem_trytake(alarm_sem))
            continue;

        n->next->prev = n->prev;
        n->prev->next = n->next;

        // 操作完以后解锁
        rt_sem_release(alarm_sem);

        // n->next = n->prev = n;
        rt_free(n);
    }
}

swdg_info_t *getSwdgInfo(void)
{
    return &swdg_info[0];
}
alarm_dev_t *getAlarm_hp(void)
{
    return alarm_hp;
}

uint8_t getSwdgNum(void)
{
    return swdg_Index;
}
