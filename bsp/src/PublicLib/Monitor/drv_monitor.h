#ifndef __DRV_MONITOR_H__
#define __DRV_MONITOR_H__

#include <rtthread.h>
#include <rtdevice.h>

#define SWDG_INITED_FLAG (0x20)

// 最大名字限制
#define SWDG_NAMEMAXLEN 30

// 最多看门狗对象
#define SWDG_DEV_MAXNUM_S 18

/*要拓展更多的RGB报警颜色需要在RGB驱动文件.h增加三元色宏；
在Alarm_Thread线程的swicth语句内增加case选项；*/
typedef enum
{
    ALARM_NULL = -1,
    ALARM_WHITE = 0,
    ALARM_RED,
    ALARM_BLUE,
    ALARM_GREEN,
    ALARM_YELLOW,
    ALARM_PURPLE,
    ALARM_BROWN,

} Alarm_color_e;

/*这里采用链表套结构体的方法，优点：思路简单易实现；缺点：通用性不够*/
// 看门狗句柄
typedef struct swdg_dev
{
    uint16_t ID; // 小看门狗跟踪设备的id，id是唯一的

    Alarm_color_e color; // 监视器报警的RGB灯颜色

    rt_uint32_t time_threshold; // 时间阈值，超过该时间未响应则报警，单位ms
    rt_tick_t time_deadline;    // 报警时刻, 到这个时刻不喂狗就直接报警

    rt_uint8_t flag_inited; // 写入 0x20 代表初始化成功, 反之表示未初始化 只有初始化才能遍历该对象 Swdg_Create将其置位

    rt_bool_t if_alarm; // RT_TRUE:启用报警，RT_FALSE:关闭报警功能 控制RGB，蜂鸣器等

    rt_err_t (*handle)(rt_bool_t); // 异常对应的处理函数指针,(触发式函数，不会一直轮询)

} swdg_dev_t;

typedef struct
{
    uint16_t swdg_Id;
    char swdg_name[SWDG_NAMEMAXLEN];
    uint8_t if_start; // 看门狗开始工作 Swdg_Start将其置1
    uint8_t isError;
    uint32_t errCounts;
    swdg_dev_t swdg_dev;
} swdg_info_t;
// 报警双向链表，挂载异常看门狗对象
typedef struct mlist_node
{
    struct mlist_node *next;
    struct mlist_node *prev;
    swdg_info_t *swdg;

} alarm_dev_t;

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
                     rt_uint8_t init_flag);

swdg_info_t *getSwdgInfo(void);
alarm_dev_t *getAlarm_hp(void);

void InitSwdgHandle(rt_err_t (*handle)(rt_bool_t));
/**
 * @brief initialize a list
 * @param l list to be initialized
 */
void Mlist_Init(alarm_dev_t *l);

/***
 * @brief  insert a node after alarm_head
 * @param  insertSwdgDev   没有及时被喂狗的swdg指针
 ***/

void Mlist_Insert(swdg_info_t *insertSwdgDev);

/***
 * @brief  remove node from list and release memory block
 * @param  mID: 对应监视器的ID
 * @return None
 * @author Lvfp
 ***/
void Mlist_Remove(uint16_t mID);

uint8_t getSwdgNum(void);
#endif
