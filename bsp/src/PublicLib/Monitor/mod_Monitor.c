/************************************** Copyright ******************************
 *                 (C) Copyright 2020,China, HITwh.
 *                            All Rights Reserved
 *
 *                     HITwh Excellent Robot Organization
 *                     https://github.com/HERO-ECG
 *                     https://gitee.com/HIT-718LC
 *
 * FileName   : app_monitor.c
 * Version    : v3.2
 * Author     : mqy,LvFp
 * Date       : 2020-02-14
 * Instructions：1.在drv_Monitor.h文件内枚举uint16_t里添加新的ID值。
 *               2.在All_Swdg_Create(void)函数里，用Swdg_Create添加新的软件看门狗对象
 *				  3.在func_MonCallback.c内添加新的异常处理函数。
 *	              4.在需要监视的地方使用Swdg_Feed来喂狗，注意喂狗地方要有周期性，且当监视的设备离线时这个地方不能被执行到
 *				  5.拨动核心板左拨码开关，可关闭报警中的蜂鸣器
 *				  6.推荐喂狗写法
 *					#ifdef  CORE_USING_MONITOR
 *						Swdg_Feed(F_Left_ID);
 *					#endif
 *				  7.硬件看门狗，当使能宏BSP_USING_WDG时，如果监视器线程被阻塞超过1s，
 *					则程序会一直复位。通过在main函数开头写报警函数提示程序崩了。
 *					而main.c里的预定义里的内容是监视器除了Swdg_Feed之外唯一暴露在监视器文件外的代码。
 *					推荐写法：左写在main.c包含的头文件里，右包含在main函数开头
 *					#ifdef BSP_USING_WDG			| #ifdef BSP_USING_WDG
 *						#include "func_Monitor.h"	|   PROGRAM_RESET    //程序一直复位提醒程序
 *					#endif							| #endif
 ********************************************************************************/
#include "mod_Monitor.h"
#include "string.h"

extern uint16_t swdg_Index; // 当前已用ID数 按顺序自增
static swdg_info_t *modSwdg_info; // 名字和ID
static uint8_t isInit_mod = 0;

/**
 * @brief    给看门狗喂食(移出ID对应的报警节点,复位剩余时间)
 * @param    mID 看门狗id
 */
void Swdg_Feed(uint16_t mID)
{
    if (mID >= swdg_Index)
        return;

    // 需要判断该模块是否真的已经被初始化了
    if ((modSwdg_info[mID].if_start == 1))
    {
        // 直接将阈值增加相当于喂狗
        modSwdg_info[mID].swdg_dev.time_deadline = rt_tick_get() + modSwdg_info[mID].swdg_dev.time_threshold;

        // 当错误线程再次运行时则表示又恢复正常
        if (modSwdg_info[mID].isError == RT_TRUE) // 对异常看门狗节点进行恢复
        {
            modSwdg_info[mID].isError = RT_FALSE;
            if (modSwdg_info[mID].swdg_dev.handle != RT_NULL)
                (*modSwdg_info[mID].swdg_dev.handle)(modSwdg_info[mID].isError); // 恢复函数

#if (defined(BSP_USING_RGB_LIGHT) || defined(BSP_USING_BUZZER)) && (defined(MONITOR_NEED_ALARM))
            Mlist_Remove(mID); // 移出对应的报警链表
#endif
        }
    }
}

/**
 * @brief 启动一个监视器
 * @author fwlh
 * @param  mID              待启动监视器的 ID
 */
void Swdg_Start(uint16_t mID)
{
    if (mID >= swdg_Index)
        return;

    // 需要判断该模块是否真的已经被初始化了
    if ((modSwdg_info[mID].swdg_dev.flag_inited == SWDG_INITED_FLAG))
    {
        modSwdg_info[mID].if_start = RT_TRUE;
        // 更新标志位的时候一定要注意更新时刻
        modSwdg_info[mID].swdg_dev.time_deadline = rt_tick_get() + modSwdg_info[mID].swdg_dev.time_threshold;
        return;
    }
}

/**
 * @brief 查询看门狗对象是否异常
 * @param {uint16_t} mID  mID 看门狗id
 * @return {*}  RT_FALSE:正常，RT_TRUE:异常 -1未初始化
 */
int16_t Swdg_If_Error(uint16_t mID)
{
    if (mID >= swdg_Index)
        return -1;

    if ((modSwdg_info[mID].swdg_dev.flag_inited == SWDG_INITED_FLAG))
        return modSwdg_info[mID].isError;
    else
        return -1;
}

/*------------------------------------通过名字动态申请与查找----------------------------------------------------------*/

/**
 * @brief 通过名字查找看门狗信息
 * @param {swdg_info_t} *swdg_info_temp 返回找到的看门狗信息 未找到则为NULL
 * @param {char*} name 名字
 * @return {*} 当为RT_TRUE表示已经有该名字
 */
static rt_bool_t Find_Swdg_Info(swdg_info_t *swdg_info_temp, const char *name)
{
    swdg_info_temp = &modSwdg_info[0];
    rt_bool_t IsSameName = RT_FALSE;
    for (int i = 0; i < swdg_Index; i++)
    {
        if (strcmp(name, swdg_info_temp->swdg_name) == 0)
        {
            IsSameName = RT_TRUE;
            break;
        }
        else
        {
            swdg_info_temp++;
        }
    }
    if (IsSameName != RT_TRUE)
    {
        swdg_info_temp = NULL;
    }

    return IsSameName;
}
/**
 * @brief 通过名字返回ID
 * @param {char} *name
 * @return {*} 如果已经有该名字则返回id>0 否则返回-1
 */
int16_t Find_Swdg_ID(const char *name)
{
    swdg_info_t *swdg_info_temp = &modSwdg_info[0];
    rt_bool_t IsSameName = RT_FALSE;
    for (int i = 0; i < swdg_Index; i++)
    {
        if (strcmp(name, swdg_info_temp->swdg_name) == 0)
        {
            IsSameName = RT_TRUE;
        }
        else
        {
            swdg_info_temp++;
        }
    }
    if (IsSameName != RT_TRUE)
    {
        swdg_info_temp = NULL;
        return -1;
    }
    else
        return swdg_info_temp->swdg_Id;
}

/**
 * @brief: 拷贝字符串当超过最大长度时直接截断
 * @param {char} *dst
 * @param {char} *src
 * @return {返回拷贝好的字符串指针}
 */
static char *strnamecpy(char *dst, const char *src)
{
    uint8_t max_num = SWDG_NAMEMAXLEN;
    char *d = dst;
    const char *s = src;
    do
    {
        if ((*d++ = *s++) == 0)
        {
            while (--max_num != 0)
                *d++ = 0;
            break;
        }
    } while (--max_num != 0);

    return (dst);
}

/**
 * @brief 通过姓名添加看门狗对象不启动需要外部再启动
 * @param {char} *name 注意姓名不要超过SWDG_DEV_MAXNUM 字节
 * @param {rt_uint32_t} time_threshold 两次未喂狗相差最大时间单位 ms
 * @param {Alarm_color_e} color
 * @param {rt_bool_t} if_alarm
 * @return 当名字不重复时返回ID>0 否则返回-1
 */
int16_t Swdg_Add_to_Monitor(char *name, rt_uint32_t time_threshold,
                            rt_err_t (*handle)(rt_bool_t), Alarm_color_e color,
                            rt_bool_t if_alarm)
{
    swdg_info_t *swdg_info_temp = NULL;
    rt_bool_t IsSameName = RT_FALSE;

    IsSameName = Find_Swdg_Info(swdg_info_temp, name);

    // 当名字不相同
    if (IsSameName == RT_FALSE && isInit_mod)
    {
        if (swdg_Index < SWDG_DEV_MAXNUM_S)
        {
            strnamecpy(modSwdg_info[swdg_Index].swdg_name, name);

            // 创建加入监视链表当中
            Swdg_Create(color, if_alarm, time_threshold, handle, SWDG_INITED_FLAG);

            return modSwdg_info[swdg_Index - 1].swdg_Id;
        }
        return -1; // 程序未初始化等
    }
    else
        return -1; // 当名字相同时
}

/**
 * @brief 得到当前监视对象是否异常
 * @return {返回-1表示输入错误 0正常,1表示监视对象异常}
 */
int16_t SwdgGetIsSErr_Name(char *name)
{
    int16_t xID = RT_FALSE;
    xID = Find_Swdg_ID(name);

    if (xID >= swdg_Index || xID == -1)
        return -1;

    if ((modSwdg_info[xID].swdg_dev.flag_inited == SWDG_INITED_FLAG))
    {
        if (modSwdg_info[xID].isError==1)
            return 1;
        else
            return 0;
    }
    else
        return -1;
}

rt_err_t Swdg_Stop(uint16_t mID)
{
    if (mID >= swdg_Index)
        return RT_ERROR;

    if ((modSwdg_info[mID].swdg_dev.flag_inited == SWDG_INITED_FLAG))
    {
        modSwdg_info[mID].if_start = RT_FALSE;
        return RT_EOK;
    }
    else
        return RT_ERROR;
}

/**
 * @brief 通过名字喂狗
 * @param {char*} name
 * @return {*}当有这个名字则喂狗成功返回 RT_TRUE
 */
rt_bool_t Swdg_Feed_UseName(char *name)
{
    int16_t find_id = Find_Swdg_ID(name);

    // 当确实有这个名字时
    if (find_id != -1)
    {
        Swdg_Feed((uint16_t)find_id);
        return RT_TRUE;
    }
    else
        return RT_FALSE;
}

void modMonitorInit(void)
{
    modSwdg_info = getSwdgInfo();
    isInit_mod = 1;
}
