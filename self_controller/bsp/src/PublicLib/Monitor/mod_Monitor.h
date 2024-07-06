#ifndef __MOD_MONITOR_H__
#define __MOD_MONITOR_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_Monitor.h"
#include "func_Alarm.h"

#ifdef BSP_USING_WDG
#include "drv_HardWdt.h"
#endif

void modMonitorInit(void);
/**
 * @brief    给看门狗喂食(移出ID对应的报警节点,复位剩余时间)
 * @param    mID 看门狗id
 */
void Swdg_Feed(uint16_t mID);

/**
 * @brief 查询看门狗对象是否异常
 * @param {uint16_t} mID  mID 看门狗id
 * @return {*}   RT_FALSE：正常，RT_TRUE：异常 -1未初始化
 */
int16_t Swdg_If_Error(uint16_t mID);

/**
 * @brief 启动一个监视器
 * @author fwlh
 * @param  mID              待启动监视器的 ID
 */
void Swdg_Start(uint16_t mID);

/*------------------------------------通过名字动态申请与查找----------------------------------------------------------*/
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
                            rt_bool_t if_alarm);

rt_err_t Swdg_Stop(uint16_t mID);

/**
 * @brief 通过名字喂狗
 * @param {char*} name
 * @return {*}当有这个名字则喂狗成功返回 RT_TRUE
 */
rt_bool_t Swdg_Feed_UseName(char *name);

/**
 * @brief 通过名字返回ID
 * @param {char} *name
 * @return {*} 如果已经有该名字则返回id>0 否则返回-1
 */
int16_t Find_Swdg_ID(const char *name);

/**
 * @brief 得到当前监视对象是否异常
 * @return {返回-1表示输入错误 0正常,1表示监视对象异常}
 */
int16_t SwdgGetIsSErr_Name(char *name);
#endif
