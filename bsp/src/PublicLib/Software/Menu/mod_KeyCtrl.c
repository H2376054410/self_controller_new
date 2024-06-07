#include "mod_KeyCtrl.h"
#include <rtthread.h>
#include "drv_thread.h"

Key_Menu_Str_t *MenuLast = NULL;
Key_Menu_Str_t *MenuNow = NULL; // 记录当前所在菜单层

// 初始化触发函数配置块
static void TrigFun_ConfigStr_Init(Trig_Func_Config_t *ConfigStr)
{
    ConfigStr->Trig_Source = 0; // 没有触发源
    ConfigStr->Trig_Func = NULL;
}

// 配置触发函数配置块
static void TrigFun_ConfigStr_Set(Trig_Func_Config_t *ConfigStr, int Trig_Source, TrigFun_t Trig_Fun)
{
    ConfigStr->Trig_Source = Trig_Source;
    ConfigStr->Trig_Func = Trig_Fun;
}

// 初始化子菜单配置块
static void Submenu_ConfigStr_Init(SubMenu_Config_t *ConfigStr)
{
    ConfigStr->Trig_Source = 0; // 没有触发源
    ConfigStr->SubMenuStr = NULL;
}

/**
 * @brief 初始化单个菜单节点
 * @param Key_Menu_Str_t* MenuStr：需要初始化的菜单节点
 * @return void
 * @author ych
 */
void KeyCtrl_MenuStr_Init(Key_Menu_Str_t *MenuStr)
{
    int fori;

    // 初始化时没有触发函数
    for (fori = 0; fori < KEYCTRL_SET_ENTRYFUN_NUM; fori++)
    {
        TrigFun_ConfigStr_Init(&(MenuStr->EntryFun_Config[fori])); // 初始化入口函数指针配置块
    }
    for (fori = 0; fori < KEYCTRL_SET_MAINFUN_NUM; fori++)
    {
        TrigFun_ConfigStr_Init(&(MenuStr->MainFun_Config[fori])); // 初始化入口函数指针配置块
    }
    for (fori = 0; fori < KEYCTRL_SET_QUITFUN_NUM; fori++)
    {
        TrigFun_ConfigStr_Init(&(MenuStr->QuitFun_Config[fori])); // 初始化入口函数指针配置块
    }

    // 初始化时没有子菜单
    for (fori = 0; fori < KEYCTRL_SET_SUBMENU_NUM; fori++)
    {
        Submenu_ConfigStr_Init(&MenuStr->SubMenuConfig[fori]); // 默认没有子菜单
    }
}

/**
 * @brief 在指定菜单节点下添加子菜单节点
 * @param Key_Menu_Str_t* MenuStr：      指定菜单节点
 * @param Key_Menu_Str_t* SubMenuStr：   待添加的子菜单节点
 * @param int             Trig_Key：     触发键
 * @return [int] 0表示子节点已满，其它数字表示本次添加的子菜单是指定菜单的几号子菜单
 * @author ych
 */
int MenuSet_Add_SubMenu(Key_Menu_Str_t *MenuStr, Key_Menu_Str_t *SubStr, int Trig_Key)
{
    int fori = 0;
    if (SubStr != NULL)
    {
        while (fori < KEYCTRL_SET_SUBMENU_NUM)
        {
            if (MenuStr->SubMenuConfig[fori].SubMenuStr == NULL)
            {
                MenuStr->SubMenuConfig[fori].SubMenuStr = SubStr;    // 设定子菜单
                MenuStr->SubMenuConfig[fori].Trig_Source = Trig_Key; // 设定按键
                return fori + 1;
            }
            fori++; // 查找下一个位置
        }
    }
    return 0;
}

/**
 * @brief 在指定菜单节点下添加入口触发函数
 * @param Key_Menu_Str_t* MenuStr：      指定菜单节点
 * @param TrigFun_t Trig_Fun：   待添加的入口触发函数
 * @param Key_Select_E Trig_Source： 为入口触发函数指定触发按键 若不需要指定则使用 KeyEVT_ALL
 * @return [int] 0表示入口触发函数已满，其它数字表示本次添加的入口触发函数是指定菜单的几号入口触发函数
 * @author ych
 */
int MenuSet_Add_EntryFun(Key_Menu_Str_t *MenuStr, int Trig_Source, TrigFun_t Trig_Fun)
{
    int fori = 0;
    while (fori < KEYCTRL_SET_ENTRYFUN_NUM)
    {
        if (MenuStr->EntryFun_Config[fori].Trig_Func == NULL)
        {
            TrigFun_ConfigStr_Set(&MenuStr->EntryFun_Config[fori], Trig_Source, Trig_Fun);
            return fori + 1;
        }
        fori++; // 查找下一个位置
    }
    return 0;
}

/**
 * @brief 在指定菜单节点下添加功能触发函数
 * @param Key_Menu_Str_t* MenuStr：  指定菜单节点
 * @param TrigFun_t Trig_Fun：       待添加的功能触发函数
 * @param Key_Select_E Trig_Source： 为功能触发函数指定触发按键 若不需要指定则使用 KeyEVT_ALL
 * @return [int] 0表示功能触发函数已满，其它数字表示本次添加的功能触发函数是指定菜单的几号功能触发函数
 * @author ych
 */
int MenuSet_Add_MainFun(Key_Menu_Str_t *MenuStr, int Trig_Source, TrigFun_t Trig_Fun)
{
    int fori = 0;
    while (fori < KEYCTRL_SET_ENTRYFUN_NUM)
    {
        if (MenuStr->MainFun_Config[fori].Trig_Func == NULL)
        {
            TrigFun_ConfigStr_Set(&MenuStr->MainFun_Config[fori], Trig_Source, Trig_Fun);
            return fori + 1;
        }
        fori++; // 查找下一个位置
    }
    return 0;
}

/**
 * @brief 在指定菜单节点下添加出口触发函数
 * @param Key_Menu_Str_t* MenuStr：  指定菜单节点
 * @param TrigFun_t Trig_Fun：       待添加的出口触发函数
 * @param Key_Select_E Trig_Source： 为出口触发函数指定触发按键 若不需要指定则使用 KeyEVT_ALL
 * @return [int] 0表示出口触发函数已满，其它数字表示本次添加的出口触发函数是指定菜单的几号出口触发函数
 * @author ych
 */
int MenuSet_Add_QuitFun(Key_Menu_Str_t *MenuStr, int Trig_Source, TrigFun_t Trig_Fun)
{
    int fori = 0;
    while (fori < KEYCTRL_SET_ENTRYFUN_NUM)
    {
        if (MenuStr->QuitFun_Config[fori].Trig_Func == NULL)
        {
            TrigFun_ConfigStr_Set(&MenuStr->QuitFun_Config[fori], Trig_Source, Trig_Fun);
            return fori + 1;
        }
        fori++; // 查找下一个位置
    }
    return 0;
}

/**
 * @brief 指定主菜单，用于初始化菜单过程
 * @param Key_Menu_Str_t* MenuStr：  指定菜单节点
 * @author ych
 */
void MenuCmd_SetMainMenu(Key_Menu_Str_t *MenuStr)
{
    MenuNow = MenuStr;
}

static rt_thread_t Key_MenuCTRL_ThreadTid = RT_NULL; // 菜单线程句柄

// 获取指定菜单中所有的子菜单触发按键
static int GetEVT_MenuTrig(Key_Menu_Str_t *MenuStr)
{
    int Temp = 0;
    int fori;
    for (fori = 0; fori < KEYCTRL_SET_SUBMENU_NUM; fori++)
    {
        Temp |= MenuStr->SubMenuConfig[fori].Trig_Source;
    }
    return Temp;
}

// 检查并执行入口触发函数
static void EntryFun_Check(Key_Menu_Str_t *MenuStr, int EVT_Check)
{
    int fori;
    int EVT_Trig;
    for (fori = 0; fori < KEYCTRL_SET_ENTRYFUN_NUM; fori++)
    {
        EVT_Trig = MenuStr->EntryFun_Config[fori].Trig_Source & EVT_Check;
        if (EVT_Trig)
        { // 符合触发条件 调用回调函数
            if (MenuStr->EntryFun_Config[fori].Trig_Func != NULL)
            {                                                       // 回调函数已经初始化
                MenuStr->EntryFun_Config[fori].Trig_Func(EVT_Trig); // 正常调用函数
            }
        }
    }
}
// 检查并执行功能触发函数
static void MainFun_Check(Key_Menu_Str_t *MenuStr, int EVT_Check)
{
    int fori;
    int EVT_Trig;
    for (fori = 0; fori < KEYCTRL_SET_MAINFUN_NUM; fori++)
    {
        EVT_Trig = MenuStr->MainFun_Config[fori].Trig_Source & EVT_Check;
        if (EVT_Trig)
        { // 符合触发条件 调用回调函数
            if (MenuStr->MainFun_Config[fori].Trig_Func != NULL)
            {                                                      // 回调函数已经初始化
                MenuStr->MainFun_Config[fori].Trig_Func(EVT_Trig); // 正常调用函数
            }
        }
    }
}
// 检查并执行出口触发函数
static void QuitFun_Check(Key_Menu_Str_t *MenuStr, int EVT_Check)
{
    int fori;
    int EVT_Trig;
    for (fori = 0; fori < KEYCTRL_SET_QUITFUN_NUM; fori++)
    {
        EVT_Trig = MenuStr->QuitFun_Config[fori].Trig_Source & EVT_Check;
        if (EVT_Trig)
        { // 符合触发条件 调用回调函数
            if (MenuStr->QuitFun_Config[fori].Trig_Func != NULL)
            {                                                      // 回调函数已经初始化
                MenuStr->QuitFun_Config[fori].Trig_Func(EVT_Trig); // 正常调用函数
            }
        }
    }
}

// 根据按键情况切换到下一菜单 返回值：匹配到的触发事件（按键EVT）
static int Key_MenuSwitch(Key_Menu_Str_t *MenuStr_Now, int EVT)
{
    int fori;
    int EVT_Cal;
    for (fori = 0; fori < KEYCTRL_SET_SUBMENU_NUM; fori++)
    {
        EVT_Cal = MenuStr_Now->SubMenuConfig[fori].Trig_Source & EVT;
        if (EVT_Cal)
        {
            MenuLast = MenuNow;
            MenuNow = MenuStr_Now->SubMenuConfig[fori].SubMenuStr;
            return EVT_Cal; // 返回对应的事件标号
        }
    }
    return 0;
}

// 菜单处理线程
void KeyMenu_Thread(void *Para)
{
    rt_uint32_t EVT_Cal, EVT_Rec;
    rt_uint32_t EVT_TryTake;
    int SwitchEVT_Temp; // 切换菜单位置时，记录匹配到的触发事件

    // 主循环
    while (1)
    {
        if (MenuNow == NULL)
        {
            rt_thread_delay(50); // 未初始化主菜单，无法正常工作
            continue;
        }

        EVT_Cal = GetEVT_MenuTrig(MenuNow); // 计算当前菜单中需要检测的按键，用于挂起事件集
        rt_event_recv(&KeyRec_Press_EVT, EVT_Cal,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, &EVT_Rec); // 等待按键下降沿信号
        rt_event_recv(&KeyRec_Press_EVT, (rt_uint32_t)0xFFFFFFFF,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_NO, &EVT_TryTake); // 清空其它按键数据

        // 有按键按下，查找子菜单
        SwitchEVT_Temp = Key_MenuSwitch(MenuNow, EVT_Rec);

        // 如果匹配到了按键并完成菜单切换，则调用退出函数
        if (SwitchEVT_Temp && MenuNow != NULL && MenuLast != NULL)
        {
            QuitFun_Check(MenuLast, SwitchEVT_Temp);
            EntryFun_Check(MenuNow, SwitchEVT_Temp);
            MainFun_Check(MenuNow, SwitchEVT_Temp);
        }
    }
}

/**
 * @brief 启动菜单程序
 * @author ych
 */
void Key_Menu_Start(void)
{
    // 初始化菜单按键线程
    /* 创建线程，名称是 KeyMenu，入口是 KeyMenu_Thread */
    Key_MenuCTRL_ThreadTid = rt_thread_create("KeyMenu",
                                              KeyMenu_Thread, RT_NULL,
                                              1024,
                                              THREAD_PRIO_KEYMENU, 2);
    if (Key_MenuCTRL_ThreadTid != RT_NULL)
        rt_thread_startup(Key_MenuCTRL_ThreadTid); // 启动菜单线程
}
