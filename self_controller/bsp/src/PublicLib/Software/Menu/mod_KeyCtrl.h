#ifndef __MOD_KEYCTRL_H__
#define __MOD_KEYCTRL_H__

#include "func_Key_Record.h"

#define KEYCTRL_SET_ENTRYFUN_NUM (3) // 定义二维按键程序中，支持的最大入口触发函数个数（至少1个）
#define KEYCTRL_SET_MAINFUN_NUM (3)  // 定义二维按键程序中，支持的最大主功能触发函数个数（至少1个）
#define KEYCTRL_SET_QUITFUN_NUM (5)  // 定义二维按键程序中，支持的最大出口触发函数个数（至少1个）

#define KEYCTRL_SET_SUBMENU_NUM (6) // 定义每个菜单支持的最大子菜单的个数

typedef void (*TrigFun_t)(int Trig_Key); // 触发函数数据类型

typedef struct Trig_Func_Config
{
    int Trig_Source;     // 设置触发条件 （选择指定按键）
    TrigFun_t Trig_Func; // 函数指针
} Trig_Func_Config_t;

struct Key_Menu_Str;
typedef struct SubMenu_Config
{
    int Trig_Source;                 // 设置触发条件 （选择指定按键）
    struct Key_Menu_Str *SubMenuStr; // 定义子菜单或功能结构体指针
} SubMenu_Config_t;

typedef struct Key_Menu_Str
{
    Trig_Func_Config_t EntryFun_Config[KEYCTRL_SET_ENTRYFUN_NUM]; // 定义入口钩子函数配置结构体
    Trig_Func_Config_t MainFun_Config[KEYCTRL_SET_MAINFUN_NUM];   // 定义菜单主功能函数配置结构体
    Trig_Func_Config_t QuitFun_Config[KEYCTRL_SET_QUITFUN_NUM];   // 定义出口钩子函数配置结构体
    SubMenu_Config_t SubMenuConfig[KEYCTRL_SET_SUBMENU_NUM];
} Key_Menu_Str_t; // 定义菜单节点

/**
 * @brief 初始化单个菜单节点
 * @param Key_Menu_Str_t* MenuStr：需要初始化的菜单节点
 * @return void
 * @author ych
 */
extern void KeyCtrl_MenuStr_Init(Key_Menu_Str_t *MenuStr);

/**
 * @brief 在指定菜单节点下添加子菜单节点
 * @param Key_Menu_Str_t* MenuStr：      指定菜单节点
 * @param Key_Menu_Str_t* SubMenuStr：   待添加的子菜单节点
 * @param int             Trig_Key：     触发键
 * @return [int] 0表示子节点已满，其它数字表示本次添加的子菜单是指定菜单的几号子菜单
 * @author ych
 */
extern int MenuSet_Add_SubMenu(Key_Menu_Str_t *MenuStr, Key_Menu_Str_t *SubStr, int Trig_Key);

/**
 * @brief 在指定菜单节点下添加入口触发函数
 * @param Key_Menu_Str_t* MenuStr：      指定菜单节点
 * @param TrigFun_t Trig_Fun：   待添加的入口触发函数
 * @param Key_Select_E Trig_Source： 为入口触发函数指定触发按键 若不需要指定则使用 KeyEVT_ALL
 * @return [int] 0表示入口触发函数已满，其它数字表示本次添加的入口触发函数是指定菜单的几号入口触发函数
 * @author ych
 */
extern int MenuSet_Add_EntryFun(Key_Menu_Str_t *MenuStr, int Trig_Source, TrigFun_t Trig_Fun);

/**
 * @brief 在指定菜单节点下添加功能触发函数
 * @param Key_Menu_Str_t* MenuStr：  指定菜单节点
 * @param TrigFun_t Trig_Fun：       待添加的功能触发函数
 * @param Key_Select_E Trig_Source： 为功能触发函数指定触发按键 若不需要指定则使用 KeyEVT_ALL
 * @return [int] 0表示功能触发函数已满，其它数字表示本次添加的功能触发函数是指定菜单的几号功能触发函数
 * @author ych
 */
extern int MenuSet_Add_MainFun(Key_Menu_Str_t *MenuStr, int Trig_Source, TrigFun_t Trig_Fun);

/**
 * @brief 在指定菜单节点下添加出口触发函数
 * @param Key_Menu_Str_t* MenuStr：  指定菜单节点
 * @param TrigFun_t Trig_Fun：       待添加的出口触发函数
 * @param Key_Select_E Trig_Source： 为出口触发函数指定触发按键 若不需要指定则使用 KeyEVT_ALL
 * @return [int] 0表示出口触发函数已满，其它数字表示本次添加的出口触发函数是指定菜单的几号出口触发函数
 * @author ych
 */
extern int MenuSet_Add_QuitFun(Key_Menu_Str_t *MenuStr, int Trig_Source, TrigFun_t Trig_Fun);

/**
 * @brief 指定主菜单，用于初始化菜单过程
 * @param Key_Menu_Str_t* MenuStr：  指定菜单节点
 * @author ych
 */
extern void MenuCmd_SetMainMenu(Key_Menu_Str_t *MenuStr);

/**
 * @brief 启动菜单程序
 * @author ych
 */
extern void Key_Menu_Start(void);
#endif
