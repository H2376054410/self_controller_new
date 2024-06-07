#ifndef __FUNC_KEY_RECORD__
#define __FUNC_KEY_RECORD__

#include <rtdef.h>

#include "drv_Key_Set.h"

#define KEY_CONFIRM_COUNT_DEFAULT_SET (2)// 默认去抖动次数

typedef struct
{
    rt_uint32_t ID;// 按键ID，32位中每一个位对应一个按键
    rt_int8_t Press;// 记录当前按键状态
    rt_int16_t State_Confirm_Set;// 记录去抖动水平
    rt_int16_t State_Rec;// 用于去抖动的变量
    void (*Press_Func)(void);// 可能存在的按键回调函数
    void (*Rise_Func)(void);// 可能存在的弹起回调函数
} Key_Record_Type;

extern void RC_Key_Process(void); // 需要在每帧遥控器数据接收完成后调用，此函数会对遥控器发来的数据的所有按键信息进行统一处理

/**
* @brief 对指定的按键添加设置回调函数
* @param [Key_Select_E] Key_EVT：需要查询的按键
* @param [void(*p)(void)] PressFun：需要添加至该按键的按下回调函数的函数指针
* @param [void(*p)(void)] RiseFun：需要添加至该按键的弹起回调函数的函数指针
* @return [void]
* @author ych
*/
extern void Key_SetCallBack(Key_Select_E Key_EVT, void (*PressFun)(void), void (*RiseFun)(void));

/**
* @brief 修改指定按键的去抖动次数
* @param [Key_Select_E] Key_EVT：需要查询的按键
* @param [rt_int16_t] Confirm_Count_Set：需要设置的去抖动次数参数
* @return [void]
* @author ych
*/
extern void Key_SetPressConfirm(Key_Select_E Key_EVT, rt_int16_t Confirm_Count_Set);

/**
* @brief 读取按键状态
* @param [Key_Select_E] Key_EVT：需要查询的按键
* @return [rt_int8_t] 该按键现在的状态，数据经过去抖动处理
* @author ych
*/
extern rt_int8_t Key_GetState(Key_Select_E Key_EVT);

extern struct rt_event KeyRec_Press_EVT; // 按键下降沿事件集

extern int KeyMenu_EVT_Init(void);  // 初始化二维按键使用的事件集
extern rt_err_t KeyRec_Press_EVT_INI_STATE; // 二维按键事件集初始化记录变量
#endif
