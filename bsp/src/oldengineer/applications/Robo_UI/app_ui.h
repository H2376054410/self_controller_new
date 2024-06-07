#ifndef _APP_UI_H_
#define _APP_UI_H_
#include "board.h"

#if defined ARM_MATH_CM4
#include <arm_math.h>
#else
#include <math.h>
#endif

// 裁判系统规定名字字节大小
#define NAME_MAXSIZE 3

// 绘制ui图形次数 为0则是一直发送
#define UI_RUN_FOREVER (-1)

// UI只运行一次的参数
#define UI_RUN_FOREVER_PARA UI_RUN_FOREVER, 1

// UI只运行一次的参数
#define UI_RUN_ONETIME_PARA 1, 1

// UI背景图像的参数
#define UI_BAKG_PARA 4,10

// UI背景图像的参数
#define UI_BAKG_PARA2 4,20

// UI背景图像的参数
#define UI_BAKG_PARA3 2, 8

// UI清除图像的参数
#define UI_CLEAR_PARA 1, 1

//需要单独发送例如为字符串--便于后期修改
#define UI_SEND_SEPA 1

// 不需要单独发送--便于后期修改
#define UI_SEND_NORM 0

typedef enum
{
    commonList = 0,
    uiCharList,
    UI_LIST_ENEMY,
    UI_LIST_URGENT,

    UI_LIST_NUM,
} UI_List_e;

// UI绘制线程
void CustomUI_Transmit_Thread(void *parameter);

// 获取绘制 UI 需要用到的外部数据
void CustomUI_Update_Thread(void *parameter);

// 线程初始化
rt_err_t UI_Init(void);

#endif
