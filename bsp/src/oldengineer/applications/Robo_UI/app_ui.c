#include "app_ui.h"
#include "app_monitor.h"
#include "drv_thread.h"
#include "ui_interface.h"
#include "fun_ui_list.h"
#include "func_UI_data.h"
#include "fun_drawCustomUI.h"
#include "func_Dataserver.h"
#include "drv_RemoteCtrl_data.h"
#include "drv_uiSend.h"
#include "app_monitor.h"
static struct rt_timer task_50ms; // 定时器

// 各个链表
static List_t ui_list[UI_LIST_NUM];
static rt_uint8_t ui_loop_flag; // 完成一次循环或成功发送一次后置1
rt_uint32_t ui_tick;
rt_uint32_t transID; // 监视器全局id,因为UI可能不是一直在绘制

// 遍历整个链表并执行相应回调函数
static uint16_t Travel_ui_list(List_t *List_head, uint8_t reset_flag)
{
    // 节点--由节点可以找到自定义结构体
    ListItem_t *List_Item;
    ui_TCB *UI_TCB;
    static uint32_t period_cnt; // 记录运行次数 即使1MS一次 一个月以内不会溢出

    if (reset_flag)
    { // 若复位则运行次数置0
        period_cnt = 0;
    }

    // 仅用于标志位
    if (ui_loop_flag)
        ui_loop_flag = 0;

    if (List_head && !listLIST_IS_EMPTY(List_head))
    {
        // 得到当前链表的第一个挂载的节点
        List_Item = listGET_HEAD_ENTRY(List_head);
        // 当不为最后一个节点时
        while (listGET_END_MARKER(List_head) != List_Item)
        {
            /* 获取节点拥有者 */
            if (List_Item->pvOwner)
            {
                UI_TCB = listGET_LIST_ITEM_OWNER(List_Item);
                // 当达到运行周期时
                if (period_cnt % UI_TCB->member.func.period == 0)
                {
                    // 执行函数,同时保证线程间同步
                    UI_TCB->member.func.func(0);
                    if (UI_TCB->sendSeparatley) // 当为字符串等需要单独发送时重复一次
                        UI_TCB->member.func.func(0);

                    if (UI_TCB->member.func.run_times != UI_RUN_FOREVER) // 只执行有限次的函数
                    {
                        UI_TCB->member.func.run_times--;
                        if (UI_TCB->member.func.run_times == 0)
                        {
                            // 删除UI_TCB并将指定指针指向前一节点
                            UI_List_Delete(UI_TCB, List_Item);
                        }
                    }
                }
                // 得到下一个节点
                List_Item = listGET_NEXT(List_Item);
                ui_tick = rt_tick_get();
            }
        }
        SWDG_FEED(transID); // 因为UI可能不是一直在绘制，所以在这里喂狗
    }
    return period_cnt++;
}

//	UI链表所执行的回调函数初始化
static void UI_Func_List_Init(void)
{
    UI_FuncListAdd(&ui_list[commonList], Clear_Screen, UI_CLEAR_PARA, UI_Name_clear, UI_SEND_SEPA);
    //  UI_FuncListAdd(&ui_list[commonList], Fill_Full_Screen, UI_RUN_FOREVER, 1, UI_Name_Fill_Full_Screen, UI_SEND_NORM); // 充满屏幕（测试用

    // 绘制机械臂yaw轴的状态
    UI_FuncListAdd(&ui_list[commonList], Draw_armYaw_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_armYaw_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 绘制机械臂pitch轴的状态
    UI_FuncListAdd(&ui_list[commonList], Draw_armPitch_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_armPitch_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // // // 绘制底盘控制模式
    // UI_FuncListAdd(&ui_list[commonList], Draw_ctrlMenu_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    // UI_FuncListAdd(&ui_list[commonList], Draw_ctrlMenu_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 绘制气泵状态
    UI_FuncListAdd(&ui_list[commonList], Draw_pump_bkgd, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_pump_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 绘制前臂的roll轴状态
    UI_FuncListAdd(&ui_list[commonList], Draw_forearmRoll_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_forearmRoll_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 绘制图传状态
    UI_FuncListAdd(&ui_list[commonList], Draw_imageTrans_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_imageTrans_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 绘制瞄准框
    UI_FuncListAdd(&ui_list[commonList], Draw_aimingFrame_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_aimingFrame_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 绘制车道线
    UI_FuncListAdd(&ui_list[commonList], Draw_laneLines_bkgd, UI_BAKG_PARA, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], Draw_laneLines_dyn, UI_RUN_FOREVER_PARA, UI_Name_dynRefresh, UI_SEND_NORM);

    // 对于动态图层  改变参数实现修改
    UI_FuncListAdd(&ui_list[commonList], UI_dynRefresh, 1, 1, UI_Name_dynRefresh, UI_SEND_NORM);
    UI_FuncListAdd(&ui_list[commonList], UI_drawEnd, UI_RUN_FOREVER, 1, UI_Name_Draw_END, UI_SEND_NORM);
}

// 用于确保发送频率较为固定
static void task_50ms_IRQHandler(void *parameter)
{
    UI_sendEvent(ui_50ms_eve);
}

// 向裁判系统发送UI数据
void CustomUI_Transmit_Thread(void *parameter)
{
    ADDTOMONITOR_ID("CustomUI_Transmit", 5000, MONITOR_DEHANDLER, ALARM_RED, 0, transID);
    SWDG_START(transID); // 喂狗在链表遍历当中

    while (1)
    {

        // 等待可以发送
        UI_waitForSend();

        // 获取当前机器人ID和客户端ID REF_ROBO_ID REF_CLIENT_ID
        Ref_Robot_ID();
        // 从数据服务器获取信息
        Get_UIData_fromServer();

        sendUIData(REF_ROBO_ID, REF_CLIENT_ID); // 向裁判系统发送数据
        ui_loop_flag = 1;
    }
}

static void uiDataInit(void)
{
    // 链表根节点初始化
    for (int i = 0; i < UI_LIST_NUM; i++)
    {
        // 首先将链表节点全部释放
        if (UI_List_IsEmpty(&ui_list[i]) == RT_FALSE)
        {
            // 如果销毁失败则卡住
            if (!UI_list_Destroy(&ui_list[i]))
                while (1)
                    ;
        }
        UI_List_Init(&ui_list[i]);
    }

    // 添加链表节点
    UI_Func_List_Init();

    // 初始化自定义ui绘制包结构体
    custom_ui_init();
    UI_Clear_Init_Flag();
}

// 获取绘制 UI 需要用到的外部数据 时间片轮询
void CustomUI_Update_Thread(void *parameter)
{
    // 获得云台ui复位标志位
    static rt_uint8_t ui_reset, ui_reset_last;
    ui_reset_last = ui_reset;

    CREAT_ID(id);
    ADDTOMONITOR_ID("CustomUI_Update", 5000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);

    while (1)
    {
        SWDG_FEED(id);

        // 获取绘制 UI 需要用到的数据--底层使用
        getUIexData();

        ui_reset = (rt_uint8_t)Get_UI_Reset_Flag();
        // 复位标志改变即复位
        if ((ui_reset_last != ui_reset))
        {
            ui_reset_last = ui_reset;
            uiDataInit(); // UI重置

        }
        Travel_ui_list(&ui_list[commonList], ui_reset_last != ui_reset);
    }
}

// 线程初始化
rt_err_t UI_Init(void)
{
    rt_err_t res = RT_EOK;
    rt_thread_t thread;

    res = uiUsartInit();
    if (res != RT_EOK)
        return RT_ERROR;

    uiDataInit();
    UI_synInit(); // UI两个线程间同步初始化

    // UI发送线程 50ms检查一次是否可以发送数据
    thread = rt_thread_create("UI Draw",
                              CustomUI_Transmit_Thread,
                              RT_NULL,
                              THREAD_STACK_UI,
                              THREAD_PRIO_UI,
                              THREAD_TICK_UI);
    if (thread != RT_NULL)
    {
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    // 得到待发送UI的数据线程 时间片轮询
    thread = rt_thread_create("UI Draw",
                              CustomUI_Update_Thread,
                              RT_NULL,
                              THREAD_STACK_UI,
                              THREAD_PRIO_UI,
                              THREAD_TICK_UI);
    if (thread != RT_NULL)
    {
        // 启动线程
        res = rt_thread_startup(thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    // 创建线程定时器
    rt_timer_init(&task_50ms,
                  "50ms_task",
                  task_50ms_IRQHandler,
                  RT_NULL,
                  70,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    // 启动定时器
    res = rt_timer_start(&task_50ms);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}
