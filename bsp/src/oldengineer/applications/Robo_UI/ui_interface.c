#include "ui_interface.h"
#include "func_UI_data.h"

/* 事件控制块 */
static struct rt_event ui_event;
UI_Used_Data_t UI_needData; // 底层UI需要使用到的外部数据

// 返回Ui重置标志位 1复位
rt_uint8_t Get_UI_Reset_Flag(void)
{
    return UI_needData.state.UI_Reset_Flag;
}

int UI_Debug_flag = 1;

// 绘制UI所需要的外部数据
UI_Used_Data_t Get_UI_Data(void)
{
    if (UI_Debug_flag)
    {
        UI_needData.Ore_State = GetState_OreState();                    // 获取矿石模式状态
        UI_needData.state.UI_Reset_Flag = GetState_UIReset();           //
        UI_needData.state.pumpStatus = GetState_AirPumpIfOpen();        // 气泵状态
        UI_needData.state.ForearmRoll = GetState_ArmIfLimit(ForearmRoll);
        UI_needData.state.largeArmYaw = GetState_ArmIfLimit(BoomYaw);
        UI_needData.state.smallArmYaw = GetState_ArmIfLimit(ForearmYaw);
        UI_needData.state.smallArmPitch1 = GetState_ArmIfLimit(BoomPitch1);
        UI_needData.state.smallArmPitch2 = GetState_ArmIfLimit(BoomPitch2);
        UI_needData.state.smallArmPitch3 = GetState_ArmIfLimit(ForearmPitch);

        UI_needData.ctrlMenu = GetState_RoboRCCtrl();
        UI_needData.LargeArmYaw = GetState_Armrad(BoomYaw);
        UI_needData.smallArmYaw = GetState_Armrad(ForearmYaw);
        UI_needData.smallArmPitch1 = GetState_Armrad(BoomPitch1);
        UI_needData.smallArmPitch2 = GetState_Armrad(BoomPitch2);
        UI_needData.smallArmPitch3 = GetState_Armrad(ForearmPitch);
    }

    return UI_needData;
}

rt_err_t UI_synInit(void)
{
    rt_err_t result = RT_ERROR;
    /* 初始化事件对象 */
    result = rt_event_init(&ui_event, "ui_event", RT_IPC_FLAG_PRIO);
    if (result != RT_ERROR)
        UI_sendEvent(ui_sendReady_eve); // 首先发送该事件表示上次发送完成
    return result;
}

/*
 每50ms检查是否可以发送
 检查是否需要发送
 检查上一次是否发送完成即是否可以发送
*/
rt_err_t UI_waitForSend(void)
{
    rt_err_t result = RT_ERROR;
    result = rt_event_recv(&ui_event, (ui_50ms_eve | ui_dataReady_eve | ui_sendReady_eve),
                           RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                           RT_WAITING_FOREVER, RT_NULL);
    return result;
}

rt_err_t UI_sendEvent(UI_threadsyn_e event)
{
    rt_err_t result = RT_ERROR;
    result = rt_event_send(&ui_event, event);
    return result;
}

rt_err_t UI_waitEvent(UI_threadsyn_e event)
{
    rt_err_t result = RT_ERROR;
    result = rt_event_recv(&ui_event, event, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                           RT_WAITING_FOREVER, RT_NULL);
    return result;
}
