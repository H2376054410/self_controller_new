#include "func_Key_Record.h"
#ifdef BSP_USING_REMOTE_DT7
#include "drv_remote.h"
#elif defined BSP_USING_REMOTE_SBUS
#include "drv_remote_sbus.h"
#endif

static Key_Record_Type KeyRec[KEY_COUNT_ALL]; // 用于记录每一个按键的状态

struct rt_event KeyRec_Press_EVT;               // 按键下降沿事件集
rt_err_t KeyRec_Press_EVT_INI_STATE = RT_ERROR; // 定义初始化记录变量

// 从EVT标志位转换成按键序号
static int Key_EVTToNum(Key_Select_E EVT)
{ // 使用二分法进行查找
    int Num = 0;

    if (!(EVT & 0xFFFF))
    {
        EVT >>= 16;
        Num += 16;
    }
    if (!(EVT & 0xFF))
    {
        EVT >>= 8;
        Num += 8;
    }
    if (!(EVT & 0xF))
    {
        EVT >>= 4;
        Num += 4;
    }
    if (!(EVT & 0x3))
    {
        EVT >>= 2;
        Num += 2;
    }
    if (!(EVT & 0x1))
    {
        Num += 1;
    }
    return Num;
}

/**
 * @brief 从遥控器数据结构体中读取指定按键的状态
 * @param [RC_Ctrl_t*] RC_data：遥控器数据结构体
 * @param [Key_Num_E] Key_Num：需要查询的按键
 * @return [rt_int8_t] 按键状态信息
 * @author ych
 */
static rt_int8_t Key_GetRCState(RC_Ctrl_t *RC_data, Key_Num_E Key_Num)
{
    // 此时Count的值为按键的序号
    switch (Key_Num)
    {
    case KeyNum_MouseL:
        return (RC_data->Mouse_Data.press_l);
    case KeyNum_MouseR:
        return (RC_data->Mouse_Data.press_r);
    case KeyNum_W:
        return (RC_data->Key_Data.W);
    case KeyNum_S:
        return (RC_data->Key_Data.S);
    case KeyNum_A:
        return (RC_data->Key_Data.A);
    case KeyNum_D:
        return (RC_data->Key_Data.D);
    case KeyNum_SHIFT:
        return (RC_data->Key_Data.shift);
    case KeyNum_CTRL:
        return (RC_data->Key_Data.ctrl);
    case KeyNum_Q:
        return (RC_data->Key_Data.Q);
    case KeyNum_E:
        return (RC_data->Key_Data.E);
    case KeyNum_R:
        return (RC_data->Key_Data.R);
    case KeyNum_F:
        return (RC_data->Key_Data.F);
    case KeyNum_G:
        return (RC_data->Key_Data.G);
    case KeyNum_Z:
        return (RC_data->Key_Data.Z);
    case KeyNum_X:
        return (RC_data->Key_Data.X);
    case KeyNum_C:
        return (RC_data->Key_Data.C);
    case KeyNum_V:
        return (RC_data->Key_Data.V);
    case KeyNum_B:
        return (RC_data->Key_Data.B);
    default:
        return 0;
    }
}

/**
 * @brief 对指定的键盘按键的状态进行刷新，同时调用相关的回调函数
 * @param [Key_Num_E] Key_Num：需要查询的按键
 * @return 无
 * @author ych
 */
static void Key_Check(Key_Num_E Key_Num)
{
    Key_Record_Type *KeyRec_p;
    KeyRec_p = &KeyRec[Key_Num];
    if (Key_GetRCState(&RC_data, Key_Num))
    {
        // 此按键的遥控数据标记为按下状态
        KeyRec_p->State_Rec++;
        if (KeyRec_p->State_Rec >= KeyRec_p->State_Confirm_Set)
        {                                                      // 此时已经正常按下
            KeyRec_p->State_Rec = KeyRec_p->State_Confirm_Set; // 对状态标志位进行限幅
            if (KeyRec_p->Press != 1)
            { // 检查是否为按下的边沿
                KeyRec_p->Press = 1;
                if (KeyRec_p->Press_Func != NULL)
                { // 在按下边沿时刻，如果用户设置了按键回调函数指针，则执行对应函数
                    KeyRec_p->Press_Func();
                }
                if (KeyRec_Press_EVT_INI_STATE == RT_EOK)
                    rt_event_send(&KeyRec_Press_EVT, 1 << Key_Num); // 发送按下事件
            }
        }
    }
    else
    {
        // 此按键的遥控数据标记为按下状态
        KeyRec_p->State_Rec--;
        if (KeyRec_p->State_Rec < 0)
        {
            KeyRec_p->State_Rec = 0;
            if (KeyRec_p->Press != 0)
            {
                KeyRec_p->Press = 0;
                if (KeyRec_p->Rise_Func != NULL)
                { // 在按下边沿时刻，如果用户设置了按键回调函数指针，则执行对应函数
                    KeyRec_p->Rise_Func();
                }
            }
        }
    }
}

/**
 * @brief 按键处理程序, 需要在遥控接收到一帧数据后调用。
 * @param 无
 * @return 无
 * @author ych
 */
void RC_Key_Process()
{
    static int16_t MouseZ_Temp = 0;
    rt_int16_t Num;

    // 接收鼠标滚轮数据
    MouseZ_Temp += RC_data.Mouse_Data.z_speed; // 在这里接收鼠标数据，在MouseZ_N中不进行接收

    // 处理一轮遥控按键信息
    for (Num = 0; Num < KeyNum_End; Num++)
    {
        switch (Num)
        {
        // 单独处理鼠标滚轮的数据
        case KeyNum_MouseZ_P:
            if (MouseZ_Temp > 0)
            {
                if (MouseZ_Temp > 5)
                {
                    MouseZ_Temp = 5; // 不能积攒太多,积攒太多说明出现了数据错误，在这里进行限幅
                }
                MouseZ_Temp--; // 每次处理时减一
                if (KeyRec_Press_EVT_INI_STATE == RT_EOK)
                    rt_event_send(&KeyRec_Press_EVT, KeyEVT_MouseZ_P); // 发送正转事件
            }
            break;
        case KeyNum_MouseZ_N:
            if (MouseZ_Temp < 0)
            {
                if (MouseZ_Temp < -5)
                {
                    MouseZ_Temp = -5; // 不能积攒太多
                }
                MouseZ_Temp++; // 每次处理时加一
                if (KeyRec_Press_EVT_INI_STATE == RT_EOK)
                    rt_event_send(&KeyRec_Press_EVT, KeyEVT_MouseZ_N); // 发送反转事件
            }
            break;
        default:
            Key_Check((Key_Num_E)Num);
            break;
        }
    }
}

/**
 * @brief 按键记录数组预初始化函数，做最基本的初始化后才能允许用户通过按键初始化函数进行进一步修改。
 * @param 无
 * @return [int] RT_EOK
 * @author ych
 */
int KeyRecData_PreInit(void)
{
    rt_uint32_t Num;
    Key_Record_Type *KeyRec_p;
    // 对KeyRec数组中的每个结构体成员进行缺省值填充
    for (Num = 0; Num < KEY_COUNT_ALL; Num++)
    {
        // 获取结构体指针，设置此按键的ID号
        KeyRec_p = &KeyRec[Num];
        KeyRec_p->ID = 1 << Num;
        // 按键默认弹起
        KeyRec_p->Press = 0;
        KeyRec_p->State_Rec = 0;

        // 默认没有回调函数
        KeyRec_p->Press_Func = NULL;
        KeyRec_p->Rise_Func = NULL;

        // 设置默认的去抖动次数
        KeyRec_p->State_Confirm_Set = KEY_CONFIRM_COUNT_DEFAULT_SET; // 默认去抖动3次
    }
    return RT_EOK;
}
INIT_BOARD_EXPORT(KeyRecData_PreInit); // 在Board层进行自动初始化

/**
 * @brief 对指定的按键添加设置回调函数
 * @param [Key_Select_E] Key_EVT：需要查询的按键
 * @param [void(*p)(void)] PressFun：需要添加至该按键的按下回调函数的函数指针
 * @param [void(*p)(void)] RiseFun：需要添加至该按键的弹起回调函数的函数指针
 * @return [void]
 * @author ych
 */
void Key_SetCallBack(Key_Select_E Key_EVT, void (*PressFun)(void), void (*RiseFun)(void))
{
    Key_Record_Type *KeyRec_p;
    KeyRec_p = &KeyRec[Key_EVTToNum(Key_EVT)]; // 获取对应结构体的地址
    KeyRec_p->Press_Func = PressFun;
    KeyRec_p->Rise_Func = RiseFun;
}

/**
 * @brief 修改指定按键的去抖动次数
 * @param [Key_Select_E] Key_EVT：需要查询的按键
 * @param [rt_int16_t] Confirm_Count_Set：需要设置的去抖动次数参数
 * @return [void]
 * @author ych
 */
void Key_SetPressConfirm(Key_Select_E Key_EVT, rt_int16_t Confirm_Count_Set)
{
    int Key_Num = Key_EVTToNum(Key_EVT);
    // 检查数值，防止数组越界
    if ((rt_uint16_t)Key_Num >= KEY_COUNT_ALL)
    {
        return;
    }
    if (Confirm_Count_Set >= 0)
    { // 不允许设定负数数值
        KeyRec[Key_Num].State_Confirm_Set = Confirm_Count_Set;
    }
}

/**
 * @brief 读取按键状态
 * @param [Key_Select_E] Key_EVT：需要查询的按键
 * @return [rt_int8_t] 该按键现在的状态，数据经过去抖动处理
 * @author ych
 */
rt_int8_t Key_GetState(Key_Select_E Key_EVT)
{
    return KeyRec[(rt_int16_t)Key_EVTToNum(Key_EVT)].Press;
}

// 初始化二维按键使用的事件集
int KeyMenu_EVT_Init(void)
{
    rt_err_t result;
    /* 初始化二维按键使用的事件集 */
    result = rt_event_init(&KeyRec_Press_EVT, "KEY_EVT", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("Init KeyRec_Press_EVT failed.\n");
    }
    else
    {
        KeyRec_Press_EVT_INI_STATE = RT_EOK;
    }
    return (int)result;
}
INIT_APP_EXPORT(KeyMenu_EVT_Init);
