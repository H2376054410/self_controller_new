#include "drv_remote_sbus.h"
#if defined BSP_USING_REMOTE_SBUS
#include "drv_thread.h"

/* 存储遥控器当前数据 */
RC_Ctrl_t RC_data;
/*存储遥控器上次数据 */
static RC_Ctrl_t RC_data_last;
static int RC_Data_Num = 0; // 对接收到的串口数据包进行计数标记，用于判断是否为新数据
/* 存储s1 s2 的动作 */
static rt_int16_t remote_s0_data = 0;
static rt_int16_t remote_s1_data = 0;
static rt_int16_t remote_s2_data = 0;
static rt_int16_t remote_s3_data = 0;
/* 在从两边往中间拨时存储RC_last中拨杆s1 s2的值 */
/* 当拨杆从中间拨到两边时，可以读到RC_data的值来判断上下拨 */
/* 但当拨杆为两边往中间拨时，RC_data=RC_lastdata,所以读到动作的时候存一下 */
static rt_int16_t temp_s_data = 0;

static rt_int8_t RC_Channel_Fix_Flag = 0; // 标记是否进行过遥控通道零位校准（接收到的第一组摇杆数据将作为零位）
static rt_int16_t CH_FIX[4];              // 用来存储零位偏移量的数组

int16_t s0 = 0;
int16_t s1 = 0;
int16_t s2 = 0;
int16_t s3 = 0;
int16_t ch0 = 0;
int16_t ch1 = 0;
int16_t ch2 = 0;
int16_t ch3 = 0;
void RemoteDataProcess(uint8_t *pData, RC_Ctrl_t *RC_CtrlData)
{
    if (pData == NULL)
    {
        return;
    }

    ch0 = ((int16_t)pData[1] >> 0 | ((int16_t)pData[2] << 8)) & 0x7ff;
    ch1 = ((int16_t)pData[2] >> 3 | ((int16_t)pData[3] << 5)) & 0x7ff;
    ch2 = ((int16_t)pData[3] >> 6 | ((int16_t)pData[4] << 2) | ((int16_t)pData[5] << 10)) & 0x7ff;
    ch3 = ((int16_t)pData[5] >> 1 | ((int16_t)pData[6] << 7)) & 0x7ff;
    s0 = ((int16_t)pData[6] >> 4 | ((int16_t)pData[7] << 4)) & 0x7ff;
    s1 = ((int16_t)pData[7] >> 7 | ((int16_t)pData[8] << 1) | ((int16_t)pData[9] << 9)) & 0x7ff;
    s2 = ((int16_t)pData[9] >> 2 | ((int16_t)pData[10] << 6)) & 0x7ff;
    s3 = ((int16_t)pData[10] >> 5 | ((int16_t)pData[11] << 3)) & 0x7ff;

    if (
        (ch0 < THROTTLE_HIGH) && (ch0 > THROTTLE_DOWN) &&
        (ch1 < THROTTLE_HIGH) && (ch1 > THROTTLE_DOWN) &&
        (ch2 < THROTTLE_HIGH) && (ch2 > THROTTLE_DOWN) &&
        (ch3 < THROTTLE_HIGH) && (ch3 > THROTTLE_DOWN))
    {
        if (RC_Channel_Fix_Flag)
        { // 如果还没有进行零位校正
            RC_CtrlData->Remote_Data.ch0 = ch0 - CH_FIX[0];
            RC_CtrlData->Remote_Data.ch1 = ch1 - CH_FIX[1];
            RC_CtrlData->Remote_Data.ch2 = ch2 - CH_FIX[2];
            RC_CtrlData->Remote_Data.ch3 = ch3 - CH_FIX[3];
        }
        else
        {
            if (RC_Data_Num >= 10)
            { // 使用第10组数据进行零位校正
                CH_FIX[0] = ch0 - THROTTLE_MIDDLE;
                CH_FIX[1] = ch1 - THROTTLE_MIDDLE;
                CH_FIX[2] = ch2 - THROTTLE_MIDDLE;
                CH_FIX[3] = ch3 - THROTTLE_MIDDLE;
                for (char fori = 0; fori < 4; fori++)
                {
                    if (CH_FIX[fori] > 30 || CH_FIX[fori] < -30)
                    { // 如果测到的数据很离谱，则不进行修正
                        CH_FIX[fori] = 0;
                    }
                }
                RC_Channel_Fix_Flag = 1;
            }
        }

        if (s0 <= SWITCH_MIDDLE)
            RC_CtrlData->Remote_Data.s0 = 1;
        else
            RC_CtrlData->Remote_Data.s0 = 2;

        if (s1 <= SWITCH_DOWN)
            RC_CtrlData->Remote_Data.s1 = 1;
        else if (s1 > SWITCH_DOWN && s1 < SWITCH_UP)
            RC_CtrlData->Remote_Data.s1 = 3;
        else if (s1 > SWITCH_UP)
            RC_CtrlData->Remote_Data.s1 = 2;

        if (s2 <= SWITCH_DOWN)
            RC_CtrlData->Remote_Data.s2 = 1;
        else if ((s2 > SWITCH_DOWN) && (s2 < SWITCH_UP))
            RC_CtrlData->Remote_Data.s2 = 3;
        else if (s2 > SWITCH_UP)
            RC_CtrlData->Remote_Data.s2 = 2;

        if (s3 <= SWITCH_MIDDLE)
            RC_CtrlData->Remote_Data.s3 = 1;
        else
            RC_CtrlData->Remote_Data.s3 = 2;

        RC_Data_Num++;
    }
}

/* 串口接收消息结构*/
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
/* 串口设备句柄 */
static rt_device_t serial;
/* 消息队列控制块 */
static struct rt_messagequeue rx_mq;

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(&rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("message queue full!\n");
    }
    return result;
}
struct rt_semaphore RoboControl_sem; /* 用于接收消息的信号量 */
uint8_t rx_buffer[RT_SERIAL_RB_BUFSZ + 1];
static void serial_thread_entry(void *parameter)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;
    rt_int8_t LastData_Valid = 0;
    // 初始化
    RC_data.Remote_Data.ch0 = 1024;
    RC_data.Remote_Data.ch1 = 1024;
    RC_data.Remote_Data.ch2 = 1024;
    RC_data.Remote_Data.ch3 = 1024;
    RC_data.Remote_Data.s0 = 3;
    RC_data.Remote_Data.s1 = 3;
    RC_data.Remote_Data.s2 = 3;
    RC_data.Remote_Data.s3 = 3;
    RC_data_last.Remote_Data.ch0 = 1024;
    RC_data_last.Remote_Data.ch1 = 1024;
    RC_data_last.Remote_Data.ch2 = 1024;
    RC_data_last.Remote_Data.ch3 = 1024;
    RC_data_last.Remote_Data.s0 = 3;
    RC_data_last.Remote_Data.s1 = 3;
    RC_data_last.Remote_Data.s2 = 3;
    RC_data_last.Remote_Data.s3 = 3;
    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rx_mq, &msg, sizeof(msg), 200);
        if (result == RT_EOK)
        {
            /* 从串口读取数据*/
            rx_length = rt_device_read(msg.dev, 0, rx_buffer, msg.size);
            if (rx_length != 0x0019)
            { // 如果长度不对，则直接跳过，但是必须从rt_device_read读出，否则缓冲区会溢出
                continue;
            }
            rx_buffer[rx_length] = '\0';

            RemoteDataProcess(rx_buffer, &RC_data);

            RC_data.ValidData_FreshTick = rt_tick_get(); // 刷新数据的更新时间

            if (LastData_Valid)
            {
                /* remote_sx_data = 1 中间往两边拨 =2 两边往中间拨 */
                /* remote_sx_data = 1 向上拨动 =2 向下拨动 */
                if (RC_data_last.Remote_Data.s0 == 1 && RC_data.Remote_Data.s0 == 2)
                {
                    remote_s0_data = 2;
                }
                else if (RC_data_last.Remote_Data.s0 == 2 && RC_data.Remote_Data.s0 == 1)
                {
                    remote_s0_data = 1;
                }

                if (RC_data_last.Remote_Data.s1 == 3 && RC_data.Remote_Data.s1 != 3)
                {
                    remote_s1_data = 1;
                }
                else if (RC_data_last.Remote_Data.s1 != 3 && RC_data.Remote_Data.s1 == 3)
                {
                    remote_s1_data = 2;
                    temp_s_data = (RC_data_last.Remote_Data.s1 << 8) | RC_data_last.Remote_Data.s2;
                }

                if (RC_data_last.Remote_Data.s2 == 3 && RC_data.Remote_Data.s2 != 3)
                {
                    remote_s2_data = 1;
                }
                else if (RC_data_last.Remote_Data.s2 != 3 && RC_data.Remote_Data.s2 == 3)
                {
                    remote_s2_data = 2;
                    temp_s_data = (RC_data_last.Remote_Data.s1 << 8) | RC_data_last.Remote_Data.s2;
                }

                if (RC_data_last.Remote_Data.s3 == 1 && RC_data.Remote_Data.s3 == 2)
                {
                    remote_s3_data = 2;
                }
                else if (RC_data_last.Remote_Data.s3 == 2 && RC_data.Remote_Data.s3 == 1)
                {
                    remote_s3_data = 1;
                }
            }
            else
            {
                if (RC_Data_Num >= 5)
                { // 前5组数据丢弃，可能不稳定
                    LastData_Valid = 1;
                }
            }
        }
        else
        {
            // 数据时效检查未通过, 准备重新进行校准
            RC_Data_Num = 0;
            LastData_Valid = 0;
            RC_Channel_Fix_Flag = 0;
            RC_data.Remote_Data.ch0 = THROTTLE_MIDDLE;
            RC_data.Remote_Data.ch1 = THROTTLE_MIDDLE;
            RC_data.Remote_Data.ch2 = THROTTLE_MIDDLE;
            RC_data.Remote_Data.ch3 = THROTTLE_MIDDLE;
            RC_data.Remote_Data.s0 = 0;
            RC_data.Remote_Data.s1 = 0;
            RC_data.Remote_Data.s2 = 0;
            RC_data.Remote_Data.s3 = 0;
        }
        rt_memcpy(&RC_data_last, &RC_data, sizeof(RC_data));
        // 释放遥控器数据处理信号量
        while (rt_sem_trytake(&RoboControl_sem) == RT_EOK)
            continue;
        rt_sem_release(&RoboControl_sem);
    }
}

int remote_uart_init(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    static char msg_pool[256];

    rt_strncpy(uart_name, REMOTE_UART_DEVICE_NAME, RT_NAME_MAX);

    /* 查找串口设备 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(REMOTE_UART_DEVICE_NAME);

    /* step2：修改串口配置参数 */
    config.baud_rate = 100000;      // 修改波特率为 9600
    config.data_bits = DATA_BITS_8; // 数据位 8
    config.stop_bits = STOP_BITS_2; // 停止位 2
    config.bufsz = 128;             // 修改缓冲区 buff size 为 128
    config.parity = PARITY_EVEN;    //

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* 初始化消息队列 */
    rt_mq_init(&rx_mq, "rx_mq",
               msg_pool,              /* 存放消息的缓冲区 */
               sizeof(struct rx_msg), /* 一条消息的最大长度 */
               sizeof(msg_pool),      /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_FIFO);     /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);

    // 初始化信号量
    rt_sem_init(&RoboControl_sem, "RoboControl_sem", 0, RT_IPC_FLAG_FIFO);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 2048, THREAD_PRIO_RC_RX, 1);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    else
        ret = RT_ERROR;
    return ret;
}

/**
 * @brief   读取拨杆动作(两档按键)
 * @param   传入S1则读取S1的动作，传入S2则读取S2的动作
 * @retval  返回往上拨或往下拨
 **/
switch_action_e Switch_Change(switch_action_e sx)
{
    if (sx == S0)
    {
        if (remote_s0_data == 1)
        {
            remote_s0_data = 0;
            return down_to_up;
        }
        else if (remote_s0_data == 2)
        {
            remote_s0_data = 0;
            return up_to_down;
        }
    }
    else if (sx == S3)
    {
        if (remote_s3_data == 1)
        {
            remote_s3_data = 0;
            return down_to_up;
        }
        else if (remote_s3_data == 2)
        {
            remote_s3_data = 0;
            return up_to_down;
        }
    }
    return NO_ACTION;
}

/**
 * @brief   读取拨杆动作(只能读取中间往两边拨的动作)
 * @param   传入S1则读取S1的动作，传入S2则读取S2的动作
 * @retval  返回往上拨(middle_to_up) 或 往下拨(middle_to_down)
 **/
switch_action_e Change_from_middle(switch_action_e sx)
{
    if (sx == S1)
    {
        if (remote_s1_data == 1)
        {
            if (RC_data.Remote_Data.s1 == 1)
            {
                remote_s1_data = 0;
                return middle_to_up;
            }
            else if (RC_data.Remote_Data.s1 == 2)
            {
                remote_s1_data = 0;
                return middle_to_down;
            }
        }
    }
    else if (sx == S2)
    {
        if (remote_s2_data == 1)
        {
            if (RC_data.Remote_Data.s2 == 1)
            {
                remote_s2_data = 0;
                return middle_to_up;
            }
            else if (RC_data.Remote_Data.s2 == 2)
            {
                remote_s2_data = 0;
                return middle_to_down;
            }
        }
    }
    return NO_ACTION;
}
/**
 * @brief   读取拨杆动作(只能读取两边往中间拨的动作)
 * @param   传入S1则读取S1的动作，传入S2则读取S2的动作
 * @retval  返回从上往中间拨(up_to_middle) 或 从下往中间拨(down_to_middle)
 **/
switch_action_e Change_to_middle(switch_action_e sx)
{
    if (sx == S1)
    {
        if (remote_s1_data == 2)
        {
            if ((rt_uint8_t)(temp_s_data >> 8) == 1)
            {
                remote_s1_data = 0;
                temp_s_data &= 0x1100;
                return up_to_middle;
            }
            else if ((rt_uint8_t)(temp_s_data >> 8) == 2)
            {
                remote_s1_data = 0;
                temp_s_data &= 0x1100;
                return down_to_middle;
            }
        }
    }
    else if (sx == S2)
    {
        if (remote_s2_data == 2)
        {
            if ((rt_uint8_t)temp_s_data == 1)
            {
                remote_s2_data = 0;
                temp_s_data &= 0x0011;
                return up_to_middle;
            }
            else if ((rt_uint8_t)temp_s_data == 2)
            {
                remote_s2_data = 0;
                temp_s_data &= 0x0011;
                return down_to_middle;
            }
        }
    }
    return NO_ACTION;
}

#endif
