#include "RoboInteract.h"
#include "mod_RefSystem.h"
#include "drv_CRC.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "HRef_ID.h"
static ext_send_user_data_t send_data;
static rt_device_t serial;
static uint8_t referee_intercom_buff[40] = {0};

static struct rt_timer task_100ms;
static struct rt_semaphore interact_100ms_sem;
static rt_err_t referee_intercom_tranamit(ext_send_user_data_t *sendData);

/***
 * @brief    //向其他机器人发送通信内容,目前只用到哨兵
 ***/
static void Ref_Message_Distribute(Ref_ROBO_CLIENT_ID_e receiver_id, uint8_t *data, uint16_t length)
{

    send_data.dataFrameHeader.data_cmd_id = 0x201; // 0x200~0x2ff取其一即可
    send_data.dataFrameHeader.send_ID = REF_ROBO_ID;
    send_data.dataFrameHeader.receiver_ID = receiver_id;

    send_data.dataLength = 0;
    rt_memcpy(send_data.userData.data, data, length);
    send_data.dataLength += length;
    referee_intercom_tranamit(&send_data);
}

static void task_100ms_IRQHandler(void *parameter)
{
    rt_sem_release(&interact_100ms_sem);
}

static void RoboInteract_Thread(void *parameter)
{
    while (1)
    {
        rt_sem_take(&interact_100ms_sem, RT_WAITING_FOREVER);
        Ref_Message_Distribute((Ref_ROBO_CLIENT_ID_e)1, NULL, 1); // unused
    }
}

void RoboInteract_Init(void)
{
    rt_thread_t thread;
    rt_sem_init(&interact_100ms_sem, "100ms_sem", 0, RT_IPC_FLAG_FIFO);
    thread = rt_thread_create("Robot Interact",
                              RoboInteract_Thread,
                              RT_NULL,
                              THREAD_STACK_INTERACT,
                              THREAD_PRIO_INTERACT,
                              THREAD_TICK_INTERACT);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    // 创建线程定时器
    rt_timer_init(&task_100ms,
                  "100ms_task",
                  task_100ms_IRQHandler,
                  RT_NULL,
                  100,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    // 启动定时器
    rt_timer_start(&task_100ms);
}

/***
 * @name
 * @brief	机器人间通信
 * @param	sendData :待发送数据结构体
 * @retval
 ***/
static rt_err_t referee_intercom_tranamit(ext_send_user_data_t *sendData)
{
    uint16_t index = 0;
    uint16_t header = ID_student_interactive_data;
    uint16_t datalength = LEN_HEADER + LEN_CMDID + LEN_TAIL + sendData->dataLength + 6;
    uint16_t tempcrc16 = 0;
    uint16_t sender_Id = (uint16_t)sendData->dataFrameHeader.send_ID;
    uint16_t receiver_id = (uint16_t)sendData->dataFrameHeader.receiver_ID;
    uint16_t cmd_id = sendData->dataFrameHeader.data_cmd_id;

    static uint8_t seq = 0x02; // sequence number of frame
    memset(referee_intercom_buff, 0x00, sizeof(referee_intercom_buff));
    referee_intercom_buff[index] = HEADER_SOF;
    index += 1; // index =1
    memcpy(referee_intercom_buff + index, &datalength, 2);
    index += 2; // index = 3
    memcpy(referee_intercom_buff + index, &seq, 1);
    // seq++;
    index += 1; // index = 4
    referee_intercom_buff[index] = Get_CRC8_Check_Sum(referee_intercom_buff, index, CRC8_INIT);
    index += 1; // index = 5
    memcpy(referee_intercom_buff + index, &header, 2);
    index += 2; // index - 7

    memcpy(referee_intercom_buff + index, &cmd_id, 2);
    index += 2; // index ■9
    memcpy(referee_intercom_buff + index, &sender_Id, 2);
    index += 2; // index■11
    memcpy(referee_intercom_buff + index, &receiver_id, 2);
    index += 2; // index = 13
    memcpy(referee_intercom_buff + index, sendData->userData.data, sendData->dataLength);
    index += sendData->dataLength; // index = 13 + lenth
    tempcrc16 = Get_CRC16_Check_Sum(referee_intercom_buff, index, CRC16_INIT);
    memcpy(referee_intercom_buff + index, &tempcrc16, 2);
    index += 2; // 1ndex = 15 + 1enth
    /*以下用自定义的串口发送函数，发送rereree_ intercom burr内的长度为index的数据*****/
    // usart6_tx_dna_enab1e (rereree_ 1ntercom_ butr，index) ;
    serial = rt_device_find(BSP_REF_USART_NAME);
    if (!rt_device_write(serial, 0, referee_intercom_buff, index))
        return RT_ERROR; // rt_kprintf("fail");//如果发送数据为0计数一次发送失败，失败次数过多发出警告
    else
        return RT_EOK;
}
