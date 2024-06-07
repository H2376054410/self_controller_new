/**
 * @file  func_encoderCail.c
 * @brief 磁编码器数据校准
 * @author mylj
 * @version 1.0
 * @date 2023-07-28
 * @copyright Copyright (c) 2023  哈尔滨工业大学(威海)HERO战队
 */

#include "func_encoderCail.h"
#include "drv_utils.h"
#include "drv_thread.h"
#include "drv_encoder.h"
#include "drv_HardWdt.h"
#include "func_Dataserver.h"
#include "func_MusicPlayer.h"

static struct rt_semaphore Encoder_CALTrig_Sem;                 // 通信结束后通知数据处理线程处理数据
static EncoderCali_State_e Encoder_Cali_State;                  // 记录编码器校正状态
static EncoderReCali_flag_e EncoderData_ReCali_flag = NoReCail; // 编码器数据重新标定标志位
float encoder_data[ENCODERDATA_NUM] = {0};
float encoder_data_count[ENCODERDATA_NUM] = {0};
CrossCircleData_s encoder_Cail =
    {
        .Circle_Len = ENCODERCIRCLE_LEN,
        .loop = 0,
        .value_extra = 0,
        .value_last = 0,
};

static rt_int8_t num_crosspoint = -1; // 跨圈点

/**
 * @brief
 * @param value_doubt
 * @param num_max
 * @param num_min
 * @return rt_uint8_t 返回较小的小标
 */
static rt_uint8_t Lookup_indexmin(float value_doubt,
                                  rt_uint8_t num_max,
                                  rt_uint8_t num_min)
{
    rt_uint8_t num;
    if (num_max <= num_min)
    {
        // 数据保护
        while (1)
            ;
    }

    if (num_crosspoint == -1)
    {
        for (rt_uint8_t i = 0; i < ENCODERDATA_NUM - 1; i++)
        {
            // 前提：原数列递增
            if (encoder_data[i + 1] <= encoder_data[i])
            {
                num_crosspoint = i;
                break;
            }
        }
    }

    if (num_crosspoint == -1 || num_crosspoint == ENCODERDATA_NUM - 1)
    {
        // 依旧为-1
        // 没有分段点，则最大值为71，原数组递增
        num_crosspoint = ENCODERDATA_NUM - 1;
        while (num_max != num_min + 1)
        {
            num = (rt_uint8_t)(((float)(num_max + num_min)) / 2.0f);
            if (value_doubt > encoder_data[num])
            {
                num_min = num;
            }
            else
            {
                num_max = num;
            }
        }
        return num_min;
    }
    else
    {
        if (value_doubt > encoder_data[num_crosspoint])
        {
            // 比数组内任何值都大
            return num_crosspoint;
        }
        else if (value_doubt < encoder_data[num_crosspoint + 1])
        {
            // 比数组内任何值都小
            return ENCODERDATA_NUM;
        }
        else if (value_doubt > encoder_data[num_max - 1])
        {
            // 比数组下标最大一个还大，说明在跨圈点之前
            num_max = num_crosspoint;
            while (num_max != num_min + 1)
            {
                num = (rt_uint8_t)(((float)(num_max + num_min)) / 2.0f);
                if (value_doubt > encoder_data[num])
                {
                    num_min = num;
                }
                else
                {
                    num_max = num;
                }
            }
            return num_min;
        }
        else
        {
            // 在跨前之后的数组内
            num_min = num_crosspoint + 1;
            while (num_max != num_min + 1)
            {
                num = (rt_uint8_t)(((float)(num_max + num_min)) / 2.0f);
                if (value_doubt > encoder_data[num])
                {
                    num_min = num;
                }
                else
                {
                    num_max = num;
                }
            }
            return num_min;
        }
    }
}

/**
 * @brief 获得编码器校正数据
 * @param value_doubt   //角度值
 * @param out_rad           //弧度值
 */
void EncoderCorrectData_Get(float value_doubt,
                            float *out_rad)
{
    float data_temp;
    rt_uint8_t num;
    num = Lookup_indexmin(value_doubt, ENCODERDATA_NUM, 0);
    if (num == 71)
    {
        data_temp = 5 * ((value_doubt - encoder_data[num]) /
                         (encoder_data[0] - encoder_data[71])) +
                    encoder_data[num];
    }
    else if (num == num_crosspoint)
    {
        data_temp = 5 * ((value_doubt - encoder_data[num]) /
                         (encoder_data[num + 1] - encoder_data[num] + 360.0f)) +
                    encoder_data[num];
    }
    else if (num == ENCODERDATA_NUM)
    {
        data_temp = 5 * ((value_doubt - 0.0f) /
                         (encoder_data[num_crosspoint + 1] -
                          encoder_data[num_crosspoint] + 360.0f)) +
                    encoder_data[num];
    }
    else
    {
        data_temp = 5 * ((value_doubt - encoder_data[num]) /
                         (encoder_data[num + 1] - encoder_data[num])) +
                    encoder_data[num];
    }
    *out_rad = DEG2RAD_f(data_temp); // 转弧度值
}

/**
 * @brief 编码器数据校准，记录校正数据
 * @param Value_true 准确角度值
 * @param Value_doubt 不准确角度值
 */
void EncoderCailData_Record(rt_int16_t Value_true,
                            float Value_doubt)
{
    rt_int8_t num;
    // 记录自开始校准后，所垮圈数
    CrossCircle_ProcessingNormal(&encoder_Cail,
                           Value_true);

    if (encoder_Cail.loop < 3) // 直到转三圈之后退出存取
    {
        num = (rt_int8_t)(Value_true / 5.0f);
        if (Value_true % 5 == 0)
        {
            // 5的倍数
            encoder_data_count[num]++;
            if (encoder_data_count[num] < 1000) // 数值保护
            {
                encoder_data[num] += Value_doubt; // 存入数据
            }
            else
            {
                encoder_data_count[num] = 1000;
            }
        }
        Encoder_Cali_State = Cali_Recording;
    }
    else if (Encoder_Cali_State == Cali_Recording)
    {
        for (rt_uint8_t i = 0; i < ENCODERDATA_NUM; i++)
        {
            encoder_data[i] = encoder_data[i] / encoder_data_count[i];
        }
        Encoder_Cali_State = Cali_RecordFinish;
        // 发送信号量，触发姿态融合算法
        rt_sem_release(&Encoder_CALTrig_Sem);
        Encoder_Cali_State = Cali_WriteStart;
        SoundDisplay_AddSound(Test1_EQ);
    }
}

/**
 * @brief Flash存储函数
 */
static int FlashRecord(void)
{
    rt_uint8_t num;
    rt_uint8_t encoder_data_num;
    rt_uint8_t temp_data[ENCODERDATA_NUM * 4 + 1];

    for (rt_uint16_t i = 0; i < ENCODERDATA_NUM * 4; i++)
    {
        num = i & (0x03);
        encoder_data_num = (rt_uint16_t)(i / 4.0f + 1);
        temp_data[i] = ((rt_uint8_t *)(&encoder_data[encoder_data_num]))[num];
    }

    temp_data[ENCODERDATA_NUM * 4] = 0x0F; // 标志位

    /* 测量完毕, 开始读写 Flash */
    // Hwdt_Feed_Slowly(RT_TRUE); // 开始操作 Flash 数据, 需要开始缓慢喂狗
    rt_enter_critical(); // 进入临界区防止操作系统调度
    stm32_flash_erase(ENCODER_CALI_DATA_ADDR, sizeof(float) * ENCODERDATA_NUM + 1);
    rt_exit_critical();    // 退出临界区继续启动调度器
    rt_thread_mdelay(500); // 擦除与写入需要间隔一段时间
    rt_enter_critical();   // 进入临界区防止操作系统调度
    stm32_flash_write(ENCODER_CALI_DATA_ADDR, temp_data, sizeof(float) * ENCODERDATA_NUM + 1);
    rt_exit_critical(); // 退出临界区继续启动调度器

    // Hwdt_Feed_Slowly(RT_FALSE);// 操作 Flash 数据结束, 重新开始正常喂狗
    Encoder_Cali_State = Cali_OK;
    EncoderCali_State_Write((uint8_t)Encoder_Cali_State);
    return 1;
}

static struct rt_semaphore Encoder_CaliFinish_Sem; // 通信结束后通知数据处理线程处理数据
static rt_thread_t Encoder_Cali_Tid = RT_NULL;     // 陀螺仪校准线程

/**
 * @brief 编码器校正线程
 * @param Para
 */
void Encoder_Cali_Thread(void *Para)
{
    while (1)
    {
        // 阻塞等待编码器数据读取完毕
        rt_sem_take(&Encoder_CALTrig_Sem, RT_WAITING_FOREVER);

        if (Encoder_Cali_State == Cali_WriteStart) // 擦除FLASH
        {
            if (FlashRecord())
            {
                Encoder_Cali_State = Cali_WriteFinish;
                rt_sem_release(&Encoder_CaliFinish_Sem); // 释放标定完成信号量
            }
        }
    }
}

/**
 * @brief 编码器校准线程
 * @brief 此函数初始化校准线程
 * @brief 并挂起等待校准线程结束
 * @brief 线程运行结束后此函数退出
 */
static void Encoder_Cali(void)
{
    // 用来挂起的信号量，校准线程运行结束后会释放
    rt_sem_init(&Encoder_CaliFinish_Sem,
                "Encoder_CaliFinish_Sem",
                0, RT_IPC_FLAG_PRIO);
    // 用来挂起的信号量，校准线程运行结束后会释放
    rt_sem_init(&Encoder_CALTrig_Sem,
                "Encoder_CALTrig_Sem",
                0, RT_IPC_FLAG_PRIO);

    // 初始化零飘校准线程
    Encoder_Cali_Tid = rt_thread_create("Encoder_Cali",           // 线程名
                                        Encoder_Cali_Thread,      // 线程入口
                                        RT_NULL,                  // 入口参数无
                                        1024,                     // 线程栈
                                        THREAD_PRIO_ENCODER_CALI, // 线程优先级
                                        5);                       // 线程时间片大小

    // 线程创建失败返回false
    if (Encoder_Cali_Tid == RT_NULL)
    {
        return;
    }

    // 线程启动失败返回false
    if (rt_thread_startup(Encoder_Cali_Tid) != RT_EOK)
    {
        return;
    }

    // 挂起等待校准结束
    rt_sem_take(&Encoder_CaliFinish_Sem, RT_WAITING_FOREVER);

    EncoderData_ReCali_flag = NoReCail;
    Encoder_Cali_State = Cali_OK;
    EncoderCali_State_Write((uint8_t)Encoder_Cali_State);
    rt_thread_delete(Encoder_Cali_Tid);
}

// 全片擦除后应为0xFF，校准完成后为0x0F
static rt_uint8_t CaliFlag_Read;

/**
 * @brief  从Flash中读取数据
 * @brief  若无数据或需要重测，则会自动重测，完成后函数返回
 * @return int
 */
int Load_EncoderCaliData(void)
{
    rt_uint8_t DataEmptyIf_flag = 0; // 数据是否为空标志位
    rt_uint8_t DataEmpty_count = 0;  // 数据为空的数量
    uint8_t ReadTemp[ENCODERDATA_NUM * 4];

    // 读取Flash中的标记
    rt_enter_critical();
    // Hwdt_Feed_Slowly(RT_TRUE);
    stm32_flash_read(ENCODER_CALI_FLAG_ADDR, &CaliFlag_Read, sizeof(rt_uint8_t));
    stm32_flash_read(ENCODER_CALI_DATA_ADDR, &ReadTemp[0], sizeof(ReadTemp));
    // Hwdt_Feed_Slowly(RT_FALSE);
    rt_exit_critical();

    if ((CaliFlag_Read == 0x0F) && (EncoderData_ReCali_flag != ReCali))
    {
        for (rt_uint16_t i = 0; i < ENCODERDATA_NUM * 4; i++)
        {
            if (ReadTemp[i] == 0xFF)
            {
                DataEmpty_count++;
                // 数据全为空，无需加载flash中的数据
                // 准备重新校准磁编码器数据
                if (DataEmpty_count == ENCODERDATA_NUM * 4)
                {
                    DataEmptyIf_flag = 1;
                    Encoder_Cali_State = Cali_Error;
                    EncoderCali_State_Write((uint8_t)Encoder_Cali_State);
                    break;
                }
                else
                {
                    DataEmptyIf_flag = 0;
                }
            }
        }

        if (DataEmptyIf_flag != 1)
        {
            // 数据不为空
            for (rt_uint16_t i = 0; i < ENCODERDATA_NUM; i++)
            {
                // 取值
                encoder_data[i] = *((float *)&ReadTemp[4 * i]);
            }

            encoder_data[55] = 356.25827f;
            encoder_data[56] = 1.321655f;
            encoder_data[71] = 73.3206f;
            
            Encoder_Cali_State = Cali_OK;
            EncoderCali_State_Write((uint8_t)Encoder_Cali_State);
            return 0;
        }
    }
    else
    {
        Encoder_Cali_State = Cali_Error;
        EncoderCali_State_Write((uint8_t)Encoder_Cali_State);
        for (size_t i = 0; i < ENCODERDATA_NUM * 4; i++)
        {
            if (ReadTemp[i] == 0xFF)
            {
                DataEmpty_count++;
                // 数据全为空，无需加载flash中的数据
                // 准备重新校准磁编码器数据
                if (DataEmpty_count == ENCODERDATA_NUM * 4)
                {
                    DataEmptyIf_flag = 1;
                    break;
                }
                else
                {
                    DataEmptyIf_flag = 0;
                }
            }
        }

        if (DataEmptyIf_flag == 1)
        {
            // 数据不全为空
            // 需要此时只有被全片擦除才会开始自动重新标定
            SoundDisplay_AddSound(EncoderCali_fail_EQ);
            rt_thread_mdelay(1000);
            return 0;
        }
    }
    // FLASH 未写入数据或需要重新标定编码器的值
    Encoder_Cali_State = Cali_Start;
    EncoderCali_State_Write((uint8_t)Encoder_Cali_State);
    Encoder_Cali();
    return 1;
}
