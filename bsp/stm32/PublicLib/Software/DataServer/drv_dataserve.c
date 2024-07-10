#include "drv_dataserve.h"

#include "drv_utils.h"

static struct Data_Server Data[DataServer_Handle_MAX]; // 数据服务器句柄
static rt_uint8_t Data_Msg[DATA_MAX_SIZE];             // 实际数据存放处
static void *Package_Data_Msg_Now = Data_Msg;          // 现在数据存放区的位置
volatile float Dataserver_Memory_Usage = 0;

static rt_uint8_t Package_Num_Now = 0; // 当前数据包的序号动态分配到的位置;
/**
 * @brief 检查可以动态分配的最小序号，主要用于数据被删除以后再次利用该编号
 * @author zqjj
 */
static void check_Package_Num_Now()
{
    while (Data[Package_Num_Now].Enable_Flag == 1)
    {
        Package_Num_Now++;
    }
}

/**
 * @brief 申请一个字节数为size的数据包并初始化
 *
 * @param name       数据包名称（字符串）
 * @param name_size  数据包名称的长度（strlen）
 * @param RecBuff    初始化数据包内容部分的头指针，如果RecBuff为RT_NULL的话就清空
 * @param Buff_size  初始化数据包内容部分的长度（sizeof）
 * @return rt_int8_t 数据包编号
 * @author zqjj
 */
rt_int8_t Request_Add_Package(char *name, rt_uint8_t name_size, void *RecBuff, rt_uint16_t Buff_size)
{
    rt_uint8_t Package_Num;
    rt_uint32_t end_adress = 0; // 暂时存放数据的末尾地址

    if (name_size >= NAME_MAX_SIZE)
        return SERVER_PACKAGE_CHECK_ERR;

    end_adress = (rt_uint32_t)Package_Data_Msg_Now + (((Buff_size / 4) * 4) + ((Buff_size % 4) ? 4 : 0));
    if (end_adress >= (rt_uint32_t)(Data_Msg + DATA_MAX_SIZE))
        return SERVER_PACKAGE_CHECK_ERR;

    /* 动态分配当前分配的序号 */
    check_Package_Num_Now();
    if (Package_Num_Now >= DataServer_Handle_MAX)
        return SERVER_PACKAGE_CHECK_ERR;

    /* 初始化各个模块 */
    rt_memcpy(Data[Package_Num_Now].Data_Name, name, name_size + 1); // 初始化名称(输入strlen取得名字的长度，不含\0)
    Data[Package_Num_Now].Refresh_Tick = rt_tick_get();              // 获取当前刷新时刻
    Data[Package_Num_Now].Frequency = -1;                            // 失能当前刷新频率
    Data[Package_Num_Now].Len = Buff_size;                           // 初始化数据包长度
    Data[Package_Num_Now].Msg_Pool = Package_Data_Msg_Now;
    if (RecBuff!=RT_NULL)                                               // 如果初始化的数据包地址是0的话可以直接全部初始化为零
        rt_memcpy(Data[Package_Num_Now].Msg_Pool, RecBuff, Buff_size); // 初始化数据包内容
    else
        rt_memset(Data[Package_Num_Now].Msg_Pool, 0, Buff_size);
    *((rt_uint32_t *)(&Package_Data_Msg_Now)) = end_adress;
    // 移动当前存放数据所指向的地方,保证内存对齐

    /* 初始化数据锁相关的信号量 */
    Data[Package_Num_Now].Lock_sem = rt_sem_create((const char *)(Data[Package_Num_Now].Data_Name), 1, RT_IPC_FLAG_PRIO); // 设置为1，相当于起振下一个要执行的带锁的函数
    if (Data[Package_Num_Now].Lock_sem == RT_NULL)
    {
        return SERVER_RTOS_ERR; // 没有成功初始化信号量
    }

    /* 初始化回调函数相关 */
    Data[Package_Num_Now].Callbacknode = (struct Read_Callback_Node *)rt_malloc(sizeof(struct Read_Callback_Node));
    if (Data[Package_Num_Now].Callbacknode == RT_NULL)
    {
        return SERVER_RTOS_ERR; // 没有成功申请内存
    }
    Data[Package_Num_Now].Callbacknode->Callback = RT_NULL;
    Data[Package_Num_Now].Callbacknode->Num = 0;
    Data[Package_Num_Now].Callbacknode->next = RT_NULL;

    /* 使能数据包有效 */
    Data[Package_Num_Now].Enable_Flag = 1;

    Package_Num = Package_Num_Now;
    Package_Num_Now++;

    /*内存占有率计算*/
    Dataserver_Memory_Usage = ((rt_uint32_t)Package_Data_Msg_Now - (rt_uint32_t)(&Data_Msg)) / (float)DATA_MAX_SIZE*100;

    return Package_Num;
}

/**
 * @brief 删除一个已经使能好的数据包
 *
 * @param Package_Num 数据包编号
 * @return rt_int8_t
 * @author zqjj
 */
rt_int8_t Request_Erase_Package(rt_uint8_t Package_Num)
{
    Data[Package_Num].Enable_Flag = 0;         // 首先失能这个数据包防止后续读取有错误
    rt_sem_delete(Data[Package_Num].Lock_sem); // 删除信号量释放系统资源
    rt_memset(Data[Package_Num].Msg_Pool, 0, (Data[Package_Num].Len / 4) * 4 + (Data[Package_Num].Len % 4) ? 4 : 0);
    // 清除数据存放地方，计算内存对齐
    rt_memset(&Data[Package_Num], 0, sizeof(struct Data_Server)); // 所有数据归零

    /* 释放回调函数存储中之前malloc的内存，否则会出现堆溢出 */
    struct Read_Callback_Node *Del_Temp_node = Data[Package_Num].Callbacknode->next;
    while (Del_Temp_node != RT_NULL)
    {
        rt_free(Data[Package_Num].Callbacknode);
        Data[Package_Num].Callbacknode = Del_Temp_node;
        Del_Temp_node = Del_Temp_node->next;
    }

    Package_Num_Now = Package_Num; // 此处序号可以再次被占用，但是数据的存放地方就不可以在被使用
    return SERVER_OK;
}

/**
 * @brief 申请为序号为Num的数据包添加读取回调函数
 *
 * @param Num 数据包编号
 * @param Call_back 回调函数的函数名（即添加的回调函数的函数指针）
 * @return rt_int8_t
 * @author zqjj
 */
rt_int8_t Request_Add_CallBack(rt_int8_t Num, DataServer_Callback Call_back)
{
    if (Data[Num].Enable_Flag != 1 || Num < 0) // 没有经过合法的初始化的数据包，丢掉(╯°Д°)╯
    {
        return SERVER_PACKAGE_CHECK_ERR;
    }

    rt_sem_take(Data[Num].Lock_sem, RT_WAITING_FOREVER); // 上锁，开始运行下面的内容

    /* 遍历到链表尾 */
    struct Read_Callback_Node *Temp = Data[Num].Callbacknode;
    while (Temp->next != RT_NULL)
    {
        Temp = Temp->next;
    }

    /* 创建一个新节点 */
    struct Read_Callback_Node *Newnode = RT_NULL;
    Newnode = (struct Read_Callback_Node *)rt_malloc(sizeof(struct Read_Callback_Node));
    if (Newnode == RT_NULL)
    {
        return -1;
    }
    rt_memset(Newnode, 0, sizeof(struct Read_Callback_Node));
    Newnode->Callback = Call_back;
    Newnode->Num = Temp->Num + 1;
    Newnode->next = RT_NULL;

    Temp->next = Newnode;

    rt_sem_release(Data[Num].Lock_sem); // 开锁，等待的部分可以进行了
    return SERVER_OK;
}

/**
 * @brief 查询名称为name的数据包分配的序号
 *
 * @param name 数据包名称字符串
 * @return rt_int8_t 数据包编号
 * @author zqjj
 */
rt_int8_t Package_Find_Num(const char *name)
{
    /* 挖个坑，这里可以改哈希什么的 */
    /* 这里先简单写一个逐字符匹配的，时间复杂度是O(N^2) */
    /* 经过粗浅验证过可行性 */
    rt_uint8_t Find_Num; // 当前查询到数据包的序号

    for (Find_Num = 0; Find_Num <= Package_Num_Now; Find_Num++)
    {
        rt_uint8_t cmp_now = 0;

        while (Data[Find_Num].Data_Name[cmp_now] == *(name + cmp_now))
        {
            if (Data[Find_Num].Data_Name[cmp_now] == '\0')
            {
                return Find_Num; // 返回该数据包的序号
            }
            cmp_now++;
        }
    }
    return SERVER_FIND_ERR; // 没有查询到名为name的数据包
}

/**
 * @brief 读取数据(全部读取)
 *
 * @param Num        数据包编号
 * @param RecBuff    存储输出数据内容的头指针
 * @param Buff_size  存储输出数据内容的长度（sizeof）
 * @return rt_int8_t
 * @author zqjj
 */
rt_int8_t Package_Read_All_Data(rt_int8_t Num, void *RecBuff, rt_uint16_t Buff_size)
{
    /* 校验部分 */
    /* 拿到的数据标号小于0 或 如果数据长度不匹配 或 数据包没有正确使能 */

    if (Num < 0 || Buff_size != Data[Num].Len || Data[Num].Enable_Flag != 1)
        return -1;

    /* 读取主体 */
    rt_sem_take(Data[Num].Lock_sem, RT_WAITING_FOREVER); // 上锁，开始运行下面的内容
    rt_memcpy((rt_uint8_t *)RecBuff, (rt_uint8_t *)Data[Num].Msg_Pool, Buff_size); // 把数据全扔进去
    rt_sem_release(Data[Num].Lock_sem);                  // 开锁，等待的部分可以进行了

    return 1;
}

/**
 * @brief 读取数据（读取第oirgin到end字节的数据）
 *
 * @param Num        数据包编号
 * @param RecBuff    存储输出数据内容的头指针
 * @param Buff_size  存储输出数据内容的长度（sizeof）
 * @param origin     存储输出数据内容部分的前端偏移量
 * @param end        存储输出数据内容部分的后端偏移量
 * @return rt_int8_t
 * @author zqjj
 */
rt_int8_t Package_Read_Part_Data(rt_int8_t Num, void *RecBuff, rt_uint16_t Buff_size, rt_uint8_t origin, rt_uint8_t end)
{
    /* 校验部分 */
    /*  拿到的数据标号小于0 或  数据包没有正确使能    */
    if (Num < 0 || Data[Num].Enable_Flag != 1 || Buff_size != end - origin + 1)
        return -1;

    /* 读取主体 */
    rt_sem_take(Data[Num].Lock_sem, RT_WAITING_FOREVER);                                                 // 上锁，开始运行下面的内容
    rt_memcpy(((rt_uint8_t *)RecBuff + origin), ((rt_uint8_t *)Data[Num].Msg_Pool + origin), Buff_size); // 把数据全扔进去
    rt_sem_release(Data[Num].Lock_sem);                                                                  // 开锁，等待的部分可以进行了

    return 1;
}

/**
 * @brief 按照链表序调用回调
 *
 * @param Num 数据包编号
 * @return rt_int8_t
 * @author zqjj
 */
static rt_int8_t Package_CallBack(rt_int8_t Num)
{
    /* 留坑,这里有点不太确定 */
    /* 拿到的数据标号小于0 */
    if (Num < 0)
        return -1;
    struct Read_Callback_Node *List_Temp = Data[Num].Callbacknode;

    while (List_Temp->next != RT_NULL) // 如果有下一级回调函数
    {
        List_Temp = List_Temp->next;
        List_Temp->Callback();
    }
    return 1;
}

/**
 * @brief 一阶低通滤波
 *
 * @param Data_now  当前值
 * @param Data_Last 上次值
 * @param input     输入值
 * @param D         参数D
 * @author zqjj
 */
static void utils_Low_Pass_Filter_Float(float *Data_now, float *Data_Last, float input, float D)
{
    if (*Data_Last == -1)
    {
        *Data_Last = input;
        return;
    }

    *Data_now = UTILS_LP_FAST(*Data_Last, input, D);
    *Data_Last = *Data_now;
}

/**
 * @brief 写入数据（全部写入）
 *
 * @param Num        数据包编号
 * @param SendBuff   为数据包写入的数据源的头指针
 * @param Buff_size  数据源长度（sizeof）
 * @return rt_int8_t
 * @author zqjj
 */
rt_int8_t Package_Write_All_Data(rt_int8_t Num, void *SendBuff, rt_uint16_t Buff_size)
{
    /* 校验部分 */
    /*  拿到的数据标号小于0 或  如果数据长度不匹配       或      数据包没有正确使能    */
    if (Num < 0 || Buff_size != Data[Num].Len || Data[Num].Enable_Flag != 1)
        return -1;

    /* 读取主体 */
    rt_sem_take(Data[Num].Lock_sem, RT_WAITING_FOREVER); // 上锁，开始运行下面的内容
    rt_memcpy(Data[Num].Msg_Pool, SendBuff, Buff_size);  // 把数据全扔进去
    /* 保存上次的刷新时刻和频率 */
    rt_tick_t Temp_Tick = Data[Num].Refresh_Tick;
    float Temp_Frequency = Data[Num].Frequency;

    /* 更新刷新时间 */
    Data[Num].Refresh_Tick = rt_tick_get();

    /* 对刷新频率取低通滤波（函数内部考虑了初值为0）*/
    utils_Low_Pass_Filter_Float(&Data[Num].Frequency, &Temp_Frequency, 100000 / (Data[Num].Refresh_Tick - Temp_Tick), 0.5);

    Package_CallBack(Num);              // 调用回调
    rt_sem_release(Data[Num].Lock_sem); // 开锁，等待的部分可以进行了

    return SERVER_OK;
}

/**
 * @brief 写入数据（返回指针并统计频率，无回调，会上锁需要手动解锁）
 * @brief !!! 不能两次连续使用 !!!，使用一次必须跟着一次结束
 * @param Num        数据包编号
 * @param Buff_size  数据源长度（sizeof）
 * @return void*
 * @author wdl
 */
void* Package_Pionter(rt_int8_t Num, rt_uint16_t Buff_size)
{
    rt_err_t ans=RT_EOK;
    /* 校验部分 */
    /*  拿到的数据标号小于0 或  如果数据长度不匹配       或      数据包没有正确使能    */
    if (Num < 0 || Buff_size != Data[Num].Len || Data[Num].Enable_Flag != 1)
        while (1)
            ;
    /* 上锁 */
    ans = rt_sem_take(Data[Num].Lock_sem, 15);
    if(ans != RT_EOK)
        while(1)//大概率是你忘了给上一次读或者写的数据解锁了，也有可能是15ms有一个线程没产生数据释放信号量
            ;

    return Data[Num].Msg_Pool;
}

/**
 * @brief 写入数据（部分写入）
 *
 * @param Num        数据包编号
 * @param SendBuff   为数据包写入的数据源的头指针
 * @param Buff_size  数据源长度（sizeof）
 * @param origin     写入部分开头指针偏移量
 * @param end        写入部分结尾指针偏移量
 * @return rt_int8_t
 * @author zqjj
 */
rt_int8_t Package_Write_Part_Data(rt_int8_t Num, void *SendBuff, rt_uint16_t Buff_size, rt_uint8_t origin, rt_uint8_t end)
{
    /* 校验部分 */
    /*  拿到的数据标号小于0 或  数据包没有正确使能 或   */
    if (Num < 0 || Data[Num].Enable_Flag != 1 || Buff_size != end - origin + 1)
        return -1;

    /* 读取主体 */
    rt_sem_take(Data[Num].Lock_sem, RT_WAITING_FOREVER);                                                         // 上锁，开始运行下面的内容
    rt_memcpy(((rt_uint8_t *)Data[Num].Msg_Pool + origin), ((rt_uint8_t *)SendBuff + origin), end - origin + 1); // 把数据全扔进去

    /* 保存上次的刷新时刻和频率 */
    rt_tick_t Temp_Tick = Data[Num].Refresh_Tick;
    float Temp_Frequency = Data[Num].Frequency;

    /* 更新刷新时间 */
    Data[Num].Refresh_Tick = rt_tick_get();

    /* 对刷新频率取低通滤波（函数内部考虑了初值为0）*/
    utils_Low_Pass_Filter_Float(&Data[Num].Frequency, &Temp_Frequency, 100000 / (Data[Num].Refresh_Tick - Temp_Tick), 0.5);

    Package_CallBack(Num); // 调用回调

    rt_sem_release(Data[Num].Lock_sem); // 开锁，等待的部分可以进行了

    return SERVER_OK;
}

/**
 * @brief 结束读数据包,当读写采用指针的时候用来结束
 *
 * @param Num        数据包编号
 * @param Buff_size  数据源长度（sizeof）
 * @return rt_err_t
 * @author wdl
 */
rt_int8_t Package_Pionter_Read_End(rt_int8_t Num, rt_uint16_t Buff_size)
{
    if (Num < 0 || Buff_size != Data[Num].Len || Data[Num].Enable_Flag != 1)
        return SERVER_PACKAGE_CHECK_ERR;
    rt_sem_release(Data[Num].Lock_sem); // 开锁，等待的部分可以进行了
    return SERVER_OK;
}

/**
 * @brief 结束写数据包,当读写采用指针的时候用来结束
 *
 * @param Num        数据包编号
 * @param Buff_size  数据源长度（sizeof）
 * @return rt_err_t
 * @author wdl
 */
rt_int8_t Package_Pionter_Write_End(rt_int8_t Num, rt_uint16_t Buff_size)
{
    if (Num < 0 || Buff_size != Data[Num].Len || Data[Num].Enable_Flag != 1)
        return SERVER_PACKAGE_CHECK_ERR;
    /* 保存上次的刷新时刻和频率 */
    rt_tick_t Temp_Tick = Data[Num].Refresh_Tick;
    float Temp_Frequency = Data[Num].Frequency;

    /* 更新刷新时间 */
    Data[Num].Refresh_Tick = rt_tick_get();

    /* 对刷新频率取低通滤波（函数内部考虑了初值为0）*/
    utils_Low_Pass_Filter_Float(&Data[Num].Frequency, &Temp_Frequency, 100000 / (Data[Num].Refresh_Tick - Temp_Tick), 0.5);

    /* 调用回调函数 */
    Package_CallBack(Num);

    rt_sem_release(Data[Num].Lock_sem); // 开锁，等待的部分可以进行了
    return SERVER_OK;
}
