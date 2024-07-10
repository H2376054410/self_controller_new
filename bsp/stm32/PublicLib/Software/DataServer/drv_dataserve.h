
#ifndef __DRV_DATASERVE_H__
#define __DRV_DATASERVE_H__

#include <rtthread.h>

#define NAME_MAX_SIZE 20

#if (DATA_MAX_SIZE % 4) || (DATA_MAX_SIZE == 0)
#error "Data_Server's Data size is wrong, it must be a multiple of 4!"
#endif

typedef void (*DataServer_Callback)(void); // 数据服务器的回调函数类型

struct Read_Callback_Node
{
    rt_uint8_t Num;               // 节点编号
    DataServer_Callback Callback; // 回调函数指针
    struct Read_Callback_Node *next;
};

struct Data_Server
{
    char Data_Name[NAME_MAX_SIZE + 1];       // 数据名称
    rt_uint8_t Pos;                          // 数据存储位置
    rt_uint16_t Len;                          // 数据长短（单位字节）
    rt_tick_t Refresh_Tick;                  // 刷新时刻
    float Frequency;                         // 刷新频率
    rt_sem_t Lock_sem;                       // 上锁相关信号量
    struct Read_Callback_Node *Callbacknode; // 回调函数的链表的表头
    rt_uint8_t Enable_Flag;                  // 该数据包是否被使能
    void *Msg_Pool;                          // 数据段内容指针
};

/* 错误码 */
#define SERVER_OK 1 // 不是错误码，一切OK

#define SERVER_PACKAGE_CHECK_ERR -1 // 申请有关数据包的操作时，数据包使能校验或长度校验未通过，爆内存
#define SERVER_RTOS_ERR -2          // 服务器中涉及操作系统的部分出现错误（如信号量未成功初始化，线程未创建成功）
#define SERVER_FIND_ERR -3          // 没有查找到该数据包
#define SERVER_OTHER_ERR -4         // 未知类型错误

/* 数据初始化, 只有数据服务器极其相关数据被初始化以后工程内的各模块才能正常初始化 */
#define DATASERVER_DATAPACKAGE_INIT(fn) INIT_BOARD_EXPORT(fn) // 初始化各个模块对于数据包的申请

/* 设计快速读取和写入数据的宏 */
/* 复制式 */
/**
 * @brief 复制式的快速申请(内部数据在初始化之后全为零)，申请的名字为这个变量的名字
 * @param Name       数据包编号
 * @param Varia      变量本身（里面含有对变量取地址和大小的操作）
 * @return rt_int8_t 数据包编号
 * @author wdl
 */
#define Package_Add(Name, Varia) Request_Add_Package(Name, rt_strlen(Name), &(Varia), sizeof(Varia))
/**
 * @brief 读取或写入数据(直接返回指针，复制式)
 * @param Num       数据包编号
 * @param Varia     变量本身（里面含有对变量取地址和大小的操作）
 * @return int8_t
 * @author wdl
 */
#define Package_Read(Num, Varia) Package_Read_All_Data((Num), &(Varia), sizeof((Varia)))
#define Package_Write(Num, Varia) Package_Write_All_Data((Num), &(Varia), sizeof((Varia)))

/* 指针式的，需要手动结束读写 */
/**
 * @brief 指针式的快速申请(内部数据在初始化之后全为零)，申请的名字为这个变量的名字
 * @param Name       数据包编号
 * @param type      数据类型，自动把地址转换类型到指针上
 * @return rt_int8_t 数据包编号
 * @author wdl
 */
#define Package_Pionter_Add(Name, type) Request_Add_Package(Name, rt_strlen(Name), 0, sizeof(type))
/**
 * @brief 读取或写入数据(直接返回指针，需要手动结束，一次读写，一次结束)
 * @brief !!! 不能两次连续使用Package...Single !!!，使用一次必须跟着一次结束
 * @param Num       数据包编号
 * @param type      数据类型，自动把地址转换类型到指针上
 * @return p
 * @author wdl
 */
#define Package_Pionter_Single(Num, type) (type *)Package_Pionter((Num), sizeof(type))
/**
 * @brief 读取或写入数据的结束函数
 * @brief write会调用回调
 * @param Num       数据包编号
 * @param type      数据类型，自动把地址转换类型到指针上
 * @return p
 * @author wdl
 */
#define Package_Read_Pionter_End(Num, type) Package_Pionter_Read_End((Num), sizeof(type))
#define Package_Write_Pionter_End(Num, type) Package_Pionter_Write_End((Num), sizeof(type))

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
extern rt_int8_t Request_Add_Package(char *name, rt_uint8_t name_size, void *RecBuff, rt_uint16_t Buff_size);
/**
 * @brief 申请为序号为Num的数据包添加读取回调函数
 *
 * @param Num 数据包编号
 * @param Call_back 回调函数的函数名（即添加的回调函数的函数指针）
 * @return rt_int8_t
 * @author zqjj
 */
extern rt_int8_t Request_Add_CallBack(rt_int8_t Num, DataServer_Callback Call_back);
/**
 * @brief 删除一个已经使能好的数据包
 *
 * @param Package_Num 数据包编号
 * @return rt_int8_t
 * @author zqjj
 */
extern rt_int8_t Request_Erase_Package(rt_uint8_t Package_Num);

/**
 * @brief 查询名称为name的数据包分配的序号
 *
 * @param name 数据包名称字符串
 * @return rt_int8_t 数据包编号
 * @author zqjj
 */
extern rt_int8_t Package_Find_Num(const char *name);
/**
 * @brief 读取数据(全部读取)
 *
 * @param Num        数据包编号
 * @param RecBuff    存储输出数据内容的头指针
 * @param Buff_size  存储输出数据内容的长度（sizeof）
 * @return rt_int8_t
 * @author zqjj
 */
extern rt_int8_t Package_Read_All_Data(rt_int8_t Num, void *RecBuff, rt_uint16_t Buff_size);
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
extern rt_int8_t Package_Read_Part_Data(rt_int8_t Num, void *RecBuff, rt_uint16_t Buff_size, rt_uint8_t origin, rt_uint8_t end);
/**
 * @brief 写入数据（返回指针并统计频率，无回调，会上锁需要手动解锁）
 * @brief !!! 不能两次连续使用!!!，使用一次必须跟着一次结束
 * @param Num        数据包编号
 * @param Buff_size  数据源长度（sizeof）
 * @return void*
 * @author wdl
 */
extern void *Package_Pionter(rt_int8_t Num, rt_uint16_t Buff_size);
/**
 * @brief 写入数据（全部写入）
 *
 * @param Num        数据包编号
 * @param SendBuff   为数据包写入的数据源的头指针
 * @param Buff_size  数据源长度（sizeof）
 * @return rt_int8_t
 * @author zqjj
 */
extern rt_int8_t Package_Write_All_Data(rt_int8_t Num, void *SendBuff, rt_uint16_t Buff_size);
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
extern rt_int8_t Package_Write_Part_Data(rt_int8_t Num, void *SendBuff, rt_uint16_t Buff_size, rt_uint8_t origin, rt_uint8_t end);
/**
 * @brief 结束读数据包,当读写采用指针的时候用来结束
 *
 * @param Num        数据包编号
 * @param Buff_size  数据源长度（sizeof）
 * @return rt_err_t
 * @author wdl
 */
extern rt_int8_t Package_Pionter_Read_End(rt_int8_t Num, rt_uint16_t Buff_size);
/**
 * @brief 结束写数据包,当读写采用指针的时候用来结束
 *
 * @param Num        数据包编号
 * @param Buff_size  数据源长度（sizeof）
 * @return rt_err_t
 * @author wdl
 */
extern rt_int8_t Package_Pionter_Write_End(rt_int8_t Num, rt_uint16_t Buff_size);

#endif
