// 使用这个接口时，一定要将 SD 卡发送线程置最低优先级（或者至少放在所有要发送 SD 数据的线程优先级之下）

#ifndef __FUNC_SD_MASTER_H_
#define __FUNC_SD_MASTER_H_

#include <rtdef.h>

/* 数据包类型，要与主机约定好 */
#define SD_LOG_IS_MAKE  (0)
#define SD_LOG_IS_WRITE (1)
#define SD_LOG_IS_PRINT (2)
#define SD_LOG_IS_VOFA_CREATE  (3)
#define SD_LOG_IS_VOFA_WRITE  (4)

// 从机内核重启需要的时间
#define SD_SLAVE_INIT_DELAY  (30)
// 从机可用延时
#define SD_SLAVE_GOOD_DELAY  (200)

// debug 接收线程
#define THREAD_STACK_SD_MASTER (1024)
// #define THREAD_PRIO_SD_MASTER (27)
#define THREAD_TICK_SD_MASTER (1)

extern rt_int32_t sd_master_printf(rt_uint8_t file_index, const char *fmt, ...);
extern rt_uint16_t sd_master_vofa_create(rt_uint8_t file_index, rt_uint8_t* name, rt_uint8_t cnt, ...);
extern rt_uint16_t sd_master_vofa_write(rt_uint8_t file_index, rt_uint8_t cnt, ...);

rt_uint8_t sd_master_wFile(rt_uint8_t file_index, rt_uint8_t* buff, rt_uint8_t size);
void sd_master_mkFile(rt_uint8_t file_index, rt_uint8_t *name);

rt_uint8_t sd_master_write(rt_uint8_t *buff, rt_uint8_t size, rt_uint8_t file_index, rt_uint8_t pack_type);
void sd_master_init(rt_uint8_t priority);

// 使用示例：
/* 
int main()
{
    // 上电延时 100ms
    // 这个时间是给 SD 卡启动用的

    // 初始化 SD 驱动
    sd_master_init(prio);  // prio 为 SD 线程优先级，请低于所有重要线程

// 推荐统一创建文件
// 文本型记录示例：
    // 创建一个文件
    sd_master_mkFile(f_index, "file");

    // 写入指定文件
    sd_master_wFile(buff, size, f_index);
    // 以 printf 格式写入指定文件（推荐）
    sd_master_printf(f_index, "%.4f %s %-6.d", 5.14f, "Hello World! ", 666);

// Vofa 图线型记录示例：
    // 创建一个 .csv 文件
    sd_master_vofa_create(f_index, "file.csv", 2, "Accl", "Gyro");

    // 写入 .csv 文件
    sd_master_vofa_write(f_index, 2, accl_data, gyro_data);
}
 */

#endif
