#include "func_sd_master.h"
#include "sd_loop_queue.h"
#include "drv_sd_master.h"
#include "func_sd_master_ex.h"
#include <rtthread.h>
#include <rthw.h>

typedef struct
{
    LoopQueue_Ctrl_Type loop_queue; // 用于管理的环形队列
    rt_uint8_t *buff;               // 记录缓冲区地址，和 loop_queue 里记录的一样
} SD_Master_Ctrl_Type;

//========================================================================

//========================================================================

// sd 发送缓冲区
ALIGN(RT_ALIGN_SIZE)
rt_uint8_t sd_buff[SD_MASTER_BUFF_SIZE];

SD_Master_Ctrl_Type sd_master_ctrl;

// 信号量，配合 idle_flag 更新缓冲区的有效数据
struct rt_semaphore sd_master_sem;
// 是否空闲（没有正在发送）
rt_uint8_t sd_idle_flag = 1;

rt_size_t sd_slave_init_delay_tick = 0;

// 发送回调函数
static void (*sd_send)(rt_uint8_t *buff, rt_uint16_t size) = RT_NULL;

//========================================================================

//========================================================================

// 只占位不写
// return   lq->size：失败（队列空间不足） 其它：成功，返回写入开始的队列下标
/* <ATOMIC> */ static rt_uint32_t lq_takeSpace(LoopQueue_Ctrl_Type *lq, rt_uint32_t size)
{
    rt_uint32_t tail;

    // 特判：size == 0 表示获取当前尾指针
    if (size == 0)
        return lq->tail;

    if (lq_esz(lq) < size)
        return lq->size;

    tail = lq->tail;

    if (lq->size >= lq->tail + size)
    {
        // rt_memcpy(&lq->buff[lq->tail], buff, size);

        lq->tail = (lq->tail + size) % lq->size;
    }
    else
    {
        // rt_memcpy(&lq->buff[lq->tail], buff, lq->size - lq->tail);
        // rt_memcpy(&lq->buff[0], buff + lq->size - lq->tail, size + lq->tail - lq->size);

        lq->tail = size + lq->tail - lq->size;
    }

    // lq_pushJudgeFull(lq);
    if (lq->head == lq->tail)
        lq->status = 2;
    else
        lq->status = 0;

    return tail;
}

// 调整 index 位置，配合 lq_takeSpace() 和报文的各段大小
// return 调整后 index 队列写入开始的下标
static inline rt_uint32_t lq_adjustIndex(rt_uint32_t lq_size, rt_uint32_t index, rt_uint32_t data_size)
{
    if (lq_size >= index + data_size)
    {
        return (index + data_size) % lq_size;
    }
    else
    {
        return data_size + index - lq_size;
    }
}

// 配合 lq_takeSpace() 对占位写入
// param    lq：队列 index：lq_takeSpace() 成功时的返回值 src：源地址 size：写入长度
static inline void lq_writeSpace(LoopQueue_Ctrl_Type *lq, rt_uint32_t index, rt_uint8_t *src, rt_uint32_t size)
{
    if (lq->size >= index + size)
    { // 未跨圈
        rt_memcpy(&lq->buff[index], src, size);
    }
    else
    {
        rt_memcpy(&lq->buff[index], src, lq->size - index);
        rt_memcpy(&lq->buff[0], src + lq->size - index, size + index - lq->size);
    }
}

//========================================================================
// 处理
//========================================================================

/**
 * @brief 主机实际发送函数
 *
 * @return
 * 实际发送长度
 */
static rt_uint32_t sd_master_send(void)
{
    rt_uint8_t isFull;
    rt_uint32_t head_index, tail_index, length;
    rt_uint32_t send_size, send_size_temp;

    // head_index isFull 注意顺序
    head_index = sd_master_ctrl.loop_queue.head; // 保存快照-头指针
    isFull = sd_master_ctrl.loop_queue.status;   // 保存快照-空满
    tail_index = sd_master_ctrl.loop_queue.tail; // 保存快照-尾指针
    length = sd_master_ctrl.loop_queue.size;     // 保存快照-lq 长度（一次就够，无序）

    // 空直接退出
    if (isFull == 1)
        return 0;

    // 跨圈特殊处理
    // 存在跨圈则分两次发送，从机有对不完整包的兼容
    if (isFull == 2 || head_index > tail_index)
    {
        send_size = length - head_index;
        send_size_temp = tail_index - 0;
    }
    else
    {
        send_size = tail_index - head_index;
        send_size_temp = 0;
    }

    // 等待发送完成
    if (send_size != 0)
        sd_send(&sd_master_ctrl.buff[head_index], send_size);
    if (send_size_temp != 0)
        sd_send(&sd_master_ctrl.buff[0], send_size_temp);

    send_size += send_size_temp;

    /* 返回实际发送长度 */
    return send_size;
}

/**
 * @brief 封装数据包到缓冲区
 *
 * @param src 数据源地址
 *
 * @param cnt 数据长度 Bytes
 *
 * @param time 当前时刻
 * 0: 这是个文件包  !0: 这是个数据包
 * @param file_index 文件编号
 *
 * @return
 * 0: 封包失败 cnt: 封包的数据段长度，允许为 0，不当作失败处理
 */
static rt_uint32_t sd_write_pack(rt_uint8_t *src, rt_uint8_t cnt, rt_uint32_t time, rt_uint8_t file_index)
{
    rt_base_t level;
    rt_uint8_t head[4] = {0}, tail[4] = {0};
    rt_uint8_t reserved;        // 用于对齐
    rt_uint32_t index, lq_size; // 用于写入队列

    lq_size = sd_master_ctrl.loop_queue.size;

    // 填充头尾帧
    head[0] = head[3] = 0x3C;
    head[1] = head[2] = cnt;

    tail[0] = 0x80;
    tail[3] = 0x7F;
    tail[1] = tail[2] = file_index;

    // 失败：空间不足
    if (lq_esz(&sd_master_ctrl.loop_queue) <= cnt + 12)
        return 0;

    // 获取空间，调整 lq.tail，包含 head + time + data [+ reserved] + tail
    // 多线程情况，先占后写
    // 由于 SD 线程优先级最低的设定，一定会在发送前完成写入
    // 原子的
    level = rt_hw_interrupt_disable();
    reserved = cnt % 4;
    if (reserved)
        // 这种写法保证队列始终 4 字节对齐（前提：缓冲区 4 字节对齐）
        index = lq_takeSpace(&sd_master_ctrl.loop_queue, 16 + cnt - reserved);
    else
        index = lq_takeSpace(&sd_master_ctrl.loop_queue, 12 + cnt);
    rt_hw_interrupt_enable(level);

    // 头帧 + cnt
    lq_writeSpace(&sd_master_ctrl.loop_queue, index, (rt_uint8_t *)head, 4);
    index = lq_adjustIndex(lq_size, index, 4);
    // 时间戳
    lq_writeSpace(&sd_master_ctrl.loop_queue, index, (rt_uint8_t *)&time, 4);
    index = lq_adjustIndex(lq_size, index, 4);
    // 数据帧
    if (cnt % 4 == 0)
    {
        lq_writeSpace(&sd_master_ctrl.loop_queue, index, src, cnt);
        index = lq_adjustIndex(lq_size, index, cnt);
    }
    else
    {
        lq_writeSpace(&sd_master_ctrl.loop_queue, index, src, cnt);
        index = lq_adjustIndex(lq_size, index, cnt + (4 - cnt % 4));
    }
    // 尾帧 + file_index
    lq_writeSpace(&sd_master_ctrl.loop_queue, index, (rt_uint8_t *)tail, 4);

    return cnt;

    /*
        struct pack
        {
            rt_uint8_t head[4];
            rt_uint32_t time;
            rt_uint8_t data[size];  // 由发送数据决定
            rt_uint8_t reserved[(0) || (4 - size % 4)];  // 4 字节对齐
            rt_uint8_t tail[4];
        };  // sizeof(pack) == 4 + 4 + size + 4 + reserved == 8 + size + reserved + 4
     */
}

/**
 * @brief SD 线程更新有效数据
 *
 */
static void sd_master_discard(rt_uint32_t send_size)
{
    lq_discardString(&sd_master_ctrl.loop_queue, send_size);
}

// 注册发送函数
static void sd_master_attach_send(void (*send_func)(rt_uint8_t *buff, rt_uint16_t size))
{
    sd_send = send_func;
}

//========================================================================
// 回调
//========================================================================

ALIGN(RT_ALIGN_SIZE)
static char sd_thread_stack[THREAD_STACK_SD_MASTER];
static struct rt_thread sd_thread;
static void sd_master_send_entry(void *parameter)
{
    static rt_uint32_t send_size;

    /* 从机从上电到可用的延时 */
    // 加这些东西是有原因的
    while (SD_SLAVE_GOOD_DELAY > rt_tick_get())
        ;

    /* 等待 SD 从机内核重启 */
    // 记录时间
    sd_slave_init_delay_tick = rt_tick_get();
    while (SD_SLAVE_INIT_DELAY > rt_tick_get() - sd_slave_init_delay_tick)
        ;

    while (1)
    {
        rt_sem_take(&sd_master_sem, RT_WAITING_FOREVER);

        // 开始发送
        send_size = sd_master_send();
        sd_master_discard(send_size);

        // 发送完成，进入空闲
        sd_idle_flag = 1;
    }
}

//========================================================================
// 用户接口
//========================================================================

/**
 * @brief 【内部使用】主机写函数
 *
 * @param buff 源地址
 *
 * @param size 数据长度
 *
 * @param file_index 文件编号
 * 
 * @param pack_type 数据包类型
 * 
 * @return
 * 0: 封包失败  cnt: 封包的数据段长度，允许为 0，也会发送，不当作失败处理，需要用户特判
 * 
 * @note  1. 该函数是所有写文件操作的基础
 *        2. 不同文件类型通过 pack_type 区分
 *        3. 请提前创建好文件
 */
rt_uint8_t sd_master_write(rt_uint8_t *buff, rt_uint8_t size, rt_uint8_t file_index, rt_uint8_t pack_type)
{
    rt_uint32_t time;
    rt_uint8_t cnt;

    // 获取时间戳
    time = rt_tick_get();

    // 确定包格式
    ((rt_uint8_t *)&time)[3] = pack_type;

    // 封包
    cnt = sd_write_pack(buff, size, time, file_index);

    // 没有数据正在发送
    if (sd_idle_flag == 1)
    {
        // 退出空闲
        sd_idle_flag = 0;
        rt_sem_release(&sd_master_sem);
    }

    return cnt;

    /*
        <ATOMIC> index = buffer_get_index(buff_free);  // 获取一块 free 下标
        rt_memcpy(buffer[index].data, buff, size);  // 向 buffer 的 data 段写入 size 长度的数据
     */
}

/**
 * @brief 写入文件
 * 
 * @param file_index 文件编号
 * @param buff 源地址
 * @param size 数据长度
 * @return rt_uint8_t 0: 写入失败或写入 0 个字节  cnt: 写入 cnt 个字节
 * 
 * @note  该函数会将 buff 的内容原封不动地写入文件
 */
rt_uint8_t sd_master_wFile(rt_uint8_t file_index, rt_uint8_t* buff, rt_uint8_t size)
{
    return sd_master_write(buff, size, file_index, SD_LOG_IS_WRITE);
}

/**
 * @brief 创建文件
 * 
 * @param file_index 文件编号，之后访问该文件都通过这个编号
 * @param name 文件名
 * 
 * @note  1. 建议在其它记录开始前先创建文件
 *        2. 当前支持范围 0-15
 *        3. 每个文件编号只可被创建一次，重复创建将被忽视
 *        4. 对未创建的文件写入将统一存入 INVALID 文件
 */
void sd_master_mkFile(rt_uint8_t file_index, rt_uint8_t *name)
{
    rt_uint8_t size;

    size = rt_strlen((char *)name);

    rt_uint32_t time = rt_tick_get();
    ((rt_uint8_t *)&time)[3] = SD_LOG_IS_MAKE;
    sd_write_pack(name, size, 0, file_index);

    // 没有数据正在发送
    if (sd_idle_flag == 1)
    {
        // 退出空闲
        sd_idle_flag = 0;
        rt_sem_release(&sd_master_sem);
    }
}

//========================================================================
// 初始化
//========================================================================

static void sd_master_handle_init(rt_uint8_t priority)
{
    sd_master_attach_send(sd_master_send_func);

    rt_memset(&sd_master_ctrl, 0, sizeof(SD_Master_Ctrl_Type));
    sd_master_ctrl.buff = sd_buff;
    rt_memset(sd_master_ctrl.buff, 0, SD_MASTER_BUFF_SIZE);
    lq_init(&sd_master_ctrl.loop_queue, SD_MASTER_BUFF_SIZE, sd_buff);

    rt_sem_init(&sd_master_sem, "sd_master_sem", 0, RT_IPC_FLAG_FIFO);

    /* 初始化 SD 线程 */
    rt_thread_init(&sd_thread,
                   "sd_th",
                   sd_master_send_entry,
                   RT_NULL,
                   &sd_thread_stack[0],
                   sizeof(sd_thread_stack),
                   priority, THREAD_TICK_SD_MASTER);
    rt_thread_startup(&sd_thread);
}

/**
 * @brief SD 主机初始化
 * 
 * @param priority SD 线程优先级，请低于所有重要线程
 * 
 * @note  1. 硬件配置通过 menuconfig 修改，需要与从机的连接一致
 *        2. 建议在上电至少 100ms 后开始记录，SD 卡初始化需要时间
 */
void sd_master_init(rt_uint8_t priority)
{
    /* 驱动 */
    sd_master_drv_init();

    /* 拓展 */
    sd_master_ex_init();

    /* 主功能 */
    sd_master_handle_init(priority);

    // 防止 SD 卡未初始化
    sd_master_write("dummy", sizeof("dummy"), 0, SD_LOG_IS_WRITE);
}
