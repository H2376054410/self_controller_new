#include "func_sd_master_ex.h"
#include <rtthread.h>
#include "func_sd_master.h"

#define ZEROPAD (1 << 0) /* pad with zero */
#define SIGN (1 << 1)    /* unsigned/signed long */
#define PLUS (1 << 2)    /* show plus */
#define SPACE (1 << 3)   /* space if plus */
#define LEFT (1 << 4)    /* left justified */
#define SPECIAL (1 << 5) /* 0x */
#define LARGE (1 << 6)   /* use 'ABCDEF' instead of 'abcdef' */

//========================================================================
// print support
//========================================================================
/* private function */

#define isdigit_private(c) ((unsigned)((c) - '0') < 10)

rt_inline int skip_atoi_private(const char **s)
{
    register int i = 0;
    while (isdigit_private(**s))
        i = i * 10 + *((*s)++) - '0'; // 使用二级指针可以修改一级指针的值

    return i;
}

// 返回接下来的数字字符长度
rt_inline int skip_with_len(const char *s)
{
    register int i = 0;
    while (isdigit_private(*(s++)))
        i++;

    return i;
}

/**
 * @brief 将变量内存直接打印到 buf
 * 
 * @param buf 
 * @param end 
 * @param num 
 * @param len 
 * @return char* 
 * 
 * @note 格式化由从机进行
 */
static char *print_mem(char *buf,
                       char *end,
                       long num,
                       int len)
{
    int len_rec;

    // 超出缓冲区则不写入
    if (buf + len <= end)
    {
        len_rec = len;
        while (len--)
        {
            buf[len] = ((rt_uint8_t *)&num)[len];
        }
        buf += len_rec;
    }

    return buf;
}

static rt_int32_t sd_log_vsnprintf(char *buf,
                            rt_size_t size,
                            const char *fmt,
                            va_list args)
{
    rt_uint32_t num;

    int i, len;
    char *str, *end, c;
    const char *s;

    // rt_uint8_t base;        /* the base of number */
    rt_uint8_t flags;       /* flags to print number */
    rt_uint8_t qualifier;   /* 'h', 'l', or 'L' for integer fields */
    rt_int32_t field_width; /* width of output field */  // 字段宽度

    int precision;      /* min. # of digits for integers and max for a string */

    char *str_temp;  // 用于 %c %s 时不打印控制字符
    int index_temp;  // 用于保留控制符中的数字字符

    str = buf;
    end = buf + size;

    /* Make sure end is always >= buf */
    if (end < buf)
    {
        end = ((char *)-1);
        size = end - buf;
    }

    for (; *fmt; ++fmt)
    {
        if (*fmt != '%')
        {
            if (str < end)
                *str = *fmt;
            ++str;
            continue;
        }

        /* process flags */
        flags = 0;

        // * NEW 将 '%' 复制
        str_temp = str;
        if (str < end) *str = *fmt;
        ++ str;

        while (1)
        {
            /* skips the first '%' also */
            ++fmt;
            if (*fmt == '-')
                flags |= LEFT;
            else if (*fmt == '+')
                flags |= PLUS;
            else if (*fmt == ' ')
                flags |= SPACE;
            else if (*fmt == '#')
                flags |= SPECIAL;
            else if (*fmt == '0')
                flags |= ZEROPAD;
            else
                break;
            
            // * NEW 将每个控制字符复制
            if (str < end) *str = *fmt;
            ++ str;
        }

        /* get field width */
        // 保留这段，只是为了跳过控制段
        field_width = -1;
        if (isdigit_private(*fmt))
        {
            // * NEW 将 field_width 部分数字字符复制
            index_temp = skip_with_len(fmt);
            while (index_temp--)
            {
                if (str < end) *str = *fmt;
                ++ str;
            }

            field_width = skip_atoi_private(&fmt);
        }
        // else if (*fmt == '*')
        // {
        //     ++fmt;
        //     /* it's the next argument */
        //     field_width = va_arg(args, int);  // '*' 表示字段宽度由一个可变参数决定，这个参数规定为 int
        //     if (field_width < 0)
        //     {
        //         field_width = -field_width;  // 带 '-' 相当于 %-，即左对齐
        //         flags |= LEFT;
        //     }
        // }

        /* get the precision */
        // 保留这段，只是为了跳过控制段
        precision = -1;
        if (*fmt == '.')
        {
            // * NEW 将 '.' 复制
            if (str < end) *str = *fmt;
            ++ str;

            ++fmt;
            if (isdigit_private(*fmt))
            {
                // * NEW 将 field_width 部分数字字符复制
                index_temp = skip_with_len(fmt);
                while (index_temp--)
                {
                    if (str < end) *str = *fmt;
                    ++ str;
                }

                precision = skip_atoi_private(&fmt);
            }
            // else if (*fmt == '*')
            // {
            //     ++fmt;
            //     /* it's the next argument */
            //     precision = va_arg(args, int);
            // }
            // if (precision < 0)
            //     precision = 0;
        }

        /* get the conversion qualifier */
        // 保留这段，只是为了跳过控制段
        qualifier = 0;
        if (*fmt == 'h' || *fmt == 'l')
        {
            // * NEW 将 'h' or  'l' 复制
            if (str < end) *str = *fmt;
            ++str;

            qualifier = *fmt;
            ++fmt;
        }

        /* the default base */
        // base = 10;

        switch (*fmt)
        {
        case 'c':
            if (!(flags & LEFT))
            {
                while (--field_width > 0)
                {
                    if (str < end) *str = ' ';
                    ++ str;
                }
            }

            // * NEW %c 不保留控制字符
            str = str_temp;

            /* get character */
            c = (rt_uint8_t)va_arg(args, int);
            if (str < end) *str = c;
            ++ str;

            /* put width */
            while (--field_width > 0)
            {
                if (str < end) *str = ' ';
                ++ str;
            }
            continue;

        case 's':

            // * NEW %s 不保留控制字符
            str = str_temp;

            s = va_arg(args, char *);
            if (!s) s = "(NULL)";

            len = rt_strlen(s);

            if (precision > 0 && len > precision) len = precision;

            // 仅保留字段宽度内的字符，减少发送
            if (!(flags & LEFT))
            {
                while (len < field_width--)
                {
                    if (str < end) *str = ' ';
                    ++ str;
                }
            }

            for (i = 0; i < len; ++i)
            {
                if (str < end) *str = *s;
                ++ str;
                ++ s;
            }

            while (len < field_width--)
            {
                if (str < end) *str = ' ';
                ++ str;
            }
            continue;

        case 'p':
            // if (field_width == -1)
            // {
            //     field_width = sizeof(void *) << 1;
            //     flags |= ZEROPAD;
            // }
            str = print_mem(str, end, (long)va_arg(args, void *), sizeof(void *));
            continue;

        case '%':
            if (str < end) *str = '%';
            ++ str;
            continue;

        /* integer number formats - set up the flags and "break" */
        case 'o':
            // base = 8;
            // break;

        case 'X':
            // flags |= LARGE;
        case 'x':
            // base = 16;
            // break;

        case 'd':
        case 'i':
            // flags |= SIGN;
        case 'u':

            // * 将这些复制进去
            if (str < end) *str = *fmt;
            ++ str;

            break;

        case 'f':
            if (str < end) *str = *fmt;
            ++ str;
            float f;
            f = va_arg(args, double);
            if (str + sizeof(float) <= end)
            {
                *(str++) = *(((rt_uint8_t *)&f) + 0);
                *(str++) = *(((rt_uint8_t *)&f) + 1);
                *(str++) = *(((rt_uint8_t *)&f) + 2);
                *(str++) = *(((rt_uint8_t *)&f) + 3);
            }
            continue;

        default:
            // * 已经在最开始填入 '%' 了
            // if (str < end) *str = '%';
            // ++ str;

            if (*fmt)
            {
                if (str < end) *str = *fmt;
                ++ str;
            }
            else
            {
                -- fmt;
            }
            continue;
        }

        if (qualifier == 'l')
        {
            num = va_arg(args, rt_uint32_t);
            if (flags & SIGN) num = (rt_int32_t)num;
        }
        else if (qualifier == 'h')
        {
            num = (rt_uint16_t)va_arg(args, rt_int32_t);
            if (flags & SIGN) num = (rt_int16_t)num;
        }
        else
        {
            num = va_arg(args, rt_uint32_t);
            if (flags & SIGN) num = (rt_int32_t)num;
        }
        str = print_mem(str, end, num, sizeof(rt_uint32_t));
    }

    if (size > 0)
    {
        if (str < end) *str = '\0';
        else
        {
            end[-1] = '\0';
        }
    }

    /* the trailing null byte doesn't count towards the total
    * ++str;
    */
    return str - buf;
}

//========================================================================
// print support
//========================================================================

/**
 * @brief Vofa+ 创建 .csv 支持函数
 * 
 * @param buf 缓冲区，存储格式化后数据
 * @param sz 缓冲区大小
 * @param cnt str 个数
 * @param args 可变参数
 * @return rt_uint16_t 字符串长度
 */
static rt_uint16_t sd_log_vnvofa_create(rt_uint8_t* buf, rt_uint8_t sz, rt_uint8_t cnt, va_list args)
{
    rt_uint8_t len, sz_l;  // len: 单字符串长度  sz_l: 已写入长度
    char *s;

    sz_l = 0;

    while (cnt--)
    {
        // 获取字符串
        s = va_arg(args, char *);
        if (!s) s = "(NULL)";
        len = rt_strlen(s);

        // 空间不足
        if (sz - sz_l < len)
        {
            *(buf + sz - 1) = '\n';  // 强行结尾
            break;
        }

        // 空间足够，拷贝字符串
        rt_memcpy(&buf[sz_l], s, len);
        sz_l += len;

        // 如果仍有空间
        if (sz > sz_l)
        {
            // 添加 .csv 格式字符
            if (cnt == 0)
                buf[sz_l] = '\n';
            else
                buf[sz_l] = ',';
            
            sz_l++;

            // 空间不足
            if (sz <= sz_l)
            {
                *(buf + sz - 1) = '\n';  // 强行结尾
                break;
            }
        }
    }

    return sz_l;
}

/**
 * @brief Vofa+ 写入数据支持函数
 * 
 * @param buf 缓冲区，存储格式化后数据
 * @param sz 缓冲区大小
 * @param cnt float 个数
 * @param args 可变参数
 * @return rt_uint16_t 传输的字节数（即 float 个数 * 4）
 * 
 * @note  将 float 直接打印内存到缓冲区，解析由从机进行
 */
static rt_uint16_t sd_log_vnvofa_write(rt_uint8_t* buf, rt_uint8_t sz, rt_uint8_t cnt, va_list args)
{
    rt_uint8_t n;
    float f;

    n = 0;

    if (sz < cnt * 4)  // 若存不下全部 float
    {
        if (sz >= 4)  // 至少能存下一个 float
        {
            cnt = sz / 4;
        }
        else
        {
            return 0;
        }
    }

    while (cnt--)
    {
        f = va_arg(args, double);
        rt_memcpy(&buf[n * 4], &f, 4);
        n++;
    }

    return n * 4;
}

//========================================================================
// 处理
//========================================================================

static rt_uint8_t print_buffer[SD_PRINT_BUFF_CNT][SD_PRINT_BUFF_SIZE];  // 缓冲区
static rt_uint8_t print_buff_is_used[SD_PRINT_BUFF_CNT] = {0}, print_buff_top = 0;  // 是否占用
static rt_sem_t print_block_sem = RT_NULL;

/**
 * @brief 试图获取 printf 缓冲区
 * 
 * @return rt_uint8_t*
 * RT_NULL: 失败  other: 成功，缓冲区地址
 * @note
 * 用完记得释放缓冲区
 */
static rt_uint8_t *sd_master_try_get_buffer(void)
{
    rt_uint8_t *buff;

    buff = RT_NULL;

    // 如果一直等不到调度就返回错误
    if (RT_EOK != rt_sem_take(print_block_sem, 3))
        return RT_NULL;

    rt_uint8_t top = print_buff_top;
    do
    {
        ++ print_buff_top;
        if (print_buff_top == SD_PRINT_BUFF_CNT)
            print_buff_top = 0;
    } while (print_buff_is_used[print_buff_top] != 0 && top != print_buff_top);
    if (top != print_buff_top)
    {
        print_buff_is_used[print_buff_top] = 1;
        buff = print_buffer[print_buff_top];
    }

    rt_sem_release(print_block_sem);
    return buff;
}

static void sd_master_release_buffer(rt_uint8_t* buf)
{
    print_buff_is_used[((rt_uint32_t)buf - (rt_uint32_t)print_buffer) / SD_PRINT_BUFF_SIZE] = 0;
}

//========================================================================
// 用户接口
//========================================================================

/**
 * @brief 写入文件（以 printf 方式）
 * 
 * @param file_index 文件编号
 * @param fmt 格式
 * @param ... 可变参数
 * @return rt_int32_t 写入字节数
 * 
 * @note  1. 请提前创建好文件
 *        2. 支持 printf 的大部分格式控制符
 */
rt_int32_t sd_master_printf(rt_uint8_t file_index, const char *fmt, ...)
{
    rt_int32_t n;
    va_list args;

    rt_uint8_t *buf;
    // 获取缓冲区
    buf = sd_master_try_get_buffer();
    if (buf == RT_NULL)
        return 0;

    // 格式化
    va_start(args, fmt);
    n = sd_log_vsnprintf((char *)buf, SD_PRINT_BUFF_SIZE, fmt, args);
    va_end(args);

    // 带一个 '\0'
    n++;

    // 写入发送
    sd_master_write(buf, n, file_index, SD_LOG_IS_PRINT);

    // 释放缓冲区
    sd_master_release_buffer(buf);

    return n;
}

/**
 * @brief 创建一个图线型文件
 * 
 * @param file_index 文件编号
 * @param name 文件名
 * @param cnt 数据组数
 * @param ... cnt 个字符串，分别代表对应组的名称
 * @return rt_uint16_t 字符串长度
 * 
 * @note  1. 图线型文件不用 sd_master_mkFile 创建
 *        2. 使用 Vofa+ 导入查看图线
 */
rt_uint16_t sd_master_vofa_create(rt_uint8_t file_index, rt_uint8_t* name, rt_uint8_t cnt, ...)
{
    rt_int32_t n;
    va_list args;

    rt_uint8_t *buf;
    // 获取缓冲区
    buf = sd_master_try_get_buffer();
    if (buf == RT_NULL)
        return 0;

    // 格式化
    va_start(args, cnt);
    n = sd_log_vnvofa_create(buf, SD_PRINT_BUFF_SIZE, cnt, args);
    va_end(args);

    // 创建文件
    sd_master_mkFile(file_index, name);
    // 写入发送
    sd_master_write(buf, n, file_index, SD_LOG_IS_VOFA_CREATE);

    // 释放缓冲区
    sd_master_release_buffer(buf);

    return n;
}

/**
 * @brief 写入图线型数据
 * 
 * @param file_index 文件编号
 * @param cnt 数据组数
 * @param ... cnt 个 float 类型数据
 * @return rt_uint16_t 写入的字节数（ r/4 就是写入的 float 数量）
 * 
 * @note  1. 使用 Vofa+ 导入查看图线
 *        2. 请提前创建文件
 *        3. 数据的顺序请与各组名称对应
 */
rt_uint16_t sd_master_vofa_write(rt_uint8_t file_index, rt_uint8_t cnt, ...)
{
    rt_int32_t n;
    va_list args;

    rt_uint8_t *buf;
    // 获取缓冲区
    buf = sd_master_try_get_buffer();
    if (buf == RT_NULL)
        return 0;

    // 格式化
    va_start(args, cnt);
    n = sd_log_vnvofa_write(buf, SD_PRINT_BUFF_SIZE, cnt, args);
    va_end(args);

    // 写入发送
    sd_master_write(buf, n, file_index, SD_LOG_IS_VOFA_WRITE);

    // 释放缓冲区
    sd_master_release_buffer(buf);

    return n;
}

//========================================================================
// 初始化
//========================================================================

/**
 * @brief SD 主机额外功能初始化
 * 
 * @note  内部使用，用户无需再次调用
 */
void sd_master_ex_init(void)
{
    print_block_sem = rt_sem_create("print", 1, RT_IPC_FLAG_PRIO);
}
