#include <rtthread.h>
#include <rthw.h>

#define CPU_USAGE_CALC_TICK 15
#define CPU_USAGE_LOOP 100

volatile float CPU_Usage;

static rt_uint64_t total_count = 0;

static volatile rt_uint64_t count;
static void cpu_usage_idle_hook(void)
{
    rt_tick_t tick;
    volatile rt_uint32_t loop;

    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();
        tick = rt_tick_get();
        while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
        {
            total_count++;
            loop = 0;

            while (loop < CPU_USAGE_LOOP)
                loop++;
        }
        rt_exit_critical();
    }

    count = 0;
    /* get CPU usage */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count++;
        loop = 0;
        while (loop < CPU_USAGE_LOOP)
            loop++;
    }

    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;
    }
    else
    {
        total_count = count;
    }
    CPU_Usage = (count * 100) / (float)total_count;
}

int cpu_usage_init(void)
{
    /* set idle thread hook */
    rt_thread_idle_sethook(cpu_usage_idle_hook);
    return 0;
}
INIT_APP_EXPORT(cpu_usage_init);
