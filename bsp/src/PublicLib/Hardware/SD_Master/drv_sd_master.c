#include <rtthread.h>
#include <board.h>
#include "drv_spi.h"
#include <rtconfig.h>

#define CONVERT_GET_PIN(x, n) (16*(x-'A') + n)
#define GIT_PVT CONVERT_GET_PIN(*SD_MASTER_SPI_NSS_PORT, SD_MASTER_SPI_NSS_PIN)

#define __GPIO_PORT(x) (shit[*x - 'A'])
#define __GPIO_PIN(n)  (GPIO_PIN_0 << n)

struct rt_spi_device *master_spi;
static GPIO_TypeDef *(shit[8]) = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOE, GPIOH};

//========================================================================
// 用户接口
//========================================================================

void sd_master_send_func(rt_uint8_t *buff, rt_uint16_t size)
{
    rt_size_t len;

    len = size;
    rt_spi_send(master_spi, buff, len);
}

//========================================================================
// 初始化
//========================================================================

static rt_err_t sd_master_spi_init(void)
{
    rt_err_t res = RT_NULL;

    /* 初始化 CS，一定要最先进行 */
    rt_pin_mode(GIT_PVT, PIN_MODE_OUTPUT);
    rt_pin_write(GIT_PVT, PIN_HIGH);

    /* 将 CS 挂载到 SPI 设备 */
    res |= rt_hw_spi_device_attach(SD_MASTER_SPI_BUS, SD_MASTER_DEV_NAME,
                                    __GPIO_PORT(SD_MASTER_SPI_NSS_PORT),
                                    __GPIO_PIN(SD_MASTER_SPI_NSS_PIN));

    /* 查找 SPI 设备 */
    master_spi = (struct rt_spi_device *)rt_device_find(SD_MASTER_DEV_NAME);
    if (!master_spi)
    {
        rt_kprintf("spi run failed! can't find %s device\n", SD_MASTER_SPI_BUS);
        /* 为稳定运行，不要加这种东西 */
        // while (1)
        //     ;
        res |= RT_ERROR;
        return res;
    }

    /* config spi */
    struct rt_spi_configuration spi_config;
    spi_config.mode = RT_SPI_MSB | RT_SPI_MASTER | RT_SPI_MODE_0;
    spi_config.data_width = 8;
    spi_config.max_hz = 5e6;  // 5M
    res |= rt_spi_configure(master_spi, &spi_config);

    return res;
}

void sd_master_drv_init(void)
{
    /* 驱动初始化 */
    sd_master_spi_init();
}
