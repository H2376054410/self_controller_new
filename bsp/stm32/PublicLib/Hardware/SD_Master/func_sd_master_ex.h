#ifndef __FUNC_SD_MASTER_EX_H__
#define __FUNC_SD_MASTER_EX_H__

#include <rtdef.h>

// * 此文件已经包含在 func_sd_master.h 中，无需单独包含

rt_int32_t sd_master_printf(rt_uint8_t file_index, const char *fmt, ...);
rt_uint16_t sd_master_vofa_create(rt_uint8_t file_index, rt_uint8_t* name, rt_uint8_t cnt, ...);
rt_uint16_t sd_master_vofa_write(rt_uint8_t file_index, rt_uint8_t cnt, ...);
void sd_master_ex_init(void);

#endif
