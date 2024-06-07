#ifndef __DRV_CRC_H__
#define __DRV_CRC_H__
#include <rtthread.h>

extern const rt_uint8_t CRC8_INIT;
extern const rt_uint16_t CRC_INIT;
extern const rt_uint8_t CRC8_TAB[256];
extern const rt_uint16_t wCRC_Table[256];

/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
rt_uint8_t Get_CRC8_Check_Sum(rt_uint8_t *pchMessage, rt_uint32_t dwLength, rt_uint8_t ucCRC8);

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
rt_uint32_t Verify_CRC8_Check_Sum(rt_uint8_t *pchMessage, rt_uint32_t dwLength);

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(rt_uint8_t *pchMessage, rt_uint32_t dwLength);

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
rt_uint16_t Get_CRC16_Check_Sum(rt_uint8_t * pchMessage, rt_uint32_t dwLength, rt_uint16_t wCRC);

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
rt_uint32_t Verify_CRC16_Check_Sum(rt_uint8_t *pchMessage, rt_uint32_t dwLength);

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(rt_uint8_t *pchMessage, rt_uint32_t dwLength);

#endif 
