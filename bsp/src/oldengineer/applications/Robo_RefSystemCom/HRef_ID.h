#ifndef __HREF_ID_H__
#define __HREF_ID_H__
#include "rtconfig.h"
#include <rtthread.h>

typedef enum
{
    //红方机器人
    REF_ROBO_R1_HERO = 1,
    REF_ROBO_R2_ENGINEER = 2,
    REF_ROBO_R3_STANDARD = 3,
    REF_ROBO_R4_STANDARD = 4,
    REF_ROBO_R5_STANDARD = 5,
    REF_ROBO_R6_AERIAL = 6,
    REF_ROBO_R7_SENTRY = 7,
    REF_ROBO_R8_DART = 8,
    REF_ROBO_R9_RADAR = 9,
    //蓝方机器人
    REF_ROBO_B1_HERO = 101,
    REF_ROBO_B2_ENGINEER = 102,
    REF_ROBO_B3_STANDARD = 103,
    REF_ROBO_B4_STANDARD = 104,
    REF_ROBO_B5_STANDARD = 105,
    REF_ROBO_B6_AERIAL = 106,
    REF_ROBO_B7_SENTRY = 107,
    REF_ROBO_B8_DART = 108,
    REF_ROBO_B9_RADAR = 109,
    //红方客户端
    REF_CLIENT_R1_HERO = 0x101,
    REF_CLIENT_R2_ENGINEER = 0x0102,
    REF_CLIENT_R3_STANDARD = 0x0103,
    REF_CLIENT_R4_STANDARD = 0x0104,
    REF_CLIENT_R5_STANDARD = 0x0105,
    REF_CLIENT_R6_AERIAL = 0x0106,
    //蓝方客户端
    REF_CLIENT_B1_HERO = 0x0165,
    REF_CLIENT_B2_ENGINEER = 0x0166,
    REF_CLIENT_B3_STANDARD = 0x0167,
    REF_CLIENT_B4_STANDARD = 0x0168,
    REF_CLIENT_B5_STANDARD = 0x0169,
    REF_CLIENT_B6_AERIAL = 0x016A,

} Ref_ROBO_CLIENT_ID_e;

extern rt_uint16_t REF_ROBO_ID;   //该机器人ID
extern rt_uint16_t REF_CLIENT_ID; //该客户端ID


#endif
