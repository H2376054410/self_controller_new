#ifndef __APP_ARM_H__
#define __APP_ARM_H__
#include "drv_thread.h"
#include "drv_SetPlanning3D.h"
#include "rtdef.h"

#define POS_ERRORMAX 0.05f
#define POS_TOLERANCE 0.00001f
#define SPE_TOLERANCE 0.00005f
#define SPEEDMAX_R 0.6f
#define SPEEDMAX_T 0.6f
#define ACCLMAX_R 1.2f
#define ACCLMAX_T 1.2f
#define SETPLANNING_PERIOD (ROBOCTRL_TIMER_PIRIOD / 1000.0)

typedef struct
{
    rt_uint8_t Arm_IfValid : 1;            // 机械臂通信是否有效
    rt_uint8_t Chassis_IfValid : 1;        // 底盘通信是否有效
    rt_uint8_t Image_IfValid : 1;          // 图传通信是否有效
    rt_uint8_t BoomYawEncoder_IfValid : 1; // 编码器通信是否有效
    rt_uint8_t GypeScope_IfValid : 1;      // 陀螺仪通信是否有效
    rt_uint8_t ImageEncoder_IfValid : 1;   // 陀螺仪通信是否有效
} DataValid_s;

/**
 * @brief 机械臂控制线程初始化
 * @return rt_err_t
 */
extern rt_err_t ArmCtrl_Thread_Init(void);

#endif /*__APP_ARM_H__*/
