#ifndef __DRV_REMOTE_SBUS_H__
#define __DRV_REMOTE_SBUS_H__
#define BSP_USING_REMOTE_SBUS
#include <rtthread.h>
#if defined BSP_USING_REMOTE_SBUS
#include <rtdevice.h>
#include <board.h>

#define REMOTE_UART_DEVICE_NAME "uart3"

#define SWITCH_MIDDLE 1024
#define SWITCH_UP 1500
#define SWITCH_DOWN 600

#define THROTTLE_HIGH 1700
#define THROTTLE_DOWN 330
#define THROTTLE_MIDDLE 1024

/* 拨杆动作枚举 */
typedef enum
{
    /* 拨杆 动作 */
    NO_ACTION = 0x00,
    up_to_middle = 0x01,
    down_to_middle = 0x02,
    middle_to_up = 0x03,
    middle_to_down = 0x04,
    /* 按键 动作 */
    down_to_up = 0x05,
    up_to_down = 0x06,

    S0 = 0x07,
    S1 = 0x08,
    S2 = 0x09,
    S3 = 0x0A,

} switch_action_e;

typedef struct __Remote
{
    /* 遥杆数据 */
    uint16_t ch0; // 右侧遥感水平向右拨动数值增大
    uint16_t ch1; // 右侧遥感竖直向上拨动数值增大
    uint16_t ch2; // 左侧遥感水平向右拨动数值增大
    uint16_t ch3; // 左侧遥感竖直向上拨动数值增大
    /* 拨杆数据 */
    uint8_t s0;
    uint8_t s1;
    uint8_t s2;
    uint8_t s3;
} Remote_t;
/* 鼠标数据 */
typedef struct __Mouse
{
    int16_t x_speed; //向左负
    int16_t y_speed; //向下正
    int16_t z_speed;
    int8_t press_l;
    int8_t press_r;
} Mouse_t;
/* 键盘数据 */
typedef struct __Key_Data
{
    uint8_t W;
    uint8_t S;
    uint8_t A;
    uint8_t D;
    uint8_t shift;
    uint8_t ctrl;
    uint8_t Q;
    uint8_t E;
    uint8_t R;
    uint8_t F;
    uint8_t G;
    uint8_t Z;
    uint8_t X;
    uint8_t C;
    uint8_t V;
    uint8_t B;
} Key_Data_t;

typedef struct __RC_Ctrl
{
    Remote_t Remote_Data;
    Mouse_t Mouse_Data;
    rt_uint16_t v;
    Key_Data_t Key_Data;
    rt_tick_t ValidData_FreshTick; // 数据更新的时间
} RC_Ctrl_t;

/* 外部需读取遥控器某按键或拨杆状态时调用 直接读结构体的值 */

extern RC_Ctrl_t RC_data;
extern struct rt_semaphore RoboControl_sem;

/* 外部在main.c中调用 */
extern int SBUS_Init(void);
/* 读取拨杆动作(只能读取中间往两边拨的动作) */
extern switch_action_e Change_from_middle(switch_action_e sx);
/* 读取拨杆动作(只能读取两边往中间拨的动作) */
extern switch_action_e Change_to_middle(switch_action_e sx);

extern switch_action_e Switch_Change(switch_action_e sx);
/* 读取按键动作(仅键盘数据) */
extern switch_action_e Key_action_read(rt_uint8_t *targetdata);
/* 读取按键动作(仅鼠标按键数据) */
extern switch_action_e Mouse_action_read(switch_action_e mouse_x);
/* 获取当前遥控数据的数据包编号，用于判断数据是否更新 */
int Get_RC_DatNum(void);

extern int remote_uart_init(void);
#endif
#endif
