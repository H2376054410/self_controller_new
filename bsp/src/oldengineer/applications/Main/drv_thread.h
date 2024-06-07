#ifndef __DRV_THREAD_H__
#define __DRV_THREAD_H__

/*定时器周期*/
#define ROBOREMOTE_TIMER_PIRIOD (2) // 遥控线程周期
#define ROBOCTRL_TIMER_PIRIOD (2)   // 机器人控制周期

/*线程栈*/
#define THREAD_STACK_Can1Rx (1024)       // can1接收线程
#define THREAD_STACK_Can1Tx (1024)       // can1发送线程
#define THREAD_STACK_Can2Rx (1024)       // can2接收线程
#define THREAD_STACK_Can2Tx (1024)       // can2发送线程
#define THREAD_STACK_ArmCtrl (4096)      // 机械臂控制线程
#define THREAD_STACK_KEYMOUSEDATA (2048) // 遥控器数据线程
#define THREAD_STACK_REMOTECTRL (4096)   // 遥控器控制线程
#define THREAD_STACK_MUSICCTRL (1024)    // 音乐控制线程
#define THREAD_STACK_CUSTCTRLER (2048)   // 自定义控制器线程

/*线程优先级*/
#define THREAD_PRIO_MONITOR (1)       // 监视器线程
#define THREAD_PRIO_ArmCtrl (2)       // 机械臂控制线程
#define THREAD_PRIO_REMOTECTRL (2)    // 遥控器控制线程
#define THREAD_PRIO_Can1Tx (3)        // can1发送线程
#define THREAD_PRIO_Can2Tx (3)        // can2发送线程
#define THREAD_PRIO_Can1Rx (4)        // can1接收线程
#define THREAD_PRIO_Can2Rx (4)        // can2接收线程
#define THREAD_PRIO_RC_RX (5)         // 遥控器数据接收线程
#define THREAD_PRIO_KEYMOUSEDATA (7)  // 遥控器数据线程
#define THREAD_PRIO_ENCODER_CALI (13) // 编码器校准线程
#define THREAD_PRIO_KEYMENU (19)      // 按键触发线程
#define THREAD_PRIO_MUSICCTRL (22)    // 音乐数据线程
#define THREAD_PRIO_CUSTCTRLER (30)   // 自定义控制器线程

/*线程的时间片*/
#define THREAD_TICK_KEYMOUSEDATA (5) // 遥控器数据线程
#define THREAD_TICK_Can1Rx (5)       // can1接收线程
#define THREAD_TICK_Can1Tx (5)       // can1发送线程
#define THREAD_TICK_Can2Rx (5)       // can2接收线程
#define THREAD_TICK_Can2Tx (5)       // can2发送线程
#define THREAD_TICK_ArmCtrl (5)      // 机械臂控制线程
#define THREAD_TICK_REMOTECTRL (5)   // 遥控器控制线程
#define THREAD_TICK_MUSICCTRL (1)    // 音乐控制线程
#define THREAD_TICK_CUSTCTRLER (5)   // 自定义控制器线程

// 裁判系统线程
#define THREAD_STACK_DJI (4096)
#define THREAD_PRIO_DJI (7)
#define THREAD_TICK_DJI (10)

// UI
#define THREAD_STACK_UI (2048) //
#define THREAD_PRIO_UI (10)
#define THREAD_TICK_UI (10)

#endif
