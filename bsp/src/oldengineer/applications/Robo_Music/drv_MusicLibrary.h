#ifndef DRV_MUSICLIBRARY_H
#define DRV_MUSICLIBRARY_H

/*蜂鸣器开启时长*/
#define Music_4division (250)
#define PPTMode (250)

/**
 * @brief 纯音乐:The truth that you  leave
 */
void The_truth_that_you_leave(void);

typedef enum
{
    EmptyCtrl_EQ = 0,   // 不控制模式音效
    StartUp_EQ,         // 开机音效
    Slowstartfinish_EQ, // 缓启动完成音效
    Remote_EQ,          // 遥控模式音效
    Client_EQ,          // 客户端模式音效
    Boom_EQ,            // 大机械臂模式音效
    Forearm_EQ,         // 小机械臂模式音效
    Chassis_EQ,         // 底盘模式音效
    Sucker_EQ,          // 吸盘模式音效
    Test1_EQ,           // 测试1音效
    Test2_EQ,           // 测试2音效
    Test3_EQ,           // 测试3音效
    IsZero_EQ,          // 0值音效
    DatainValid_EQ,     // 数据失效音效
    EncoderCali_fail_EQ, // 编码器器标定失败音乐
} PrompEQ_e;            // 提示音音效模式

/**
 * @brief 提示音音效设置
 * @param Mode
 */
void PromptEQ_Set(PrompEQ_e Soundeffect);

#endif
