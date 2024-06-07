#include "drv_MusicLibrary.h"
#include "drv_buzzer.h"

/**
 * @brief C大调音符
 *
 * @param Music_region   1：低音 2：中音 3：高音
 * @param Music_symbol   数字1~7，分别代表音符1~7
 * @param time           声音持续时间 单位：ms
 */
static void Phonogram_C(rt_uint8_t Music_region,
                        rt_uint8_t Music_symbol,
                        rt_uint16_t time)
{
    if (Music_region == 1)
    {
        switch (Music_symbol)
        {
        case 1:
            set_buzzer(262, 1);
            rt_thread_mdelay(time);
            set_buzzer(262, 0);
            break;
        case 2:
            set_buzzer(294, 1);
            rt_thread_mdelay(time);
            set_buzzer(294, 0);
            break;
        case 3:
            set_buzzer(330, 1);
            rt_thread_mdelay(time);
            set_buzzer(330, 0);
            break;
        case 4:
            set_buzzer(349, 1);
            rt_thread_mdelay(time);
            set_buzzer(349, 0);
            break;
        case 5:
            set_buzzer(392, 1);
            rt_thread_mdelay(time);
            set_buzzer(392, 0);
            break;
        case 6:
            set_buzzer(440, 1);
            rt_thread_mdelay(time);
            set_buzzer(440, 0);
            break;
        case 7:
            set_buzzer(494, 1);
            rt_thread_mdelay(time);
            set_buzzer(494, 0);
            break;
        }
    }
    else if (Music_region == 2)
    {
        switch (Music_symbol)
        {
        case 1:
            set_buzzer(523, 1);
            rt_thread_mdelay(time);
            set_buzzer(523, 0);
            break;
        case 2:
            set_buzzer(587, 1);
            rt_thread_mdelay(time);
            set_buzzer(587, 0);
            break;
        case 3:
            set_buzzer(659, 1);
            rt_thread_mdelay(time);
            set_buzzer(659, 0);
            break;
        case 4:
            set_buzzer(698, 1);
            rt_thread_mdelay(time);
            set_buzzer(698, 0);
            break;
        case 5:
            set_buzzer(784, 1);
            rt_thread_mdelay(time);
            set_buzzer(784, 0);
            break;
        case 6:
            set_buzzer(880, 1);
            rt_thread_mdelay(time);
            set_buzzer(880, 0);
            break;
        case 7:
            set_buzzer(988, 1);
            rt_thread_mdelay(time);
            set_buzzer(988, 0);
            break;
        }
    }
    else if (Music_region == 3)
    {
        switch (Music_symbol)
        {
        case 1:
            set_buzzer(1047, 1);
            rt_thread_mdelay(time);
            set_buzzer(1047, 0);
            break;
        case 2:
            set_buzzer(1175, 1);
            rt_thread_mdelay(time);
            set_buzzer(1175, 0);
            break;
        case 3:
            set_buzzer(1319, 1);
            rt_thread_mdelay(time);
            set_buzzer(1319, 0);
            break;
        case 4:
            set_buzzer(1397, 1);
            rt_thread_mdelay(time);
            set_buzzer(1397, 0);
            break;
        case 5:
            set_buzzer(1568, 1);
            rt_thread_mdelay(time);
            set_buzzer(1568, 0);
            break;
        case 6:
            set_buzzer(1760, 1);
            rt_thread_mdelay(time);
            set_buzzer(1760, 0);
            break;
        case 7:
            set_buzzer(1975, 1);
            rt_thread_mdelay(time);
            set_buzzer(1975, 0);
            break;
        }
    }
}

/**
 * @brief 纯音乐:The truth that you  leave
 */
void The_truth_that_you_leave(void)
{

    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    /*第一小结*/
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 5, 2 * Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 5, Music_4division);

    /*第二小结*/
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 5, 2 * Music_4division);
    Phonogram_C(1, 6, 2 * Music_4division);
    Phonogram_C(2, 1, Music_4division);

    /*第三小结*/
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(2, 1, 2 * Music_4division);
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(2, 1, Music_4division);

    /*第四小结*/
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    Phonogram_C(2, 4, 3 * Music_4division);

    /*第五小结*/
    Phonogram_C(3, 5, Music_4division);
    Phonogram_C(2, 6, Music_4division);
    Phonogram_C(3, 1, Music_4division);
    Phonogram_C(3, 5, 2 * Music_4division);
    Phonogram_C(2, 6, Music_4division);
    Phonogram_C(3, 1, Music_4division);
    Phonogram_C(3, 5, 2 * Music_4division);

    /*第六小结*/
    Phonogram_C(3, 5, Music_4division);
    Phonogram_C(2, 6, Music_4division);
    Phonogram_C(3, 1, Music_4division);
    Phonogram_C(3, 5, 2 * Music_4division);
    Phonogram_C(2, 6, Music_4division);
    Phonogram_C(3, 3, Music_4division);
    Phonogram_C(3, 4, Music_4division);

    /*第七小结*/
    Phonogram_C(3, 4, Music_4division);
    Phonogram_C(3, 5, 3 * Music_4division);
    Phonogram_C(3, 5, 4 * Music_4division);
    rt_thread_mdelay(600);

    /*第八小结*/
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 1, Music_4division);

    /*第九小结*/
    rt_thread_mdelay(250);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    Phonogram_C(2, 4, Music_4division);

    /*第十小结*/
    Phonogram_C(2, 4, Music_4division);
    Phonogram_C(2, 3, 3 * Music_4division);
    Phonogram_C(2, 3, 4 * Music_4division);

    /*第十一小结*/
    rt_thread_mdelay(250);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 2, Music_4division);

    /*第十二小结*/
    rt_thread_mdelay(250);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 5, Music_4division);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    Phonogram_C(2, 4, Music_4division);

    /*第十三小结*/
    Phonogram_C(2, 4, Music_4division);
    Phonogram_C(2, 5, 3 * Music_4division);
    rt_thread_mdelay(250);
    Phonogram_C(2, 2, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    Phonogram_C(2, 4, Music_4division);

    /*第十四小结*/
    Phonogram_C(2, 4, 4 * Music_4division);
    Phonogram_C(1, 6, Music_4division);
    Phonogram_C(1, 7, Music_4division);
    Phonogram_C(2, 1, Music_4division);
    Phonogram_C(2, 5, Music_4division);

    /*第十五小结*/
    Phonogram_C(2, 4, 4 * Music_4division);
    rt_thread_mdelay(250);
    Phonogram_C(2, 4, Music_4division);
    Phonogram_C(2, 3, Music_4division);
    Phonogram_C(2, 2, Music_4division);

    /*第十六小结*/
    Phonogram_C(2, 2, 8 * Music_4division);

    /*第十七小结*/
    Phonogram_C(2, 3, 2 * Music_4division);
    Phonogram_C(2, 4, Music_4division);
    Phonogram_C(2, 3, 2 * Music_4division);
    Phonogram_C(2, 2, 2 * Music_4division);
    Phonogram_C(2, 1, Music_4division);

    /*第十八小结*/
    Phonogram_C(2, 1, 8 * Music_4division);
}

/**
 * @brief 提示音音效设置
 * @param Mode
 */
void PromptEQ_Set(PrompEQ_e Soundeffect)
{
    switch (Soundeffect)
    {
    case EmptyCtrl_EQ:
        Phonogram_C(1, 6, PPTMode);
        Phonogram_C(1, 5, PPTMode);
        break;
    case StartUp_EQ:
        Phonogram_C(2, 1, PPTMode);
        Phonogram_C(2, 2, PPTMode);
        Phonogram_C(2, 3, PPTMode);
        break;
    case Slowstartfinish_EQ:
        Phonogram_C(2, 3, PPTMode);
        Phonogram_C(2, 2, PPTMode);
        Phonogram_C(2, 1, PPTMode);
        break;
    case Remote_EQ:
        Phonogram_C(1, 1, PPTMode);
        break;
    case Client_EQ:
        Phonogram_C(3, 3, PPTMode);
        break;
    case Boom_EQ:
        Phonogram_C(1, 1, PPTMode);
        break;
    case Forearm_EQ:
        Phonogram_C(2, 2, PPTMode);
        break;
    case Chassis_EQ:
        Phonogram_C(1, 1, PPTMode);
        break;
    case Sucker_EQ:
        Phonogram_C(3, 3, PPTMode);
        break;
    case Test1_EQ:
        Phonogram_C(1, 3, PPTMode);
        break;
    case Test2_EQ:
        Phonogram_C(2, 3, PPTMode);
        break;
    case Test3_EQ:
        Phonogram_C(3, 3, PPTMode);
        break;
    case IsZero_EQ:
        Phonogram_C(2, 1, PPTMode);
        break;
    case DatainValid_EQ:
        Phonogram_C(3, 1, PPTMode);
        break;
    case EncoderCali_fail_EQ:
        Phonogram_C(3, 7, PPTMode);
        break;
    default:
        break;
    }
}
