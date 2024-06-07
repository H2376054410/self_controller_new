#ifndef __DRV_PLLFILTER_H__
#define __DRV_PLLFILTER_H__

#include <rtthread.h>

typedef struct
{
    float Pos;
    float Spe;
} PLLFilter_OutData;

typedef struct
{
    float Kp;                 // ���໷�˲�����ʹ�õ� PI �˲����ı�������
    float Ki;                 // ���໷�˲�����ʹ�õ� PI �˲����Ļ��ֲ���
    rt_uint8_t CrossCircle_Flag; // �Ƿ���Ҫ��Ȧ����
    float CircleMax;          // ��Ȧ����ʱ��Ȧ�������ֵ
    float CircleMin;          // ��Ȧ����ʱ��Ȧ����С��ֵ
} PLLFilter_Settings;

typedef struct
{
    PLLFilter_Settings Settings; // ���໷�˲�����������ò���
    float Pos_Sense;             // ��ǰ���뵽�˲����е�λ����Ϣ
    float PosErr;                // �˲������е���λ�����
    float Err_Integral;          // �������Ļ���
    rt_tick_t Last_FreshTick;    // ��һ���˲�������ʱ��ʱ��
    PLLFilter_OutData OutData;   // ��ǰ�˲������������
} PLLFilter_Ctrl_Struct;

/**
 * @brief ��ʼ�����໷�˲������ƿ�
 * @author fwlh
 * @param  PLLFilter_Ctrl   ����ʼ�������໷�˲����ṹ��
 * @param  Kp               ���໷�˲����ı���ϵ��
 * @param  Ki               ���໷�˲����Ļ���ϵ��
 * @param  StartPos         ���໷�˲����ĳ�ʼλ��
 */
extern void PLLFilter_Init(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float Kp, float Ki, float StartPos);

/**
 * @brief �������໷�˲����Ŀ�Ȧ����
 * @author fwlh
 * @param  PLLFilter_Ctrl   �����õ����໷�˲����ṹ��
 * @param  CircleMax        ��Ȧ����ʱ��Ȧ�������ֵ
 * @param  CircleMin        ��Ȧ����ʱ��Ȧ����С��ֵ
 */
extern void PLLFilter_CrossCircle_Set(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float CircleMax, float CircleMin);

/**
 * @brief ���໷�˲����ļ���
 * @author fwlh
 * @param  PLLFilter_Ctrl   �������˲���������໷�˲������ƿ�
 * @param  NowPos           ��ǰ���˲�������
 */
extern void PLLFilter_Calc(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float NowPos);

/**
 * @brief �������໷�˲���
 * @author fwlh
 * @param  PLLFilter_Ctrl   ��Ҫ���õ����໷�˲������ƿ�
 * @param  NowPos           �����Ժ�ĵ�ǰλ��
 */
extern void PLLFilter_Restart(PLLFilter_Ctrl_Struct *PLLFilter_Ctrl, float NowPos);

#endif /* __DRV_PLLFILTER_H__ */
