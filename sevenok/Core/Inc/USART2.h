
#ifndef CAPIMG_USART_H
#define CAPIMG_USART_H


#include "CRC.h"
#include "include.h"

    struct FrameHead
    {
        uint8_t SOF;
        int data_length;
        uint8_t seq;
        uint8_t CRC8;
    };

    struct SendPack
    {
        struct FrameHead head;
        float BY ;
        float BP1 ;
        float BP2 ;
        float FY ;
        float FP ;
        float FR ;
        int tail;
        uint8_t work_flag;
    };

#endif //CAPIMG_USART_H
