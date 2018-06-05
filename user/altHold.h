/************************************************************
 @������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 +���ߣ��ո�
 +�������߶ȿ���ͷ�ļ�
 +��д���ڣ�2017/3/10
 +��Ȩ���У�����ؾ���
*************************************************************/
#ifndef __ALTHOLD_H
#define __ALTHOLD_H

#include "stm32f10x.h"
#include "PID.h"
#include "control.h"

extern uint16_t acc_offset;
extern uint16_t altTHR;
extern uint16_t initialThrottleHold;
extern float barAltRateOut;
extern float barAltHeightOut;
extern float rate;
extern int16_t acc_error;
extern PID_DATA barAltHoldRate;
extern PID_DATA barAltHoldHeight;

extern PID_DATA barAltHoldRate;
extern PID_DATA barAltHoldHeight;

void AltHoldChange(void);
void AltDataDeal(void);
#endif

