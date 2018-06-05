#ifndef _FILTER_H
#define _FILTER_H

#include "stm32f10x.h"
#include "queue.h"

extern SqQueue CH_Queue[9];   //构造数组指针


void FilterInit(void);
u16 GetMedianNum(u16 * bArray, int iFilterLen);
 void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
 int16_t MovMiddle(int16_t input);
#endif

