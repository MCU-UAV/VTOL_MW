/************************************************************
 @本程序只供学习使用，未经作者许可，不得用于其它任何用途
 +编者：普哥
 +描述：高度控制头文件
 +编写日期：2017/3/10
 +版权所有，盗版必纠。
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

