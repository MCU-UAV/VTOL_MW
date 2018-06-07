#ifndef _PID_H
#define _PID_H

#include "stm32f10x.h"
#include "imuAPI.h"
#include "control.h"
float PID_Postion_Cal( PID_DATA *data);
void PID_Set(PID_DATA *data, float Input, float Desire, float Measure, float IntDifZone, float Integral_max,float OutLim);
void parameterInit(void);


#endif

