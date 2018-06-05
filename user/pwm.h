#ifndef _MOTO_PWM_H
#define _MOTO_PWM_H

#include "stm32f10x.h"

#define PERIOD1 		(60000-1)				//ARR计数值为：7200，不分频时，则pwm频率为400Hz，注意这里要用小括号括起来
#define PERIOD2         (60000-1)

void MOTO_PwmConfig(void);
void SERVO_PwmConfig(void);


void MOTO1_SetPulse(u16 pulse);
void MOTO2_SetPulse(u16 pulse);
void MOTO3_SetPulse(u16 pulse);
void MOTO4_SetPulse(u16 pulse);

void SERVO1_SetPulse(u16 pulse);
void SERVO2_SetPulse(u16 pulse);
void SERVO3_SetPulse(u16 pulse);
void SERVO4_SetPulse(u16 pulse);
#endif
