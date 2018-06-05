#ifndef MS5611_H
#define MS5611_H

#include "stm32f10x.h"

#define MS5611_Adress 0xEC
/*Command*/
#define Reset 0x1E
/*不同采样率下的转换命令*/
#define Convert_D1_8bit 0x40
#define Convert_D1_9bit 0x42
#define Convert_D1_10bit 0x44
#define Convert_D1_11bit 0x46
#define Convert_D1_12bit 0x48

#define Convert_D2_8bit 0x50
#define Convert_D2_9bit 0x52
#define Convert_D2_10bit 0x54
#define Convert_D2_11bit 0x56
#define Convert_D2_12bit 0x58
/*读命令*/
#define ADC_Read 0x00
/*0xA0~0xAE*/
#define PROM_Read 0xA0
/*10ms读一次数据*/
#define MS5611_Time 10

void MS5611_Read_Colibration(void);
void Start_Convert(u8 Command);
u32 MS5611_Read_ADC(void);
float MS5611_Calculate(u32 D1,u32 D2);
#endif
