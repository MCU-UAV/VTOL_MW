#ifndef __I2C_MONI_H
#define __I2C_MONI_H

#include "stm32f10x.h"

void I2C_MoniConfig(void);
void I2C_Start(void);
void I2C_Stop(void);

void I2C_SetAck(FunctionalState ackState);
FunctionalState MoniI2C_WriteByte(u8 data);

u8 MoniI2C_ReadByte(FunctionalState ackState); 

u8 MoniI2c_WriteByteToSlave(u8 addr,u8 reg,u8 data);
u8 MoniI2c_WriteSomeDataToSlave(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MoniI2c_ReadFromSlave(u8 addr,u8 reg,u8 *buf);
u8 MoniI2c_ReadSomeDataFromSlave(u8 addr,u8 reg,u8 len,u8 *buf);

#endif

