
/************************************************************************
模拟I2C
只需根据需要修改下面修改区中的I2C_PORT，I2C_SCL，I2C_SDA，RCC_I2C_PORT
对应的管脚和端口即可

注意：
1.初始化I2C_MoniConfig()函数前，请先初始化SysTick_init()，以便可以使用延迟函数
************************************************************************/
#include "i2c_soft.h"
#include "delay.h"
#include "usart.h"
#include "stdio.h"
/************************************************************************
代码移植修改区
只需要修改模拟I2C使用的是使用的的端口和SCL/SDA引脚即可
************************************************************************/
#define I2C_PORT 				GPIOB
#define I2C_SCL 				GPIO_Pin_14
#define I2C_SDA 				GPIO_Pin_15
#define RCC_I2C_PORT 			RCC_APB2Periph_GPIOB
/***********************************************************************/

//控制管脚输出高低电平
#define I2C_SCL_LOW 			(I2C_PORT->BRR |= I2C_SCL)
#define I2C_SCL_HIGH 			(I2C_PORT->BSRR |= I2C_SCL)
#define I2C_SDA_LOW 			(I2C_PORT->BRR |= I2C_SDA)
#define I2C_SDA_HIGH 			(I2C_PORT->BSRR |= I2C_SDA)

//读取管脚的电平状态
#define I2C_SDA_READ 			(I2C_PORT->IDR & I2C_SDA)
//==========================================================================



/****************************函数区*******************************************/
//I2C初始化配置
void I2C_MoniConfig(void){
	GPIO_InitTypeDef GPIO_initStruct;
	
	RCC_APB2PeriphClockCmd(RCC_I2C_PORT,ENABLE);
	
	GPIO_initStruct.GPIO_Pin = I2C_SCL|I2C_SDA;
	GPIO_initStruct.GPIO_Mode = GPIO_Mode_Out_OD;		//必须为通用开漏输出GPIO_Mode_Out_OD][[
	GPIO_initStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_PORT,&GPIO_initStruct);
	
	I2C_SCL_LOW;
	delay_us(1);
	I2C_SDA_HIGH;
	delay_us(1);
}


//内部函数，判断总线是否处于空闲状态，若不是，则循环的提供SCL时钟驱动，直到从机释放SDA线
void _I2C_IsBusy(void)
{
	//读取SDA线上的电平状态,若为低电平则说明总线被从机控制，若为高电平则说明总线空闲，可以准备发送开始条件
	while(!I2C_SDA_READ){
		
		I2C_SCL_LOW;
		delay_us(1);
		I2C_SCL_HIGH;
		delay_us(1);
	}
}	


//产生开始信号
void I2C_Start(void){
	//判断下总线是否处于空闲状态
	_I2C_IsBusy();
	
	//先让SCL中的电平为低，防止因为SCL处于高电平而使后面将SDA拉高时，可能会触发一个stop信号
	I2C_SCL_LOW;
	delay_us(1);
	
	I2C_SDA_HIGH;
	delay_us(1);
	I2C_SCL_HIGH;
	delay_us(1);
	I2C_SDA_LOW;
	delay_us(1);
	
	I2C_SCL_LOW;
	delay_us(1);
}

//产生结束信号
void I2C_Stop(void){
	I2C_SCL_LOW;
	delay_us(1);
	
	I2C_SDA_LOW;
	delay_us(1);
	I2C_SCL_HIGH;
	delay_us(1);
	I2C_SDA_HIGH;
	delay_us(1);
	
}

//产生应答信号或者非应答信号,ackState->为ENABLE时，则产生应答信号
void I2C_SetAck(FunctionalState ackState){
	I2C_SCL_LOW;
	delay_us(1);
	
	if(ackState == ENABLE){
		I2C_SDA_LOW;
		delay_us(1);
	}else{
		I2C_SDA_HIGH;
		delay_us(1);
	}
	
	I2C_SCL_HIGH;
	delay_us(1);
	
	I2C_SCL_LOW;
	delay_us(1);
	
	I2C_SDA_HIGH;
	delay_us(1);
}

//获得应答信号（ENABLE）或者非应答信号（DISABLE）
FunctionalState I2C_GetAck(void){
	FunctionalState ask;
	
	I2C_SCL_HIGH;
	delay_us(1);
	
	//读取SDA线上的电平状态
	if(I2C_SDA_READ){
		ask = DISABLE;
	}else{
		ask = ENABLE;
	}
	
	I2C_SCL_LOW;
	delay_us(1);
	
	return ask;
}

//写出数据给从机，并返回应答或者非应答信号
FunctionalState MoniI2C_WriteByte(u8 data){
	u8 i;
	
	//类似移位寄存器的功能，将数据通过I/O口发送出去
	for(i=0;i<8;i++){
		I2C_SCL_LOW;
		delay_us(1);
		
		if(data & 0x80){
			I2C_SDA_HIGH;
		}else{
			I2C_SDA_LOW;
		}
		data <<=1;
		delay_us(1);
		
		I2C_SCL_HIGH;
		delay_us(1);
	}
	
	I2C_SCL_LOW;
	delay_us(1);
	I2C_SDA_HIGH;
	delay_us(1);
	
	return I2C_GetAck();
}

//读取从机发送的数据,并决定是应答还是非应答
 u8 MoniI2C_ReadByte(FunctionalState ackState){
	u8 i;
	u8 data = 0x00;
	
	//类似移位寄存器的功能，将数据从I/O口中读取进来
	for(i=0;i<8;i++){
		I2C_SCL_HIGH;
		delay_us(1);
		
		//先移位，后赋值
		data <<=1;
		if(I2C_SDA_READ){
			data |= 0x01;
		}
		
		I2C_SCL_LOW;
		delay_us(1);
	}
	
	//发送响应（或非响应）给MPU6050
	I2C_SetAck(ackState);
	
	return data;
}


/********************************************************
往从机中的寄存器写入一个字节的数据
*addr:设备地址
*reg：寄存器地址
*data：要写入的数据
*返回值：0为正常写入，1为写入异常
**********************************************************/
u8 MoniI2c_WriteByteToSlave(u8 addr,u8 reg,u8 data)
{
	FunctionalState state;							//用来记录ACK还是NACK
	                                                
	I2C_Start();                                    //产生起始信号
	state = MoniI2C_WriteByte(addr<<1|0);           //将从机地址和写方向写入总线
	if(state == ENABLE){                            //判断从机是否响应了地址
		state = MoniI2C_WriteByte(reg);     		//将要写入数据的寄存器地址
		if(state == ENABLE){                        
			MoniI2C_WriteByte(data);                //将数据发送给从机
			
			//正常写入，产生停止信号，并返回0
			I2C_Stop();
			return 0;
		}
	}
	
	//写入异常，产生停止信号，并返回1
	I2C_Stop();
	return 1;
}                                                  


/************************************************************
往从机中的寄存器写入多个数据
*addr：器件的地址
*reg：首个写入数据的寄存器地址
*len：要写入的数据的个数
*buf：要写入的数据区的首地址
*返回值：成功写入则返回0，失败则返回1
**************************************************************/
u8 MoniI2c_WriteSomeDataToSlave(u8 addr,u8 reg,u8 len,u8 *buf)
{
	FunctionalState state;								//用来记录ACK还是NACK
	u8 i;
	
	I2C_Start();                                    	//产生起始信号
	//state = MoniI2C_WriteByte(addr);               //将从机地址和写方向写入总线
    state = MoniI2C_WriteByte(addr<<1|0);               //将从机地址和写方向写入总线
	if(state == ENABLE){                            	//判断从机是否响应了地址，
		state = MoniI2C_WriteByte(reg);      			//将要写入数据的首个寄存器地址发送给从机
		if(state == ENABLE){
			for(i=0;i<len;i++){							//开始将数据区的数据都发送给从机
				state = MoniI2C_WriteByte(*(buf+i));    //将数据发送给从机
				if(state == DISABLE){					
					I2C_Stop();                         //从机未应答，产生停止信号结束数据传输，并返回1
					return 1;
				}
			}
			
			//正常产生停止信号，返回0
			I2C_Stop();                                      	
			return 0;
		}                                          
	}                              
	
	//写入异常，产生结束信号，返回1
	I2C_Stop();
	return 1;
}                                                  


/********************************************************
往从机中的寄存器读取一个字节的数据
*addr:设备地址
*reg：寄存器地址
*buf:读取到的数据存储的内存区
*返回值：0为正常读取，1为读取异常
**********************************************************/
u8 MoniI2c_ReadFromSlave(u8 addr,u8 reg,u8 *buf)
{
	FunctionalState state;
	
	I2C_Start();
	state = MoniI2C_WriteByte(addr<<1|0);			//发送从机地址和写方向
    
	if(state == ENABLE){
        
		state = MoniI2C_WriteByte(reg);				//发送要读取数据的寄存器的地址
		if(state == ENABLE){
           
			I2C_Start();							//发送Sr信号，
			state = MoniI2C_WriteByte(addr<<1|1);	//发送从机地址和读方向
			if(state == ENABLE){
				*buf = MoniI2C_ReadByte(DISABLE);	//读取完一个数据后，向从机发送一个NACK信号，结束从机发送数据
				
				//正常数据读取，产生停止信号，并返回0
				I2C_Stop();
				return 0;
			}
		}
	}
	
	//数据读取异常，产生stop信号，并返回1
	I2C_Stop();
	return 1;		//数据读取失败，返回0
}


/************************************************************************
从从机中读取采样到的高8位和低8位的数据

*addr：器件的地址
*reg：要读取数据的首个数据的寄存器地址
*len：读取的数据个数
*buf：读取到的8位数据存放区的地址，即通过指针的方式将读取到的数据返回给调用者。
*返回值：成功读取则返回0，失败则返回1
*************************************************************************/
u8 MoniI2c_ReadSomeDataFromSlave(u8 addr,u8 reg,u8 len,u8 *buf)
{
	FunctionalState state;
	
	I2C_Start();
	state = MoniI2C_WriteByte(addr<<1|0);							//写入从机地址和写方向
	if(state == ENABLE){
		state = MoniI2C_WriteByte(reg);								//写入要读取数据的首个寄存器的地址
		if(state == ENABLE){
			I2C_Start();
			state = MoniI2C_WriteByte(addr<<1|1);					//重新发送起始信号，转为读方向，开始读取数据
			if(state == ENABLE){
				while(len){
					if(len==1){										
						*buf = MoniI2C_ReadByte(DISABLE);			//当最后一个数据接收时，给从机发送一个NACK信号
					}else{
						*buf = MoniI2C_ReadByte(ENABLE);			//当一个数据接收时，给从机发送一个ACK信号
					}
					len--;
					buf++;
				}
				
				//数据正常读取，产生停止信号，结束此次数据的传输，并返回0
				I2C_Stop();
				return 0;
			}
		}
	}
	
	//数据读取异常，产生stop信号，结束此次数据读取
	I2C_Stop();
	return 1;		//数据读取失败，返回1
}



