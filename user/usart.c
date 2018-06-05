
/************************************************************************
USART传输数据或者打印数据
移植时，只需修改下面代码修改区中的代码即可

测试代码为：
	USART_Config();
	printf("vvvv");
*************************************************************************/
#include "usart.h"
#include "GroundStation.h"

/***********************************************************************
移植代码修改区
************************************************************************/
//#define USE_USART1					//若使用的不是USART1，则把这句注释掉即可
#define USART						USART3
#define RCC_PORT					RCC_APB2Periph_GPIOB
#define RCC_USART					RCC_APB1Periph_USART3
#define PORT						GPIOB
#define TX							GPIO_Pin_10
#define RX							GPIO_Pin_11
/************************************************************************/



/******************************函数区************************************/
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	USART_InitTypeDef USART_initStructure;
	
	RCC_APB2PeriphClockCmd(RCC_PORT,ENABLE);
	
	#ifdef USE_USART1
		RCC_APB2PeriphClockCmd(RCC_USART,ENABLE);
	#else
		RCC_APB1PeriphClockCmd(RCC_USART,ENABLE);
	#endif
	
	//作为USART的TX端和RX端的引脚初始化
	GPIO_initStructure.GPIO_Pin = TX;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//复用推挽输出
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT,&GPIO_initStructure);
	
	GPIO_initStructure.GPIO_Pin = RX;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
	GPIO_Init(PORT,&GPIO_initStructure);
	
	//配置USART
	USART_initStructure.USART_BaudRate = 57600;
	USART_initStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_initStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_initStructure.USART_Parity = USART_Parity_No;
	USART_initStructure.USART_StopBits = USART_StopBits_1;
	USART_initStructure.USART_WordLength = USART_WordLength_8b;
    
	USART_Init(USART,&USART_initStructure);
	USART_ITConfig(USART, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART,ENABLE);
	
}	

//内部调用函数，注意要勾选OPTIONS中的USE Micro LIB选项
int fputc(int ch,FILE *f)
{
	USART_SendData(USART,(u8)ch);
	while(USART_GetFlagStatus(USART,USART_FLAG_TXE)==RESET);
	return ch;
}

void USART3_IRQHandler(void){  
      unsigned char RxData;  
      if (USART_GetITStatus(USART, USART_IT_RXNE) != RESET) {  
            USART_ClearITPendingBit(USART1, USART_IT_RXNE);  
            RxData=USART_ReceiveData(USART1);   
            ANO_DT_Data_Receive_Prepare(RxData); 
      }  
} 

