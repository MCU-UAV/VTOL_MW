
/***************************************************************************************
高级定时器TIM1输出PWM波控制四个电机

主函数测试代码为：
	MOTO_PwmConfig();
	MOTO_Start();
	
	while(1);
	
****************************************************************************************/
#include "pwm.h"


/***********************************函数区***********************************************/
//定时器TIM1初始化配置
void MOTO_PwmConfig(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	
	//配置TIM1输出通道的引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
	
	//使能TIM1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	//配置时基
	TIM_timeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;    //向上计数
	TIM_timeBaseInitStructure.TIM_Period = PERIOD1;						//设置ARR值
	TIM_timeBaseInitStructure.TIM_Prescaler = 2;						    //时钟预分频值
	TIM_timeBaseInitStructure.TIM_RepetitionCounter = 0;				//重复计数值
	TIM_TimeBaseInit(TIM1,&TIM_timeBaseInitStructure);
	
	//配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平（这个和刹车有关，刹车后通道的电平状态就取决于这个）
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;		//设置CHN通道的空闲状态的电平
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			//设置CHN通道的有效电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	//关闭CHN通道
	TIM_OCInitStructure.TIM_Pulse = 0;									//设置TIM1的CCR值
	
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
	
	//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出
	TIM_ARRPreloadConfig(TIM1,ENABLE);									//使能TIM1的寄存器ARR的预装载功能，DISABLE时将会使改变ARR值时立即生效
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Disable);                   //使能TIM1通道1的CCR的预装载功能，DISABLE时将会使改变CRR值时立即生效
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Disable);
	
	TIM_Cmd(TIM1,ENABLE);
	
	//开始启动定时器输出pwm,这句是高级定时器才有的，输出pwm必须打开
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

void SERVO_PwmConfig(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
 
	
    
    //配置TIM4输出通道的引脚
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   //开启复用时钟	
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //使能TIM4的时钟
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    //配置时基
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;  
    //向上计数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = PERIOD2 ;   //设置ARR值
	TIM_TimeBaseStructure.TIM_Prescaler =23;  //时钟预分频值
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //重复计数值
	
	 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	 //配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
     TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset; 
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;        
  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	
	
 
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);  
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);  
    
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);   
    TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Disable);
	
	
	TIM_Cmd(TIM4, ENABLE);  
}


u32 map(int val, int I_Min, int I_Max, int O_Min, int O_Max)
{
   return((float)val/(float)(I_Max-I_Min)*(float)(O_Max-O_Min) + (float)O_Min);
}






//=================================================================
/***********************************************
更改MOTO1的pulse
输入参数为pwm的占空数值
************************************************/
void MOTO1_SetPulse(u16 pulse)
{
	TIM_SetCompare1(TIM1, map(pulse,0, 2500, 0, 60000));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void MOTO2_SetPulse(u16 pulse)
{
	TIM_SetCompare2(TIM1,map(pulse,0, 2500, 0, 60000));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void MOTO3_SetPulse(u16 pulse)
{
	TIM_SetCompare3(TIM1,map(pulse,0, 2500, 0, 60000));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void MOTO4_SetPulse(u16 pulse)
{
	TIM_SetCompare4(TIM1,map(pulse,0, 2500, 0, 60000));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void SERVO1_SetPulse(u16 pulse)
{
	TIM_SetCompare1(TIM4,map(pulse,0, 2500, 0, 7500));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void SERVO2_SetPulse(u16 pulse)
{
	TIM_SetCompare2(TIM4,map(pulse,0, 2500, 0, 7500));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void SERVO3_SetPulse(u16 pulse)
{
	TIM_SetCompare3(TIM4,map(pulse,0, 2500, 0, 7500));
}

/************************************************
更改MOTO2的pulse
输入参数为pwm占空数值
************************************************/
void SERVO4_SetPulse(u16 pulse)
{
	TIM_SetCompare4(TIM4,map(pulse,0, 2500, 0, 7500));
}

