
#include "rc.h"
#include "delay.h"
#include "usart.h"
#include "string.h"
#include "filter.h"
#include "queue.h"

u8 chan = 0;
u16	TIM3CH1_CAPTURE_VAL;	//PWM输入捕获值
vs16 CHdata[10];  //遥控器原始数据，滤波后数据

void RC_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    FilterInit();
    
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* GPIOA and GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* TIM3 channel 2 pin (PA.07) configuration */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);

    TIM_TimeBaseStructure.TIM_Period = 0XFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);       //选择IC2为始终触发源
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);    //TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
    TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);  //启动定时器的被动触发


    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM3,  TIM_IT_CC2 , ENABLE);

    /* TIM enable counter */
    TIM_Cmd(TIM3, ENABLE);
    
    
}


void TIM3_IRQHandler(void)
{


    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);                //清除TIM的中断待处理位

    TIM3CH1_CAPTURE_VAL = TIM_GetCapture2(TIM3);                         //读取IC2捕获寄存器的值，即为PWM周期的计数值
    if(TIM3CH1_CAPTURE_VAL > 900 && TIM3CH1_CAPTURE_VAL < 2200 && chan < 8)
    {
        CHdata[chan] = TIM3CH1_CAPTURE_VAL ;      //存储原始数据
        //printf("%d %d\n",CHdata[chan],chan);
//        EnQueue(&CH_Queue[chan],CHdata[chan]);    //新值入队列  队列长度为9 
//        DeQueue(&CH_Queue[chan]);                  //旧值滚开
        chan++;
    }
    else if(TIM3CH1_CAPTURE_VAL > 3000)    //同步帧   
        chan = 0;
      
   

}





