
#include "rc.h"
#include "delay.h"
#include "usart.h"
#include "string.h"
#include "filter.h"
#include "queue.h"

u8 chan = 0;
u16	TIM3CH1_CAPTURE_VAL;	//PWM���벶��ֵ
vs16 CHdata[10];  //ң����ԭʼ���ݣ��˲�������

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
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);       //ѡ��IC2Ϊʼ�մ���Դ
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);    //TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
    TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);  //������ʱ���ı�������


    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM3,  TIM_IT_CC2 , ENABLE);

    /* TIM enable counter */
    TIM_Cmd(TIM3, ENABLE);
    
    
}


void TIM3_IRQHandler(void)
{


    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);                //���TIM���жϴ�����λ

    TIM3CH1_CAPTURE_VAL = TIM_GetCapture2(TIM3);                         //��ȡIC2����Ĵ�����ֵ����ΪPWM���ڵļ���ֵ
    if(TIM3CH1_CAPTURE_VAL > 900 && TIM3CH1_CAPTURE_VAL < 2200 && chan < 8)
    {
        CHdata[chan] = TIM3CH1_CAPTURE_VAL ;      //�洢ԭʼ����
        //printf("%d %d\n",CHdata[chan],chan);
//        EnQueue(&CH_Queue[chan],CHdata[chan]);    //��ֵ�����  ���г���Ϊ9 
//        DeQueue(&CH_Queue[chan]);                  //��ֵ����
        chan++;
    }
    else if(TIM3CH1_CAPTURE_VAL > 3000)    //ͬ��֡   
        chan = 0;
      
   

}





