#include "flightmode.h"

#include "control.h"
#include "stdio.h"
#include "RC.h"
#include "altHold.h"

#define USE_TOE_IN_UNLOCK 1
vu8 Mode = Vert;   //��ʼ������Ϊ��ֱ
vu8 FlightMode = Stabilze; //Ĭ�Ϸ���ģʽΪ����
vu8 State = Armed;  //��ʼ������״̬Ϊ����

void go_arm()
{
    if(State == DisArmed) // ���û�н���
    {
        State = Armed;

    }
    else if(State == Armed)
    {
        //do something
    }

}

void go_disarm()
{
    if(State == Armed)
    {
        State = DisArmed;
    }
    else if(State == DisArmed)
    {
        //do something
    }
}




s16 ready_cnt = 0;
/*----------------------------------------------------------
 + ʵ�ֹ��ܣ������ж�
 + ���ò��������ε���ʱ��� ��λ����
----------------------------------------------------------*/

void FlightStateTask(float T)
{
    if( CHdata[2] < 1200 ) //���ſ����ź�С��1200
    {
        if( !State && ready_cnt != -1 ) //������ɣ������˳�������������
        {

        }
        /* �ڰ˽�����ʽ*/

        if( CHdata[0] < 1200 )
        {

            if( CHdata[1] < 1200 )

                if( CHdata[3] > 1700 )

                    if( ready_cnt != -1 )	//�ڰ��������˳������������̣���ֹ�����л�

                        ready_cnt +=  100 * T; //������ʱ

        }

        /* �Ѿ�������������*/
        else if( ready_cnt == -1 )
   
            ready_cnt = 0; //���������ʱ
     
    }
    else//�����Ͻ�������������
    {
        ready_cnt = 0; //���������ʱ
    }


    if( ready_cnt > 1000 ) // ������1000������
    {
        ready_cnt = -1;//��ֹ�����л�����
        if( !State )//������
        {
            go_disarm();//����

        }
        else//���Ѿ�����
        {
            go_arm(); //����
        }
    }

}

void ModeTask(vs16 AUX_Mode, int8_t tow)
{
    if(tow * (AUX_Mode - 1500) > 0 ) Mode = Vert;
    else Mode = Aero;  //ʹ�õ�7ͨ���ı�ɻ�״̬
}

void FlightTask(vs16 AUX_Flight, int8_t tow) //����ģʽ
{
    if(tow * (AUX_Flight -1700) > 0 ) 
    {
        FlightMode = Stabilze;
        rate = 0;
    }
    else 
    {
        FlightMode = AltHold;
         barAltHoldHeight.Desire = 0;
         initialThrottleHold = CHdata[THR];
         barAltHoldHeight.I = 0;
         barAltRateOut=0;
    }
}

void SafeTask(void)
{
    static uint8_t safe_check_cnt = 0;
    if(!(loop_cnt %10))
      safe_check_cnt++;
    if(safe_check_cnt == 100)
    {
        go_arm();
        safe_check_cnt = 0;
    }
}




