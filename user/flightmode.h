#ifndef _FLIGHTMODE_H
#define _FLIGHTMODE_H

#include "stm32f10x.h"

extern vu8 Mode;   //��ʼ������ģʽΪ��ֱ
extern vu8 State;  //��ʼ������״̬Ϊ����
extern vu8 FlightMode;  //����ģʽ


enum FlightMode   //����ģʽ
{
    Manual,    //�ֶ���ֻ�����ڻ� ���ٶȻ����޷���״̬��
    Stabilze,  //���ȣ�˫������
    AltHold,   //����
    Rtl,       //�Զ�����
    Guided,    //����վ����
    Loiter     //����������+��������    
};

enum Mode
{
    Vert ,     //��ֱģʽ 0
    Aero,     //�̶���ģʽ  1 
};

enum FlightState  //����״̬
{
    
    Armed,    //����״̬ 0
    DisArmed  //����״̬  1
    
};

enum Operation
{
    Default, //Ĭ�� 0
    StateSW, //��/��������  1
    Normal //�������  2
     
};

void ModeTask(vs16 AUX_Mode,int8_t tow);
void FlightTask(vs16 AUX_Flight,int8_t tow);

void SafeTask(void);
void FlightStateTask(float T);
    
#endif


