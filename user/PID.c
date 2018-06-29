#include "stm32f10x.h"

#include "control.h"
#include "imuAPI.h"
#include "PID.h"
#include "RC.h"
#include "flightmode.h"
#include "altHold.h"



#define remote_normal_value 1500 //ң����������ͨ����������ֵ ��λ��ms


struct PITCH pitch;
struct ROLL roll;
struct YAW yaw;

float PID_Postion_Cal( PID_DATA *data)
{
    char Index = 0;

    data->error  = data->Input + data->Desire  - data->Measure  +  data->correct  ;     
    data->Diff =  data->error -  data->lastError;     //����΢��
    if(myabs(data->error) < data->IntDifZone) Index = 1;    //���ַ���
    else  Index = 0;
    if(data->Integral > data->Integral_max) data->Integral = data->Integral_max;   //�����޷�
    if(data->Integral < -data->Integral_max) data->Integral = -data->Integral_max;
    if(CHdata[THR] < 1200) data->Integral = 0;   //���ŵ�λ ����������
    data->lastError = data->error;   //���汾�����
    data->Output =  data->P * data->error +  Index * data->I *  data->Integral +  data->D *  data->Diff;  //����
    return limf( data->Output , -data->OutLim, data->OutLim);  //����
}

void PID_Set(PID_DATA *data, float Input, float Desire, float Measure, float IntDifZone, float Integral_max,float OutLim)
{
    data->Input = Input;
    data->Desire = Desire;
    data->Measure = Measure;
    data->IntDifZone = IntDifZone;
    data->OutLim = OutLim;
    data->Integral_max = Integral_max;
}

void parameterInit()
{
    pitch.outer.correct = 0;
    roll.outer.correct = 0;
    yaw.outer.correct = 0;

    pitch.inner.correct = 0;
    roll.inner.correct = 0;
    yaw.inner.correct = 0;

    pitch.outer.Output = 0;
    roll.outer.Output = 0;
    yaw.outer.Output = 0;
    
    roll.inner.P = 3.5f;   //1.5
    roll.inner.I = 0.01f;
    roll.inner.D = 12.0f;  //40.0

    pitch.inner.P  = 5.0f;   //40.00
    pitch.inner.I = 0.0f;
    pitch.inner.D = 10.0f;
    
    yaw.inner.P = 10.0f;
    yaw.inner.I = 0.0f;
    yaw.inner.D = 5.0f;

    roll.outer.P = 1.0f;  //-2.0
    roll.outer.I = 0.01f;   //-0.01
    roll.outer.D = 1.8f;  //0.0
    
    pitch.outer.P = 10.0f;    //1.0
    pitch.outer.I = 0.01f;
    pitch.outer.D = 20.0f;
  

    yaw.outer.P = 0.0f;
    yaw.outer.I = 0.0f;
    yaw.outer.D = 0.0f;
    
    barAltHoldHeight.P = 100.0;
    barAltHoldHeight.I = 0.0;
    barAltHoldHeight.D = 0.0;
    
    barAltHoldRate.P = 100.0;
    barAltHoldRate.I = 0.0;
    barAltHoldRate.D = 0.0;
    
    lastgg.X = 0;
    lastgg.Y = 0;
    lastgg.Z = 0;

    State = Armed;  //Ĭ������״̬
    Mode = Stabilze;  //Ĭ������ģʽ
    
    setCalibration(3);  //��ѹ�߶���0

  
    PID_Set(&(barAltHoldHeight), 0, 0, 0, 0.5,2000, 1000.0); //��0.5���ڻ���
    PID_Set(&(barAltHoldRate),0,0,0,1.0,2000,1000.0);
    
   
   // yaw_desire=getYawToward(); //�õ���ǰ����Ƕ�
    
    setOpenGPS(); //����ģ���GPS
    setGPSBaud(9600);  //����ģ����GPS��ͨ�Ų�����
   

}

