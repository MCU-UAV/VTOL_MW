#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f10x.h"
#include "imuAPI.h"


#define remote_dead_zone 10     //ң����������ͨ��������������Χ ��λ ��ms
#define remote_normal_value 1500 //ң����������ͨ����������ֵ ��λ��ms
#define motor_idle_throttle 1250 //�����������
#define throttle_low_dead_value  1200
#define motor_min_throttle 1100

#define AIL 0  //���

#define ELE 1  //������
#define THR 2

#define RUD 3  //�����   
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7




typedef struct DATA
{
    float P;
    float I;
    float D;
    float lastError;
    float Integral;
    float Diff;
    float Output;
    float error;
    float correct;
    float Input;  //����
    float Measure; //����ֵ
    float Desire;  //����
    float IntDifZone;
    float Integral_max;
    float OutLim;
  

}PID_DATA;

struct PITCH
{
    struct DATA outer;
    struct DATA inner;
};
struct ROLL
{
    struct DATA outer;
    struct DATA inner;
};
struct YAW
{
    struct DATA outer;
    struct DATA inner;
};

extern vs16 PWM1, PWM2, PWM3, PWM4;
extern vs16 ch[10];

extern IMUdata gyro;
extern IMUdata ang;
extern IMUdata acc;
extern IMUdata mag;
extern AddData  add;
extern QuadNum  quad;

extern struct PITCH pitch;
extern struct ROLL roll;
extern struct YAW yaw;
extern vu8 loop_cnt;
extern float yaw_desire;

extern float Debug_temp;
void PIDinit(void);
void Control(void);
void ControlInit(void);
float limf(float pat, float min, float max);
uint16_t myabs(int16_t x);
#endif

