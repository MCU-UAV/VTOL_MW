#ifndef _FLIGHTMODE_H
#define _FLIGHTMODE_H

#include "stm32f10x.h"

extern vu8 Mode;   //初始化飞行模式为垂直
extern vu8 State;  //初始化飞行状态为锁定
extern vu8 FlightMode;  //飞行模式


enum FlightMode   //飞行模式
{
    Manual,    //手动，只保留内环 角速度环（无飞行状态）
    Stabilze,  //增稳，双环控制
    AltHold,   //定高
    Rtl,       //自动返航
    Guided,    //地面站引导
    Loiter     //留待，定高+光流定点    
};

enum Mode
{
    Vert ,     //垂直模式 0
    Aero,     //固定翼模式  1 
};

enum FlightState  //飞行状态
{
    
    Armed,    //锁定状态 0
    DisArmed  //解锁状态  1
    
};

enum Operation
{
    Default, //默认 0
    StateSW, //加/解锁操作  1
    Normal //常规操作  2
     
};

void ModeTask(vs16 AUX_Mode,int8_t tow);
void FlightTask(vs16 AUX_Flight,int8_t tow);

void SafeTask(void);
void FlightStateTask(float T);
    
#endif


