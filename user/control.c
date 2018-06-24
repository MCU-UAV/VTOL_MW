
/*****************************************************************
PID控制核心算法
******************************************************************/
#include "stm32f10x.h"
#include "control.h"

#include "usart.h"
#include "i2c_soft.h"
#include "nvic.h"
#include "pwm.h"
#include "systick.h"
#include "queue.h"
#include "filter.h"
#include "rc.h"
#include "stdio.h"
#include "imuAPI.h"
#include "math.h"
#include "PID.h"
#include "flightmode.h"
#include "altHold.h"





/*********************************************************************************
全局变量定义区
************************************************************************************/


vu8 loop_cnt = 0;
vs16 PWM1, PWM2, PWM3, PWM4;
vs16 lastPWM3, lastPWM4;

float Debug_temp;
float yaw_desire;

IMUdata ang;
IMUdata gyro;
IMUdata acc;
IMUdata mag;

AddData  add;
QuadNum  quad;
//====================================================================

u16 lim(vs16 pat, u16 min, u16 max)
{
    if(pat < min) return min;
    else if(pat > max) return max;
    else return pat;
}
uint16_t myabs(int16_t x)
{
    return  x >= 0 ?  x : -x;
}
float limf(float pat, float min, float max)
{
    if(pat < min) return min;
    else if(pat > max) return max;
    else return pat;
}

void Control(void)  //200HZ
{

    //更新角度数据

    //getAngle(&ang);  //由模块直接读出的角度数据

    getQuad (&quad);   //由模块读出的四元数，解算为角度，较灵活，以防万向轴死锁
    Quad2Angle(quad, &ang);
    getGyro(&gyro);    //得到角速度便于内环控制
    getAcc(&acc);       //得到加速度数据
    getMag(&mag);       //得到罗盘数据



    /*      坐标系                                                 对应飞机
    z ^             y   (row)                              ------------------------------------
    |           /                                         /                                   /
    |         /                                          /                                   /
    |       /                                           /                                   /
    |     /                                            /    ------             ------      /     水平平放
    |   /                                             /    /      /           /     /     /
    | /                                               -----------------------------------
    ----------------------->x （pitch）
    */
//PID运算
//外环PID
    //1通副翼，2通升降，3通油门，4通方向
    //按照图意 X轴为升降 Y轴为横滚 Z轴为偏航
    if(loop_cnt % 2 )    //100hz
    {
        
        FlightStateTask(0.1);//10ms执行一次  进行飞机锁定与加锁状态
        FlightTask(CHdata[AUX1], 1); //飞行模式   取反变为-1 自稳/定高
        ModeTask(CHdata[AUX2], -1); //飞机姿态 取反变为-1  垂直/水平      
        
        
       
        if(myabs(CHdata[AIL] - remote_normal_value) <= remote_dead_zone) CHdata[AIL] = remote_normal_value; //副翼死区设置
        if(myabs(CHdata[ELE] - remote_normal_value) <= remote_dead_zone) CHdata[ELE] = remote_normal_value; //升降死区设置
        if(myabs(CHdata[RUD] - remote_normal_value) <= remote_dead_zone) CHdata[RUD] = remote_normal_value; //方向死区设置

        if(Mode  == Vert)  //垂直状态
        {
            //外环PID计算

            //ROLL 外环 X轴
            PID_Set(&(roll.outer), limf( (CHdata[AIL] - remote_normal_value) / 30.0f, -16.0, 16.0 ), 0, -ang.Y, 5.0,45.0 ,1000.0);
            roll.outer.Output = PID_Postion_Cal(&(roll.outer));
            //PITCH 外环 X轴
            PID_Set(&(pitch.outer), limf(- (CHdata[ELE] - remote_normal_value) / 25.0f, -20.0, 20.0), 0, ang.X, 5.0,45.0, 1000.0);
            pitch.outer.Output = PID_Postion_Cal(&(pitch.outer));
            //YAW 外环
            PID_Set(&(yaw.outer),yaw_desire,0, ang.Z, 5.0,45.0 , 500.0);
            yaw.outer.Output = PID_Postion_Cal(&(yaw.outer));
            yaw_desire += (-(CHdata[RUD] - remote_normal_value) / 2.5) * 0.005;
            if(yaw_desire >= 180) yaw_desire -=360;
            
            else if(yaw_desire <= 180) yaw_desire +=360;
            if(FlightMode == AltHold)
            {
            getAddData(&add); //得到气压、气压高度、GPS相关数据
           
            AltDataDeal(); //计算 Z轴速度测量量 ， Z轴高度测量量
            
            barAltHeightOut = PID_Postion_Cal(&barAltHoldHeight);    //高度环计算
            barAltHoldRate.Desire = barAltHeightOut;              //高度环输出为速度环输入
            barAltRateOut = PID_Postion_Cal(&barAltHoldRate);      //速度环输入
                
            AltHoldChange();
            }
            
            
        }
        else if(Mode == Aero)
        {
            printf("Flight State is ERROR!\n");
            // roll.outer.Output = PID_Postion_Cal(&(roll.outer), (CHdata[AIL] - remote_normal_value) / 25.0f , ang.Y, 0, angle_max);      //外环PID计算
            //oll.outer.Output = PID_Postion_Cal(&(roll.outer),limf( (CHdata[AIL] - remote_normal_value) / 30.0f, -200.0, 200.0 ), ang.Y, 0, angle_max);
            // pitch.outer.Output = PID_Postion_Cal(&(pitch.outer), (CHdata[ELE] - remote_normal_value) / 25.0f, ang.X, 0, angle_max);
            //  yaw.outer.Output = PID_Postion_Cal(&(yaw.outer), (CHdata[RUD] - remote_normal_value) *(-80.0f),ang.Z, 0, angle_max);

        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //内环PID  200HZ

    PID_Set(&(roll.inner), roll.outer.Output, 0, gyro.Y, 50.0,200.0, 1000.0); //ROLL 内环 Z轴
    roll.inner.Output = PID_Postion_Cal(&(roll.inner));

    PID_Set(&(pitch.inner), pitch.outer.Output, 0, gyro.X, 50.0,200.0 ,1000.0); //PITCH 内环 X轴
    pitch.inner.Output = PID_Postion_Cal(&(pitch.inner));

    PID_Set(&(yaw.inner), yaw.outer.Output,0, gyro.Z, 50.0, 200.0 ,1000.0); //YAW 内环 Y轴
    yaw.inner.Output = PID_Postion_Cal(&(yaw.inner));
     



    //处理PWM数值，根据油门和是否解锁飞机来判断是否正常起飞，以及进行限幅

    //如图坐标系的水平安装情况下，垂直时坐标发生旋转


    if(Mode  == Vert)  //垂直状态
    {
        if(FlightMode == Stabilze)
        {

        PWM1 = CHdata[THR] - 100 + roll.inner.Output ;
        PWM2 = CHdata[THR] - 100 - roll.inner.Output;
        PWM3 = 1500  + (+ pitch.inner.Output ) * 0.5 + (+yaw.inner.Output) * 0.5;
        PWM4 = 1500  + (+ pitch.inner.Output ) * 0.5 + (-yaw.inner.Output) * 0.5;
        }
        else if(FlightMode == AltHold)
        {
         PWM1 = altTHR - 100 + roll.inner.Output ;
        PWM2 = altTHR - 100 - roll.inner.Output;
        PWM3 = 1500  + (+ pitch.inner.Output ) * 0.5 + (+yaw.inner.Output) * 0.5;
        PWM4 = 1500  + (+ pitch.inner.Output ) * 0.5 + (-yaw.inner.Output) * 0.5;
        }
    }
    else if(Mode == Aero)//固定翼状态
    {
//            PWM1 =  ch[2]+ roll.inner.Output ;
//            PWM2 = ch[2] - roll.inner.Output ;
//            PWM3 = 1500  - pitch.inner.Output   + yaw.inner.Output;
//            PWM4 = 1500  - pitch.inner.Output   - yaw.inner.Output;
    }

    PWM1  = limf(PWM1, 1100, 1950);
    PWM2  = limf(PWM2, 1100, 1950);
    PWM3  = limf(PWM3, 1000, 2000);
    PWM4  = limf(PWM4, 1000, 2000);
    if(myabs(PWM3 - lastPWM3) < 20)   PWM3 = lastPWM3; //舵机去抖动
    if(myabs(PWM4 - lastPWM4) < 20)   PWM4 = lastPWM4;

    if(State == Armed)  //锁定状态
    {
        PWM1 = motor_min_throttle;
        PWM2 = motor_min_throttle;
        MOTO1_SetPulse(PWM1)   ;   //油门最低
        MOTO2_SetPulse(PWM2)   ;
        SERVO1_SetPulse(3000 - PWM3)  ;
        SERVO2_SetPulse(PWM4)  ;

    }
    else if(CHdata[THR] < throttle_low_dead_value && State == DisArmed)  //油门杆在下面且 解锁了，此时怠速
    {
        PWM1 = motor_idle_throttle;  
        PWM2 = motor_idle_throttle;
        MOTO1_SetPulse(PWM1)   ;  //怠速油门
        MOTO2_SetPulse(PWM2)   ;
        SERVO1_SetPulse(3000 - PWM3)  ;
        SERVO2_SetPulse(PWM4)  ;
        SafeTask();   //解锁5秒后无动作重新锁定


    }
    else if(State == DisArmed)  //正常飞行
    {
   
        MOTO1_SetPulse(PWM1)   ;
        MOTO2_SetPulse(PWM2)   ;
        SERVO1_SetPulse(3000 - PWM3)  ;
        SERVO2_SetPulse(PWM4)  ;
    }


    lastPWM3  = PWM3;
    lastPWM4 = PWM4;


    loop_cnt ++;  //循环周期控制
    loop_cnt %= 200;  //限制计数变量

}

void  ControlInit()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 4999;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE );

    TIM_Cmd(TIM2, ENABLE);
}



void TIM2_IRQHandler(void) //TIM2 中断  5ms 200hz
{

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查 TIM2 更新中断发生与否
    {

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //清除 TIM2 更新中断标志
        Control();   //控制函数

    }
}









