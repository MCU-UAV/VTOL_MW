
#include "stm32f10x.h"

#include "usart.h"
#include "i2c_soft.h"
#include "nvic.h"
#include "pwm.h"
#include "systick.h"
#include "control.h"
#include "queue.h"
#include "filter.h"
#include "rc.h"
#include "stdio.h"
#include "imuAPI.h"
#include "math.h"
#include "PID.h"
#include "GroundStation.h"
#include "altHold.h"
#include "flightmode.h"

//存在问题：delay函数无效，有一定几率卡死，原因未知
//完成：PPM,Gyro,Angle,气压高度等值的读取，PID双环控制算法，四组PWM输出，系统总运行逻辑

//2018.6.5.15.10更新日志：
//1.修改了一系列BUG
//2.支持部分数据的SerialChart查看方式
//注意：定高部分虽然修改，但仍未测试


int main(void)
{
   uint16_t i,j;
   //串口初始化
	USART_Config();
	printf("usart is ready\r\n");
	printf ("TX->B10  RX->B11\n");
    
    
         
    //各中断优先级配置
	NVIC_PriorityConfig();
	printf("NVIC is ready\r\n");
    
     //部署延时函数
    delay_init ();
    printf("SysTick is ready\r\n");
    
	  
	//初始化模拟I2C
	I2C_MoniConfig();
	printf("I2C is ready!\n");
    printf("SCL->B3  SDA->B4");
    
    ///参数初始化
    parameterInit();
    printf("Parameter is ready\r\n");
    
    //遥控信号读取初始化
	RC_init();
    printf("RC is ready!\n");
    printf("Mode is PPM\n");
    
	//初始化电机控制引脚A8,A9,A10,A11
	MOTO_PwmConfig();
    SERVO_PwmConfig();
    printf("PWM is ready!\n");
    printf("A8,A9,A10,A11 for 400HZ\n");
    printf("B6,B7,B8,B9 for 50HZ\n");
    
    //控制函数启动（TIM2）
    ControlInit(); //200hz更新频率
    printf("Control System is ready\r\n");
        
    printf("--------------------\n");
    printf("All is well!\n");

 
	while(1)
    {
        for(i=  0;i<100;i++)
            for(j = 0;j<100;j++);
        /***地面站数据***/
            ANO_DT_Data_Exchange(); //地面站数据发送/接收
        /***基础数据***/
        //1.三轴加速度
        // printf("%.2f,%.2f,%.2f\n",acc.X,acc.Y,acc.Z);
        
        //2.三轴角速度
        //printf("%.2f,%.2f,%.2f\n",gyro.X,gyro.Y,gyro.Z);
        
        //3.三轴罗盘
        //printf("%.2f,%.2f,%.2f\n",mag.X,mag.Y,mag.Z);
        
        /***定高相关***/
        //当前重力加速度  z轴加速度（减去重力）定高输出油门
        //printf("%.2f,%d,%.2f\n",GravityAcc,acc_error,rate);  
        
        //定高输出油门 原始油门
        //printf("%d,%d\n", altTHR,CHdata[2]);
        
        //高度测量值 速度测量值 定高高度环输出 定高速度环输出
        //printf("%.2f,%.2f,%.2f,%.2f\n", barAltHoldHeight.Measure,barAltHoldRate.Measure,barAltHeightOut,barAltRateOut);
        
    }

}

