/*********************************************************
 @本程序只供学习使用，未经作者许可，不得用于其它任何用途
 +编者：普哥
 +描述：高度控制
 +编写日期：2017/3/10
 +版权所有，盗版必纠。
**********************************************************/
#include "altHold.h"

#include "rc.h"
#include "filter.h"
#include "imuAPI.h"
#include "flightmode.h"
#include "control.h"
#include "PID.h"

#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 50
uint16_t initialThrottleHold = 0;
float rate = 0;
uint16_t altTHR = 0;
uint16_t acc_offset = 0;
int16_t acc_error;

PID_DATA barAltHoldRate;
PID_DATA barAltHoldHeight;

float barAltRateOut = 0;
float barAltHeightOut = 0;

void AltHoldChange()
{

        static uint8_t isAltHoldChanged = 0;

        static int16_t AltHoldCorr = 0;

        if (myabs(CHdata[THR] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
        {


            AltHoldCorr += CHdata[THR] - initialThrottleHold;

            if(myabs(AltHoldCorr) > 512) {

                barAltHoldHeight.Desire += AltHoldCorr / 512;

                AltHoldCorr %= 512;

            }

            isAltHoldChanged = 1;

        }
        else if (isAltHoldChanged) {

            barAltHoldHeight.Desire = add.PreHeight;

            isAltHoldChanged = 0;

        }

        altTHR = initialThrottleHold + barAltRateOut;

    
}
void AltDataDeal()  //100HZ
{

    

   
    add.PreHeight = MovMiddle(add.PreHeight);

    {   //获取垂直速度数据

        static uint8_t cnt = 0;
        const uint8_t RATE_AVR_NUM = 25; //将得到的加速度数据做25组滑动均值滤波
        static 	int16_t average[RATE_AVR_NUM];

        uint8_t i;
        int32_t sum = 0;
        int16_t max = -32767;
        int16_t min = 32767;

        acc_error = getNormAccZ(quad, acc) - GravityAcc;  //加速度单位cm/s^2

        average[cnt] = acc_error;

        cnt++;

        if(cnt >= RATE_AVR_NUM)
        {
            cnt = 0;
        }

        for(i = 0; i < RATE_AVR_NUM; i++) //排除极值对均值的干扰
        {
            sum += average[i];
            if(average[i] > max)
                max = average[i];
            if(average[i] < min)
                min = average[i];
        }
        acc_error = (sum - max - min) / (RATE_AVR_NUM - 2);



        if(acc_error > 300 || acc_error < -300)	//加速度太小，判定为抖动
        {
            rate += (float)acc_error * 0.01;	//累加速度值  速度m/s
        }

        {   //此处做一个速度与高度的互补滤波
            const float factor = 1.0f; //100hz cm换m/s 系数为1
            static float last_high;

            barAltHoldRate.Measure = rate + (add.PreHeight - last_high) * factor ; 
            last_high =  add.PreHeight;
        }
        barAltHoldHeight.Measure = add.PreHeight;

    }
  
}


