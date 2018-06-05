#ifndef _IMU_API_H
#define _IMU_API_H

#include "stm32f10x.h"
#define SAVE 			0x00
#define CALSW 		0x01
#define RSW 			0x02
#define RRATE			0x03
#define BAUD 		0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define MAGRANGX	0x1c
#define MAGRANGY	0x1d
#define MAGRANGZ	0x1e
#define BANDWIDTH	0x1f
#define GYRORANGE	0x20
#define ACCRANGE	0x21
#define SLEEP			0x22
#define ORIENT		0x23
#define AXIS6		  0x24
#define FILTK			0x25
#define GPSBAUD		0x26
#define READADDR	0x27
#define MOVETHR		0x28
#define MOVESTA		0x29
#define ACCFILT		0x2A
#define GYROFILT	0x2b
#define MAGFILT		0x2c
#define RSV6			0x2d
#define RSV7			0x2e
#define RSV8			0x2f
#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS				0x33
#define AX				0x34
#define AY				0x35
#define AZ				0x36
#define GX				0x37
#define GY				0x38
#define GZ				0x39
#define HX				0x3a
#define HY				0x3b
#define HZ				0x3c			
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw				0x3f
#define TEMP				0x40
#define D0Status			0x41
#define D1Status			0x42
#define D2Status			0x43
#define D3Status			0x44
#define PressureL			0x45
#define PressureH			0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight   		0x4d
#define GPSYAW      		0x4e
#define GPSVL				0x4f
#define GPSVH			0x50
#define q0				0x51
#define q1				0x52
#define q2				0x53
#define q3				0x54
#define SVNUM			0x55
#define PDOP				0x56
#define HDOP				0x57
#define VDOP				0x58
#define DELAYT			0x59
#define XMIN				0x5a
#define XMAX				0x5b
#define GXMIN			0x5c
#define GXMAX			0x5d
#define YMIN				0x5e
#define YMAX				0x5f
#define GYMIN			0x60
#define GYMAX			0x61
#define ALARMLEVEL		0x62
#define GYRONOCAL		0x63



typedef struct IMUdata
{
	float X,Y,Z;
}IMUdata;

typedef struct QuadNum
{
    float Q0,Q1,Q2,Q3;
}QuadNum;
typedef struct DegreeUnit
{
    double minute;
    double second;
}DegreeUnit;
typedef struct IMU
{
	IMUdata IMU_Acc;
    IMUdata IMU_Grro;	
	IMUdata IMU_Angle;
	IMUdata IMU_Mag;
    QuadNum IMU_Quad;
}IMU;

typedef struct Time
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
}Time;

typedef struct addData
{
 	DegreeUnit Lon,Lat;
	float GPSheight;
	float GPSYaw;
	float GPSV;
	uint16_t SN;   //卫星数
    float GPS_PDOP;    //位置定位精度
    float GPS_HDOP;     //水平定位精度
    float GPS_VDOP;    //垂直定位精度
	float Pressure;
	float PreHeight;
}AddData;

extern IMUdata lastgg;
extern float GravityAcc;

void getAngle(IMUdata  *aa);
void getQuad(QuadNum *qq);
void Quad2Angle(QuadNum qq,IMUdata *aa);
float getNormAccZ(QuadNum qq,IMUdata acc);
float getGravityAcc(void);
void getAddData(AddData *add);
void getGyro(IMUdata *gg);
void getMag(IMUdata *mm);
void getAcc(IMUdata *aa);
void setCalibration(uint8_t setState);
#endif

