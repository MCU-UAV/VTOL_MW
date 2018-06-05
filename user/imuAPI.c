#include "stm32f10x.h"
#include "i2c_soft.h"
#include "imuAPI.h"
#include "usart.h"
#include "math.h"
#include "systick.h"
#define IMUREADADDR  0x50
#define GRAVITY 9.8f
#define AerFa 0.5
IMUdata lastgg;
float GravityAcc; //重力加速度
/********************************************************
往从机中的寄存器写入一个字节的数据
*addr:设备地址
*reg：寄存器地址
*data：要写入的数据
*返回值：0为正常写入，1为写入异常
**********************************************************/
//u8 MoniI2c_WriteByteToSlave(u8 addr,u8 reg,u8 data)

/************************************************************
往从机中的寄存器写入多个数据
*addr：器件的地址
*reg：首个写入数据的寄存器地址
*len：要写入的数据的个数
*buf：要写入的数据区的首地址
*返回值：成功写入则返回0，失败则返回1
**************************************************************/
//u8 MoniI2c_WriteSomeDataToSlave(u8 addr,u8 reg,u8 len,u8 *buf)

/********************************************************
往从机中的寄存器读取一个字节的数据
*addr:设备地址
*reg：寄存器地址
*buf:读取到的数据存储的内存区
*返回值：0为正常读取，1为读取异常
**********************************************************/
//u8 MoniI2c_ReadFromSlave(u8 addr,u8 reg,u8 *buf)

/************************************************************************
从从机中读取采样到的高8位和低8位的数据

*addr：器件的地址
*reg：要读取数据的首个数据的寄存器地址
*len：读取的数据个数
*buf：读取到的8位数据存放区的地址，即通过指针的方式将读取到的数据返回给调用者。
*返回值：成功读取则返回0，失败则返回1
*************************************************************************/
//u8 MoniI2c_ReadSomeDataFromSlave(u8 addr,u8 reg,u8 len,u8 *buf)

/********************************************************************************************/
/********************************************************************************************/

void ShortToChar(short sData, unsigned char cData[])
{
    cData[0] = sData & 0xff;
    cData[1] = sData >> 8;
}
short CharToShort(unsigned char cData[])
{
    return ((short)cData[1] << 8) | cData[0];
}

//需要实现的API
//1.

//功能：返回系统时间
//参数：结构体指针
void getTime(Time *tt)
{
}

//2.

//功能：独立返回加速度
//参数：结构体指针
void getAcc(IMUdata *aa)
{
    uint8_t temp[6];

    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, AX, 6, temp);

    aa->X = (float)CharToShort(&temp[0]) / 32768 * 16 * GRAVITY;
    aa->Y = (float)CharToShort(&temp[2]) / 32768 * 16 * GRAVITY;
    aa->Z = (float)CharToShort(&temp[4]) / 32768 * 16 * GRAVITY;
}

//3.
//功能：独立返回角速度
//参数：结构体指针
void getGyro(IMUdata *gg)
{
    uint8_t temp[6];
    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, GX, 6, temp);
    gg->X = AerFa * (float)CharToShort(&temp[0]) / 32768 * 2000 + (1 - AerFa) * lastgg.X;
    gg->Y = AerFa * (float)CharToShort(&temp[2]) / 32768 * 2000 + (1 - AerFa) * lastgg.Y;
    gg->Z = AerFa * (float)CharToShort(&temp[4]) / 32768 * 2000 + (1 - AerFa) * lastgg.Z;
    lastgg.X = gg->X;
    lastgg.Y = gg->Y;
    lastgg.Z = gg->Z;
}

//4.
//功能：独立返回角度
//参数：结构体指针
void getAngle(IMUdata  *aa)
{
    uint8_t  temp[6];
    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, Roll, 6, temp);



    aa->X = (float)((int16_t)(temp[1] << 8) | temp[0]) / 32768.0 * 180.0;
    aa->Y = (float)((int16_t)(temp[3] << 8) | temp[2]) / 32768.0 * 180.0;
    aa->Z = (float)((int16_t)(temp[5] << 8) | temp[4]) / 32768.0 * 180.0;
}

//5.
//功能：独立返回磁场
//参数：结构体指针
void getMag(IMUdata *mm)
{
    uint8_t temp[6];
    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, HX, 6, temp);
    mm->X = (float)CharToShort(&temp[0]);
    mm->Y = (float)CharToShort(&temp[2]);
    mm->Z = (float)CharToShort(&temp[4]);
}

//6.
//功能：iic连续读取获得所有IMU数据
//参数：结构体指针
void getIMU(IMU *ii)
{
}

//7
//功能：独立返回四元数
//参数 ： 结构体指针
void getQuad(QuadNum *qq)
{
    u8 temp[8];
    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, q0, 8, temp);

    qq->Q0 = (float)((short)(temp[1] << 8) | temp[0]) / 32768.0f;
    qq->Q1 = (float)((short)(temp[3] << 8) | temp[2]) / 32768.0f;
    qq->Q2 = (float)((short)(temp[5] << 8) | temp[4]) / 32768.0f;
    qq->Q3 = (float)((short)(temp[7] << 8) | temp[6]) / 32768.0f;

}
float getNormAccX(QuadNum qq, IMUdata acc)
{
    IMUdata vec;
    vec.X  = qq.Q0 * qq.Q0 + qq.Q1 * qq.Q1 - qq.Q2 * qq.Q2 - qq.Q3 * qq.Q3; //地理坐标系X轴加速度
    vec.Y = 2 * qq.Q1 * qq.Q2 - 2 * qq.Q3 * qq.Q0; /*矩阵(3,2)项*/
    vec.Z =  2 * qq.Q1 * qq.Q3 + 2 * qq.Q2 * qq.Q0 ;	/*矩阵(3,3)项*/
    return (acc.X * vec.X + acc.Y * vec.Y + acc.Z * vec.Z) * 100; //cm/s^2
}

float getNormAccY(QuadNum qq, IMUdata acc)
{
    IMUdata vec;
    vec.X =  2 * qq.Q1 * qq.Q2 + 2 * qq.Q0 * qq.Q3;             //地理坐标系Y轴加速度
    vec.Y = qq.Q0 * qq.Q0 - qq.Q1 * qq.Q1 + qq.Q2 * qq.Q2 - qq.Q3 * qq.Q3; ;
    vec.Z = 2 * qq.Q2 * qq.Q3 - 2 * qq.Q1 * qq.Q0 ;
    return (acc.X * vec.X + acc.Y * vec.Y + acc.Z * vec.Z) * 100; //cm/s^2

}

float getNormAccZ(QuadNum qq, IMUdata acc)
{
    IMUdata vec;

    vec.X = 2 * qq.Q1 * qq.Q3 - 2 * qq.Q0 * qq.Q2 ;                //地理坐标系Z轴加速度
    vec.Y = 2 * qq.Q2 * qq.Q3 + 2 * qq.Q0 * qq.Q1 ;
    vec.Z = qq.Q0 * qq.Q0 - qq.Q1 * qq.Q1 - qq.Q2 * qq.Q2 + qq.Q3 * qq.Q3;

    return (acc.X * vec.X + acc.Y * vec.Y + acc.Z * vec.Z) * 100;	/*Z轴加速度*/ //cm/s^2
}

float getGravityAcc(void)
{
    float medAcc = 0;
    IMUdata acc;
    QuadNum qq;
    uint8_t i = 0;
    for(i=0;i<200;i++)
    {
        getAcc(&acc);
        getQuad(&qq);
        medAcc += getNormAccZ(qq,acc);
        delay_ms(1);  //延时1毫秒，保证数据的平均性
    }
    medAcc /= 200.0;
    return medAcc;
      
}
//8.
//功能：iic连续读取获得所有附加数据
//参数：结构体指针
void getAddData(AddData *add)
{
    u8 temp[24];
    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, PressureL, 24, temp);

    add->Pressure  = ((uint32_t )temp[3] << 24) | ((uint32_t )temp[2] << 16) | ((uint32_t )temp[1] << 8) | temp[0];  //Pa
    add->PreHeight  = (int32_t )(((int32_t )temp[7] << 24) | ((int32_t )temp[6] << 16) | ((int32_t )temp[5] << 8) | temp[4]);  //cm

    add->Lon.minute =  (double)(((uint32_t )temp[11] << 24) | ((uint32_t )temp[10] << 16) | ((uint32_t )temp[9] << 8) | temp[8]) / 10000000.0;
    add->Lon.second = (double)(((((uint32_t )temp[11] << 24) | ((uint32_t )temp[10] << 16) | ((uint32_t )temp[9] << 8) | temp[8])) % 10000000) / 100000.0;
    add->Lat.minute =  (double)(((uint32_t )temp[15] << 24) | ((uint32_t )temp[14] << 16) | ((uint32_t )temp[13] << 8) | temp[12]) / 10000000.0;
    add->Lat.second = (double)(((((uint32_t )temp[15] << 24) | ((uint32_t )temp[14] << 16) | ((uint32_t )temp[13] << 8) | temp[12])) % 10000000) / 100000.0;
    add->GPSheight  = (float)(((uint16_t)temp[17] << 8) | temp[16]) / 10;
    add->GPSYaw = (float)(((uint16_t)temp[19] << 8) | temp[18]) / 10;
    add->GPSV = (float)(((uint32_t )temp[23] << 24) | ((uint32_t )temp[22] << 16) | ((uint32_t )temp[21] << 8) | temp[20]) / 1000.0;

    MoniI2c_ReadSomeDataFromSlave(IMUREADADDR, SVNUM, 8, temp);

    add->SN = ((uint16_t)temp[1] << 8) | temp[0];
    add->GPS_PDOP = (float)(((uint16_t)temp[3] << 8) | temp[2]) / 100.0;
    add->GPS_HDOP = (float)(((uint16_t)temp[5] << 8) | temp[4]) / 10.0;
    add->GPS_VDOP = (float)(((uint16_t)temp[7] << 8) | temp[6]) / 10.0;
}

//9.
//功能：保持设置
//参数：0：保持当前配置 1：恢复默认设置
void keepSet(uint8_t setState)
{
}

//10.
//功能：设置校准
//参数：0：退出 1：加速度校准 2：磁场校准 3：高度置0
void setCalibration(uint8_t setState)
{
    uint8_t com[2] = {0x00, 0x00};
    com[0] = setState;
    MoniI2c_WriteSomeDataToSlave(IMUREADADDR, CALSW, 2, com);

}
//11.
//功能：设置安装方向
//参数：0：水平 1：垂直
void setDirection(uint8_t setState)
{
}
//12.
//功能：陀螺仪自动校准
//参数：0：开启 1：关闭
void GyroAutoCal(uint8_t setState)
{
}
//13.
//功能：设置三轴加速度零偏
//返回值：1：成功 0：失败
uint8_t setAccOffset(void)
{
    return 0;
}
//14.
//功能：设置三轴角速度零偏
//返回值：1：成功 0：失败
uint8_t setGyroOffset(void)
{
    return 0;
}
//15.
//功能：设置三轴磁场零偏
//返回值：1：成功 0：失败
uint8_t setMagOffset(void)
{
    return 0;
}
//17.
//功能：设置D0,D1连接GPS
void setOpenGPS(void)
{
}

//16.
//功能：设置GPS通信速度
//参数：波特率
void setGPSBaud(uint16_t GPSBaud)
{
}

//18.
//功能：四元数转角度
//参数：角度结构体指针 四元数结构体指针
void Quad2Angle(QuadNum qq, IMUdata *aa)
{
    //Z-Y-X 默认
    aa->X = atan2(2 * qq.Q2  * qq.Q3  + 2 * qq.Q0  * qq.Q1 , -2 * qq.Q1  * qq.Q1  - 2 * qq.Q2 * qq.Q2  + 1) * 57.217f;
    aa->Y = asin(-2 * qq.Q1  * qq.Q3  + 2 * qq.Q0 * qq.Q2 ) * 57.217f;
    aa->Z = atan2(2 * qq.Q1 *  qq.Q2 + 2 *  qq.Q0  *  qq.Q3 , -2 *  qq.Q2 * qq.Q2  - 2 *  qq.Q3 *  qq.Q3  + 1) * 57.217f;
    //Z-X-Y
//     aa->X= atan2(2 * qq.Q1  *qq.Q3  + 2 * qq.Q0  * qq.Q2 , -2 * qq.Q1  * qq.Q1  - 2 * qq.Q2 * qq.Q2  + 1)* 57.217f;
//     aa->Y= asin(-2 * qq.Q2  * qq.Q3  + 2 * qq.Q0 * qq.Q1 )* 57.217f;
//     aa->Z=atan2(2 * qq.Q1 *  qq.Q2 + 2 *  qq.Q0  *  qq.Q3 , -2 *  qq.Q1 * qq.Q1  - 2 *  qq.Q3 *  qq.Q3  + 1)* 57.217f;

}



