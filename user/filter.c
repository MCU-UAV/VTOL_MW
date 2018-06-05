#include "filter.h"
#include "queue.h"
#include "string.h"

SqQueue CH_Queue[9];   //8���˲�����


void FilterInit()          //�˲���ʼ��
{
	u8 InitQueueIndex = 0, DefaultQueueIndex = 0;
	for (InitQueueIndex = 0; InitQueueIndex < 8; InitQueueIndex++)
	{
		InitQueue(&CH_Queue[InitQueueIndex]);
		for (DefaultQueueIndex = 0; DefaultQueueIndex < 8; DefaultQueueIndex++)  //��ʼʱ�����ֻʣһ����λ���ȴ�����
		{
			EnQueue(&CH_Queue[InitQueueIndex],1500) ;          //Ĭ��ֵ��Ϊ1500
		}
	}
}

u16 GetMedianNum(u16 * bArray, int iFilterLen)
{
	u8 i, j;// ѭ������  
	u16 bTemp;

	// ��ð�ݷ��������������  
	for (j = 0; j < iFilterLen - 1; j++)
	{
		for (i = 0; i < iFilterLen - j - 1; i++)
		{
			if (bArray[i] > bArray[i + 1])
			{
				// ����  
				bTemp = bArray[i];
				bArray[i] = bArray[i + 1];
				bArray[i + 1] = bTemp;
			}
		}
	}

 
	bTemp = bArray[(iFilterLen + 1) / 2 ];


	return bTemp;
} 

/*----------------------------------------------------------
 + ʵ�ֹ���:float�������ݻ��������˲�
 + ���ò�����in����������� moavarray[]:������������ len����ȡ���ݸ��� fil_cnt[2]:�����±� 
 + ���ز�����*out���������λ��
----------------------------------------------------------*/
 void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}


/*=================================== ================================================================*/
/*====================================================================================================*
**���� : ��ֵ�˲�
**���� : 
**���� : 
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
int16_t MovMiddle(int16_t input)
{	
	uint8_t i,j;
	const uint8_t MOV_MIDDLE_NUM = 5;
	static int16_t middle[5]={0};
	int16_t middle_t[5];
//	MOV_MIDDLE_NUM = pidHeightRate.ki;
	for(i=1;i<MOV_MIDDLE_NUM;i++)
	{
		 middle[i-1] =  middle[i];
	}
	middle[MOV_MIDDLE_NUM-1] = input;
	memcpy(middle_t,middle,MOV_MIDDLE_NUM*sizeof(uint32_t));
	for(i=0;i<MOV_MIDDLE_NUM-1;i++)
	{
		for(j=i+1;j<MOV_MIDDLE_NUM;j++)
		{
			if(middle_t[i] > middle_t[j])
			{
				middle_t[i] ^= middle_t[j];
				middle_t[j] ^= middle_t[i];
				middle_t[i] ^= middle_t[j];
			}
		}
	}
	return middle_t[(MOV_MIDDLE_NUM+1)>>1];
}	


