#include "stdio.h" 
#include "stdlib.h"    
#include "queue.h"  

  
u8 InitQueue(SqQueue *Q)   //����һ��ѭ������
{
	
	Q->front = Q->rear = 0;
	return OK;
}


u8 QueueLength(SqQueue Q)   //�õ����г���
{
	return (Q.rear - Q.front + MAXQSIZE) %MAXQSIZE;
}

u8 EnQueue(SqQueue *Q, u16 e)   //����һ��Ԫ��
{
	if (( (Q->rear) + 1) % MAXQSIZE == Q->front) return ERROR; // ������
	Q->base[Q->rear] = e;
	Q->rear = ( (Q->rear) + 1) % MAXQSIZE;
		return OK;
}

u8 DeQueue(SqQueue *Q)       //ɾ��һ��Ԫ��
{
	if (Q->front == Q->rear) return ERROR;
	Q->front = (Q->front + 1) % MAXQSIZE;
	return OK;
}



