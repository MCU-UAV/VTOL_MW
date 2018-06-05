#include "stdio.h" 
#include "stdlib.h"    
#include "queue.h"  

  
u8 InitQueue(SqQueue *Q)   //建立一个循环队列
{
	
	Q->front = Q->rear = 0;
	return OK;
}


u8 QueueLength(SqQueue Q)   //得到队列长度
{
	return (Q.rear - Q.front + MAXQSIZE) %MAXQSIZE;
}

u8 EnQueue(SqQueue *Q, u16 e)   //插入一个元素
{
	if (( (Q->rear) + 1) % MAXQSIZE == Q->front) return ERROR; // 队列满
	Q->base[Q->rear] = e;
	Q->rear = ( (Q->rear) + 1) % MAXQSIZE;
		return OK;
}

u8 DeQueue(SqQueue *Q)       //删除一个元素
{
	if (Q->front == Q->rear) return ERROR;
	Q->front = (Q->front + 1) % MAXQSIZE;
	return OK;
}



