#ifndef __QUEUE_H_  
#define __QUEUE_H_  
#include "stm32f10x.h"
#define OK 1
#define ERROR 0
#define QLen 9
#define MAXQSIZE (QLen)

typedef struct queue   
{  
    u16 base[QLen];  
    int front;    //指向队列第一个元素  
    int rear;    //指向队列最后一个元素的下一个元素  
}SqQueue;  
  
u8 InitQueue(SqQueue *Q);  
u8 QueueLength(SqQueue Q);
u8 EnQueue(SqQueue *Q, u16 e);
u8 DeQueue(SqQueue *Q);
 
#endif  


