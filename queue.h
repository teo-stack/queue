#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern "C"
{
#endif
//-----------------libraries--------------//
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

//-----------------type------------------//
typedef enum{
  Empty,
  Available,
  Full
}QueueState;

//----------------struct-----------------//
typedef struct{
    unsigned int front, rear, size;
    unsigned int capacity;
    int* array;
}Queue;

//----------------fuction----------------//
extern Queue* createQueue(unsigned int capacity);
extern QueueState checkQueue(Queue* queue);
extern QueueState enQueue(Queue* queue, int item);
extern QueueState deQueue(Queue* queue, int* item);
extern QueueState getFront(Queue* queue, int* item);
extern QueueState getRear(Queue* queue, int* item);
extern void deleteQueue(Queue* queue);
#ifdef __cplusplus
}
#endif
#endif // QUEUE_H
