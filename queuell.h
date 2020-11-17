#ifndef QUEUELL_H
#define QUEUELL_H

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
  Available
}QueueState;

//----------------struct-----------------//
struct Qnode{
    int value;
    struct QNode* next;
};

struct Queue{
    struct Qnode *front, *rear;
};


//----------------fuction----------------//
extern struct Qnode* newNode(int value);
extern struct Queue* createQueue(void);
extern QueueState checkQueue(struct Queue* queue);
extern QueueState enQueue(struct Queue* queue, int value);
extern QueueState deQueue(struct Queue* queue, int* value);
extern QueueState getRear(struct Queue* queue, int* value);
extern QueueState getFront(struct Queue* queue, int* value);
#ifdef __cplusplus
}
#endif
#endif // QUEUE_H
