#include "queuell.h"

struct Qnode* newNode(int value){
    struct Qnode* newnode = (struct Qnode*)malloc(sizeof(struct Qnode));
    newnode->value = value;
    newnode->next = NULL;
    return newnode;
}

struct Queue* createQueue(void){
    struct Queue* queue = (struct Queue*)malloc(sizeof(struct Queue));
    queue->rear = queue->front = NULL;
    return queue;
}

QueueState checkQueue(struct Queue* queue){
    QueueState queuestate;
    if( queue->rear == NULL || queue->front == NULL )
        queuestate = Empty;
    else
        queuestate = Available;
    return queuestate;
}


QueueState enQueue(struct Queue* queue, int value){
    struct Qnode* newnode = newNode(value);
    QueueState queuestate = checkQueue(queue);
    if( queuestate == Empty ){
        queue->rear = queue->front = newnode;
        return queuestate;
    }
    queue->rear->next = newnode;
    queue->rear = newnode;
    return queuestate;
}

QueueState deQueue(struct Queue* queue, int* value){
    QueueState queuestate = checkQueue(queue);
    if( queuestate == Empty )
        return queuestate;
    struct Qnode* oldnode = queue->front;
    *value = oldnode->value;
    queue->front = queue->front->next;
    queuestate = checkQueue(queue);
    if( queue->front == NULL ){
        queue->rear = queue->front = NULL;
    }
    free(oldnode);
    return queuestate;
}

QueueState getRear(struct Queue* queue, int* value){
    QueueState queuestate = checkQueue(queue);
    if( queuestate == Empty )
        return queuestate;
    *value = queue->rear->value;
    return queuestate;
}

QueueState getFront(struct Queue* queue, int* value){
    QueueState queuestate = checkQueue(queue);
    if( queuestate == Empty )
        return queuestate;
    *value = queue->front->value;
    return queuestate;
}

void deleteQueue(struct Queue* queue){
    int temp;
    while( checkQueue(queue) != Empty )
    deQueue(queue,&temp);
    free(queue);
}
