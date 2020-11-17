#include "queue.h"

Queue* createQueue(unsigned int capacity){
    Queue* queue = (Queue*)malloc( sizeof(Queue) );
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;
    queue->array = (int*)malloc( sizeof(int) );
    return queue;
}

QueueState isQueueEmpty(Queue* queue){
    QueueState queueState;
    if( queue->size == queue->capacity )
        queueState = Full;
    else if ( queue->size == 0 )
        queueState = Empty;
    else
        queueState = Available;
    return queueState;
}

QueueState enQueue(Queue* queue, int item){
    QueueState queueState = isQueueEmpty(queue);
    if( queueState == Full )
        return queueState;
    queue->rear = ( queue->rear + 1 ) % queue->capacity;
    queue->array[queue->rear] = item;
    queue->size++;
    return queueState;
}

QueueState deQueue(Queue* queue, int* item){
    QueueState queueState = isQueueEmpty(queue);
    if( queueState == Empty )
        return queueState;
    *item = queue->array[queue->front];
    queue->front = ( queue->front + 1 ) % queue->capacity;
    queue->size--;
    return queueState;
}

QueueState frontQueue(Queue* queue, int* item){
    QueueState queueState = isQueueEmpty(queue);
    if( queueState == Empty )
        return queueState;
    *item = queue->array[queue->front];
    return queueState;
}

QueueState rearQueue(Queue* queue, int* item){
    QueueState queueState = isQueueEmpty(queue);
    if( queueState == Empty )
        return queueState;
    *item = queue->array[queue->rear];
    return queueState;
}
