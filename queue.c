#include "queue.h"

//-----------create-new-queue----------//
Queue* createQueue(unsigned int capacity){
    Queue* queue = (Queue*)malloc( sizeof(Queue) );
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;
    queue->array = (int*)malloc( sizeof(int) );
    return queue;
}

//-----------check-queue-empty-or-full----------//
QueueState checkQueue(Queue* queue){
    QueueState queueState;
    if( queue->size == queue->capacity )
        queueState = Full;
    else if ( queue->size == 0 )
        queueState = Empty;
    else
        queueState = Available;
    return queueState;
}

//-----------add-new-value-to-queue----------//
QueueState enQueue(Queue* queue, int item){
    QueueState queueState = checkQueue(queue);
    if( queueState == Full )
        return queueState;
    queue->rear = ( queue->rear + 1 ) % queue->capacity;
    queue->array[queue->rear] = item;
    queue->size++;
    return queueState;
}

//-----------remove-earliest-updated-value-to-queue----------//
QueueState deQueue(Queue* queue, int* item){
    QueueState queueState = checkQueue(queue);
    if( queueState == Empty )
        return queueState;
    *item = queue->array[queue->front];
    queue->front = ( queue->front + 1 ) % queue->capacity;
    queue->size--;
    return queueState;
}

//-----------get-front-value-and-keep----------//
QueueState getFront(Queue* queue, int* item){
    QueueState queueState = checkQueue(queue);
    if( queueState == Empty )
        return queueState;
    *item = queue->array[queue->front];
    return queueState;
}

//-----------get-rear-value-and-keep----------//
QueueState getRear(Queue* queue, int* item){
    QueueState queueState = checkQueue(queue);
    if( queueState == Empty )
        return queueState;
    *item = queue->array[queue->rear];
    return queueState;
}

//--------------delete-queue-----------------//
void deleteQueue(Queue* queue){
    for(unsigned int i = 0; i < queue->size; i++ ){
    free(queue->array + i);
    }
    free(queue);
}
