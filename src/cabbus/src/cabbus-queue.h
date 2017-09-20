#ifndef CABBUS_QUEUE_H
#define CABBUS_QUEUE_H

#include "cabbus-constants.h"

typedef struct 
{
	uint8_t pkt[CABBUS_BUFFER_SIZE];
	uint8_t len;
} CabBusPacket;

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t tailIdx;
	volatile uint8_t full;
	CabBusPacket* pktBufferArray;
	uint8_t pktBufferArraySz;
} CabBusPktQueue;

void cabBusPktQueueInitialize(CabBusPktQueue* q, CabBusPacket* pktBufferArray, uint8_t pktBufferArraySz);
uint8_t cabBusPktQueueDepth(CabBusPktQueue* q);
uint8_t cabBusPktQueuePush(CabBusPktQueue* q, uint8_t* data, uint8_t dataLen);
uint8_t cabBusPktQueuePopInternal(CabBusPktQueue* q, uint8_t* data, uint8_t dataLen, uint8_t snoop);
uint8_t cabBusPktQueueDrop(CabBusPktQueue* q);

#define cabBusPktQueueFull(q) ((q)->full?1:0)
#define cabBusPktQueueEmpty(q) (0 == cabBusPktQueueDepth(q))

#define cabBusPktQueuePeek(q, data, dataLen) cabBusPktQueuePopInternal((q), (data), (dataLen), 1)
#define cabBusPktQueuePop(q, data, dataLen) cabBusPktQueuePopInternal((q), (data), (dataLen), 0)


#endif
