#ifndef XPRESSNET_QUEUE_H
#define XPRESSNET_QUEUE_H

#include "xpressnet-constants.h"

typedef struct 
{
	uint8_t pkt[XPRESSNET_BUFFER_SIZE];
	uint8_t len;
} XpressNetPacket;

typedef struct
{
	volatile uint8_t headIdx;
	volatile uint8_t tailIdx;
	volatile uint8_t full;
	XpressNetPacket* pktBufferArray;
	uint8_t pktBufferArraySz;
} XpressNetPktQueue;

void xpressnetPktQueueInitialize(XpressNetPktQueue* q, XpressNetPacket* pktBufferArray, uint8_t pktBufferArraySz);
uint8_t xpressnetPktQueueDepth(XpressNetPktQueue* q);
uint8_t xpressnetPktQueuePush(XpressNetPktQueue* q, uint8_t* data, uint8_t dataLen);
uint8_t xpressnetPktQueuePopInternal(XpressNetPktQueue* q, uint8_t* data, uint8_t dataLen, uint8_t snoop);
uint8_t xpressnetPktQueueDrop(XpressNetPktQueue* q);

#define xpressnetPktQueueFull(q) ((q)->full?1:0)
#define xpressnetPktQueueEmpty(q) (0 == xpressnetPktQueueDepth(q))

#define xpressnetPktQueuePeek(q, data, dataLen) xpressnetPktQueuePopInternal((q), (data), (dataLen), 1)
#define xpressnetPktQueuePop(q, data, dataLen) xpressnetPktQueuePopInternal((q), (data), (dataLen), 0)


#endif
