#ifdef __AVR__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#endif

#include <stdlib.h>
#include <string.h>

#include "xpressnet-constants.h"
#include "xpressnet-queue.h"
#include "xpressnet-macros.h"

void xpressnetPktQueueInitialize(XpressNetPktQueue* q, XpressNetPacket* pktBufferArray, uint8_t pktBufferArraySz)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		q->pktBufferArray = pktBufferArray;
		q->pktBufferArraySz = pktBufferArraySz;
		q->headIdx = q->tailIdx = 0;
		q->full = 0;
		memset(q->pktBufferArray, 0, pktBufferArraySz * sizeof(XpressNetPacket));
	}
}

uint8_t xpressnetPktQueueDepth(XpressNetPktQueue* q)
{
	uint8_t result = 0;
	if(q->full)
		return(q->pktBufferArraySz);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		result = (uint8_t)(q->headIdx - q->tailIdx) % q->pktBufferArraySz;
	}
	return(result);
}

uint8_t xpressnetPktQueuePush(XpressNetPktQueue* q, uint8_t* data, uint8_t dataLen)
{
	uint8_t* pktPtr;
	// If full, bail with a false
	if (q->full)
		return(0);

	dataLen = min(XPRESSNET_BUFFER_SIZE, dataLen);
	pktPtr = (uint8_t*)q->pktBufferArray[q->headIdx].pkt;
	memcpy(pktPtr, data, dataLen);
	memset(pktPtr+dataLen, 0, XPRESSNET_BUFFER_SIZE - dataLen);
	q->pktBufferArray[q->headIdx].len = dataLen;

	if( ++q->headIdx >= q->pktBufferArraySz )
		q->headIdx = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (q->headIdx == q->tailIdx)
			q->full = 1;
	}
	return(1);
}

uint8_t xpressnetPktQueuePopInternal(XpressNetPktQueue* q, uint8_t* data, uint8_t dataLen, uint8_t snoop)
{
	memset(data, 0, dataLen);
	if (0 == xpressnetPktQueueDepth(q))
		return(0);

	uint8_t length = min(dataLen, q->pktBufferArray[q->tailIdx].len);
	memcpy(data, (uint8_t*)&(q->pktBufferArray[q->tailIdx].pkt), length);

	// Snoop indicates that we shouldn't actually pop the packet off - just copy it out
	if (snoop)
		return(length);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if( ++q->tailIdx >= q->pktBufferArraySz )
			q->tailIdx = 0;
		q->full = 0;
	}

	return(length);
}

uint8_t xpressnetPktQueueDrop(XpressNetPktQueue* q)
{
	if (0 == xpressnetPktQueueDepth(q))
		return(0);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if( ++q->tailIdx >= q->pktBufferArraySz )
			q->tailIdx = 0;
		q->full = 0;
	}
	return(1);
}

