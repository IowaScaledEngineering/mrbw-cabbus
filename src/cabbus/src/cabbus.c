/*************************************************************************
Title:    Cab Bus Atmel AVR
Authors:  Michael Petersen <railfan@drgw.net>, Colorado, USA
          Nathan Holmes <maverick@drgw.net>, Colorado, USA
File:     cabbus.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Nathan Holmes, Michael Petersen

    UART code derived from AVR UART library by Peter Fleury, and as
    modified by Tim Sharpe.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along
    with this program. If not, see http://www.gnu.org/licenses/
    
*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "cabbus.h"

#define CABBUS_STATUS_BROADCAST_CMD        0x01
#define CABBUS_STATUS_TRANSMITTING         0x02
#define CABBUS_STATUS_XPRESSNET            0x04
#define CABBUS_STATUS_COLLISION_DETECTED   0x10
#define CABBUS_STATUS_RESPONSE_SENT        0x20
#define CABBUS_STATUS_PING_RECEIVED        0x40
#define CABBUS_STATUS_TX_PENDING           0x80

static volatile uint8_t cabBusStatus = 0;

#define XPRESSNET_ENABLED (cabBusStatus & CABBUS_STATUS_XPRESSNET)

static volatile uint8_t xpressnetAckPending = 0;

uint8_t cabBusPing(void)
{
	if(cabBusStatus & CABBUS_STATUS_PING_RECEIVED)
	{
		cabBusStatus &= ~CABBUS_STATUS_PING_RECEIVED;
		return 1;
	}
	else
		return 0;
}

uint8_t cabBusResponse(void)
{
	if(cabBusStatus & CABBUS_STATUS_RESPONSE_SENT)
	{
		cabBusStatus &= ~CABBUS_STATUS_RESPONSE_SENT;
		return 1;
	}
	else
		return 0;
}

uint8_t cabBusCollision(void)
{
	if(cabBusStatus & CABBUS_STATUS_COLLISION_DETECTED)
	{
		cabBusStatus &= ~CABBUS_STATUS_COLLISION_DETECTED;
		return 1;
	}
	else
		return 0;
}


static uint8_t cabBusTxBuffer[CABBUS_BUFFER_SIZE];
static uint8_t cabBusRxBuffer[CABBUS_BUFFER_SIZE];  // Only accessed within ISR so doesn't need to be volatile
static uint8_t cabBusTxLength = 0;
static volatile uint8_t cabBusTxIndex = 0;
static uint8_t myCabNumber = 0;
CabBusPktQueue cabBusTxQueue;
CabBusPktQueue cabBusRxQueue;

#include <util/parity.h>

void enableTransmitter(void)
{
	CABBUS_UART_CSR_A |= _BV(CABBUS_TXC);
	CABBUS_PORT |= _BV(CABBUS_TXE);

	// Disable receive interrupt while transmitting
	CABBUS_UART_CSR_B &= ~_BV(CABBUS_RXCIE);

	// Enable transmit interrupt
	CABBUS_UART_CSR_B |= _BV(CABBUS_UART_UDRIE);

	cabBusStatus |= CABBUS_STATUS_TRANSMITTING;
}

void startTxTimer(void)
{
	TCNT2 = 0;  // Reset timer2
	TIFR2 |= _BV(OCF2A);  // Clear any previous interrupts
	TIMSK2 |= _BV(OCIE2A);  // Enable timer2 interrupt
#ifdef DEBUG
	PORTB |= _BV(PB6);
#endif
}

ISR(TIMER2_COMPA_vect)
{
	TIMSK2 &= ~_BV(OCIE2A);  // Disable timer2 interrupt
#ifdef DEBUG
	PORTB &= ~_BV(PB6);
#endif
	enableTransmitter();
}

ISR(CABBUS_UART_RX_INTERRUPT)
{
	uint8_t data = 0;
	static uint8_t byte_count = 0;

	if (CABBUS_UART_CSR_A & CABBUS_RX_ERR_MASK)
	{
		// Handle framing errors
		data = CABBUS_UART_DATA;  // Clear the data register and discard
	}
	else if(XPRESSNET_ENABLED)
	{
		// XpressNet
		if(CABBUS_UART_CSR_B & _BV(CABBUS_RXB8))
		{
			// Bit 9 set, Address byte
			data = CABBUS_UART_DATA;  // Must be read after reading RXB8 bits
			uint8_t address = data & 0x1F;
			if( (!parity_even_bit(data)) && (0x00 == (data & 0x60)) && (address == myCabNumber) )
			{
				// Request Acknowledgement
				xpressnetAckPending = 1;
				startTxTimer();
			}
			else if( (!parity_even_bit(data)) && (0x40 == (data & 0x60)) )
			{
				// Normal inquiry
				if(myCabNumber == address)
				{
					// It's for us, so respond if anything is pending
					cabBusStatus |= CABBUS_STATUS_PING_RECEIVED;
					if(cabBusStatus & CABBUS_STATUS_TX_PENDING)
					{
						cabBusStatus |= CABBUS_STATUS_RESPONSE_SENT;
						startTxTimer();
					}
				}
				
				// If the previously stored data was a ping response from our address, signal a collision
				//   Only look at message with an actual response (byte_count > 1)
				if((myCabNumber == (cabBusRxBuffer[0] & 0x1F)) && (byte_count > 1))
				{
					if(!(cabBusStatus & CABBUS_STATUS_TRANSMITTING))
						cabBusStatus |= CABBUS_STATUS_COLLISION_DETECTED;
				}

				// Clear the flag here on every new ping
				// It will get set later once enableTransmitter is called
				cabBusStatus &= ~CABBUS_STATUS_TRANSMITTING;

				// Reset byte_count so the current data byte gets stored in the correct (index = 0) spot
				byte_count = 0;
			}
		}
		else
		{
			// Read here so we have the data for storing below if it wasn't already read above
			data = CABBUS_UART_DATA;
		}
	}
	else
	{
		// Cab Bus
		data = CABBUS_UART_DATA;

		// Dumb Cab:
		// byte 0: --> ping
		// byte 1: <-- response byte 1
		// byte 2: <-- response byte 2
		// byte 3: --> Command byte 1 (11xx xxxx)
		// byte 4: --> Command byte 2 or next ping (sometimes doesn't follow the 11xx xxxx rule)
		// byte n: --> More command bytes or next ping (sometimes doesn't follow the 11xx xxxx rule, e.g. command 0xCB)
		// Note: Bytes 4-n could look like pings but are conditional on the command byte 1 value
		//
		// Smart Cab:
		// byte 0: --> ping
		// byte 1: <-- address 1
		// byte 2: <-- address 2
		// byte 3: <-- Command
		// byte 4: <-- Value
		// byte 5: <-- Checksum
		// Note: Bytes 4-5 are allowed to have the MSB set

		// Does the byte have the characteristics of a ping (10xx xxxx)?
		if((data & 0xC0) == 0x80)
		{
			// The byte might be a ping, but we need to check the exceptions
			uint8_t cmd = cabBusRxBuffer[3];
			if(
				!( (0x15 <= cmd) && (cmd <= 0x16) && (4 <= byte_count) && (byte_count <= 5)  ) &&  // Ignore bytes 4 & 5 of smart cab FN13-28 responses (Note 1)
				!( (0xCB == cmd)                  && (4 <= byte_count) && (byte_count <= 12) ) &&  // Ignore 9 bytes following dumb cab command 0xCB
				!( (0xC0 <= cmd) && (cmd <= 0xC7) && (4 <= byte_count) && (byte_count <= 11) ) &&  // Ignore 8 bytes following dumb cab commands 0xC0 to 0xC7, special chars are not escaped (Note 2)
				!( (0xC8 == cmd)                  && (4 == byte_count)                       ) &&  // Ignore data for command 0xC8 (unverified, based on example from spec)
				!( (0xCC == cmd)                  && (4 == byte_count)                       )     // Ignore data for command 0xCC (?)
			)
			// Note 1: This could catch some cab memory access and OPS programming packets, too, but these are properly escaped so it shouldn't ever get here.
			// Note 2: Although ASCII chars are escaped in the 8 char writes, special chars are not (e.g. EXPN display for FN10 and FN20).
			//         Dumb cabs appear to be fooled by this and cause collisions, though the data appears to still get through.  Observed collisions on scope.
			{
				// Must be a ping, so handle it
				if(myCabNumber == (data & 0x3F))
				{
					// It's for us, so respond if anything is pending
					cabBusStatus |= CABBUS_STATUS_PING_RECEIVED;
					if(cabBusStatus & CABBUS_STATUS_TX_PENDING)
					{
						cabBusStatus |= CABBUS_STATUS_RESPONSE_SENT;
						startTxTimer();
					}
				}
				
				// If the previously stored data was a broadcast command, push onto the rx queue before we overwrite it
				if(0x80 == cabBusRxBuffer[0])
				{
					cabBusPktQueuePush(&cabBusRxQueue, cabBusRxBuffer, byte_count);
				}

				// If the previously stored data was a ping response from our address, signal a collision
				//   Only look at message with an actual response (byte_count > 1)
				if((myCabNumber == (cabBusRxBuffer[0] & 0x3F)) && (byte_count > 1))
				{
					if(!(cabBusStatus & CABBUS_STATUS_TRANSMITTING))
						cabBusStatus |= CABBUS_STATUS_COLLISION_DETECTED;
				}

				// Clear the flag here on every new ping
				// It will get set later once enableTransmitter is called
				cabBusStatus &= ~CABBUS_STATUS_TRANSMITTING;

				// Reset byte_count so the current data byte gets stored in the correct (index = 0) spot
				byte_count = 0;
			}
		}

		if(1 == byte_count)
		{
			// First byte after ping
			if(0x80 == cabBusRxBuffer[0])
			{
				// Broadcast command, skip 2 bytes so it aligns with normal commands
				cabBusRxBuffer[byte_count++] = 0;
				cabBusRxBuffer[byte_count++] = 0;
			}
		}
	}
	// Store the byte
	cabBusRxBuffer[byte_count] = data;
	byte_count++;
	if(byte_count >= CABBUS_BUFFER_SIZE)
		byte_count = CABBUS_BUFFER_SIZE - 1;
}

ISR(CABBUS_UART_DONE_INTERRUPT)
{
	// Transmit is complete: terminate
	CABBUS_PORT &= ~_BV(CABBUS_TXE);  // Disable driver
	// Disable the various transmit interrupts
	// Re-enable receive interrupt
	CABBUS_UART_CSR_B = (CABBUS_UART_CSR_B & ~(_BV(CABBUS_TXCIE) | _BV(CABBUS_UART_UDRIE))) | _BV(CABBUS_RXCIE);
	cabBusStatus &= ~CABBUS_STATUS_TX_PENDING;
	xpressnetAckPending = 0;
}

ISR(CABBUS_UART_TX_INTERRUPT)
{
	uint8_t done = 0;
	
	if(XPRESSNET_ENABLED && xpressnetAckPending)
	{
		CABBUS_UART_DATA = 0x20;
		xpressnetAckPending++;
		done = (xpressnetAckPending > 2);
	}
	else
	{
		CABBUS_UART_DATA = cabBusTxBuffer[cabBusTxIndex++];  //  Get next byte and write to UART
		done = (cabBusTxIndex >= CABBUS_BUFFER_SIZE || cabBusTxLength == cabBusTxIndex);
	}

	if(done)
	{
		//  Done sending data to UART, disable UART interrupt
		CABBUS_UART_CSR_A |= _BV(CABBUS_TXC);
		CABBUS_UART_CSR_B &= ~_BV(CABBUS_UART_UDRIE);
		CABBUS_UART_CSR_B |= _BV(CABBUS_TXCIE);
	}
}

uint8_t cabBusTransmit(void)
{
	uint8_t i;

	if (cabBusPktQueueEmpty(&cabBusTxQueue))
		return(0);

	//  Return if bus already active.
	if (cabBusStatus & CABBUS_STATUS_TX_PENDING)
		return(1);

	cabBusTxLength = cabBusPktQueuePeek(&cabBusTxQueue, (uint8_t*)cabBusTxBuffer, sizeof(cabBusTxBuffer));

	// If we have no packet length, or it's less than the header, just silently say we transmitted it
	// On the AVRs, if you don't have any packet length, it'll never clear up on the interrupt routine
	// and you'll get stuck in indefinite transmit busy
	if (0 == cabBusTxLength)
	{
		cabBusPktQueueDrop(&cabBusTxQueue);
		return(0);
	}

	// First Calculate XOR
	uint8_t xor_byte = 0;
	for (i=0; i<cabBusTxLength; i++)
	{
		xor_byte ^= cabBusTxBuffer[i];
	}
	cabBusTxBuffer[cabBusTxLength++] = xor_byte;  // Put it at the end and increment length so it gets transmitted

	cabBusTxIndex = 0;
	
	cabBusPktQueueDrop(&cabBusTxQueue);

	cabBusStatus |= CABBUS_STATUS_TX_PENDING;  // Notify RX routine that a packet is ready to go

	return(0);
}

void cabBusInit(uint8_t addr, uint8_t enableXpressnet)
{
#ifdef DEBUG
	DDRB |= (_BV(PB6) | _BV(PB5));  // For analyzing response time and other debug
#endif

	if(enableXpressnet)
		cabBusStatus |= CABBUS_STATUS_XPRESSNET;
	else
		cabBusStatus &= ~CABBUS_STATUS_XPRESSNET;

	// Setup Timer 2 for 800us post ping delay
	// Required minimum = 100us to bridge the 2nd stop bit (we get interrupt after the first stop bit) + 100us minimum before response (see Cab Bus spec)
	// However, in some cases, the bus is held for an additional 300us (empirical observations)
	// An 800us delay still falls within the 800us max delay since we start the timer before the 2nd stop bit
	// Besides, a ProCab and the NCE USB interface appears to wait ~1.12us before responding.
	TCNT2 = 0;
	OCR2A = XPRESSNET_ENABLED ? 1 : 250;  // 	50ns (20MHz) * 64 * 250 = 800us, disabled for XpressNet
	TCCR2A = _BV(WGM21);
	TCCR2B = _BV(CS22);  // Divide-by-64
	TIMSK2 = 0;  // Disable interrupt for now

#undef BAUD
	if(XPRESSNET_ENABLED)
	{
#define BAUD XPRESSNET_BAUD
#include <util/setbaud.h>
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	CABBUS_UART_CSR_B = _BV(UCSZ02);
	CABBUS_UART_CSR_C = _BV(UCSZ01) | _BV(UCSZ00);
#undef BAUD
	}
	else
	{
#define BAUD CABBUS_BAUD
#include <util/setbaud.h>
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	CABBUS_UART_CSR_B = 0;
	CABBUS_UART_CSR_C = _BV(USBS0) | _BV(UCSZ01) | _BV(UCSZ00);
#undef BAUD
	}

	myCabNumber = addr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	CABBUS_UART_CSR_B |= (_BV(CABBUS_RXCIE) | _BV(CABBUS_RXEN) | _BV(CABBUS_TXEN));

	CABBUS_DDR &= ~(_BV(CABBUS_RX) | _BV(CABBUS_TX));  // Set RX and TX as inputs
	CABBUS_DDR |= _BV(CABBUS_TXE);  // Set driver enable as output
	CABBUS_PORT &= ~(_BV(CABBUS_TXE));  // Disable driver
}


