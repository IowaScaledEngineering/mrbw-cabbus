/*************************************************************************
Title:    XpressNet Atmel AVR
Authors:  Michael Petersen <railfan@drgw.net>, Colorado, USA
          Nathan Holmes <maverick@drgw.net>, Colorado, USA
File:     xpressnet-avr.h
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
#include "xpressnet.h"

static volatile uint8_t xpressnetTxBuffer[XPRESSNET_BUFFER_SIZE];
static volatile uint8_t xpressnetTxLength = 0;
static volatile uint8_t xpressnetAddress = 0;
static volatile uint8_t xpressnetTxIndex = 0;
static volatile uint8_t xpressnetTxPending = 0;
static volatile uint8_t xpressnetAckPending = 0;
XpressNetPktQueue xpressnetTxQueue;

#include <util/parity.h>

void enableTransmitter(void)
{
	XPRESSNET_UART_CSR_A |= _BV(XPRESSNET_TXC);
	XPRESSNET_PORT |= _BV(XPRESSNET_TXE);

	// Disable receive interrupt while transmitting
	XPRESSNET_UART_CSR_B &= ~_BV(XPRESSNET_RXCIE);

	// Enable transmit interrupt
	XPRESSNET_UART_CSR_B |= _BV(XPRESSNET_UART_UDRIE);
}

ISR(XPRESSNET_UART_RX_INTERRUPT)
{
	uint8_t data = 0;

	if (XPRESSNET_UART_CSR_A & XPRESSNET_RX_ERR_MASK)
	{
			// Handle framing errors
			data = XPRESSNET_UART_DATA;  // Clear the data register and discard
	}
    else
    {
		if(XPRESSNET_UART_CSR_B & _BV(XPRESSNET_RXB8))
		{
			// Bit 9 set, Address byte
			data = XPRESSNET_UART_DATA;
			uint8_t address = data & 0x1F;
			if( (!parity_even_bit(data)) && (0x00 == (data & 0x60)) && (address == xpressnetAddress) )
			{
				// Request Acknowledgement
				xpressnetAckPending = 1;
				enableTransmitter();
			}
			else if( (!parity_even_bit(data)) && (0x40 == (data & 0x60)) )
			{
				// Normal inquiry
				if(xpressnetTxPending && (address == xpressnetAddress))
				{
					/* Enable transmitter since control over bus is assumed */
					enableTransmitter();
				}
			}
		}
		else
		{
			data = XPRESSNET_UART_DATA;  // Clear the data register and discard
		}
    }
}

ISR(XPRESSNET_UART_DONE_INTERRUPT)
{
	// Transmit is complete: terminate
	XPRESSNET_PORT &= ~_BV(XPRESSNET_TXE);  // Disable driver
	// Disable the various transmit interrupts
	// Re-enable receive interrupt
	XPRESSNET_UART_CSR_B = (XPRESSNET_UART_CSR_B & ~(_BV(XPRESSNET_TXCIE) | _BV(XPRESSNET_UART_UDRIE))) | _BV(XPRESSNET_RXCIE);
	xpressnetTxPending = 0;
	xpressnetAckPending = 0;
}

ISR(XPRESSNET_UART_TX_INTERRUPT)
{
	uint8_t done = 0;
	
	if(xpressnetAckPending)
	{
		XPRESSNET_UART_DATA = 0x20;
		xpressnetAckPending++;
		done = (xpressnetAckPending > 2);
	}
	else
	{
		XPRESSNET_UART_DATA = xpressnetTxBuffer[xpressnetTxIndex++];  //  Get next byte and write to UART
		done = (xpressnetTxIndex >= XPRESSNET_BUFFER_SIZE || xpressnetTxLength == xpressnetTxIndex);
	}
	if(done)
	{
		//  Done sending data to UART, disable UART interrupt
		XPRESSNET_UART_CSR_A |= _BV(XPRESSNET_TXC);
		XPRESSNET_UART_CSR_B &= ~_BV(XPRESSNET_UART_UDRIE);
		XPRESSNET_UART_CSR_B |= _BV(XPRESSNET_TXCIE);
	}
}

uint8_t xpressnetTransmit(void)
{
	uint8_t i;

	if (xpressnetPktQueueEmpty(&xpressnetTxQueue))
		return(0);

	//  Return if bus already active.
	if (xpressnetTxPending)
		return(1);

	xpressnetTxLength = xpressnetPktQueuePeek(&xpressnetTxQueue, (uint8_t*)xpressnetTxBuffer, sizeof(xpressnetTxBuffer));

	// If we have no packet length, or it's less than the header, just silently say we transmitted it
	// On the AVRs, if you don't have any packet length, it'll never clear up on the interrupt routine
	// and you'll get stuck in indefinite transmit busy
	if (0 == xpressnetTxLength)
	{
		xpressnetPktQueueDrop(&xpressnetTxQueue);
		return(0);
	}

	// First Calculate XOR
	uint8_t xor_byte = 0;
	for (i=0; i<=xpressnetTxLength; i++)
	{
		xor_byte ^= xpressnetTxBuffer[i];
	}
	xpressnetTxBuffer[xpressnetTxLength++] = xor_byte;  // Put it at the end and increment length so it gets transmitted

	xpressnetTxIndex = 0;
	
	xpressnetPktQueueDrop(&xpressnetTxQueue);

	xpressnetTxPending = 1;  // Notify RX routine that a packet is ready to go

	return(0);
}


void xpressnetInit(uint8_t addr)
{
#undef BAUD
#define BAUD XPRESSNET_BAUD
#include <util/setbaud.h>

#if defined( XPRESSNET_AT90_UART )
	// FIXME - probably need more stuff here
	UBRR = (uint8_t)UBRRL_VALUE;

#elif defined( XPRESSNET_ATMEGA_USART_SIMPLE )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X):0;
	XPRESSNET_UART_CSR_B = 0;
	XPRESSNET_UART_CSR_C = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
	
#elif defined( XPRESSNET_ATMEGA_USART0_SIMPLE )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	XPRESSNET_UART_CSR_B = 0;
	XPRESSNET_UART_CSR_C = _BV(URSEL0) | _BV(UCSZ01) | _BV(UCSZ00);
	
#elif defined( XPRESSNET_ATMEGA_USART ) || defined ( XPRESSNET_ATMEGA_USART0 )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	XPRESSNET_UART_CSR_B = _BV(UCSZ02);
	XPRESSNET_UART_CSR_C = _BV(UCSZ01) | _BV(UCSZ00);

#elif defined( XPRESSNET_ATTINY_USART )
	// Top four bits are reserved and must always be zero - see ATtiny2313 datasheet
	// Also, H and L must be written independently, since they're non-adjacent registers
	// on the attiny parts
	XPRESSNET_UART_UBRRH = 0x0F & UBRRH_VALUE;
	XPRESSNET_UART_UBRRL = UBRRL_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X):0;
	XPRESSNET_UART_CSR_B = 0;
	XPRESSNET_UART_CSR_C = _BV(UCSZ1) | _BV(UCSZ0);

#elif defined ( XPRESSNET_ATMEGA_USART1 )
	XPRESSNET_UART_UBRR = UBRR_VALUE;
	XPRESSNET_UART_CSR_A = (USE_2X)?_BV(U2X1):0;
	XPRESSNET_UART_CSR_B = _BV(UCSZ12);
	XPRESSNET_UART_CSR_C = _BV(UCSZ11) | _BV(UCSZ10);
#else
#error "UART for your selected part is not yet defined..."
#endif

#undef BAUD

	xpressnetAddress = addr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	XPRESSNET_UART_CSR_B |= (_BV(XPRESSNET_RXCIE) | _BV(XPRESSNET_RXEN) | _BV(XPRESSNET_TXEN));

	XPRESSNET_DDR &= ~(_BV(XPRESSNET_RX) | _BV(XPRESSNET_TX));  // Set RX and TX as inputs
	XPRESSNET_DDR |= _BV(XPRESSNET_TXE);  // Set driver enable as output
	XPRESSNET_PORT &= ~(_BV(XPRESSNET_TXE));  // Disable driver
}


