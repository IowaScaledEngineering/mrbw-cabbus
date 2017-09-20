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
#include "cabbus.h"

static volatile uint8_t cabBusTxBuffer[CABBUS_BUFFER_SIZE];
static volatile uint8_t cabBusTxLength = 0;
static volatile uint8_t cabBusAddress = 0;
static volatile uint8_t cabBusTxIndex = 0;
static volatile uint8_t cabBusTxPending = 0;
static volatile uint8_t cabBusAckPending = 0;
CabBusPktQueue cabBusTxQueue;

#include <util/parity.h>

void enableTransmitter(void)
{
	CABBUS_UART_CSR_A |= _BV(CABBUS_TXC);
	CABBUS_PORT |= _BV(CABBUS_TXE);

	// Disable receive interrupt while transmitting
	CABBUS_UART_CSR_B &= ~_BV(CABBUS_RXCIE);

	// Enable transmit interrupt
	CABBUS_UART_CSR_B |= _BV(CABBUS_UART_UDRIE);
}

ISR(CABBUS_UART_RX_INTERRUPT)
{
	uint8_t data = 0;

	if (CABBUS_UART_CSR_A & CABBUS_RX_ERR_MASK)
	{
			// Handle framing errors
			data = CABBUS_UART_DATA;  // Clear the data register and discard
	}
    else
    {
		if(CABBUS_UART_CSR_B & _BV(CABBUS_RXB8))
		{
			// Bit 9 set, Address byte
			data = CABBUS_UART_DATA;
			uint8_t address = data & 0x1F;
			if( (!parity_even_bit(data)) && (0x00 == (data & 0x60)) && (address == cabBusAddress) )
			{
				// Request Acknowledgement
				cabBusAckPending = 1;
				enableTransmitter();
			}
			else if( (!parity_even_bit(data)) && (0x40 == (data & 0x60)) )
			{
				// Normal inquiry
				if(cabBusTxPending && (address == cabBusAddress))
				{
					/* Enable transmitter since control over bus is assumed */
					enableTransmitter();
				}
			}
		}
		else
		{
			data = CABBUS_UART_DATA;  // Clear the data register and discard
		}
    }
}

ISR(CABBUS_UART_DONE_INTERRUPT)
{
	// Transmit is complete: terminate
	CABBUS_PORT &= ~_BV(CABBUS_TXE);  // Disable driver
	// Disable the various transmit interrupts
	// Re-enable receive interrupt
	CABBUS_UART_CSR_B = (CABBUS_UART_CSR_B & ~(_BV(CABBUS_TXCIE) | _BV(CABBUS_UART_UDRIE))) | _BV(CABBUS_RXCIE);
	cabBusTxPending = 0;
	cabBusAckPending = 0;
}

ISR(CABBUS_UART_TX_INTERRUPT)
{
	uint8_t done = 0;
	
	if(cabBusAckPending)
	{
		CABBUS_UART_DATA = 0x20;
		cabBusAckPending++;
		done = (cabBusAckPending > 2);
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
	if (cabBusTxPending)
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
	for (i=0; i<=cabBusTxLength; i++)
	{
		xor_byte ^= cabBusTxBuffer[i];
	}
	cabBusTxBuffer[cabBusTxLength++] = xor_byte;  // Put it at the end and increment length so it gets transmitted

	cabBusTxIndex = 0;
	
	cabBusPktQueueDrop(&cabBusTxQueue);

	cabBusTxPending = 1;  // Notify RX routine that a packet is ready to go

	return(0);
}


void cabBusInit(uint8_t addr)
{
#undef BAUD
#define BAUD CABBUS_BAUD
#include <util/setbaud.h>

#if defined( CABBUS_AT90_UART )
	// FIXME - probably need more stuff here
	UBRR = (uint8_t)UBRRL_VALUE;

#elif defined( CABBUS_ATMEGA_USART_SIMPLE )
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X):0;
	CABBUS_UART_CSR_B = 0;
	CABBUS_UART_CSR_C = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
	
#elif defined( CABBUS_ATMEGA_USART0_SIMPLE )
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	CABBUS_UART_CSR_B = 0;
	CABBUS_UART_CSR_C = _BV(URSEL0) | _BV(UCSZ01) | _BV(UCSZ00);
	
#elif defined( CABBUS_ATMEGA_USART ) || defined ( CABBUS_ATMEGA_USART0 )
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	CABBUS_UART_CSR_B = _BV(UCSZ02);
	CABBUS_UART_CSR_C = _BV(UCSZ01) | _BV(UCSZ00);

#elif defined( CABBUS_ATTINY_USART )
	// Top four bits are reserved and must always be zero - see ATtiny2313 datasheet
	// Also, H and L must be written independently, since they're non-adjacent registers
	// on the attiny parts
	CABBUS_UART_UBRRH = 0x0F & UBRRH_VALUE;
	CABBUS_UART_UBRRL = UBRRL_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X):0;
	CABBUS_UART_CSR_B = 0;
	CABBUS_UART_CSR_C = _BV(UCSZ1) | _BV(UCSZ0);

#elif defined ( CABBUS_ATMEGA_USART1 )
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X1):0;
	CABBUS_UART_CSR_B = _BV(UCSZ12);
	CABBUS_UART_CSR_C = _BV(UCSZ11) | _BV(UCSZ10);
#else
#error "UART for your selected part is not yet defined..."
#endif

#undef BAUD

	cabBusAddress = addr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	CABBUS_UART_CSR_B |= (_BV(CABBUS_RXCIE) | _BV(CABBUS_RXEN) | _BV(CABBUS_TXEN));

	CABBUS_DDR &= ~(_BV(CABBUS_RX) | _BV(CABBUS_TX));  // Set RX and TX as inputs
	CABBUS_DDR |= _BV(CABBUS_TXE);  // Set driver enable as output
	CABBUS_PORT &= ~(_BV(CABBUS_TXE));  // Disable driver
}


