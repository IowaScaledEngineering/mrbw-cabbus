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

static volatile uint8_t cabBusTxBuffer[CABBUS_BUFFER_SIZE];
static volatile uint8_t cabBusTxLength = 0;
static volatile uint8_t cabBusAddress = 0;
static volatile uint8_t cabBusTxIndex = 0;
static volatile uint8_t cabBusTxPending = 0;
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
	static uint8_t byte_count = 0;

	if (CABBUS_UART_CSR_A & CABBUS_RX_ERR_MASK)
	{
		// Handle framing errors
		data = CABBUS_UART_DATA;  // Clear the data register and discard
	}
    else
    {
		data = CABBUS_UART_DATA;
		if((data & 0xC0) == 0x80)
		{
			// It's (maybe) a ping
			if((3 == byte_count) || (4 == byte_count) || (5 == byte_count))
			{
				// Not a ping since bytes 3-5 are allowed to have the MSB set
				byte_count++;
			}
			else
			{
				// Must be a ping, so process it
				byte_count = 0;
				uint8_t address = data & 0x3F;
				if(address == cabBusAddress)
				{
					// It's for us, so respond if anything is pending
					if(cabBusTxPending)
					{
						_delay_us(300);  // 100us to bridge the 2nd stop bit (we're called after the first) + 100us minimum before response (see Cab Bus spec) + 100us guardband
						enableTransmitter();
					}
				}
				else
				{
					// Not for us, ignore (but still count bytes since someone else may respond as the next byte)
					byte_count++;
				}
			}
		}
		else
		{
			// Data response from someone else, ignore (but still count bytes)
			byte_count++;
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
}

ISR(CABBUS_UART_TX_INTERRUPT)
{
	uint8_t done = 0;
	
	CABBUS_UART_DATA = cabBusTxBuffer[cabBusTxIndex++];  //  Get next byte and write to UART
	done = (cabBusTxIndex >= CABBUS_BUFFER_SIZE || cabBusTxLength == cabBusTxIndex);

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
	for (i=0; i<cabBusTxLength; i++)
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
	CABBUS_UART_UBRR = UBRR_VALUE;
	CABBUS_UART_CSR_A = (USE_2X)?_BV(U2X0):0;
	CABBUS_UART_CSR_B = 0;
	CABBUS_UART_CSR_C = _BV(USBS0) | _BV(UCSZ01) | _BV(UCSZ00);
#undef BAUD

	cabBusAddress = addr;

	/* Enable USART receiver and transmitter and receive complete interrupt */
	CABBUS_UART_CSR_B |= (_BV(CABBUS_RXCIE) | _BV(CABBUS_RXEN) | _BV(CABBUS_TXEN));

	CABBUS_DDR &= ~(_BV(CABBUS_RX) | _BV(CABBUS_TX));  // Set RX and TX as inputs
	CABBUS_DDR |= _BV(CABBUS_TXE);  // Set driver enable as output
	CABBUS_PORT &= ~(_BV(CABBUS_TXE));  // Disable driver
}


