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

#define CABBUS_STATUS_BROADCAST_CMD    0x01
#define CABBUS_STATUS_DUMB_CAB         0x02
#define CABBUS_STATUS_SMART_CAB        0x04
#define CABBUS_STATUS_TX_PENDING       0x80

static volatile uint8_t cabBusStatus = 0;

static volatile uint8_t cabBusTxBuffer[CABBUS_BUFFER_SIZE];
// While cabbusRxBuffer is volatile, it's only accessed within the ISR
static uint8_t cabBusRxBuffer[CABBUS_BUFFER_SIZE];
static volatile uint8_t cabBusTxLength = 0;
static volatile uint8_t cabBusAddress = 0;
static volatile uint8_t cabBusTxIndex = 0;
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
		// Dumb Cab:
		// byte 0: --> ping
		// byte 1: <-- response byte 1
		// byte 2: <-- response byte 2
		// byte 3: --> Command byte 1 (11xx xxxx)
		// byte 4: --> Command byte 2 (sometimes doesn't follow the 11xx xxxx rule)
		// byte n: --> More command bytes
		// Might missing a ping following a 2 byte command (byte = 5), but we'll catch it next time we're pinged (FIXME: maybe?)
		//
		// Smart Cab:
		// byte 0: --> ping
		// byte 1: <-- address 1
		// byte 2: <-- address 2
		// byte 3: <-- Command
		// byte 4: <-- Value
		// byte 5: <-- Checksum
		// Note: Bytes 3-5 are allowed to have the MSB set

		data = CABBUS_UART_DATA;

		if((data & 0xC0) == 0x80)
		{
			// The byte might be a ping, but we need to check the exceptions
			if(
				!( (cabBusStatus & CABBUS_STATUS_DUMB_CAB) && ((4 == byte_count)) ) &&
				!( (cabBusStatus & CABBUS_STATUS_SMART_CAB) && ((3 == byte_count) || (4 == byte_count) || (5 == byte_count)) )
			)
			{
				// Must be a ping, so handle it
				uint8_t address = data & 0x3F;
				if(0 == address)
				{
					//FIXME: is this needed?  Can it be determined later?
					// Broadcast command
					cabBusStatus |= CABBUS_STATUS_BROADCAST_CMD;
				}
				else if(cabBusAddress == address)
				{
					// It's for us, so respond if anything is pending
					if(cabBusStatus & CABBUS_STATUS_TX_PENDING)
					{
						TCNT2 = 0;  // Reset timer2
						TIFR2 |= _BV(OCF2A);  // Clear any previous interrupts
						TIMSK2 |= _BV(OCIE2A);  // Enable timer2 interrupt
						PORTB |= _BV(PB6);
					}
				}
				
				// Process previous packet, if one exists
				// FIXME

				// Reset flags
				cabBusStatus &= ~CABBUS_STATUS_SMART_CAB;
				cabBusStatus &= ~CABBUS_STATUS_DUMB_CAB;
				// Reset byte_count so the current data byte gets stored in the correct spot
				byte_count = 0;
			}
		}

		if(1 == byte_count)
		{
			// First byte of a response.  Determine the response type.
			if(data < 0x40)
				cabBusStatus |= CABBUS_STATUS_SMART_CAB;
			else
				cabBusStatus |= CABBUS_STATUS_DUMB_CAB;
		}

		// Store the byte
		cabBusRxBuffer[byte_count] = data;
		byte_count++;
	}


/*
		if(cabBusStatus & CABBUS_STATUS_BROADCAST_CMD)
		{
			if( ((data & 0xC0) == 0xC0) || (2 == byte_count) )
			{
				// Buffer data, assuming it follows the "11xx xxxx" rule for commands, unless it's broadcast command byte #2 since that sometimes doesn't follow the rule
				// byte 0: ping
				// byte 1: Command byte 1 (11xx xxxx)
				// byte 2: Command byte 2 (sometimes doesn't follow the 11xx xxxx rule)
				// byte n: More command bytes
				// We will swallow the ping immediately following the broadcast command, but we'll catch it next time we're pinged not following a broadcast command
				if(byte_count < CABBUS_BUFFER_SIZE)
				{
					cabBusRxBuffer[byte_count] = data;
					byte_count++;
				}
			}
			else
			{
				cabBusStatus &= ~CABBUS_STATUS_BROADCAST_CMD;
				cabBusPktQueuePush(&cabBusRxQueue, cabBusRxBuffer, byte_count);
			}
		}
		else
		{
			// Data response from someone else, ignore (but still count bytes)
			byte_count++;
		}
    }
*/
}

ISR(TIMER2_COMPA_vect)
{
	TIMSK2 &= ~_BV(OCIE2A);  // Disable timer2 interrupt
	PORTB &= ~_BV(PB6);
	enableTransmitter();
}

ISR(CABBUS_UART_DONE_INTERRUPT)
{
	// Transmit is complete: terminate
	CABBUS_PORT &= ~_BV(CABBUS_TXE);  // Disable driver
	// Disable the various transmit interrupts
	// Re-enable receive interrupt
	CABBUS_UART_CSR_B = (CABBUS_UART_CSR_B & ~(_BV(CABBUS_TXCIE) | _BV(CABBUS_UART_UDRIE))) | _BV(CABBUS_RXCIE);
	cabBusStatus &= ~CABBUS_STATUS_TX_PENDING;
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

void cabBusInit(uint8_t addr)
{
	DDRB |= _BV(PB6);  // For analyzing response time

	// Setup Timer 2 for 300us post ping delay
	// 100us to bridge the 2nd stop bit (we get interrupt after the first stop bit) + 100us minimum before response (see Cab Bus spec) + 100us guardband
	TCNT2 = 0;
	OCR2A = 188;  // 50ns (20MHz) * 32 * 188 = 300.8us
	TCCR2A = _BV(WGM21);
	TCCR2B = _BV(CS21) | _BV(CS20);  // Divide-by-32
	TIMSK2 = 0;  // Disable interrupt for now

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


