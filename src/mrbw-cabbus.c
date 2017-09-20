/*************************************************************************
Title:    MRBW-CABBUS MRBee Cab Bus Interface
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mrbw-cabbus.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "mrbee.h"
#include "cabbus.h"

#ifdef DEBUG
void debugInit(void)
{
	DDRD |= _BV(PD5) | _BV(PD6);
	PORTD &= ~(_BV(PD5) | _BV(PD6));
}

void debug5(uint8_t val)
{
	if(val)
		PORTD |= _BV(PD5);
	else
		PORTD &= ~_BV(PD5);
}

void debug6(uint8_t val)
{
	if(val)
		PORTD |= _BV(PD6);
	else
		PORTD &= ~_BV(PD6);
}
#endif

#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 16

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

#define CABBUS_TX_BUFFER_DEPTH 16

CabBusPacket cabBusTxPktBufferArray[CABBUS_TX_BUFFER_DEPTH];
uint8_t cabBusBuffer[CABBUS_BUFFER_SIZE];

uint8_t mrbus_dev_addr = 0;



void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 18;
	buf[MRBUS_PKT_TYPE] = 'v';
	buf[6]  = MRBUS_VERSION_WIRELESS;
	// Software Revision
	buf[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
	buf[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
	buf[9]  = 0xFF & (GIT_REV); // Software Revision
	buf[10]  = HWREV_MAJOR; // Hardware Major Revision
	buf[11]  = HWREV_MINOR; // Hardware Minor Revision
	buf[12] = 'C';
	buf[13] = 'A';
	buf[14] = 'B';
	buf[15] = 'B';
	buf[16] = 'U';
	buf[17] = 'S';
}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbeeRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		createVersionPacket(rxBuffer[MRBUS_PKT_SRC], txBuffer);
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	else if (('S' == rxBuffer[MRBUS_PKT_TYPE]) && (mrbus_dev_addr == rxBuffer[MRBUS_PKT_DEST]))
	{
		// Status packet and addressed to us
		uint16_t locoAddress = ((uint16_t)rxBuffer[6] << 8) + rxBuffer[7];
		if(locoAddress > 99)
			locoAddress |= 0xC000;
		uint8_t speedDirection = rxBuffer[8];
		uint32_t functions = ((uint32_t)rxBuffer[9] << 24) + ((uint32_t)rxBuffer[10] << 16) + ((uint16_t)rxBuffer[11] << 8) + rxBuffer[12];

		uint8_t statusFlags = rxBuffer[13];

/*  FIXME!
		// Send Speed/Direction
		xpressnetBuffer[0] = 0xE4;  // Speed & Direction, 128 speed steps
		xpressnetBuffer[1] = 0x13;
		xpressnetBuffer[2] = (locoAddress >> 8);  // Locomotive Address
		xpressnetBuffer[3] = locoAddress & 0xFF;
		xpressnetBuffer[4] = speedDirection;
		xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);

		// Send function states
		xpressnetBuffer[0] = 0xE4;  // Function Group 1
		xpressnetBuffer[1] = 0x20;
		xpressnetBuffer[2] = (locoAddress >> 8);  // Locomotive Address
		xpressnetBuffer[3] = locoAddress & 0xFF;
		xpressnetBuffer[4] = ((functions & 0x01) << 4) | ((functions & 0x1E) >> 1);  // F0 F4 F3 F2 F1
		xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);

		xpressnetBuffer[0] = 0xE4;  // Function Group 2
		xpressnetBuffer[1] = 0x21;
		xpressnetBuffer[2] = (locoAddress >> 8);  // Locomotive Address
		xpressnetBuffer[3] = locoAddress & 0xFF;
		xpressnetBuffer[4] = (functions >> 5) & 0xF;  // F8 F7 F6 F5
		xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);

		xpressnetBuffer[0] = 0xE4;  // Function Group 3
		xpressnetBuffer[1] = 0x22;
		xpressnetBuffer[2] = (locoAddress >> 8);  // Locomotive Address
		xpressnetBuffer[3] = locoAddress & 0xFF;
		xpressnetBuffer[4] = (functions >> 9) & 0xF;  // F12 F11 F10 F9
		xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);

		xpressnetBuffer[0] = 0xE4;  // Function Group 4 (http://www.opendcc.net/elektronik/opendcc/xpressnet_commands_e.html)
		xpressnetBuffer[1] = 0x23;
		xpressnetBuffer[2] = (locoAddress >> 8);  // Locomotive Address
		xpressnetBuffer[3] = locoAddress & 0xFF;
		xpressnetBuffer[4] = (functions >> 13) & 0xFF;  // F20 - F13
		xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);

		xpressnetBuffer[0] = 0xE4;  // Function Group 5 (http://www.opendcc.net/elektronik/opendcc/xpressnet_commands_e.html)
		xpressnetBuffer[1] = 0x28;
		xpressnetBuffer[2] = (locoAddress >> 8);  // Locomotive Address
		xpressnetBuffer[3] = locoAddress & 0xFF;
		xpressnetBuffer[4] = (functions >> 21) & 0xFF;  // F28 - F21
		xpressnetPktQueuePush(&xpressnetTxQueue, xpressnetBuffer, 5);
*/
	}

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
}

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 195;  // 20MHz / 1024 / 195 = 100.16Hz
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)  // 100ms
	{
		ticks = 0;
		decisecs++;
	}
}

void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	// Configure DIP switches
	DDRC &= 0x07;  // PC3 - PC7 = inputs
	PORTC |= 0xF8; // PC3 - PC7 pullups enabled

	initialize100HzTimer();
}

uint8_t oldSwitches = 0xFF;  // Default to something that can never be set on switches so it updates them the first time

void readDipSwitches(void)
{
	uint8_t switches = PINC >> 3;
	if(oldSwitches != switches)
	{
		cabBusInit(switches);
		mrbus_dev_addr = 0xD0 + switches;
		oldSwitches = switches;
	}
}

int main(void)
{
	uint8_t mrbusTxBuffer[MRBUS_BUFFER_SIZE];
	uint16_t decisecs_tmp = 0;

	init();

	wdt_reset();
	
	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);

	cabBusPktQueueInitialize(&cabBusTxQueue, cabBusTxPktBufferArray, CABBUS_TX_BUFFER_DEPTH);

	readDipSwitches();  // cabBusInit will be called here
	mrbeeInit();

	sei();	

	wdt_reset();

	// Fire off initial reset version packet
	createVersionPacket(0xFF, mrbusTxBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);

	wdt_reset();

#ifdef DEBUG
	debugInit();
#endif

	while(1)
	{
		wdt_reset();

		readDipSwitches();
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = decisecs;
		}
		if(decisecs_tmp >= 10)
		{
			// Send "I'm here" message every second so throttles know communication is active
			createVersionPacket(0xFF, mrbusTxBuffer);
			mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				decisecs = 0;
			}
		}

		// Handle any MRBus packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			PktHandler();
		}

		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			wdt_reset();
#ifndef DEBUG
			mrbeeTransmit();
#endif
		}

		if(cabBusPktQueueDepth(&cabBusTxQueue))
		{
			// Packet pending, so queue it up for transmit
			wdt_reset();
			cabBusTransmit();
		}
	}

}


