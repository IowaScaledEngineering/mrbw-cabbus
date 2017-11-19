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
#include "cabbus-cache.h"

#define LOCO_ADDRESS_SHORT 0x8000

#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 16

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

#define CABBUS_TX_BUFFER_DEPTH 16
#define CABBUS_RX_BUFFER_DEPTH 4

CabBusPacket cabBusTxPktBufferArray[CABBUS_TX_BUFFER_DEPTH];
CabBusPacket cabBusRxPktBufferArray[CABBUS_RX_BUFFER_DEPTH];
uint8_t cabBusBuffer[CABBUS_BUFFER_SIZE];

uint8_t mrbus_dev_addr = 0;

#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

uint16_t timeScaleFactor = 10;
uint8_t timeFlags = 0;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t dayOfWeek;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} TimeData;

TimeData fastTime;


void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 19;
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
	buf[15] = ' ';
	buf[16] = 'B';
	buf[17] = 'U';
	buf[18] = 'S';
}

void createTimePacket(uint8_t *buf)
{
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_DEST] = 0xFF;
	buf[MRBUS_PKT_LEN] = 18;			
	buf[5] = 'T';
	buf[6] = 0;
	buf[7] = 0;
	buf[8] = 0;
	buf[9] = timeFlags;
	buf[10] = fastTime.hours;
	buf[11] = fastTime.minutes;			
	buf[12] = fastTime.seconds;
	buf[13] = 0xFF & (timeScaleFactor>>8);
	buf[14] = 0xFF & timeScaleFactor;
	buf[15] = 0;
	buf[16] = 0;
	buf[17] = 0;
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
		CabData c;

		c.locoAddress = ((uint16_t)rxBuffer[6] << 8) + rxBuffer[7];
		if(c.locoAddress & LOCO_ADDRESS_SHORT)
		{
			// Short Address
			c.locoAddress &= ~(LOCO_ADDRESS_SHORT);
			if(c.locoAddress > 127)
				c.locoAddress = 0x27FF;
			else
				c.locoAddress += 0x2780;
		}
		else
		{
			// Long Address
			if(c.locoAddress > 9999)
				c.locoAddress = 0x270F;
		}
		
		c.speedDirection = rxBuffer[8];

		//  Yeah, yeah, not elegant but copied from code I knew worked and consolidated it here
		uint32_t functions = ((uint32_t)rxBuffer[9] << 24) + ((uint32_t)rxBuffer[10] << 16) + ((uint16_t)rxBuffer[11] << 8) + rxBuffer[12];
		c.functionGroup1 = ((functions & 0x01) << 4) | ((functions & 0x1E) >> 1);  // F0 F4 F3 F2 F1
		c.functionGroup2 = (functions >> 5) & 0xF;  // F8 F7 F6 F5
		c.functionGroup3 = (functions >> 9) & 0xF;  // F12 F11 F10 F9
		c.functionGroup4 = (functions >> 13) & 0xFF;  // F20 - F13
		c.functionGroup5 = (functions >> 21) & 0xFF;  // F28 - F21

//      Not used at the moment, so comment out to suppress compiler warnings
//		uint8_t statusFlags = rxBuffer[13];

		uint8_t delta = compareCabData(rxBuffer[MRBUS_PKT_SRC], &c);

		// Policy for sending Cab Bus updates:
		//    1) Locomotive address changed.  This throttle was controlling a different locomotive before, so update everything.
		//    2) Certain items have changed since last time.  Update only those items that changed.
		//    3) No changes since last time.  Update everything as a refresh.

		if(IS_LOCO_ADDRESS_CHANGED(delta) || IS_SPEED_DIRECTION_CHANGED(delta) || (0 == delta))
		{
			// Send Speed/Direction
			uint8_t speed = c.speedDirection & 0x7F;
			uint8_t direction = c.speedDirection & 0x80;
			if(1 == speed)
			{
				// E-stop
				cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
				cabBusBuffer[1] = c.locoAddress & 0x7F;
				cabBusBuffer[2] = (direction) ? 0x06 : 0x05;  // Direction for E-stop
				cabBusBuffer[3] = 0;
				cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
			}
			else
			{
				// Speed and direction
				cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
				cabBusBuffer[1] = c.locoAddress & 0x7F;
				cabBusBuffer[2] = (direction) ? 0x04 : 0x03;  // Direction for 128 speed step
				cabBusBuffer[3] = speed ? speed-1 : 0;  // Speed
				cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
			}
		}

		// Send function states
		if(IS_LOCO_ADDRESS_CHANGED(delta) || IS_FN_GROUP_1_CHANGED(delta) || (0 == delta))
		{
			cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
			cabBusBuffer[1] = c.locoAddress & 0x7F;
			cabBusBuffer[2] = 0x07;  // Function Group 1
			cabBusBuffer[3] = c.functionGroup1;
			cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
		}

		if(IS_LOCO_ADDRESS_CHANGED(delta) || IS_FN_GROUP_2_CHANGED(delta) || (0 == delta))
		{
			cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
			cabBusBuffer[1] = c.locoAddress & 0x7F;
			cabBusBuffer[2] = 0x08;  // Function Group 2
			cabBusBuffer[3] = c.functionGroup2;
			cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
		}

		if(IS_LOCO_ADDRESS_CHANGED(delta) || IS_FN_GROUP_3_CHANGED(delta) || (0 == delta))
		{
			cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
			cabBusBuffer[1] = c.locoAddress & 0x7F;
			cabBusBuffer[2] = 0x09;  // Function Group 3
			cabBusBuffer[3] = c.functionGroup3;
			cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
		}

		if(IS_LOCO_ADDRESS_CHANGED(delta) || IS_FN_GROUP_4_CHANGED(delta) || (0 == delta))
		{
			cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
			cabBusBuffer[1] = c.locoAddress & 0x7F;
			cabBusBuffer[2] = 0x15;  // Function Group 4
			cabBusBuffer[3] = c.functionGroup4;
			cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
		}

		if(IS_LOCO_ADDRESS_CHANGED(delta) || IS_FN_GROUP_5_CHANGED(delta) || (0 == delta))
		{
			cabBusBuffer[0] = (c.locoAddress >> 7) & 0x7F;  // Locomotive Address
			cabBusBuffer[1] = c.	locoAddress & 0x7F;
			cabBusBuffer[2] = 0x16;  // Function Group 5
			cabBusBuffer[3] = c.functionGroup5;
			cabBusPktQueuePush(&cabBusTxQueue, cabBusBuffer, 4);
		}
		
		updateCabData(rxBuffer[MRBUS_PKT_SRC], &c);
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
	uint8_t switches = (PINC >> 3) & 0x1F;
	if(oldSwitches != switches)
	{
		cabBusInit(switches);
		mrbus_dev_addr = 0xD0 + switches;
		oldSwitches = switches;
	}
}

uint8_t adjustCabBusASCII(uint8_t chr)
{
	if(chr & 0x20)
		return(chr & 0x3F);  // Clear bit 6 & 7
	else
		return(chr & 0x7F);  // Clear only bit 7
}

int main(void)
{
	uint8_t mrbusTxBuffer[MRBUS_BUFFER_SIZE];
	uint16_t decisecs_tmp = 0;
	
	uint8_t enableFastTime = 1;

	init();

	wdt_reset();
	
	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);

	cabBusPktQueueInitialize(&cabBusTxQueue, cabBusTxPktBufferArray, CABBUS_TX_BUFFER_DEPTH);
	cabBusPktQueueInitialize(&cabBusRxQueue, cabBusRxPktBufferArray, CABBUS_RX_BUFFER_DEPTH);

	readDipSwitches();  // cabBusInit will be called here
	mrbeeInit();

	sei();	

	wdt_reset();

	// Fire off initial reset version packet
	createVersionPacket(0xFF, mrbusTxBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);

	wdt_reset();

	while(1)
	{
		wdt_reset();

		readDipSwitches();
		
#ifndef DEBUG
		DDRB |= _BV(PB7);  // Set PB7 as output pulling low
		PORTB &= ~_BV(PB7);
		
		DDRB &= ~(_BV(PB5) | _BV(PB6));  // Set PB5 and PB6 as input, pull-up enabled
		PORTB |= (_BV(PB5) | _BV(PB6));
		
		// Check programming header pins 1 & 3
		// If a jumper is on 1-3, PB7 will pull PB6 low.
		enableFastTime = PINB & _BV(PB6);
		
		// Check programming header pins 4 & 6
		// If a jumper is on 4-6, GND will pull PB5 low.
		// Do something useful eventually
#endif
		
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
			mrbeeTransmit();
		}

		if (cabBusPktQueueDepth(&cabBusRxQueue))
		{
			uint8_t rxBuffer[CABBUS_BUFFER_SIZE];

			if (0 != cabBusPktQueuePop(&cabBusRxQueue, rxBuffer, sizeof(rxBuffer)))
			{
				switch(rxBuffer[1])
				{
					case 0xC1:
						// Fast Time ASCII
						fastTime.hours = ((adjustCabBusASCII(rxBuffer[3]) - '0') * 10) + (adjustCabBusASCII(rxBuffer[4]) - '0');
						fastTime.minutes = ((adjustCabBusASCII(rxBuffer[6]) - '0') * 10) + (adjustCabBusASCII(rxBuffer[7]) - '0');
						if('A' == (adjustCabBusASCII(rxBuffer[8])))
						{
							timeFlags |= TIME_FLAGS_DISP_FAST_AMPM;
							if(fastTime.hours > 11)
								fastTime.hours -= 12;
						}
						else if('P' == (adjustCabBusASCII(rxBuffer[8])))
						{
							timeFlags |= TIME_FLAGS_DISP_FAST_AMPM;
							if(fastTime.hours < 12)
								fastTime.hours += 12;
						}
						else
						{
							timeFlags &= ~TIME_FLAGS_DISP_FAST_AMPM;
						}
						timeFlags |= TIME_FLAGS_DISP_FAST;
						if(enableFastTime)
						{
							createTimePacket(mrbusTxBuffer);
							mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);
						}
						break;
					case 0xD4:
						// Fast Time Ratio
						timeScaleFactor = ((uint16_t)(rxBuffer[2] & 0x3F)) * 10;
						timeFlags |= TIME_FLAGS_DISP_FAST;
						if(enableFastTime)
						{
							createTimePacket(mrbusTxBuffer);
							mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);
						}
						break;
				}
			}
		}

		if(cabBusPktQueueDepth(&cabBusTxQueue))
		{
			// Packet pending, so queue it up for transmit
			wdt_reset();
			cabBusTransmit();
		}
	}

}


