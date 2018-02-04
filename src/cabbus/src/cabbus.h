/*************************************************************************
Title:    Cab Bus Atmel AVR Header
Authors:  Michael Petersen <railfan@drgw.net>, Colorado, USA
          Nathan Holmes <maverick@drgw.net>, Colorado, USA
File:     cabbus.h
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

#ifndef CABBUS_AVR_H
#define CABBUS_AVR_H

#include "cabbus-constants.h"
#include "cabbus-macros.h"
#include "cabbus-queue.h"
#include "cabbus-cache.h"

// AVR type-specific stuff
// Define the UART port and registers used for XBee communication
// Follows the format of the AVR UART library by Fleury/Sharpe

#if defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)

#if defined(CABBUS_ATMEGA_USART1)
#define CABBUS_UART_RX_INTERRUPT    USART1_RX_vect
#define CABBUS_UART_TX_INTERRUPT    USART1_UDRE_vect
#define CABBUS_UART_DONE_INTERRUPT  USART1_TX_vect
#define CABBUS_PORT                 PORTD
#define CABBUS_PIN                  PIND
#define CABBUS_DDR                  DDRD

#ifndef CABBUS_TXE
#define CABBUS_TXE                  4       /* PD4 */
#endif
#ifndef CABBUS_TX
#define CABBUS_TX                   3       /* PD1 */
#endif
#ifndef CABBUS_RX
#define CABBUS_RX                   2       /* PD0 */
#endif

#define CABBUS_UART_UBRR           UBRR1
#define CABBUS_UART_CSR_A          UCSR1A
#define CABBUS_UART_CSR_B          UCSR1B
#define CABBUS_UART_CSR_C          UCSR1C
#define CABBUS_UART_DATA           UDR1
#define CABBUS_UART_UDRIE          UDRIE1
#define CABBUS_RXEN                RXEN1
#define CABBUS_TXEN                TXEN1
#define CABBUS_RXCIE               RXCIE1
#define CABBUS_TXCIE               TXCIE1
#define CABBUS_TXC                 TXC1
#define CABBUS_RXB8                RXB81
#define CABBUS_RX_ERR_MASK         (_BV(FE1) | _BV(DOR1))

#else
#define CABBUS_ATMEGA_USART0
#define CABBUS_UART_RX_INTERRUPT    USART0_RX_vect
#define CABBUS_UART_TX_INTERRUPT    USART0_UDRE_vect
#define CABBUS_UART_DONE_INTERRUPT  USART0_TX_vect
#define CABBUS_PORT                 PORTD
#define CABBUS_PIN                  PIND
#define CABBUS_DDR                  DDRD

#ifndef CABBUS_TXE
#define CABBUS_TXE                  4       /* PD4 */
#endif
#ifndef CABBUS_TX
#define CABBUS_TX                   1       /* PD1 */
#endif
#ifndef CABBUS_RX
#define CABBUS_RX                   0       /* PD0 */
#endif

#define CABBUS_UART_UBRR           UBRR0
#define CABBUS_UART_CSR_A          UCSR0A
#define CABBUS_UART_CSR_B          UCSR0B
#define CABBUS_UART_CSR_C          UCSR0C
#define CABBUS_UART_DATA           UDR0
#define CABBUS_UART_UDRIE          UDRIE0
#define CABBUS_RXEN                RXEN0
#define CABBUS_TXEN                TXEN0
#define CABBUS_RXCIE               RXCIE0
#define CABBUS_TXCIE               TXCIE0
#define CABBUS_TXC                 TXC0
#define CABBUS_RXB8                RXB80
#define CABBUS_RX_ERR_MASK         (_BV(FE0) | _BV(DOR0))
#endif

#else
#error "No UART definition for MCU available"
#error "Please feel free to add one and send us the patch"
#endif

extern CabBusPktQueue cabBusTxQueue;
extern CabBusPktQueue cabBusRxQueue;

uint8_t cabBusTransmit(void);
void cabBusInit(uint8_t addr);

uint8_t cabBusPing(void);
uint8_t cabBusResponse(void);
uint8_t cabBusCollision(void);

#endif // CABBUS_AVR_H


