/*************************************************************************
Title:    XpressNet Atmel AVR Header
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

#ifndef XPRESSNET_AVR_H
#define XPRESSNET_AVR_H

#include "xpressnet-constants.h"
#include "xpressnet-macros.h"
#include "xpressnet-queue.h"

// AVR type-specific stuff
// Define the UART port and registers used for XBee communication
// Follows the format of the AVR UART library by Fleury/Sharpe

#if defined(__AVR_ATmega162__)

#define XPRESSNET_ATMEGA_USART0_SIMPLE
#define XPRESSNET_UART_RX_INTERRUPT    USART0_RXC_vect
#define XPRESSNET_UART_TX_INTERRUPT    USART0_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT  USART0_TXC_vect
#define XPRESSNET_PORT                 PORTD
#define XPRESSNET_PIN                  PIND
#define XPRESSNET_DDR                  DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                  2       /* PD2 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                   1       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                   0       /* PD0 */
#endif

#define XPRESSNET_UART_UBRR            UBRR0L
#define XPRESSNET_UART_CSR_A           UCSR0A
#define XPRESSNET_UART_CSR_B           UCSR0B
#define XPRESSNET_UART_CSR_C           UCSR0C
#define XPRESSNET_UART_DATA            UDR0
#define XPRESSNET_UART_UDRIE           UDRIE0
#define XPRESSNET_RXEN                 RXEN0
#define XPRESSNET_TXEN                 TXEN0
#define XPRESSNET_RXCIE                RXCIE0
#define XPRESSNET_TXCIE                TXCIE0
#define XPRESSNET_TXC                  TXC0
#define XPRESSNET_RX_ERR_MASK          (_BV(FE0) | _BV(DOR0))

#elif  defined(__AVR_ATmega8__)

#define XPRESSNET_ATMEGA_USART_SIMPLE
#define XPRESSNET_UART_RX_INTERRUPT    USART_RXC_vect
#define XPRESSNET_UART_TX_INTERRUPT    USART_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT  USART_TXC_vect
#define XPRESSNET_PORT                 PORTD
#define XPRESSNET_PIN                  PIND
#define XPRESSNET_DDR                  DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                  2       /* PD2 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                   1       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                   0       /* PD0 */
#endif

#define XPRESSNET_UART_UBRR            UBRRL
#define XPRESSNET_UART_CSR_A           UCSRA
#define XPRESSNET_UART_CSR_B           UCSRB
#define XPRESSNET_UART_CSR_C           UCSRC
#define XPRESSNET_UART_DATA            UDR
#define XPRESSNET_UART_UDRIE           UDRIE
#define XPRESSNET_RXEN                 RXEN
#define XPRESSNET_TXEN                 TXEN
#define XPRESSNET_RXCIE                RXCIE
#define XPRESSNET_TXCIE                TXCIE
#define XPRESSNET_TXC                  TXC
#define XPRESSNET_RX_ERR_MASK          (_BV(FE) | _BV(DOR))



#elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega48P__) || defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
    defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
#define XPRESSNET_ATMEGA_USART
#define XPRESSNET_UART_RX_INTERRUPT    USART_RX_vect
#define XPRESSNET_UART_TX_INTERRUPT    USART_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT  USART_TX_vect
#define XPRESSNET_PORT                 PORTD
#define XPRESSNET_PIN                  PIND
#define XPRESSNET_DDR                  DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                  2       /* PD2 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                   1       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                   0       /* PD0 */
#endif

#define XPRESSNET_UART_UBRR            UBRR0
#define XPRESSNET_UART_CSR_A           UCSR0A
#define XPRESSNET_UART_CSR_B           UCSR0B
#define XPRESSNET_UART_CSR_C           UCSR0C
#define XPRESSNET_UART_DATA            UDR0
#define XPRESSNET_UART_UDRIE           UDRIE0
#define XPRESSNET_RXEN                 RXEN0
#define XPRESSNET_TXEN                 TXEN0
#define XPRESSNET_RXCIE                RXCIE0
#define XPRESSNET_TXCIE                TXCIE0
#define XPRESSNET_TXC                  TXC0
#define XPRESSNET_RX_ERR_MASK          (_BV(FE0) | _BV(DOR0))

#elif defined(__AVR_ATmega32U4__)

#define XPRESSNET_ATMEGA_USART1
#define XPRESSNET_UART_RX_INTERRUPT    USART1_RX_vect
#define XPRESSNET_UART_TX_INTERRUPT    USART1_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT  USART1_TX_vect
#define XPRESSNET_PORT                 PORTD
#define XPRESSNET_PIN                  PIND
#define XPRESSNET_DDR                  DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                  2       /* PD2 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                   1       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                   0       /* PD0 */
#endif


#define XPRESSNET_UART_UBRR           UBRR1
#define XPRESSNET_UART_CSR_A          UCSR1A
#define XPRESSNET_UART_CSR_B          UCSR1B
#define XPRESSNET_UART_CSR_C          UCSR1C
#define XPRESSNET_UART_DATA           UDR1
#define XPRESSNET_UART_UDRIE          UDRIE1
#define XPRESSNET_RXEN                RXEN1
#define XPRESSNET_TXEN                TXEN1
#define XPRESSNET_RXCIE               RXCIE1
#define XPRESSNET_TXCIE               TXCIE1
#define XPRESSNET_TXC                 TXC1
#define XPRESSNET_RX_ERR_MASK         (_BV(FE1) | _BV(DOR1))


#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)

#if defined(XPRESSNET_ATMEGA_USART1)
#define XPRESSNET_UART_RX_INTERRUPT    USART1_RX_vect
#define XPRESSNET_UART_TX_INTERRUPT    USART1_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT  USART1_TX_vect
#define XPRESSNET_PORT                 PORTD
#define XPRESSNET_PIN                  PIND
#define XPRESSNET_DDR                  DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                  4       /* PD4 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                   3       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                   2       /* PD0 */
#endif

#define XPRESSNET_UART_UBRR           UBRR1
#define XPRESSNET_UART_CSR_A          UCSR1A
#define XPRESSNET_UART_CSR_B          UCSR1B
#define XPRESSNET_UART_CSR_C          UCSR1C
#define XPRESSNET_UART_DATA           UDR1
#define XPRESSNET_UART_UDRIE          UDRIE1
#define XPRESSNET_RXEN                RXEN1
#define XPRESSNET_TXEN                TXEN1
#define XPRESSNET_RXCIE               RXCIE1
#define XPRESSNET_TXCIE               TXCIE1
#define XPRESSNET_TXC                 TXC1
#define XPRESSNET_RXB8                RXB81
#define XPRESSNET_RX_ERR_MASK         (_BV(FE1) | _BV(DOR1))

#else
#define XPRESSNET_ATMEGA_USART0
#define XPRESSNET_UART_RX_INTERRUPT    USART0_RX_vect
#define XPRESSNET_UART_TX_INTERRUPT    USART0_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT  USART0_TX_vect
#define XPRESSNET_PORT                 PORTD
#define XPRESSNET_PIN                  PIND
#define XPRESSNET_DDR                  DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                  4       /* PD4 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                   1       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                   0       /* PD0 */
#endif

#define XPRESSNET_UART_UBRR           UBRR0
#define XPRESSNET_UART_CSR_A          UCSR0A
#define XPRESSNET_UART_CSR_B          UCSR0B
#define XPRESSNET_UART_CSR_C          UCSR0C
#define XPRESSNET_UART_DATA           UDR0
#define XPRESSNET_UART_UDRIE          UDRIE0
#define XPRESSNET_RXEN                RXEN0
#define XPRESSNET_TXEN                TXEN0
#define XPRESSNET_RXCIE               RXCIE0
#define XPRESSNET_TXCIE               TXCIE0
#define XPRESSNET_TXC                 TXC0
#define XPRESSNET_RXB8                RXB80
#define XPRESSNET_RX_ERR_MASK         (_BV(FE0) | _BV(DOR0))
#endif

#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)

#define XPRESSNET_ATTINY_USART
#define XPRESSNET_UART_RX_INTERRUPT   USART_RX_vect
#define XPRESSNET_UART_TX_INTERRUPT   USART_UDRE_vect
#define XPRESSNET_UART_DONE_INTERRUPT USART_TX_vect
#define XPRESSNET_PORT                PORTD
#define XPRESSNET_PIN                 PIND
#define XPRESSNET_DDR                 DDRD

#ifndef XPRESSNET_TXE
#define XPRESSNET_TXE                 2       /* PD2 */
#endif
#ifndef XPRESSNET_TX
#define XPRESSNET_TX                  1       /* PD1 */
#endif
#ifndef XPRESSNET_RX
#define XPRESSNET_RX                  0       /* PD0 */
#endif

#define XPRESSNET_UART_UBRRH          UBRRH
#define XPRESSNET_UART_UBRRL          UBRRL
#define XPRESSNET_UART_CSR_A          UCSRA
#define XPRESSNET_UART_CSR_B          UCSRB
#define XPRESSNET_UART_CSR_C          UCSRC
#define XPRESSNET_UART_DATA           UDR
#define XPRESSNET_UART_UDRIE          UDRIE
#define XPRESSNET_RXEN                RXEN
#define XPRESSNET_TXEN                TXEN
#define XPRESSNET_RXCIE               RXCIE
#define XPRESSNET_TXCIE               TXCIE
#define XPRESSNET_TXC                 TXC
#define XPRESSNET_RX_ERR_MASK         (_BV(FE) | _BV(DOR))
#else
#error "No UART definition for MCU available"
#error "Please feel free to add one and send us the patch"
#endif

extern XpressNetPktQueue xpressnetTxQueue;

uint8_t xpressnetTransmit(void);
void xpressnetInit(uint8_t addr);

#endif // XPRESSNET_AVR_H


