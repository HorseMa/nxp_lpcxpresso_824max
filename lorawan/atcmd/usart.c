/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "modem.h"
#include "board.h"
#include "utilities.h"
#include "chip.h"
#include "string.h"

#define USART             USART1
#define USART_TX_PORT     GPIOA
#define USART_TX_PIN      9
#define USART_RX_PORT     GPIOA
#define USART_RX_PIN      10
#define USART_AF          0x07
#define USART_RCCREG      APB2ENR
#define USART_RCCVAL      RCC_APB2ENR_USART1EN
#define USART_IRQN        USART0_IRQn
#define USART_IRQHANDLER  UART0_IRQHandler

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Transmit and receive ring buffers */
RINGBUFF_T txring, rxring;

/* Ring buffer size */
#define UART_RB_SIZE 64

/* Set the default UART, IRQ number, and IRQ handler name */
#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

/* Default baudrate for testing */
#define UART_TEST_DEFAULT_BAUDRATE 115200

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];
#define	UART_CLOCK_DIV	1
void usart_init () {
    hal_disableIRQs();
    /* Enable the clock to the Switch Matrix */
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

    Chip_Clock_SetUARTClockDiv(UART_CLOCK_DIV);

    /* Connect the U0_TXD_O and U0_RXD_I signals to port pins(P0.4, P0.0) */
    Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);
    Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 0);
    Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 4);
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
    /* Setup UART */
    Chip_UART_Init(LPC_USART);
    Chip_UART_ConfigData(LPC_USART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
    Chip_Clock_SetUSARTNBaseClockRate((115200 * 16), true);
    Chip_UART_SetBaud(LPC_USART, UART_TEST_DEFAULT_BAUDRATE);
    Chip_UART_Enable(LPC_USART);
    Chip_UART_TXEnable(LPC_USART);

    /* Before using the ring buffers, initialize them using the ring
    buffer init function */
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(LPC_USART, UART_INTEN_RXRDY);
    Chip_UART_IntDisable(LPC_USART, UART_INTEN_TXRDY);	/* May not be needed */

    /* preemption = 1, sub-priority = 1 */
    NVIC_EnableIRQ(LPC_IRQNUM);
    hal_enableIRQs();
}

void usart_starttx () {
    //USART->CR1 |= (USART_CR1_TE | USART_CR1_TXEIE);
    Chip_UART_IntEnable(LPC_USART0, UART_INTEN_TXRDY);
}

void usart_startrx () {
    //USART->CR1 |= (USART_CR1_RE | USART_CR1_RXNEIE);
    Chip_UART_IntEnable(LPC_USART0, UART_INTEN_RXRDY);
}

void USART_IRQHANDLER (void) {
    //int bytes;
    //hal_disableIRQs();
    Chip_UART_IRQRBHandler(LPC_USART0, &rxring, &txring);
#if 0
    hal_disableIRQs();
    // check status reg (clears most of the flags)
    /*u4_t sr = USART->SR;
    if (sr & (USART_SR_PE|USART_SR_ORE|USART_SR_FE))  {
    hal_failed();
    }*/

    // check for tx reg empty
    if((Chip_UART_GetStatus(LPC_USART0) & UART_STAT_TXRDY) != 0) {
        uint16_t c = frame_tx(1);
        if((c & 0xFF00) == 0) { // send next char
            //USART->DR = c;
            Chip_UART_SendByte(LPC_USART0, c);
        } else { // no more chars - wait for completion
                //USART->CR1 &= ~USART_CR1_TXEIE;
                //USART->CR1 |= USART_CR1_TCIE;
            Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXRDY);
            Chip_UART_IntEnable(LPC_USART0, UART_INTEN_TXIDLE);

        }

        // check for tx complete
        if((Chip_UART_GetStatus(LPC_USART0) & UART_STAT_TXIDLE) != 0) {
        //    USART->CR1 &= ~(USART_CR1_TE | USART_CR1_TCIE); // stop transmitter
        Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXRDY);
        Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXIDLE);
        frame_tx(0);
        }
    }

    // check for rx reg not empty
    if((Chip_UART_GetStatus(LPC_USART0) & UART_STAT_RXRDY) != 0) {
    if(frame_rx(Chip_UART_ReadByte(LPC_USART0)) == 0) {
            Chip_UART_IntDisable(LPC_USART0, UART_INTEN_RXRDY);
        //USART->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // stop receiver
    }
    }
#endif
#if 0
    // check status reg (clears most of the flags)
    u4_t sr = USART->SR;
    if (sr & (USART_SR_PE|USART_SR_ORE|USART_SR_FE))  {
    hal_failed();
    }

    // check for tx reg empty
    if( (USART->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE) ) {
    u2_t c = frame_tx(1);
    if((c & 0xFF00) == 0) { // send next char
        USART->DR = c;
    } else { // no more chars - wait for completion
            USART->CR1 &= ~USART_CR1_TXEIE;
            USART->CR1 |= USART_CR1_TCIE;
        }
    }

    // check for tx complete
    if( (USART->CR1 & USART_CR1_TCIE) && (sr & USART_SR_TC) ) {
        USART->CR1 &= ~(USART_CR1_TE | USART_CR1_TCIE); // stop transmitter
    frame_tx(0);
    }

    // check for rx reg not empty
    if( (USART->CR1 & USART_CR1_RXNEIE) && (sr & USART_SR_RXNE) ) {
    if(frame_rx(USART->DR) == 0) {
        USART->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // stop receiver
    }
    }
#endif
    //hal_enableIRQs();
}

