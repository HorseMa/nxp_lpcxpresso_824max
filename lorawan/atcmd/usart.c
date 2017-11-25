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
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#if 0
// USART1
#define USART             USART1
#define USART_TX_PORT     GPIOA
#define USART_TX_PIN      9
#define USART_RX_PORT     GPIOA
#define USART_RX_PIN      10
#define USART_AF          0x07
#define USART_RCCREG      APB2ENR
#define USART_RCCVAL      RCC_APB2ENR_USART0EN
#define USART_IRQN        USART0_IRQn
#define USART_IRQHANDLER  UART0_IRQHandler

void usart_init () {
#if 0
    taskENTER_CRITICAL();

    // configure USART (115200/8N1)
    RCC->USART_RCCREG |= USART_RCCVAL;
    hw_cfg_pin(USART_TX_PORT, USART_TX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|USART_AF);
    hw_cfg_pin(USART_RX_PORT, USART_RX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|USART_AF);
    USART->BRR = 277; // 115200
    // configure NVIC
    NVIC->IP[USART_IRQN]      = 0x70; // interrupt priority
    NVIC->ISER[USART_IRQN>>5] = 1 << (USART_IRQN&0x1F);  // set enable IRQ
    // enable usart
    USART->CR1 = USART_CR1_UE;
    taskEXIT_CRITICAL();
#endif
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
    hal_disableIRQs();
    // check status reg (clears most of the flags)
    /*uint32_t sr = USART->SR;
    if (sr & (USART_SR_PE|USART_SR_ORE|USART_SR_FE))  {
    hal_failed();
    }*/
    // check for tx reg empty
    if((Chip_UART_GetStatus(LPC_USART0) & UART_STAT_TXRDY) != 0) {
    uint16_t c = frame_tx(1);
    if((c & 0xFF00) == 0) { // send next char
        Chip_UART_SendByte(LPC_USART0, c);
    } else { // no more chars - wait for completion
            frame_tx(0);
            //USART->CR1 &= ~USART_CR1_TXEIE;
            //USART->CR1 |= USART_CR1_TCIE;
            //Chip_UART_IntDisable(LPC_USART0, UART_INTEN_TXRDY);
        }
    }

    // check for tx complete
    /*if( (USART->CR1 & USART_CR1_TCIE) && (sr & USART_SR_TC) ) {
        //USART->CR1 &= ~(USART_CR1_TE | USART_CR1_TCIE); // stop transmitter
    frame_tx(0);
    }*/

    // check for rx reg not empty
    if((Chip_UART_GetStatus(LPC_USART0) & UART_STAT_RXRDY) != 0) {
    if(frame_rx(Chip_UART_ReadByte(LPC_USART0)) == 0) {
            Chip_UART_IntDisable(LPC_USART0, UART_INTEN_RXRDY);
        //USART->CR1 &= ~(USART_CR1_RE | USART_CR1_RXNEIE); // stop receiver
    }
    }

    hal_enableIRQs();
}
#endif

#define USART             USART1
#define USART_TX_PORT     GPIOA
#define USART_TX_PIN      9
#define USART_RX_PORT     GPIOA
#define USART_RX_PIN      10
#define USART_AF          0x07
#define USART_RCCREG      APB2ENR
#define USART_RCCVAL      RCC_APB2ENR_USART1EN
#define USART_IRQN        USART1_IRQn
#define USART_IRQHANDLER  USART1_IRQHandler

void usart_init () {
    hal_disableIRQs();
    #if 0
    // configure USART (115200/8N1)
    RCC->USART_RCCREG |= USART_RCCVAL;
    hw_cfg_pin(USART_TX_PORT, USART_TX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|USART_AF);
    hw_cfg_pin(USART_RX_PORT, USART_RX_PIN, GPIOCFG_MODE_ALT|GPIOCFG_OSPEED_40MHz|GPIOCFG_OTYPE_PUPD|GPIOCFG_PUPD_PUP|USART_AF);
    USART->BRR = 277; // 115200
    // configure NVIC
    NVIC->IP[USART_IRQN]      = 0x70; // interrupt priority
    NVIC->ISER[USART_IRQN>>5] = 1 << (USART_IRQN&0x1F);  // set enable IRQ
    // enable usart
    USART->CR1 = USART_CR1_UE;
    #endif
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
    hal_disableIRQs();
    // check status reg (clears most of the flags)
    /*u4_t sr = USART->SR;
    if (sr & (USART_SR_PE|USART_SR_ORE|USART_SR_FE))  {
    hal_failed();
    }*/

    // check for tx reg empty
    if((Chip_UART_GetStatus(LPC_USART0) & UART_STAT_TXRDY) != 0) {
        u2_t c = frame_tx(1);
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
    hal_enableIRQs();
}

