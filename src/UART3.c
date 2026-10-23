/**********************************************************************
* CPEG222 Project 6 - UART3 (USART3) serial monitor helper (source)
* On the Nucleo-F446ZE the ST-Link virtual COM port is USART3:
*   PD8 = TX (AF7), PD9 = RX (AF7)
**********************************************************************/
#include "UART3.h"

void UART3_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;              // GPIOD clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;             // USART3 clock

    // PD8, PD9 -> alternate function mode (10), AF7 = USART3
    GPIOD->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)));
    GPIOD->MODER |=  ((2U << (8 * 2)) | (2U << (9 * 2)));
    GPIOD->AFR[1] &= ~((0xFU << ((8 - 8) * 4)) | (0xFU << ((9 - 8) * 4)));
    GPIOD->AFR[1] |=  ((7U   << ((8 - 8) * 4)) | (7U   << ((9 - 8) * 4)));

    // Baud rate: USART3 is on APB1 = 45 MHz.
    // BRR = fPCLK / baud = 45,000,000 / 115,200 = 390.6 -> 391 (0.1 % error)
    USART3->BRR = (45000000U + 57600U) / 115200U;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // 8N1, enable
}

void UART3_SendString(const char *s)
{
    while (*s) {
        while (!(USART3->SR & USART_SR_TXE)) { }      // wait: TX register empty
        USART3->DR = (uint8_t)*s++;
    }
    while (!(USART3->SR & USART_SR_TC)) { }           // wait: last byte fully sent
}
