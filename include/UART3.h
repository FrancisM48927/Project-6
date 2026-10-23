/**********************************************************************
* CPEG222 Project 6 - UART3 (USART3) serial monitor helper (header)
**********************************************************************/
#ifndef UART3_H
#define UART3_H

#include "stm32f4xx.h"

void UART3_Init(void);              // 115200 baud, 8N1, TX only
void UART3_SendString(const char *s);

#endif
