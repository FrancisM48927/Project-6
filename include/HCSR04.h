/**********************************************************************
* CPEG222 Project 6 - HC-SR04 ultrasonic sensor helper (header)
* TRIG = PE3 (output), ECHO = PG1 (input, EXTI1), TIM7 = 1 us timer
**********************************************************************/
#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f4xx.h"
#include <stdint.h>

void HCSR04_Init(void);          // GPIO, EXTI1 and TIM7 setup
void HCSR04_Trigger(void);       // send the ~10 us TRIG pulse
void delay_us(uint16_t us);      // busy-wait using TIM7 (1 us per count)

// Current TIM7 count (1 count = 1 us, wraps every 65536 us)
static inline uint16_t HCSR04_Now(void) { return (uint16_t)TIM7->CNT; }

#endif
