/**********************************************************************
* CPEG222 Project 6 - HC-SR04 ultrasonic sensor helper (source)
*
* Pins (Q: "Which pins and peripherals does the HC-SR04 use?")
*   VCC  : +5 V from the shield        -> powers the sensor
*   GND  : ground                      -> common reference
*   TRIG : PE3 (GPIO output)           -> we send a 10 us HIGH pulse here
*                                         to START a measurement
*   ECHO : PG1 (GPIO input + EXTI1)    -> sensor holds this HIGH for as long
*                                         as the sound took to go out and
*                                         come back (150 us ... 25 ms)
*   TIM7 : free-running 16-bit timer   -> counts once per microsecond so we
*                                         can time the ECHO pulse
*   EXTI1: external interrupt line 1   -> fires on every ECHO edge
**********************************************************************/
#include "HCSR04.h"

#define TRIG_PIN 3U   // PE3
#define ECHO_PIN 1U   // PG1

void HCSR04_Init(void)
{
    // ---- Clocks: GPIOE (TRIG), GPIOG (ECHO), SYSCFG (routes pin -> EXTI), TIM7
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOGEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // ---- TRIG (PE3): output, idle LOW
    GPIOE->ODR   &= ~(1U << TRIG_PIN);
    GPIOE->MODER &= ~(3U << (TRIG_PIN * 2));
    GPIOE->MODER |=  (1U << (TRIG_PIN * 2));          // 01 = output

    // ---- ECHO (PG1): input (MODER = 00) with pull-down so it never floats
    GPIOG->MODER &= ~(3U << (ECHO_PIN * 2));
    GPIOG->PUPDR &= ~(3U << (ECHO_PIN * 2));
    GPIOG->PUPDR |=  (2U << (ECHO_PIN * 2));          // 10 = pull-down

    // ---- EXTI1: connect PG1 to EXTI line 1.
    // EXTICR[0] holds lines 0-3, 4 bits each; port G = 0b0110.
    SYSCFG->EXTICR[0] &= ~(0xFU << 4);                // line 1 field = bits 7:4
    SYSCFG->EXTICR[0] |=  (0x6U << 4);                // 6 = port G

    // Interrupt on BOTH edges. Rising edge = echo started (save the time),
    // falling edge = echo ended (width = end - start).
    EXTI->RTSR |= (1U << ECHO_PIN);
    EXTI->FTSR |= (1U << ECHO_PIN);
    EXTI->PR    =  (1U << ECHO_PIN);                  // clear stale flag
    EXTI->IMR  |= (1U << ECHO_PIN);                   // unmask line 1

    // ECHO timing is the most time-critical thing -> highest priority (0).
    // (TIM5 SSD refresh is priority 1, SysTick is lower.)
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_EnableIRQ(EXTI1_IRQn);

    // ---- TIM7: free-running 1 MHz counter.
    // TIM7 is on APB1. APB1 = 45 MHz but its prescaler is 4 (not 1), so the
    // timer clock is doubled = 90 MHz.  PSC = 90 - 1 -> 90 MHz / 90 = 1 MHz
    // = one count per microsecond.  ARR = 0xFFFF -> wraps every 65.536 ms.
    TIM7->PSC = 90 - 1;
    TIM7->ARR = 0xFFFF;
    TIM7->EGR = TIM_EGR_UG;                           // load PSC/ARR now
    TIM7->CR1 |= TIM_CR1_CEN;                         // start counting forever
}

// Busy-wait for 'us' microseconds. Subtracting as uint16_t keeps it correct
// even when the 16-bit counter wraps from 65535 back to 0.
void delay_us(uint16_t us)
{
    uint16_t start = HCSR04_Now();
    while ((uint16_t)(HCSR04_Now() - start) < us) { }
}

// The sensor starts a measurement when TRIG goes HIGH for about 10 us.
// It then sends 8 bursts of 40 kHz ultrasound and raises ECHO.
void HCSR04_Trigger(void)
{
    GPIOE->BSRR = (1U << TRIG_PIN);                   // TRIG high
    delay_us(10);                                     // ~10 us
    GPIOE->BSRR = (1U << (TRIG_PIN + 16));            // TRIG low
}
