/**********************************************************************
* Author: F. MAILOM
* CPEG222 Project 6, 10/22/26
* NucleoF466ZE CMSIS Scanning HC-SR04 Radar
**********************************************************************/
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include "SysClock180MHz.h"
#include "ssd.h"
#include "OLED.h"
#include "UART3.h"
#include "HCSR04.h"

// ---------------- Settings ----------------
// The rubric asks for ~10 serial lines in 5 s, i.e. one every 0.5 s.
// (The slide says "about 250 ms"; change this to 250 if your TA wants that.)
#define MEASURE_PERIOD_MS   500
#define ECHO_TIMEOUT_MS     60      // give up if no echo (> 38 ms max)
#define US_PER_CM           58.3f   // 1 cm of distance = 58.3 us of echo (343 m/s, round trip)
#define US_PER_INCH         148.1f  // 58.3 x 2.54 (same constant as the slides)

#define SWITCH_S1_PIN       8U      // S1 on PC8: 1 = inches, 0 = centimeters

// ---------------- Shared with interrupts ----------------
// volatile: ISRs change these behind main's back, so the compiler must
// re-read them from memory every time instead of caching them in a register.
volatile uint32_t msTicks        = 0;  // ms since start, incremented by SysTick
volatile uint8_t  measureRequest = 0;  // set by SysTick every 500 ms
volatile uint8_t  echoDone       = 0;  // set by EXTI1 when falling edge seen
volatile uint16_t echoStart      = 0;  // TIM7 count at the rising edge
volatile uint16_t pulseWidth     = 0;  // ECHO width in us (= TIM7 counts)

// ---------------- Interrupt handlers ----------------

// SysTick: 1 ms tick. Every MEASURE_PERIOD_MS we ask main() for a reading.
void SysTick_Handler(void)
{
    static uint16_t elapsed = 0;
    msTicks++;
    if (++elapsed >= MEASURE_PERIOD_MS) {
        elapsed = 0;
        measureRequest = 1;
    }
}

// TIM5: refreshes ONE seven-segment digit per interrupt (from ssd.c).
void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR &= ~TIM_SR_UIF;       // clear the flag or it fires forever
        SSD_Refresh();
    }
}

// EXTI1: fires on BOTH edges of ECHO (PG1).
void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1U << 1)) {
        EXTI->PR = (1U << 1);          // clear pending flag (write 1 to clear)

        if (GPIOG->IDR & (1U << 1)) {
            // Both edges are needed: width = (time of fall) - (time of rise).
            // PG1 is HIGH -> RISING edge: echo pulse just started
            echoStart = HCSR04_Now();
        } else {
            // PG1 is LOW -> FALLING edge: echo pulse just ended.
            // uint16_t subtraction is correct even if TIM7 wrapped around.
            pulseWidth = (uint16_t)(HCSR04_Now() - echoStart);
            echoDone = 1;
        }
    }
}

// ---------------- Helpers ----------------

static void Switch_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3U << (SWITCH_S1_PIN * 2));      // 00 = input
}

static uint8_t Switch_IsInches(void)
{
    return (GPIOC->IDR >> SWITCH_S1_PIN) & 1U;
}

// Blocking delay using the SysTick millisecond counter
static void delay_ms_systick(uint32_t ms)
{
    uint32_t t0 = msTicks;
    while ((msTicks - t0) < ms) { }
}

// One full measurement: trigger, wait for ECHO, convert, display.
static void DoMeasurement(void)
{
    char line[48];
    char oled1[24];
    char oled2[24];

    echoDone = 0;
    HCSR04_Trigger();                                  // start the sensor

    // Wait for EXTI1 to finish timing the echo (with a timeout so a missing
    // object can never freeze the program)
    uint32_t t0 = msTicks;
    while (!echoDone && (msTicks - t0) < ECHO_TIMEOUT_MS) { }

    if (!echoDone) {
        UART3_SendString(" No echo (out of range)\r\n");
        OLED_Print(3, 10, "No echo             ");
        OLED_Print(4, 10, "                    ");
        return;
    }

    uint16_t pulse = pulseWidth;                       // copy the volatile once
    uint8_t inches = Switch_IsInches();

    // Floating-point math (FPU enabled in main): us -> distance
    float distance = inches ? ((float)pulse / US_PER_INCH)
                            : ((float)pulse / US_PER_CM);

    // Print as integers (whole.hundredths) so we do not need printf's float
    // support, which is disabled by default in newlib-nano.
    uint32_t hundredths = (uint32_t)(distance * 100.0f + 0.5f);
    uint32_t whole = hundredths / 100U;
    uint32_t frac  = hundredths % 100U;
    const char *units = inches ? "inch" : "cm";

    // Serial monitor (UART3, 115200 baud)
    snprintf(line, sizeof(line), " Pulse: %4u us,      Distance: %lu.%02lu %s\r\n",
             (unsigned)pulse, (unsigned long)whole, (unsigned long)frac, units);
    UART3_SendString(line);

    // OLED (padded with spaces so shorter text overwrites old text)
    snprintf(oled1, sizeof(oled1), "Pulse : %4u us    ", (unsigned)pulse);
    snprintf(oled2, sizeof(oled2), "Dist  : %lu.%02lu %-4s  ",
             (unsigned long)whole, (unsigned long)frac, inches ? "in" : "cm");
    OLED_Print(3, 10, oled1);
    OLED_Print(4, 10, oled2);

    // Seven-segment display: SSD_DisplayValue(1234) shows "12.34"
    SSD_DisplayValue((uint16_t)(hundredths > 9999U ? 9999U : hundredths));
}

// ---------------- main ----------------
int main(void)
{
    // Enable the FPU (coprocessors CP10 and CP11 full access) BEFORE any
    // float instruction, otherwise the CPU faults.
    SCB->CPACR |= (0xFU << 20);
    __DSB();
    __ISB();

    SystemClock_Config();          // 16 MHz HSI -> 180 MHz PLL
    SystemCoreClockUpdate();       // make SystemCoreClock read 180000000

    SSD_Init();
    // ssd.c computes TIM5's ARR from SystemCoreClock (180 MHz), but TIM5 is on
    // APB1 and really runs at 90 MHz, which would make each digit refresh
    // twice as slow (visible flicker). Reload ARR for the intended
    // 4 digits x 100 Hz = 400 interrupts/s.
    TIM5->ARR = (90000000U / (4U * 100U)) - 1U;

    OLED_Init();
    UART3_Init();
    HCSR04_Init();
    Switch_Init();

    // SysTick: 1 ms interrupt. 180,000 counts fit in the 24-bit reload value
    // (a direct 0.5 s would need 90,000,000 and would not). The handler counts
    // 500 of these ticks, then sets measureRequest.
    SysTick_Config(SystemCoreClock / 1000U);

    // Splash screen, then static text
    SH1106_RenderFullScreenLogo();
    delay_ms_systick(1500);
    OLED_Clear();
    OLED_Print(0, 0, "CPEG222 Project 6");
    OLED_Print(1, 0, "Ultrasonic Range");

    UART3_SendString("CPEG222 Project6 Ultrasonic Range Finder\r\n");

    while (1) {
        if (measureRequest) {      // set every 0.5 s by SysTick
            measureRequest = 0;
            DoMeasurement();
        }
    }
}