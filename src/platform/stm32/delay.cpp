#include "Common.hpp"
#include <stm32g4xx.h>

volatile uint32_t millisCounter;
extern "C" void SysTick_Handler()
{
    ++millisCounter;
}

uint32_t millis() { return millisCounter; }

uint32_t tick() { return SysTick->VAL; }

void delay(uint32_t ms)
{
    for (uint32_t startT = millis(); (millis() - startT) < ms;)
    {
        yield();
        wdtFeed();
    }
}