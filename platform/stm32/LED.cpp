#include "indicators/LED.hpp"
#include "SRT/SRT.hpp"
#include <stm32g4xx.h>

namespace LED
{
    void init()
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

        GPIOA->MODER &= ~(GPIO_MODER_MODE15);
        GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED15_0;
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD15;
    }

    void enable()
    {
        LED::setLED(LED::Color::red, LED::Action::double_short_blink);
    }

    void setState(bool state)
    {
        if (state == false or color == Color::off)
            GPIOA->MODER &= ~(GPIO_MODER_MODE15);
        else
        {
            GPIOA->MODER |= GPIO_MODER_MODE15_0;
            if (color == Color::green)
                GPIOA->BSRR = GPIO_BSRR_BR15;
            else
                GPIOA->BSRR = GPIO_BSRR_BS15;
        }
    }

    REGISTER_SRT_MODULE(ledDriver, init, enable, handler);
}