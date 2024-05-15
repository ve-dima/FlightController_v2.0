#include <stm32g4xx.h>
#include "ICM-20948/ICM-20948.hpp"
#include "modes/Modes.hpp"
#include "control/Control.hpp"

namespace IMU
{
    void init()
    {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
        __DMB();

        TIM6->PSC = 1;
        TIM6->ARR = 35590; // 224,77 Hz
        TIM6->DIER |= TIM_DIER_UIE;
        TIM6->CR1 |= TIM_CR1_CEN;

        NVIC_DisableIRQ(TIM6_DAC_IRQn);
        NVIC_SetPriority(TIM6_DAC_IRQn, 2);
    }

    void enable()
    {

    }

    void handler()
    {

    }

    extern "C" void TIM6_DAC_IRQHandler(void)
    {
        TIM6->SR = ~TIM_SR_UIF;
        ICM20948::isr();
        FlightModeDispatcher::switchHandler();
        FlightModeDispatcher::attitudeTickHandler();
        Control::velocityHandler();
        Control::rateHandler();
        Control::updateMotorPower();
    }
};