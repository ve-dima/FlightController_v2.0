#include <stm32g4xx.h>
#include "ICM-20948/ICM-20948.hpp"
#include "BMP280/BMP280.hpp"
#include "modes/Modes.hpp"
#include "control/Control.hpp"
#include "SRT/SRT.hpp"
#include "rc/RC.hpp"
#include "Common.hpp"
#include <algorithm>

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
        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }

    void handler()
    {
    }

    extern "C" void TIM6_DAC_IRQHandler(void)
    {
        TIM6->SR = ~TIM_SR_UIF;

        static uint32_t maxClk = 0;
        uint32_t startTime = tick();
        ICM20948::handler();
        if (not ICM20948::magIsRead)
            BMP280::handler();
        else
            ICM20948::magIsRead = false;

        maxClk = std::max(maxClk, tick() - startTime);
        // 11340 - empty -Og
        // 10271 - empty -O3

        // 14583 - my lib -Og
        // . - my lib -O3

        // eigen -Og 17023
        // eigen -O3 12223
        RC::ahrsTickHandler();
        FlightModeDispatcher::switchHandler();
        FlightModeDispatcher::attitudeTickHandler();
        Control::positionControlHandler();
        Control::linearVelocityHandler();
        Control::rotateVelocityHandler();
        Control::rateHandler();
        Control::updateMotorPower();
    }

    REGISTER_SRT_MODULE(imu, init, enable, handler);
};