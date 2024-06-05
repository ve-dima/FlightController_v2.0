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

        TIM6->PSC = 19;
        TIM6->ARR = 35590; // 224,77 Hz
        TIM6->DIER |= TIM_DIER_UIE;
        TIM6->CR1 |= TIM_CR1_CEN;

        NVIC_DisableIRQ(TIM6_DAC_IRQn);
        NVIC_SetPriority(TIM6_DAC_IRQn, 2);

        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
        GPIOC->MODER &= ~(GPIO_MODER_MODE13);
        GPIOC->MODER |= GPIO_MODER_MODE13_0;
        GPIOC->BSRR = GPIO_BSRR_BS13;
    }

    void enable()
    {
        GPIOC->BSRR = GPIO_BSRR_BS13;
        delay(300);
        GPIOC->BSRR = GPIO_BSRR_BR13;

        NVIC_EnableIRQ(TIM6_DAC_IRQn);
    }

    void handler()
    {
    }

    extern "C" void TIM6_DAC_IRQHandler(void)
    {
        TIM6->SR = ~TIM_SR_UIF;

        ICM20948::handler();
        if (not ICM20948::magIsRead)
            BMP280::handler();
        else
            ICM20948::magIsRead = false;

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