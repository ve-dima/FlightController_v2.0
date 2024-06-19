#include "motor/motor.hpp"
#include "SRT/SRT.hpp"
#include <algorithm>
#include <stm32g4xx.h>

namespace Motor
{
    static constexpr volatile uint32_t *assignTable[] = {
        &TIM2->CCR2,
        &TIM1->CCR4,
        &TIM1->CCR3,
        &TIM2->CCR1,
        &TIM16->CCR1,
        &TIM17->CCR1,
    };

    void init()
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM17EN | RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM1EN;

        GPIOA->MODER &= ~(
            GPIO_MODER_MODE0 |
            GPIO_MODER_MODE1 |
            GPIO_MODER_MODE10 |
            GPIO_MODER_MODE11 |
            GPIO_MODER_MODER12);
        GPIOB->MODER &= ~GPIO_MODER_MODE5;

        GPIOA->MODER |= ((0b10 << GPIO_MODER_MODE0_Pos) |
                         (0b10 << GPIO_MODER_MODE1_Pos) |
                         (0b10 << GPIO_MODER_MODE10_Pos) |
                         (0b10 << GPIO_MODER_MODE11_Pos) |
                         (0b10 << GPIO_MODER_MODE12_Pos));
        GPIOB->MODER |= 0b10 << GPIO_MODER_MODE5_Pos;

        GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0 |
                           GPIO_OSPEEDR_OSPEED1 |
                           GPIO_OSPEEDR_OSPEED10 |
                           GPIO_OSPEEDR_OSPEED11 |
                           GPIO_OSPEEDR_OSPEED12);
        GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED5;

        GPIOA->AFR[0] |= ((1 << GPIO_AFRL_AFSEL0_Pos) |
                          (1 << GPIO_AFRL_AFSEL1_Pos));
        GPIOA->AFR[1] |= ((6 << GPIO_AFRH_AFSEL10_Pos) |
                          (11 << GPIO_AFRH_AFSEL11_Pos) |
                          (1 << GPIO_AFRH_AFSEL12_Pos));
        GPIOB->AFR[0] |= 10 << GPIO_AFRL_AFSEL5_Pos;

        static constexpr uint32_t pwmFreq = 100;
        TIM1->PSC = TIM2->PSC = TIM16->PSC = TIM17->PSC = F_CPU / 1'000'000 - 1;
        TIM1->ARR = TIM2->ARR = TIM16->ARR = TIM17->ARR = 1'000'000 / pwmFreq - 1;
        static constexpr uint32_t pwmOutputChannelSettings = (TIM_CCMR1_OC1PE |
                                                              0b110 << TIM_CCMR1_OC1M_Pos);
        TIM1->CCMR2 |= pwmOutputChannelSettings;
        TIM1->CCMR2 |= pwmOutputChannelSettings << 8;
        TIM2->CCMR1 |= pwmOutputChannelSettings;
        TIM2->CCMR1 |= pwmOutputChannelSettings << 8;
        TIM16->CCMR1 |= pwmOutputChannelSettings;
        TIM17->CCMR1 |= pwmOutputChannelSettings;

        TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
        TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
        TIM16->CCER |= TIM_CCER_CC1E;
        TIM17->CCER |= TIM_CCER_CC1E;

        for (auto c : assignTable)
            *c = 1'000;

        TIM1->BDTR |= TIM_BDTR_MOE;
        TIM16->BDTR |= TIM_BDTR_MOE;
        TIM17->BDTR |= TIM_BDTR_MOE;
        TIM1->CR1 = TIM2->CR1 = TIM16->CR1 = TIM17->CR1 = TIM_CR1_CEN;
    }

    extern float power[8];
    void updateOutput(unsigned motor)
    {
        if (power[motor] > 0 and
            motor < (sizeof(assignTable) / sizeof(assignTable[0])))
            *assignTable[motor] = 1'000 + std::clamp<uint32_t>(power[motor] * 1'000, 0, 1'000);
        else
            *assignTable[motor] = 900;
    }

    void enable() { stateHandler(); }

    void handler() { stateHandler(); }

    REGISTER_SRT_MODULE(pwmDriver, init, enable, handler);
}