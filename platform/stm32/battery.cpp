#include "battery/battery.hpp"
#include "SRT/SRT.hpp"
#include "Common.hpp"
#include <stm32g4xx.h>

namespace Battery
{
    constexpr float adcScale = 3.3 / ((1 << 12) - 1);

    void init()
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_GPIOBEN;

        __DMB();

        ADC1->CR &= ~ADC_CR_DEEPPWD;
        ADC12_COMMON->CCR = (0b1001 << ADC_CCR_PRESC_Pos) | ADC_CCR_CKMODE_0;

        ADC1->CR = ADC_CR_ADVREGEN;
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();

        ADC1->CR |= ADC_CR_ADCAL;
        while (ADC1->CR & ADC_CR_ADCAL)
            ;

        ADC1->CR |= ADC_CR_ADEN;
        while (!(ADC1->ISR & ADC_ISR_ADRDY))
            ;

        ADC1->ISR = ADC_ISR_ADRDY;

        ADC1->SMPR2 = (0b111 << ADC_SMPR2_SMP15_Pos);
        ADC1->SQR1 = (15 << ADC_SQR1_SQ1_Pos) | (1 << ADC_SQR1_L_Pos);
        ADC1->CFGR |= ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
        ADC1->CR |= ADC_CR_ADSTART;
    }

    void enable()
    {
    }

    void handler()
    {
        if (ADC1->ISR & ADC_ISR_EOSMP)
        {
            ADC1->ISR = ADC_ISR_EOSMP;
            uint16_t adcValue = ADC1->DR;
            Battery::updateByADC(adcValue * adcScale);
        }
    }

    REGISTER_SRT_MODULE(batteryDriver, init, enable, handler);
}