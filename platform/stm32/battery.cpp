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

        ADC12_COMMON->CCR = (0b1111 << ADC_CCR_PRESC_Pos) | ADC_CCR_CKMODE_0;
        ADC12_COMMON->CCR |= ADC_CCR_VREFEN;

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

        ADC1->SMPR2 = (0b111 << ADC_SMPR2_SMP12_Pos) | (0b111 << ADC_SMPR2_SMP15_Pos) | (0b111 << ADC_SMPR2_SMP18_Pos);

        // Voltage, Current, VrefInt,
        ADC1->SQR1 = (15 << ADC_SQR1_SQ1_Pos) | (12 << ADC_SQR1_SQ2_Pos) | (18 << ADC_SQR1_SQ3_Pos) | (2 << ADC_SQR1_L_Pos);
        ADC1->CFGR |= ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
        ADC1->IER = ADC_IER_EOCIE;

        NVIC_SetPriority(ADC1_2_IRQn, 1);
        NVIC_EnableIRQ(ADC1_2_IRQn);
    }

    void enable()
    {
        ADC1->CR |= ADC_CR_ADSTART;
    }

    enum class WhatScan
    {
        undefined,
        voltage,
        current,
        reference,
    };
    WhatScan whatScan = WhatScan::undefined;

#define VREFINT_CAL_ADDR ((uint16_t *)(0x1FFF75AAUL))
    uint16_t VREFINT_DATA = 1200;

    extern "C" void ADC1_2_IRQHandler(void)
    {
        if (ADC1->ISR & ADC_ISR_EOS)
        {
            ADC1->ISR = ADC_ISR_EOS;
            whatScan = WhatScan::reference;
        }

        if (ADC1->ISR & ADC_ISR_OVR)
        {
            whatScan = WhatScan::undefined;
            ADC1->ISR = ADC_ISR_OVR;
        }

        const uint16_t adcData = ADC1->DR;
        ADC1->ISR = ADC_ISR_EOC;
        const float voltage = 3.0 * float(uint32_t(*VREFINT_CAL_ADDR) * uint32_t(adcData)) / float(uint32_t(VREFINT_DATA) * uint32_t((1 << 12) - 1));

        switch (whatScan)
        {
        case WhatScan::voltage:
            Battery::updateVoltage(voltage);
            whatScan = WhatScan::current;
            break;
        case WhatScan::current:
            whatScan = WhatScan::reference;
            break;
        case WhatScan::reference:
            VREFINT_DATA = adcData;
            whatScan = WhatScan::voltage;
            break;
        default:
            break;
        }
    }

    void handler() {}

    REGISTER_SRT_MODULE(batteryDriver, init, enable, handler);
}