#include "Common.hpp"
#include <stm32g4xx.h>

void setup()
{
    SystemInit();
    SystemCoreClockUpdate();
    SysTick_Config(F_CPU / 1'000);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE15);
    GPIOA->MODER |= GPIO_MODER_MODE15_0;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED15_0;

    GPIOB->MODER &= ~(
        GPIO_MODER_MODE8 |
        GPIO_MODER_MODE9);

    //===============

    GPIOB->MODER |= ((0b10 << GPIO_MODER_MODE8_Pos) |
                     (0b10 << GPIO_MODER_MODE9_Pos));
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8 |
                       GPIO_OSPEEDR_OSPEED9);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 |
                      GPIO_OTYPER_OT9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_1 |
                     GPIO_PUPDR_PUPD9_1);
    GPIOB->AFR[1] |= ((4 << GPIO_AFRH_AFSEL8_Pos) |
                      (4 << GPIO_AFRH_AFSEL9_Pos));

    I2C1->TIMINGR |= (0x3 << I2C_TIMINGR_PRESC_Pos) |
                     (0x13 << I2C_TIMINGR_SCLL_Pos) |
                     (0xF << I2C_TIMINGR_SCLH_Pos) |
                     (0x2 << I2C_TIMINGR_SDADEL_Pos) |
                     (0x4 << I2C_TIMINGR_SCLDEL_Pos);
}