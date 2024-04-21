#include "Common.hpp"
#include <stm32g4xx.h>
#include "UART/Uart.hpp"

void setup()
{
    SystemInit();
    SystemCoreClockUpdate();
    SysTick_Config(F_CPU / 1'000);

    //========================================================================
    // Led init

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODE15);
    GPIOA->MODER |= GPIO_MODER_MODE15_0;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED15_0;

    //========================================================================
    // I2C1 100kHz

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    GPIOB->MODER &= ~(
        GPIO_MODER_MODE8 |
        GPIO_MODER_MODE9);

    GPIOB->MODER |= ((0b10 << GPIO_MODER_MODE8_Pos) |
                     (0b10 << GPIO_MODER_MODE9_Pos));
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8 |
                       GPIO_OSPEEDR_OSPEED9);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 |
                      GPIO_OTYPER_OT9);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 |
                     GPIO_PUPDR_PUPD9_0);
    GPIOB->AFR[1] |= ((4 << GPIO_AFRH_AFSEL8_Pos) |
                      (4 << GPIO_AFRH_AFSEL9_Pos));
                      
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C1_FMP;
    I2C1->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos) |
                     (0x9 << I2C_TIMINGR_SCLL_Pos) |
                     (0x3 << I2C_TIMINGR_SCLH_Pos) |
                     (0x2 << I2C_TIMINGR_SDADEL_Pos) |
                     (0x3 << I2C_TIMINGR_SCLDEL_Pos);

    //========================================================================

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    GPIOB->MODER &= ~(
        GPIO_MODER_MODE10 |
        GPIO_MODER_MODE11);

    GPIOB->MODER |= ((0b10 << GPIO_MODER_MODE10_Pos) |
                     (0b10 << GPIO_MODER_MODE11_Pos));
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 |
                       GPIO_OSPEEDR_OSPEED11);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_1 |
                     GPIO_PUPDR_PUPD11_1);
    GPIOB->AFR[1] |= ((7 << GPIO_AFRH_AFSEL10_Pos) |
                      (7 << GPIO_AFRH_AFSEL11_Pos));

    NVIC_SetPriority(USART3_IRQn, 0);
    NVIC_EnableIRQ(USART3_IRQn);
    uart3.begin(921'600);

    __enable_irq();
}

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, uint16_t length)
{
    if (chan == MAVLINK_COMM_0)
        uart3.write(ch, length);
}