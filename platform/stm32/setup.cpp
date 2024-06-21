#include "Common.hpp"
#include "Board.hpp"
#include <stm32g4xx.h>
#include "UART/Uart.hpp"
#include "rc/RC.hpp"

void setup()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    PWR->CR5 &= ~PWR_CR5_R1MODE;
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM |
                      RCC_PLLCFGR_PLLN |
                      RCC_PLLCFGR_PLLP |
                      RCC_PLLCFGR_PLLQ |
                      RCC_PLLCFGR_PLLR |
                      RCC_PLLCFGR_PLLSRC);

    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLM_Pos) |
                    (40 << RCC_PLLCFGR_PLLN_Pos) |
                    (2 << RCC_PLLCFGR_PLLPDIV_Pos) |
                    (0 << RCC_PLLCFGR_PLLQ_Pos) |
                    (0 << RCC_PLLCFGR_PLLR_Pos) |
                    RCC_PLLCFGR_PLLSRC_HSE |
                    RCC_PLLCFGR_PLLPEN |
                    RCC_PLLCFGR_PLLQEN |
                    RCC_PLLCFGR_PLLREN;

    RCC->CR |= RCC_CR_HSEON;
    while (not(RCC->CR & RCC_CR_HSERDY))
        ;

    RCC->CR |= RCC_CR_PLLON;
    while (not(RCC->CR & RCC_CR_PLLRDY))
        ;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    SystemInit();
    SystemCoreClockUpdate();
    SysTick_Config(F_CPU / 1'000);
    NVIC_SetPriority(SysTick_IRQn, 0);

    //========================================================================

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
    I2C1->TIMINGR |= 0x00F08BFF;

    //========================================================================

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __DMB();
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    __DMB();
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    __DMB();

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    GPIOB->MODER &= ~(
        GPIO_MODER_MODE10 |
        GPIO_MODER_MODE11);

    GPIOB->MODER |= ((0b10 << GPIO_MODER_MODE10_Pos) |
                     (0b10 << GPIO_MODER_MODE11_Pos));
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 |
                       GPIO_OSPEEDR_OSPEED11);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD10_0 |
                     GPIO_PUPDR_PUPD11_0);
    GPIOB->AFR[1] |= ((7 << GPIO_AFRH_AFSEL10_Pos) |
                      (7 << GPIO_AFRH_AFSEL11_Pos));

    DMAMUX1_Channel4->CCR = 29 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    DMAMUX1_Channel5->CCR = 28 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    NVIC_SetPriority(DMA1_Channel5_IRQn, 1);
    NVIC_SetPriority(DMA1_Channel6_IRQn, 1);
    NVIC_SetPriority(USART3_IRQn, 1);
    NVIC_EnableIRQ(USART3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    //========================================================================

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    GPIOB->MODER &= ~(
        GPIO_MODER_MODE3 |
        GPIO_MODER_MODE4);

    GPIOB->MODER |= ((0b10 << GPIO_MODER_MODE3_Pos) |
                     (0b10 << GPIO_MODER_MODE4_Pos));
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3 |
                       GPIO_OSPEEDR_OSPEED4);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD3_0 |
                     GPIO_PUPDR_PUPD4_0);
    GPIOB->AFR[0] |= ((7 << GPIO_AFRL_AFSEL3_Pos) |
                      (7 << GPIO_AFRL_AFSEL4_Pos));

    DMAMUX1_Channel2->CCR = 27 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    DMAMUX1_Channel3->CCR = 26 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    NVIC_SetPriority(DMA1_Channel3_IRQn, 1);
    NVIC_SetPriority(DMA1_Channel4_IRQn, 1);
    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    //========================================================================

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOB->MODER &= ~(
        GPIO_MODER_MODE6 |
        GPIO_MODER_MODE7);

    GPIOB->MODER |= ((0b10 << GPIO_MODER_MODE6_Pos) |
                     (0b10 << GPIO_MODER_MODE7_Pos));
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 |
                       GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 |
                     GPIO_PUPDR_PUPD7_0);
    GPIOB->AFR[0] |= ((7 << GPIO_AFRL_AFSEL6_Pos) |
                      (7 << GPIO_AFRL_AFSEL7_Pos));

    DMAMUX1_Channel0->CCR = 25 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    DMAMUX1_Channel1->CCR = 24 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 1);
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    //========================================================================

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

    GPIOA->MODER &= ~(
        GPIO_MODER_MODE2 |
        GPIO_MODER_MODE3);

    GPIOA->MODER |= ((0b10 << GPIO_MODER_MODE2_Pos) |
                     (0b10 << GPIO_MODER_MODE3_Pos));
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 |
                       GPIO_OSPEEDR_OSPEED3);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD2_0 |
                     GPIO_PUPDR_PUPD3_0);
    GPIOA->AFR[0] |= ((12 << GPIO_AFRL_AFSEL2_Pos) |
                      (12 << GPIO_AFRL_AFSEL3_Pos));

    DMAMUX1_Channel6->CCR = 35 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    DMAMUX1_Channel7->CCR = 34 << DMAMUX_CxCR_DMAREQ_ID_Pos;
    NVIC_SetPriority(DMA2_Channel1_IRQn, 1);
    NVIC_SetPriority(DMA2_Channel2_IRQn, 1);
    NVIC_SetPriority(LPUART1_IRQn, 1);
    NVIC_EnableIRQ(LPUART1_IRQn);
    NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    NVIC_EnableIRQ(DMA2_Channel2_IRQn);

    __enable_irq();
    
    mav0Uart.setAutoSend(false);
    mav0Uart.begin(115'200);
    mav1Uart.setAutoSend(false);
    mav1Uart.begin(115'200);
    
    EXTI->IMR1 |= EXTI_IMR1_IM0;
    EXTI->RTSR1 |= EXTI_RTSR1_RT0;
    rcUart.idleCallback = []()
    {
        EXTI->SWIER1 = EXTI_SWIER1_SWI0;
    };
    NVIC_SetPriority(EXTI0_IRQn, 3);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, uint16_t length)
{
    if (chan == MAVLINK_COMM_0)
        mav0Uart.write(ch, length);
    else if (chan == MAVLINK_COMM_1)
        mav1Uart.write(ch, length);
}

void mavlink_end_uart_send(mavlink_channel_t chan, int length)
{
    if (chan == MAVLINK_COMM_0)
        mav0Uart.startTX();
    else if (chan == MAVLINK_COMM_1)
        mav1Uart.startTX();
}

extern "C" void EXTI0_IRQHandler(void)
{
    RC::callBackHandler();
    EXTI->PR1 = EXTI_PR1_PIF0;
}