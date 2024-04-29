#include "Uart.hpp"

#ifdef USART1
UART uart1(USART1, F_CPU, DMA1, DMA1_Channel1, DMA1_Channel2, 0, 1);
extern "C" void USART1_IRQHandler(void) { uart1.uartIqrHandler(); }
extern "C" void DMA1_Channel1_IRQHandler(void) { uart1.txDmaIrqHandler(); }
extern "C" void DMA1_Channel2_IRQHandler(void) { uart1.rxDmaIrqHandler(); }
#endif

#ifdef USART2
UART uart2(USART2, F_CPU, DMA1, DMA1_Channel3, DMA1_Channel4, 2, 3);
extern "C" void USART2_IRQHandler(void) { uart2.uartIqrHandler(); }
extern "C" void DMA1_Channel3_IRQHandler(void) { uart2.txDmaIrqHandler(); }
extern "C" void DMA1_Channel4_IRQHandler(void) { uart2.rxDmaIrqHandler(); }
#endif

#ifdef USART3
UART uart3(USART3, F_CPU, DMA1, DMA1_Channel5, DMA1_Channel6, 4, 5);
extern "C" void USART3_IRQHandler(void) { uart3.uartIqrHandler(); }
extern "C" void DMA1_Channel5_IRQHandler(void) { uart3.txDmaIrqHandler(); }
extern "C" void DMA1_Channel6_IRQHandler(void) { uart3.rxDmaIrqHandler(); }
#endif

#ifdef LPUART1
UART lpuart1(LPUART1, F_CPU, DMA2, DMA2_Channel1, DMA2_Channel2, 0, 1);
extern "C" void LPUART1_IRQHandler(void) { lpuart1.uartIqrHandler(); }
extern "C" void DMA2_Channel1_IRQHandler(void) { lpuart1.txDmaIrqHandler(); }
extern "C" void DMA2_Channel2_IRQHandler(void) { lpuart1.rxDmaIrqHandler(); }
#endif