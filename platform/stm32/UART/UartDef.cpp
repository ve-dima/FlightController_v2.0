#include "Uart.hpp"

#ifdef USART1
UART uart1(USART1, F_CPU);
extern "C" void USART1_IRQHandler(void) { uart1.iqrHandler(); }
#endif

#ifdef USART2
UART uart2(USART2, F_CPU);
extern "C" void USART2_IRQHandler(void) { uart2.iqrHandler(); }
#endif

#ifdef USART3
UART uart3(USART3, F_CPU);
extern "C" void USART3_IRQHandler(void) { uart3.iqrHandler(); }
#endif

#ifdef USART4
UART uart4(USART4, F_CPU);
extern "C" void USART4_IRQHandler(void) { uart4.iqrHandler(); }
#endif

#ifdef UART1
UART uart1(UART1, F_CPU);
extern "C" void UART1_IRQHandler(void) { uart1.iqrHandler(); }
#endif

#ifdef UART2
UART uart2(UART4, F_CPU);
extern "C" void UART2_IRQHandler(void) { uart2.iqrHandler(); }
#endif

#ifdef UART3
UART uart4(UART3, F_CPU);
extern "C" void UART3_IRQHandler(void) { uart3.iqrHandler(); }
#endif

#ifdef UART4
UART uart4(UART4, F_CPU);
extern "C" void UART4_IRQHandler(void) { uart4.iqrHandler(); }
#endif

#ifdef UART5
UART uart5(UART5, F_CPU);
extern "C" void UART5_IRQHandler(void) { uart5.iqrHandler(); }
#endif

#ifdef UART6
UART uart6(UART6, F_CPU);
extern "C" void UART6_IRQHandler(void) { uart6.iqrHandler(); }
#endif

#ifdef UART7
UART uart7(UART7, F_CPU);
extern "C" void UART7_IRQHandler(void) { uart7.iqrHandler(); }
#endif

#ifdef UART8
UART uart8(UART8, F_CPU);
extern "C" void UART8_IRQHandler(void) { uart8.iqrHandler(); }
#endif

#ifdef LPUART1
UART lpuart1(LPUART1, F_CPU);
extern "C" void LPUART1_IRQHandler(void) { lpuart1.iqrHandler(); }
#endif