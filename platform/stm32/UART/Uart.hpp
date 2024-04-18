/**
 * @file Uart.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Драйвер U(S)ART-блоков H7 камушка
 */
#pragma once
#include <stm32g4xx.h>
#include "ArduinoAPI/Stream.h"
#include "Algo/RingBuff.hpp"

static constexpr uint16_t rxBufferSize = 512;
static constexpr uint16_t txBufferSize = 512;

class UART : public Stream
{
private:
    USART_TypeDef *const usart;
    const uint32_t usartFreq;

    RingBuffer<txBufferSize, char, uint16_t> usart_tx_rb;
    RingBuffer<rxBufferSize, char, uint16_t> usart_rx_rb;

    void (*onReceive)() = nullptr;
    bool parityErrorFlag = false;

public:
    enum class ParityControl
    {
        none,
        even,
        odd
    };
    enum class StopBit
    {
        half,
        one,
        one_and_half,
        two
    };
    enum class WordLen
    {
        seven,
        eight,
        nine, // not support
    };

    UART(USART_TypeDef *const usart, uint32_t usartFreq);

    void begin(uint32_t baudRate = 115'200,
               WordLen wordLen = WordLen::eight, StopBit stopBit = StopBit::one, ParityControl parityControl = ParityControl::none,
               bool swapRxTx = false,
               bool txLogicInverse = false,
               bool rxLogicInverse = false);

    void end();

    size_t write(uint8_t) override final;
    size_t write(const uint8_t *, size_t) override final;

    int availableForWrite() override final;

    int available() override final;

    int read() override final;

    int peek() override final;

    void attachOnReceiveIRQ(void (*handler)());

    void iqrHandler();

    bool getParityErrorFlag() { return parityErrorFlag; }
    void clearParityErrorFlag() { parityErrorFlag = 0; }
};

#ifdef USART1
extern UART uart1;
#endif
#ifdef USART2
extern UART uart2;
#endif
#ifdef USART3
extern UART uart3;
#endif
#ifdef UART4
extern UART uart4;
#endif
#ifdef UART5
extern UART uart5;
#endif
#ifdef UART6
extern UART uart6;
#endif
#ifdef UART7
extern UART uart7;
#endif
#ifdef UART8
extern UART uart8;
#endif