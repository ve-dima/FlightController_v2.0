#pragma once
#include <stm32g4xx.h>
#include "ArduinoAPI/Stream.h"
#include "lwrb/lwrb.h"

static constexpr uint16_t rxBufferSize = 128;
static constexpr uint16_t rxDmaBufferSize = 64;
static constexpr uint16_t txBufferSize = 256;

class UART : public Stream
{
private:
    USART_TypeDef *const usart;
    const uint32_t usartFreq;
    DMA_TypeDef *const dma;
    DMA_Channel_TypeDef *const tx_Stream, *const rx_Stream;
    const uint32_t txStreamChannel, rxStreamChannel;

    bool parityErrorFlag = false;

    uint8_t usart_rx_rb_data[rxBufferSize];
    uint8_t usart_rx_dma_buffer[rxDmaBufferSize];
    lwrb_t usart_rx_rb;
    size_t old_pos = 0;

    lwrb_t usart_tx_rb;
    uint8_t usart_tx_rb_data[txBufferSize];
    volatile size_t usart_tx_dma_current_len;

    void usart_rx_check(void);
    void usart_process_data(const void *data, size_t len);
    uint8_t usart_start_tx_dma_transfer(void);

    bool autoSend = true;

    void (*idleCallback)(void) = nullptr;
    void (*halfFullCallback)(void) = nullptr;

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

    UART(USART_TypeDef *const usart, uint32_t usartFreq,
         DMA_TypeDef *const dma, DMA_Channel_TypeDef *const tx_Stream, DMA_Channel_TypeDef *const rx_Stream,
         const uint32_t txStreamChannel, const uint32_t rxStreamChannel);

    void begin(uint32_t baudRate = 115'200,
               WordLen wordLen = WordLen::eight, StopBit stopBit = StopBit::one, ParityControl parityControl = ParityControl::none,
               bool swapRxTx = false,
               bool txLogicInverse = false,
               bool rxLogicInverse = false);

    void end();

    size_t write(const uint8_t *, size_t) override final;
    size_t write(uint8_t) override final;
    int availableForWrite() override final;
    void setAutoSend(bool autoSend);
    void startTX(void);

    int available() override final;
    int read() override final;
    int peek() override final;

    void uartIqrHandler();
    void rxDmaIrqHandler();
    void txDmaIrqHandler();

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
#ifdef LPUART1
extern UART lpuart1;
#endif