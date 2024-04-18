#include "Uart.hpp"

UART::UART(USART_TypeDef *const usart, uint32_t usartFreq)
    : usart(usart), usartFreq(usartFreq)
{
}

void UART::begin(uint32_t baudRate,
                 WordLen wordLen, StopBit stopBit, ParityControl parityControl,
                 bool swapRxTx, bool txLogicInverse, bool rxLogicInverse)
{
    usart->CR1 = 0;
    usart->CR2 = 0;
    usart->CR3 = 0;
    usart->GTPR = 0;

    usart->CR1 |=
        USART_CR1_TE |
        USART_CR1_RE |
        USART_CR1_TXEIE_TXFNFIE |
        USART_CR1_RXNEIE_RXFNEIE;

    if (swapRxTx)
        usart->CR2 |= USART_CR2_SWAP;
    if (txLogicInverse)
        usart->CR2 |= USART_CR2_TXINV;
    if (rxLogicInverse)
        usart->CR2 |= USART_CR2_RXINV;

    switch (wordLen)
    {
    case WordLen::seven:
        usart->CR1 |= USART_CR1_M1;
        break;
    case WordLen::nine:
        usart->CR1 |= USART_CR1_M0;
        break;
    case WordLen::eight:
        break;
    }

    switch (stopBit)
    {
    case StopBit::half:
        usart->CR2 |= USART_CR2_STOP_0;
        break;
    case StopBit::one:
        break;
    case StopBit::one_and_half:
        usart->CR2 |= USART_CR2_STOP_1 | USART_CR2_STOP_0;
        break;
    case StopBit::two:
        usart->CR2 |= USART_CR2_STOP_1;
        break;
    }

    if (parityControl != ParityControl::none)
    {
        usart->CR1 |= USART_CR1_PCE;
        if (parityControl == ParityControl::odd)
            usart->CR1 |= USART_CR1_PS;
    }

    usart->CR3 |= USART_CR3_OVRDIS;
    usart->RQR |= USART_RQR_RXFRQ;

    usart->BRR = usartFreq / baudRate;

    usart->CR1 |= USART_CR1_UE;

    usart_tx_rb.clear();
    usart_rx_rb.clear();
}

void UART::end()
{
    usart->CR1 &= ~USART_CR1_UE;
}

size_t UART::write(const uint8_t c)
{
    usart_tx_rb.push(c);
    usart->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return 1;
}

size_t UART::write(const uint8_t *buffer, size_t count)
{
    for (size_t c = count; c; c--, buffer++)
        usart_tx_rb.push(*buffer);
    usart->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return count;
}

int UART::availableForWrite()
{
    return usart_tx_rb.available();
}

int UART::available()
{
    return usart_rx_rb.size();
}

int UART::read()
{
    char buff;
    if (usart_rx_rb.pop(buff))
    {
        return buff;
    }
    return -1;
}

int UART::peek()
{
    char buff;
    if (usart_rx_rb.peek(buff))
        return buff;
    return -1;
}

void UART::attachOnReceiveIRQ(void (*handler)())
{
    onReceive = handler;
}

void UART::iqrHandler()
{
    parityErrorFlag = usart->ISR & USART_ISR_PE;
    if (usart->ISR & USART_ISR_RXNE_RXFNE)
    {
        volatile uint16_t c = usart->RDR;
        usart_rx_rb.push(char(c));
        if (onReceive != nullptr)
            onReceive();
    }

    if (usart->ISR & USART_ISR_TXE_TXFNF)
    {
        if (!usart_tx_rb.empty())
            usart->TDR = usart_tx_rb.pop();
        else
            usart->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    }
}
