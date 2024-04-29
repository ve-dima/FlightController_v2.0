#include "Uart.hpp"

UART::UART(USART_TypeDef *const usart, uint32_t usartFreq,
           DMA_TypeDef *const dma, DMA_Channel_TypeDef *const tx_Stream, DMA_Channel_TypeDef *const rx_Stream,
           const uint32_t txStreamChannel, const uint32_t rxStreamChannel)
    : usart(usart), usartFreq(usartFreq),
      dma(dma), tx_Stream(tx_Stream), rx_Stream(rx_Stream),
      txStreamChannel(txStreamChannel), rxStreamChannel(rxStreamChannel)
{
}

void UART::begin(uint32_t baudRate,
                 WordLen wordLen, StopBit stopBit, ParityControl parityControl,
                 bool swapRxTx, bool txLogicInverse, bool rxLogicInverse)
{
    lwrb_init(&usart_tx_rb, usart_tx_rb_data, sizeof(usart_tx_rb_data));
    lwrb_init(&usart_rx_rb, usart_rx_rb_data, sizeof(usart_rx_rb_data));

    rx_Stream->CPAR = (uint32_t)(((__IO uint8_t *)&usart->RDR));
    rx_Stream->CMAR = (uint32_t)&usart_rx_dma_buffer;
    rx_Stream->CNDTR = rxDmaBufferSize;
    rx_Stream->CCR = (0 << DMA_CCR_PL_Pos) |
                     DMA_CCR_MINC |
                     DMA_CCR_CIRC |
                     DMA_CCR_TCIE |
                     DMA_CCR_HTIE;

    tx_Stream->CPAR = (uint32_t)(((__IO uint8_t *)&usart->TDR));
    tx_Stream->CCR |= (0 << DMA_CCR_PL_Pos) |
                      (0b01 << DMA_CCR_DIR_Pos) |
                      DMA_CCR_MINC |
                      DMA_CCR_TCIE;

    usart->GTPR = 0;
    usart->CR1 =
        USART_CR1_TE |
        USART_CR1_RE |
        USART_CR1_IDLEIE;
    usart->CR2 = 0;
    usart->CR3 = USART_CR3_DMAR |
                 USART_CR3_DMAT;

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

    rx_Stream->CCR |= DMA_CCR_EN;
    usart->CR1 |= USART_CR1_UE;
}

void UART::end()
{
    usart->CR1 &= ~USART_CR1_UE;
}

size_t UART::write(const uint8_t c)
{
    size_t len = lwrb_write(&usart_tx_rb, &c, sizeof(c));
    if (autoSend)
        usart_start_tx_dma_transfer();
    return len;
}

size_t UART::write(const uint8_t *buffer, size_t count)
{
    size_t l = lwrb_write(&usart_tx_rb, buffer, count);
    if (autoSend)
        usart_start_tx_dma_transfer();
    return l;
}

int UART::availableForWrite()
{
    return lwrb_get_free(&usart_tx_rb);
}

int UART::available()
{
    return lwrb_get_full(&usart_rx_rb);
}

void UART::setAutoSend(bool as) { autoSend = as; }
void UART::startTX() { usart_start_tx_dma_transfer(); }

int UART::read()
{
    char buff;
    if (lwrb_read(&usart_rx_rb, &buff, 1))
        return buff;
    return -1;
}

int UART::peek()
{
    char buff;
    if (lwrb_peek(&usart_rx_rb, 0, &buff, 1))
        return buff;
    return -1;
}

void UART::usart_rx_check(void)
{
    size_t pos;
    pos = rxDmaBufferSize - rx_Stream->CNDTR;
    if (pos != old_pos)
    {
        if (pos > old_pos)
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        else
        {
            usart_process_data(&usart_rx_dma_buffer[old_pos], rxDmaBufferSize - old_pos);
            if (pos > 0)
                usart_process_data(&usart_rx_dma_buffer[0], pos);
        }
        old_pos = pos;
    }
}

void UART::usart_process_data(const void *data, size_t len)
{
    lwrb_write(&usart_rx_rb, data, len);
}

uint8_t UART::usart_start_tx_dma_transfer(void)
{
    uint8_t started = 0;
    if (usart_tx_dma_current_len == 0 && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0)
    {
        tx_Stream->CCR &= ~DMA_CCR_EN;
        while (tx_Stream->CCR & DMA_CCR_EN)
            ;

        dma->IFCR = 0b1111 << (txStreamChannel * 4);

        tx_Stream->CNDTR = usart_tx_dma_current_len;
        tx_Stream->CMAR = (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_rb);

        tx_Stream->CCR |= DMA_CCR_EN;
        started = 1;
    }
    return started;
}

void UART::uartIqrHandler()
{
    if (usart->ISR & USART_ISR_IDLE)
    {
        (void)usart->RDR;
        usart->ICR = USART_ICR_IDLECF;
        usart_rx_check();
    }
}

void UART::rxDmaIrqHandler()
{
    if (dma->ISR & (DMA_ISR_HTIF1 << (4 * rxStreamChannel)))
    {
        dma->IFCR = (DMA_IFCR_CHTIF1 << (4 * rxStreamChannel));
        usart_rx_check();
    }
    if (dma->ISR & (DMA_ISR_TCIF1 << (4 * rxStreamChannel)))
    {
        dma->IFCR = (DMA_IFCR_CTCIF1 << (4 * rxStreamChannel));
        usart_rx_check();
    }
}

void UART::txDmaIrqHandler()
{
    if (dma->ISR & (DMA_ISR_TCIF1 << (4 * txStreamChannel)))
    {
        dma->IFCR = (DMA_IFCR_CTCIF1 << (4 * txStreamChannel));
        lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len);
        usart_tx_dma_current_len = 0;
        usart_start_tx_dma_transfer();
    }
}
