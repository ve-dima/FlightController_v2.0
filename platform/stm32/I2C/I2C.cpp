#include "I2C.hpp"
#include "Common.hpp"

void I2C::begin()
{
    i2c->CR1 = 0;
    while (i2c->CR1 & I2C_CR1_PE)
        ;
    i2c->CR2 = 0;
    i2c->ICR = ~0;

    i2c->CR1 |= I2C_CR1_PE;

    state = State::OK;
}

void I2C::end()
{
    i2c->CR1 &= ~I2C_CR1_PE;
}

void I2C::startTx(uint8_t addr, uint8_t cnt, bool end)
{
    i2c->CR2 = 0;
    i2c->CR2 = ((addr & 0b0111'1111) << (I2C_CR2_SADD_Pos + 1)) |
               (cnt << I2C_CR2_NBYTES_Pos) |
               (end ? I2C_CR2_AUTOEND : 0);

    __DMB();
    i2c->CR2 |= I2C_CR2_START;

    for (uint32_t timeOutTimer = millis();
         (millis() - timeOutTimer) < I2C_TIMEOUT and
         ((i2c->ISR & I2C_ISR_NACKF) == 0) and
         ((i2c->ISR & I2C_ISR_TXIS) == 0) and
         ((i2c->ISR & I2C_ISR_STOPF) == 0 and end == true);)
        ;

    if (i2c->ISR & I2C_ISR_NACKF)
    {
        state = State::ERROR;
        return;
    }
}

void I2C::startRx(uint8_t addr, uint8_t cnt, bool end)
{
    i2c->CR2 = 0;
    i2c->CR2 = ((addr & 0b0111'1111) << (I2C_CR2_SADD_Pos + 1)) |
               I2C_CR2_RD_WRN |
               (cnt << I2C_CR2_NBYTES_Pos) |
               (end ? I2C_CR2_AUTOEND : 0);

    __DMB();
    i2c->CR2 |= I2C_CR2_START;

    for (uint32_t timeOutTimer = millis();
         (millis() - timeOutTimer) < I2C_TIMEOUT and
         ((i2c->ISR & I2C_ISR_NACKF) == 0) and
         ((i2c->ISR & I2C_ISR_TXIS) == 0) and
         ((i2c->ISR & I2C_ISR_STOPF) == 0 and end == true);)
        ;

    if (i2c->ISR & I2C_ISR_NACKF)
    {
        state = State::ERROR;
        return;
    }
}

void I2C::write(uint8_t data)
{
    for (uint32_t timeOutTimer = millis();
         (millis() - timeOutTimer) < I2C_TIMEOUT and
         ((i2c->ISR & I2C_ISR_TXIS) == 0) and
         ((i2c->ISR & I2C_ISR_NACKF) == 0);)
        ;

    if (i2c->ISR & I2C_ISR_NACKF or ((i2c->ISR & I2C_ISR_TXIS) == 0))
    {
        state = State::ERROR;
        return;
    }
    *((__IO uint8_t *)&i2c->TXDR) = data;
}

uint8_t I2C::read()
{
    for (uint32_t timeOutTimer = millis(); ((i2c->ISR & I2C_ISR_RXNE) == 0) && (millis() - timeOutTimer) < I2C_TIMEOUT;)
        ;
    if ((i2c->ISR & I2C_ISR_RXNE) == 0)
    {
        state = State::ERROR;
        return uint8_t();
    }
    return i2c->RXDR;
}

void I2C::stop()
{
    i2c->CR2 |= I2C_CR2_STOP;
    // while (!(i2c->ISR & I2C_ISR_STOPF))
    //     ;
}

void I2C::waitTransferComplete()
{
    for (uint32_t timeOutTimer = millis(); (!(i2c->ISR & I2C_ISR_TC)) && (millis() - timeOutTimer) < I2C_TIMEOUT;)
        ;
    if (!(i2c->ISR & I2C_ISR_TC))
    {
        state = State::ERROR;
        return;
    }
}

void I2C::waitStop()
{
    for (uint32_t timeOutTimer = millis(); (!(i2c->ISR & I2C_ISR_STOPF)) && (millis() - timeOutTimer) < I2C_TIMEOUT;)
        ;
    if (!(i2c->ISR & I2C_ISR_STOPF))
    {
        state = State::ERROR;
        return;
    }
}

void I2C::waitReloadTransfer()
{
    for (uint32_t timeOutTimer = millis(); (!(i2c->ISR & I2C_ISR_TCR)) && (millis() - timeOutTimer) < I2C_TIMEOUT;)
        ;
    if (!(i2c->ISR & I2C_ISR_TCR))
    {
        state = State::ERROR;
        return;
    }
}

#ifdef I2C1
I2C i2c1(I2C1);
#endif

#ifdef I2C2
I2C i2c2(I2C2);
#endif

#ifdef I2C3
I2C i2c3(I2C3);
#endif

#ifdef I2C4
I2C i2c4(I2C4);
#endif