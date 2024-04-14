/**
 * @file I2C.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Драйвер I2C-шины
 */
#pragma once
#include <stm32g4xx.h>

static constexpr uint32_t I2C_TIMEOUT = 2;

class I2C
{
protected:
    I2C_TypeDef *const i2c;

public:
    explicit I2C(I2C_TypeDef *i2c) : i2c(i2c){};

    void begin();
    void end();

    void startTx(uint8_t address, uint8_t cnt, bool end = true);
    void startRx(uint8_t address, uint8_t cnt, bool end = true);

    void write(uint8_t);
    uint8_t read();
    void stop();

    void waitTransferComplete();
    void waitStop();
    void waitReloadTransfer();

    enum class State
    {
        OK,
        ERROR,
    } state;
};

#ifdef I2C1
extern I2C i2c1;
#endif

#ifdef I2C2
extern I2C i2c2;
#endif

#ifdef I2C3
extern I2C i2c3;
#endif

#ifdef I2C4
extern I2C i2c4;
#endif