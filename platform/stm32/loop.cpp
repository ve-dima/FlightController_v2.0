#include "Common.hpp"
#include "I2C/I2C.hpp"

void loop()
{
    for (uint8_t i = 0; i < 128; i++)
    {
        i2c1.begin();
        i2c1.startTx(i, 0);
        // i2c1.waitStop();
        if (i2c1.state != I2C::State::ERROR)
            __BKPT(0);
        // i2c1.stop();
        i2c1.end();
        // delay(5);
    }
    GPIOA->ODR ^= GPIO_ODR_ODR_15;
    delay(1'000);
}