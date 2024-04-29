#pragma once
#include "UART/Uart.hpp"
#include "I2C/I2C.hpp"

inline constexpr UART &RC_UART = lpuart1;

inline constexpr UART &mav0Uart = uart1;
inline constexpr UART &mav1Uart = uart2;
inline constexpr UART &debugUart = uart3;

inline constexpr I2C &imuI2C = i2c1;
inline constexpr I2C &i2c = i2c2;