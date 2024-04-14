#pragma once
#include <cstdint>

void setup();
void loop();

void yield(void) __attribute__((weak));
void wdtFeed() __attribute__((weak));

uint32_t millis();

uint32_t micros();

uint32_t tick();

void delay(uint32_t ms);

void delayMicroseconds(uint32_t us);