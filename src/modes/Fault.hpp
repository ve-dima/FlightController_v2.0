/**
 * @file Fault.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Режим fault. (Отказ каких-либо систем, отключение двигателей)
 */
#pragma once
#include "Modes.hpp"
#include <cstdint>

class Fault : public FlightMode
{
private:
    uint32_t iBusFaultTimer = 0;

public:
    static constexpr uint32_t iBusFaultTime = 250;

    virtual const char *const name() { return "fault"; }

    bool needEnter(const char *&reason) override final;
    void onEnter() override final;
    bool canExit(const char *&err) override final;
};

extern Fault faultMode;