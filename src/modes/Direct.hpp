/**
 * @file Direct.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Режим для калибровки ESC-регуляторов
 */
#pragma once
#include "Modes.hpp"

class Direct : public FlightMode
{
public:
    virtual const char *const name() { return "direct"; }

    // bool needEnter(const char *&reason) override final;

    void onEnter() override final;
    void handler() override final;
};

extern Direct directMode;