/**
 * @file Disarm.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Режим disarm. (Отключение двигателей)
 */
#pragma once
#include "Modes.hpp"

class Disarm : public FlightMode
{
public:
    virtual const char *const name() { return "disarm"; }

    bool needEnter(const char *&reason) override final;
    void onEnter() override final;
};

extern Disarm disarmMode;