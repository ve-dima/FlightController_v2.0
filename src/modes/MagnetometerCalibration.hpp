/**
 * @file MagnetometerCalibration.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Какая-никакая калибровка компаса (дописать как-нибудь надо, пока тут пусто)
 */
#pragma once
#include "Modes.hpp"

//TODO: почитать, подумать, сделать

class MagnetometerCalibrate : public FlightMode
{
    const char *const name() { return "magOffset"; }

    void onEnter() override final;

    void handler() override final;

    bool canExit(const char *&err) override final;
    bool needExit(const char *&reason, FlightMode *&to) override final;

    enum class State
    {
        measuring,
        end,
    } state;
    Vector3F max, min;
};

extern MagnetometerCalibrate magnetometerCalibrateMode;