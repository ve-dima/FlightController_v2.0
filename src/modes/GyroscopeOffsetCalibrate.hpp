/**
 * @file GyroscopeOffsetCalibrate.hpp
 * @author Vakhrushev D. E. (y200323@yandex.ru or vakhrushev@sfedu.ru)
 * @brief Режим калибровки нулевого смещения гироскопа
 */
#pragma once
#include "Modes.hpp"

class GyroscopeOffsetCalibrate : public FlightMode
{
    const char *const name() { return "gyroOffset"; }

    void onEnter() override final;

    void handler() override final;

    bool canExit(const char *&err) override final;
    bool needExit(const char *&reason, FlightMode *&to) override final;

    enum class State
    {
        measuring,
        end,
    } state;
};

extern GyroscopeOffsetCalibrate gyroscopeOffsetCalibrateMode;