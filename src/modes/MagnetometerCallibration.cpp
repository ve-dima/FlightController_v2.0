#include "Common.hpp"
#include "MagnetometerCalibration.hpp"
#include "drivers/onBoardLED/onBoardLED.hpp"
#include "drivers/I-Bus/I-Bus.hpp"
#include <cmath>
#include <algorithm>

MagnetometerCalibrate MagnetometerCalibrateMode;

void MagnetometerCalibrate::onEnter()
{
    state = State::measuring;

    onBoardLED::set(onBoardLED::BLUE, onBoardLED::FAST_FAST_BLINK);
    onBoardLED::set(onBoardLED::RED, onBoardLED::OFF);

    max = min = Vector3F(0, 0, 0);
}

void MagnetometerCalibrate::handler()
{
    for (int i = 0; i < 3; ++i)
        max.arr[i] = std::max(max.arr[i], ahrs.rawMagnetometer.arr[i]),
        min.arr[i] = std::min(min.arr[i], ahrs.rawMagnetometer.arr[i]);
}

bool MagnetometerCalibrate::canExit(const char *&err)
{
    if (state != State::end)
    {
        err = "in calibrate proccess";
        return false;
    }

    return true;
}

bool MagnetometerCalibrate::needExit(const char *&reason, FlightMode *&to)
{
    if (state != State::end)
        return false;

    reason = "end of calibrate";
    return true;
}