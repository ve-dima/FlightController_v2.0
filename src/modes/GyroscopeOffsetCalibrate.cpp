#include "Common.hpp"
#include "GyroscopeOffsetCalibrate.hpp"
#include "drivers/onBoardLED/onBoardLED.hpp"
#include "drivers/I-Bus/I-Bus.hpp"
#include <cmath>
#include <algorithm>

GyroscopeOffsetCalibrate gyroscopeOffsetCalibrateMode;

void GyroscopeOffsetCalibrate::onEnter()
{
    state = State::measuring;

    onBoardLED::set(onBoardLED::BLUE, onBoardLED::FAST_FAST_BLINK);
    onBoardLED::set(onBoardLED::RED, onBoardLED::OFF);
}

void GyroscopeOffsetCalibrate::handler()
{
    float samples[3][100];
    uint32_t timer = millis();

    for (int i = 0; i < 100;)
    {
        cycle();

        if (millis() - timer < 50)
            continue;
        timer = millis();

        if (std::abs(ahrs.rawGyroscope.x) > 0.05 or
            std::abs(ahrs.rawGyroscope.y) > 0.05 or
            std::abs(ahrs.rawGyroscope.z) > 0.05)
            continue;

        samples[0][i] = ahrs.rawGyroscope.x;
        samples[1][i] = ahrs.rawGyroscope.y;
        samples[2][i] = ahrs.rawGyroscope.z;
        ++i;
    }

    std::sort(samples[0], samples[0] + 100);
    std::sort(samples[1], samples[1] + 100);
    std::sort(samples[2], samples[2] + 100);

    ahrs.gyroscopeOffset.x = samples[0][50];
    ahrs.gyroscopeOffset.y = samples[1][50];
    ahrs.gyroscopeOffset.z = samples[2][50];

    state = State::end;
}

bool GyroscopeOffsetCalibrate::canExit(const char *&err)
{
    if (state != State::end)
    {
        err = "in calibrate proccess";
        return false;
    }

    return true;
}

bool GyroscopeOffsetCalibrate::needExit(const char *&reason, FlightMode *&to)
{
    if (state != State::end)
        return false;

    reason = "end of calibrate";
    return true;
}