#include "motor.hpp"
#include "Common.hpp"
#include <cmath>

namespace Motor
{
    State state = State::undefined;
    State getState() { return state; }

    float power[maxCount];

    bool setPower(unsigned motor, float motorPower)
    {
        if (motor > maxCount or state != State::armed)
            return false;

        if (not std::isfinite(motorPower))
            motorPower = NAN;
        else if (motorPower > 1)
            motorPower = 1;

        power[motor] = motorPower;
        updateOutput(motor);
        return true;
    }

    float getPower(unsigned motor)
    {
        if (motor > maxCount)
            return NAN;
        else
            return power[motor];
    }

    const float *const getPower() { return power; }

    uint32_t disarmTimeStamp = 0;
    void disarm()
    {
        if (state == State::disarming or state == State::disarmed)
            return;

        std::fill(power, &power[maxCount], NAN);
        for (unsigned i = 0; i < maxCount; i++)
            updateOutput(i);
        disarmTimeStamp = millis();
        state = State::disarming;
    }

    uint32_t armTimeStamp = 0;
    void arm()
    {
        if (state == State::arming or state == State::armed)
            return;

        std::fill(power, &power[maxCount], 0);
        for (unsigned i = 0; i < maxCount; i++)
            updateOutput(i);
        armTimeStamp = millis();
        state = State::arming;
    }

    void stateHandler()
    {
        switch (state)
        {
        case State::undefined:
            disarm();
            break;
        case State::disarming:
            if (millis() - disarmTimeStamp > disarmTime)
                state = State::disarmed;

            break;
        case State::arming:
            if (millis() - armTimeStamp > armTime)
                state = State::armed;

            break;
        default:
            break;
        }
    }
}