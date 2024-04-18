#include "motor.hpp"
#include "Common.hpp"
#include <cmath>

namespace Motor
{
    State state = State::undefined;
    // Protocol protocol = Protocol::pwm50;

    State getState() { return state; }
    // Protocol getProtocol() {return protocol;}

    float power[maxCount];

    bool setPower(unsigned motor, float motorPower)
    {
        if (motor > maxCount or state != State::armed)
            return false;

        if (not std::isfinite(motorPower))
            motorPower = -1;
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

    uint32_t disarmTimeStamp = 0;
    void disarm()
    {
        std::fill(power, &power[maxCount], 0);
        disarmTimeStamp = millis();
        state = State::disarming;
    }

    uint32_t armTimeStamp = 0;
    void arm()
    {
        std::fill(power, &power[maxCount], 0);
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
                state = State::disarm;
            break;
        case State::arming:
            if (millis() - armTimeStamp > armTime)
                state = State::disarm;
            break;
        default:
            break;
        }
    }
}