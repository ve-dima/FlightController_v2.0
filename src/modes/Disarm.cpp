#include "Disarm.hpp"
#include "motor/motor.hpp"
#include "control/Control.hpp"
#include "indicators/LED.hpp"
#include "rc/RC.hpp"

Disarm disarmMode;

void Disarm::onEnter()
{
    LED::setLED(LED::Color::red, LED::Action::blink);
    Motor::disarm();
}

bool Disarm::needEnter(const char *&reason)
{
    // return false;
    if (RC::channel(RC::ChannelFunction::ARMSWITCH) < 0.2 or
        RC::channel(RC::ChannelFunction::ARMSWITCH) == NAN)
    {
        reason = "manual switch";
        return true;
    }
    else if (RC::state() != RC::State::ok)
    {
        reason = "manual control loss";
        return true;
    }
    return false;
}

bool Disarm::canExit(const char *&err)
{
    // return true;

    if (RC::channel(RC::ChannelFunction::ARMSWITCH) < 0.2 or
        RC::channel(RC::ChannelFunction::ARMSWITCH) == NAN)
    {
        err = "Not armed";
        return false;
    }
    else if (RC::state() != RC::State::ok)
    {
        err = "no manual control";
        return false;
    }

    return true;
}