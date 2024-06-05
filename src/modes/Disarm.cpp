#include "Disarm.hpp"
#include "motor/motor.hpp"
#include "control/Control.hpp"
#include "indicators/LED.hpp"
#include "rc/RC.hpp"

Disarm disarmMode;

void Disarm::onEnter()
{
    LED::setLED(LED::Color::red, LED::Action::double_short_blink);
    Motor::disarm();
}

bool Disarm::needEnter(const char *&reason)
{
    if (RC::channel(RC::ChannelFunction::ARMSWITCH) < 0.2 or
        RC::channel(RC::ChannelFunction::ARMSWITCH) == NAN)
    {
        reason = "Manual switch";
        return true;
    }
    else if (RC::state() != RC::State::ok)
    {
        reason = "Manual control loss";
        return true;
    }
    return false;
}

bool Disarm::canExit(const char *&err)
{
    if (RC::channel(RC::ChannelFunction::ARMSWITCH) > 0.2 or
        RC::channel(RC::ChannelFunction::ARMSWITCH) == NAN)
    {
        err = "Not armed";
        return false;
    }
    else if (RC::state() != RC::State::ok)
    {
        err = "No manual control";
        return false;
    }

    return true;
}