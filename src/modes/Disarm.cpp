#include "Disarm.hpp"
#include "motor/motor.hpp"
#include "control/Control.hpp"
#include "indicators/LED.hpp"
#include "rc/RC.hpp"
#include "ICM-20948/ICM-20948.hpp"

Disarm disarmMode;

void Disarm::onEnter()
{
    LED::setLED(LED::Color::red, LED::Action::off);
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
    if (RC::channel(RC::ChannelFunction::ARMSWITCH) < 0 or
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

void Disarm::handler()
{
    if (ICM20948::isOK() == false)
    {
        LED::setLED(LED::Color::red, LED::Action::double_short_blink);
        return;
    }

    if (RC::state() != RC::State::ok)
    {
        LED::setLED(LED::Color::red, LED::Action::short_blink);
        return;
    }

    LED::setLED(LED::Color::red, LED::Action::off);
}
