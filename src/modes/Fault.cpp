#include "Fault.hpp"
#include "Disarm.hpp"
#include "Common.hpp"
#include "drivers/I-Bus/I-Bus.hpp"
#include "drivers/PWMOut/PWMOut.hpp"
#include "drivers/onBoardLED/onBoardLED.hpp"

Fault faultMode;

bool Fault::needEnter(const char *&reason)
{
    if (FlightModeDispatcher::getCurrentFlightMode() == &disarmMode)
        return false;

    if (iBus.isOk())
        iBusFaultTimer = millis();
    else if (millis() - iBusFaultTimer > Fault::iBusFaultTime)
    {
        reason = "Control signal is lose";
        return true;
    }

    return false;
}

void Fault::onEnter()
{
    onBoardLED::set(onBoardLED::RED, onBoardLED::FAST_FAST_BLINK);
    onBoardLED::set(onBoardLED::BLUE, onBoardLED::OFF);
    PWMOut::pwmDisable();
}

bool Fault::canExit(const char *&err)
{
    err = "Fault mode";
    const char *dummy;
    return disarmMode.needEnter(dummy);
}
