#include "Disarm.hpp"
#include "Common.hpp"
#include "drivers/PWMOut/PWMOut.hpp"
#include "drivers/I-Bus/I-Bus.hpp"
#include "drivers/PWMOut/PWMOut.hpp"
#include "drivers/onBoardLED/onBoardLED.hpp"
#include "PID/pidRegulator.hpp"
#include "Connection.hpp"

Disarm disarmMode;

void Disarm::onEnter()
{
    PID_RegulatorReset();
    throttle = 0.0f;
    altitudeMode = AltitudeMode::none;

    for (unsigned i = 0; i < PWMOut::channelCount; ++i)
        PWMOut::setPower(i, 0.0f);
    PWMOut::pwmDisable();

    onBoardLED::set(onBoardLED::RED, onBoardLED::OFF);
    onBoardLED::set(onBoardLED::BLUE, onBoardLED::OFF);
}

bool Disarm::needEnter(const char *&reason)
{
    if (safeSwitchChannel <= 1200)
    {
        reason = "manual switch";
        return true;
    }
    return false;
}