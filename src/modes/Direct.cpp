#include <algorithm>
#include "Common.hpp"
#include "Direct.hpp"
#include "drivers/PWMOut/PWMOut.hpp"
#include "drivers/I-Bus/I-Bus.hpp"
#include "drivers/onBoardLED/onBoardLED.hpp"
#include "PID/pidRegulator.hpp"
#include "Utility.hpp"

Direct directMode;

// bool Direct::needEnter(const char *&reason)
// {
//     if (safeSwitchChannel > 1700 and
//         modeSwitchChannel > 1700 and
//         subModeSwitch2 < 1500)
//     {
//         reason = "manual switch";
//         return true;
//     }
//     return false;
// }

void Direct::onEnter()
{
    // pidSetting[0] = pidSetting[1] = pidSetting[2] = PID<float>::PID_Settings{0, 0, 0, 0};
    PID_RegulatorReset();
    throttle = 0.0f;

    PWMOut::pwmDisable();
    for (unsigned i = 0; i < PWMOut::channelCount; ++i)
        PWMOut::setPower(i, 0.0f);

    onBoardLED::set(onBoardLED::BLUE, onBoardLED::ON);
    onBoardLED::set(onBoardLED::RED, onBoardLED::ON);
}

void Direct::handler()
{
    if (throttleChannel > 1150)
        throttle = std::clamp((throttleChannel - 1000) / 1000.f, 0.f, 1.f);
    else
        throttle = 0;

    PWMOut::pwmOut(subModeSwitch2 > 1500);
}
