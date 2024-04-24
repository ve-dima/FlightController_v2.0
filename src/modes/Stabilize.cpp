#include <algorithm>
#include "Common.hpp"
#include "Stabilize.hpp"
#include "drivers/PWMOut/PWMOut.hpp"
#include "drivers/I-Bus/I-Bus.hpp"
#include "drivers/onBoardLED/onBoardLED.hpp"
#include "PID/pidRegulator.hpp"
#include "Utility.hpp"

Stabilize stabilizeMode;

bool Stabilize::needEnter(const char *&reason)
{
    if (safeSwitchChannel > 1700 and
        modeSwitchChannel < 1300 and
        subModeSwitch1 < 1500 and
        subModeSwitch2 < 1500)
    {
        reason = "manual switch";
        return true;
    }
    return false;
}

void Stabilize::onEnter()
{
    PID_RegulatorReset();
    throttle = 0.0f;
    altitudeMode = AltitudeMode::none;

    for (unsigned i = 0; i < PWMOut::channelCount; ++i)
        PWMOut::setPower(i, 0.0f);

    onBoardLED::set(onBoardLED::BLUE, onBoardLED::OFF);
    onBoardLED::set(onBoardLED::RED, onBoardLED::ON);

    stickTimer = millis();
    stickWasOnCenter = false;
    setAngle.yaw = quaternionToEuler(ahrs.getAttitude()).yaw;

    PWMOut::pwmEnable();
}

void Stabilize::handler()
{
    if (not iBus.isOk())
        return;

    bool updateByStick = millis() - stickTimer > 50;
    if (updateByStick)
        stickTimer = millis();

    if (throttleChannel > 1050)
    {
        if (altitudeMode == AltitudeMode::none)
            throttle = std::clamp((throttleChannel - 1000) / 1000.f, 0.f, 1.f);
    }
    else
        throttle = 0;

    setAngle.roll = std::clamp(((rollChannel - 1500) / 1000.f) * 0.46f, -0.46f, 0.46f);
    setAngle.pitch = std::clamp(((pitchChannel - 1500) / -1000.f) * 0.46f, -0.46f, 0.46f);

    // rollOffset = std::clamp(((varAChannel - 1500) / 1000.f) * 0.087f, -0.087f, 0.087f);
    // pitchOffset = std::clamp(((varBChannel - 1500) / 1000.f) * 0.087f, -0.087f, 0.087f);

    if (updateByStick and
        (yawChannel > 1550 or yawChannel < 1450))
    {
        setAngle.yaw += std::clamp(((yawChannel - 1500) / -1000.f) * 0.05f, -0.05f, 0.05f);

        if (setAngle.yaw > M_PI)
            setAngle.yaw = -M_PI + (setAngle.yaw - M_PI);
        else if (setAngle.yaw < -M_PI)
            setAngle.yaw = M_PI + (setAngle.yaw + M_PI);
    }

    if (modeSwitchChannel > 1300)
    {
        if (altitudeMode == AltitudeMode::none)
            targetAltitude = ahrs.barometerFilter.x;
        altitudeMode = modeSwitchChannel > 1500 ? AltitudeMode::altitude_stabilization : AltitudeMode::no_corrected_altitude_stabilization;

        if (stickWasOnCenter and updateByStick and (throttleChannel > 1550 or throttleChannel < 1450))
            targetAltitude += std::clamp(((throttleChannel - 1500) / 1000.f) * 0.15f, -0.15f, 0.15f);
        else if (throttleChannel < 1550 and throttleChannel > 1450)
            stickWasOnCenter = true;

        if (subModeSwitch1 > 1500)
            altitudeLoseProtection = true;
        else
            altitudeLoseProtection = false;
    }
    else
    {
        altitudeMode = AltitudeMode::none;
        stickWasOnCenter = false;
        altitudeLoseProtection = false;
    }
}

void Stabilize::attitudeTickHandler()
{
    // targetAngle = eulerToQuaternion(EulerF{setAngle.roll, setAngle.pitch, setAngle.yaw});
    targetAngle = setAngle;
}

void Stabilize::onExit()
{
}
