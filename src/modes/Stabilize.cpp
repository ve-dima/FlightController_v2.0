#include <algorithm>
#include "Stabilize.hpp"
#include "motor/motor.hpp"
#include "control/Control.hpp"
#include "indicators/LED.hpp"
#include "ahrs/ahrs.hpp"
#include "rc/RC.hpp"
#include "param/param.hpp"

float manualMaxTilt = 30 * (M_PI / 180);
float manualYawRate = 150 * (M_PI / 180);

Stabilize stabilizeMode;

bool Stabilize::needEnter(const char *&reason)
{
    // return true;

    if (RC::channel(RC::ChannelFunction::ARMSWITCH) > 0.2)
    {
        reason = "manual switch";
        return true;
    }
    return false;
}

void Stabilize::onEnter()
{
    Control::setTargetThrust(0);
    Control::setTargetRate(Eigen::Vector3f(0, 0, 0));
    Control::setTargetAttitude(Eigen::Quaternionf::Identity());

    manualYawSetPoint = AHRS::getEulerFRD().yaw;

    LED::setLED(LED::Color::green, LED::Action::double_short_blink);
    Motor::arm();
}

float constrainAngle(float x)
{
    x = std::fmod<float>(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

void Stabilize::attitudeTickHandler()
{
    // const float yaw = AHRS::getEulerFRD().yaw;

    if (RC::channel(RC::ChannelFunction::THROTTLE) < -0.9)
    {
        // manualYawSetPoint = yaw;
        Control::setTargetThrust(0);
    }
    else
    {
        Control::setTargetThrust((RC::channel(RC::ChannelFunction::THROTTLE) + 1) / 2);

        if (not RC::inDZ(RC::ChannelFunction::YAW))
            manualYawSetPoint += RC::channel(RC::ChannelFunction::YAW) * AHRS::lastDT * manualYawRate,
                manualYawSetPoint = constrainAngle(manualYawSetPoint);
    }
    const Eigen::Quaternionf qYawSP(std::cos(manualYawSetPoint / 2.f), 0.f, 0.f, std::sin(manualYawSetPoint / 2.f));

    Eigen::Quaternionf qRPSP;
    Eigen::Vector3f targetTilt{
        RC::channel(RC::ChannelFunction::ROLL) * manualMaxTilt,
        -RC::channel(RC::ChannelFunction::PITCH) * manualMaxTilt, 0};
    float tiltAngle = targetTilt.norm();
    if (tiltAngle > manualMaxTilt)
        tiltAngle = manualMaxTilt;
    if (tiltAngle > 1e-4)
    {
        targetTilt /= tiltAngle;
        qRPSP = Eigen::Quaternionf(Eigen::AngleAxisf(tiltAngle, targetTilt));
    }
    else
        qRPSP = Eigen::Quaternionf::Identity();

    const Eigen::Quaternionf setPoint = qYawSP * qRPSP;
    Control::setTargetAttitude(setPoint);
}

PARAM_ADD(param::FLOAT, MPC_MAN_TILT_MAX, &manualMaxTilt);
PARAM_ADD(param::FLOAT, MNT_RATE_YAW, &manualYawRate);