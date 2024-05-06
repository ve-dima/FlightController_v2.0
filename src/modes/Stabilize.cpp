#include <algorithm>
#include "Stabilize.hpp"
#include "motor/motor.hpp"
#include "control/Control.hpp"
#include "indicators/LED.hpp"
#include "rc/RC.hpp"
#include "param/param.hpp"

float manualMaxTilt = 90 * (M_PI / 180);

Stabilize stabilizeMode;

bool Stabilize::needEnter(const char *&reason)
{
    if (RC::channel(RC::ChannelFunction::ARMSWITCH) > 0.2 and
        RC::channel(RC::ChannelFunction::AUX_1) != NAN)
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

    LED::setLED(LED::Color::green, LED::Action::double_short_blink);
    Motor::arm();
}

void Stabilize::attitudeTickHandler()
{
    Eigen::Quaternionf setPoint;

    Eigen::Vector3f targetTilt{
        RC::channel(RC::ChannelFunction::ROLL) * manualMaxTilt,
        RC::channel(RC::ChannelFunction::PITCH) * manualMaxTilt, 0};
    float tiltAngle = targetTilt.norm();
    if (tiltAngle > manualMaxTilt)
        tiltAngle = manualMaxTilt;
    targetTilt *= (1 / tiltAngle);
    setPoint = Eigen::Quaternionf(Eigen::AngleAxisf(tiltAngle, targetTilt));

    Control::setTargetAttitude(setPoint);
}

PARAM_ADD(param::FLOAT, MPC_MAN_TILT_MAX, &manualMaxTilt);