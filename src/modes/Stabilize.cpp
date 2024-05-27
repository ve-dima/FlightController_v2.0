#include <algorithm>
#include "Stabilize.hpp"
#include "motor/motor.hpp"
#include "control/Control.hpp"
#include "indicators/LED.hpp"
#include "ahrs/ahrs.hpp"
#include "rc/RC.hpp"
#include "param/param.hpp"
#include "math/math.hpp"
#include <stm32g4xx.h>

float manualMaxTilt = 30 * (M_PI / 180);
float manualYawRate = 150 * (M_PI / 180);
float acroRate = 600 * (M_PI / 180);
float altitudeSetSpeed = 2;

Stabilize stabilizeMode;

bool Stabilize::needEnter(const char *&reason)
{
    if (RC::channel(RC::ChannelFunction::ARMSWITCH) > 0.2 and
        RC::channel(RC::ChannelFunction::THROTTLE) < -0.9)
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

    manualYawSetPoint = AHRS::getFRD_Euler().yaw;
    homeYaw = Eigen::Quaternionf{std::cos(manualYawSetPoint / 2.f), 0.f, 0.f, std::sin(manualYawSetPoint / 2.f)};

    LED::setLED(LED::Color::green, LED::Action::double_short_blink);
    Motor::arm();
}

void Stabilize::attitudeTickHandler()
{
    if (RC::channel(RC::ChannelFunction::AUX_1) < -0.9)
        levelMode();
    else if (RC::channel(RC::ChannelFunction::AUX_1) > 0.9)
        levelMode();
    else
        altMode();
}

Eigen::Quaternionf Stabilize::getSPFromRC()
{
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

    if (not RC::inDZ(RC::ChannelFunction::YAW) and RC::channel(RC::ChannelFunction::THROTTLE) > -0.9)
        manualYawSetPoint += RC::channel(RC::ChannelFunction::YAW) * AHRS::getLastDT() * manualYawRate;

    Eigen::Quaternionf qYawSP(std::cos(manualYawSetPoint / 2.f), 0.f, 0.f, std::sin(manualYawSetPoint / 2.f));

    if (RC::channel(RC::ChannelFunction::FLTBTN_SLOT_1) > 0)
        return qRPSP * qYawSP;
    else
        return qYawSP * qRPSP;
}

float Stabilize::getThrottleFromRC()
{
    const float tht = expo(RC::channel(RC::ChannelFunction::THROTTLE), 0.5);
    return (tht + 1) / 2;
}

void Stabilize::levelMode()
{
    if (RC::channel(RC::ChannelFunction::THROTTLE) < -0.9)
        Control::setTargetThrust(0);
    else
    {
        Control::setTargetThrust(getThrottleFromRC());

        if (not RC::inDZ(RC::ChannelFunction::YAW))
            manualYawSetPoint += RC::channel(RC::ChannelFunction::YAW) * AHRS::getLastDT() * manualYawRate;
    }

    Control::setTargetAttitude(getSPFromRC());
    Control::trustMode = Control::TrustMode::MANUAL;
    Control::setTargetThrust(getThrottleFromRC());
}

void Stabilize::altMode()
{
    if (RC::channel(RC::ChannelFunction::THROTTLE) < -0.9)
        Control::setTargetThrust(0);
    else
    {
        Control::setTargetThrust(getThrottleFromRC());

        if (not RC::inDZ(RC::ChannelFunction::YAW))
            manualYawSetPoint += RC::channel(RC::ChannelFunction::YAW) * AHRS::getLastDT() * manualYawRate;
    }

    Control::setTargetAttitude(getSPFromRC());
    Control::trustMode = Control::TrustMode::ALTITUDE;
    if (not RC::inDZ(RC::ChannelFunction::THROTTLE))
    {
        Control::targetAltitude = AHRS::x[0] + expo(RC::channel(RC::ChannelFunction::THROTTLE), 0.5) * altitudeSetSpeed;
        // __BKPT(0);
    }
}

void Stabilize::acroMode()
{
    // Eigen::Vector3f rotateVec{
    //     RC::channel(RC::ChannelFunction::ROLL),
    //     RC::channel(RC::ChannelFunction::PITCH),
    //     RC::channel(RC::ChannelFunction::YAW),
    // };

    // if (RC::inDZ(RC::ChannelFunction::ROLL))
    //     rotateVec.x() = 0;
    // if (RC::inDZ(RC::ChannelFunction::PITCH))
    //     rotateVec.y() = 0;
    // if (RC::inDZ(RC::ChannelFunction::YAW))
    //     rotateVec.z() = 0;

    // for (float val : rotateVec)
    //     val = expo(val, 0.7) * acroRate;

    // Eigen::Quaternionf deltaQ = omega(AHRS::getFRD_Attitude(), rotateVec);
    // acroSP.coeffs() += deltaQ.coeffs() * AHRS::lastDT;
    // acroSP.normalize();

    // Control::setTargetAttitude(acroSP);
    // Control::trustMode = Control::TrustMode::MANUAL;
    // Control::setTargetThrust(getThrottleFromRC());
}

PARAM_ADD(param::FLOAT, MPC_MAN_TILT_MAX, &manualMaxTilt);
PARAM_ADD(param::FLOAT, MNT_RATE_YAW, &manualYawRate);