#pragma once
#include <array>
#include <PID.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Control
{
    union PIDSettings
    {
        struct
        {
            PIDf::Settings roll;
            PIDf::Settings pitch;
            PIDf::Settings yaw;
        } axis;
        std::array<PIDf::Settings, 3> settings;
    } extern rateSettings, angleSettings;

    extern float minimalTrust;
    extern float yawWeight;

    void rateHandler();
    void velocityHandler();
    // void positionHandler();
    void updateMotorPower();

    Eigen::Vector3f getTargetRate();
    Eigen::Quaternionf getTargetAttitude();
    float getTargetThrust();
    Eigen::Vector3f getTargetThrustVector();

    void setTargetRate(Eigen::Vector3f);
    void setTargetAttitude(Eigen::Quaternionf);
    void setTargetThrust(float);
}
