#pragma once
#include <array>
#include <PID.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Control
{
    enum class AngleControlMode : unsigned
    {
        velocity,
        angle,
    };

    union Modes_t
    {
        std::array<AngleControlMode, 3> modes;
        struct
        {
            AngleControlMode roll;
            AngleControlMode pitch;
            AngleControlMode yaw;
        } axis;
    } extern angleModes;

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
