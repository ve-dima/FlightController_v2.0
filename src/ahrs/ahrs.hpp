#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace AHRS
{
    void updateByIMU(Eigen::Vector3f rotate, Eigen::Vector3f acceleration, float dT);
    void updateByMagnetometer(Eigen::Vector3f field);

    Eigen::Quaternionf getAttitude();
    Eigen::Vector3f getRawRotate();
    Eigen::Vector3f getRawAcceleration();

    extern float gyroscopeOffset[3];
    extern float gyroscopeRange;

    extern float accelerometerOffset[3];
    extern float accelerometerRange;

    extern float accelerationFilterGain;
    extern float accelerationRejection;
    extern float accelerationRejectionAngle;

}