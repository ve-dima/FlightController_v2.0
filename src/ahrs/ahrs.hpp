#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace AHRS
{
    inline float accelerationFilterGain = 0.02;
    inline float accelerationRejection = 2;
 
    void updateByIMU(Eigen::Vector3f rotate, Eigen::Vector3f acceleration, float dT);
    void updateByMagnetometer(Eigen::Vector3f field);

    Eigen::Quaternionf getAttitude();
}