#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace AHRS
{
    struct Eulerf
    {
        float roll,
            pitch,
            yaw;
    };

    void updateByIMU(Eigen::Vector3f rSpeed, Eigen::Vector3f acceleration, float dT);
    void updateByMagnetometer(Eigen::Vector3f field);

    Eigen::Vector3f getRawRSpeed();
    Eigen::Vector3f getRAcceleration();

    Eigen::Vector3f getRawAcceleration();
    float getG();

    Eigen::Vector3f getRSpeed();
    Eigen::Vector3f getAcceleration();

    Eigen::Quaternionf getFRU_Attitude();
    Eigen::Quaternionf getFRD_Attitude();
    Eulerf getEulerFRU();
    Eulerf getEulerFRD();

    Eigen::Vector3f getLinearAcceleration();

    inline float lastDT = 0;
}