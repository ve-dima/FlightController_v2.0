#pragma once
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
    void updateByPressure(float P);
    void updateByTemperature(float T);

    Eigen::Vector3f getRawRotateSpeed();
    Eigen::Vector3f getRotateSpeed();
    Eigen::Vector3f getFRD_RotateSpeed();
    Eigen::Vector3f getRotateAcceleration();

    Eigen::Vector3f getRawAcceleration();
    float getG();

    float getPressure();
    float getTemperature();

    Eigen::Vector3f getMagneticField();

    Eigen::Quaternionf getFRU_Attitude();
    Eigen::Quaternionf getFRD_Attitude();
    Eulerf getFRU_Euler();
    Eulerf getFRD_Euler();

    Eigen::Vector3f getFRD_LinearAcceleration();
    float getLastDT();

    Eigen::Vector3f getZState();
    Eigen::Vector3f getZVaraince();
}