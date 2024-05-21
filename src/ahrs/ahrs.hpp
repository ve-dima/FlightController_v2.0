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

    Eigen::Vector3f getRawRSpeed();
    Eigen::Vector3f getRAcceleration();

    Eigen::Vector3f getRawAcceleration();
    float getG();

    float getPressure();
    float getTemperature();

    Eigen::Vector3f getRSpeed();
    Eigen::Vector3f getFRD_RSpeed();

    Eigen::Quaternionf getFRU_Attitude();
    Eigen::Quaternionf getFRD_Attitude();
    Eulerf getEulerFRU();
    Eulerf getEulerFRD();

    Eigen::Vector3f getLinearAcceleration();

    inline float lastDT = 0;
    
    extern Eigen::Vector3f x;
    extern Eigen::Matrix3f P;
}