#include "ahrs.hpp"

Eigen::Quaternionf adaptiveSLERP_I(Eigen::Quaternionf q, float gain, const float threshold = 0.97)
{
    // optimize LERP and SLERP with qI(1,0,0,0)======================
    if (q.w() < threshold)
    {
        // Slerp (Spherical linear interpolation):
        float angle = std::acos(q.w());
        float A = std::sin(angle * 1.0 - gain) / std::sin(angle);
        float B = std::sin(angle * gain) / std::sin(angle);

        q.w() = A + B * q.w();
        q.x() = B * q.x();
        q.y() = B * q.y();
        q.z() = B * q.z();
    }
    else
    {
        // Lerp (Linear interpolation):
        q.w() = (1.0 - gain) + gain * q.w();
        q.x() = gain * q.x();
        q.y() = gain * q.y();
        q.z() = gain * q.z();
    }

    return q;
}

namespace AHRS
{
    extern float gyroscopeOffset[3];
    extern float gyroscopeRange;

    extern float accelerometerOffset[3];
    extern float accelerometerRange;

    extern float accelerationFilterGain;
    extern float accelerationRejection;
    extern float accelerationRejectionAngle;

    Eigen::Vector3f rawRSpeed, rotateSpeed;
    Eigen::Vector3f rawAcceleration, acceleration;
    Eigen::Quaternionf attitude = Eigen::Quaternionf(1, 0, 0, 0);
    Eulerf eulerAttitude;
    float G = 1;

    Eigen::Vector3f linearAcceleration;

    Eigen::Vector3f getRawRSpeed() { return rawRSpeed; }
    Eigen::Vector3f getRAcceleration() { return Eigen::Vector3f(0, 0, 0); }

    Eigen::Vector3f getRawAcceleration() { return rawAcceleration; }

    Eigen::Vector3f getRSpeed() { return rotateSpeed; }
    Eigen::Vector3f getFRD_RSpeed()
    {
        return Eigen::Vector3f(rotateSpeed.x(), rotateSpeed.y(), -rotateSpeed.z());
    }
    Eigen::Vector3f getAcceleration() { return acceleration; }
    float getG() { return G; }

    Eigen::Quaternionf getFRU_Attitude() { return attitude; }
    Eigen::Quaternionf getFRD_Attitude() { return Eigen::Quaternionf(attitude.w(), attitude.x(), attitude.y(), -attitude.z()); }
    Eulerf getEulerFRD() { return eulerAttitude; }

    Eigen::Vector3f getLinearAcceleration() { return linearAcceleration; }

    void update()
    {
        Eigen::Quaternionf worldFrameAcc = (attitude.conjugate() *
                                            Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z())) *
                                           attitude;
        linearAcceleration = worldFrameAcc.vec() - Eigen::Vector3f(0, 0, 1);

        Eigen::Quaternionf attitude = getFRD_Attitude();
        float sinr_cosp = 2 * (attitude.w() * attitude.x() + attitude.y() * attitude.z());
        float cosr_cosp = 1 - 2 * (attitude.x() * attitude.x() + attitude.y() * attitude.y());
        eulerAttitude.roll = std::atan2(sinr_cosp, cosr_cosp);

        float sinp = 2 * (attitude.w() * attitude.y() - attitude.x() * attitude.z());
        if (std::abs(sinp) >= 1)
            eulerAttitude.pitch = std::copysign<float>(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            eulerAttitude.pitch = std::asin(sinp);

        float siny_cosp = 2 * (attitude.w() * attitude.z() + attitude.x() * attitude.y());
        float cosy_cosp = 1 - 2 * (attitude.y() * attitude.y() + attitude.z() * attitude.z());
        eulerAttitude.yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    void updateByIMU(Eigen::Vector3f rSpeed, Eigen::Vector3f rAcc, float dT)
    {
        lastDT = dT;
        rawRSpeed = rSpeed;
        rawAcceleration = rAcc;

        rotateSpeed = rSpeed - Eigen::Vector3f{gyroscopeOffset};
        acceleration = rAcc - Eigen::Vector3f{accelerometerOffset};

        Eigen::Quaternionf current = attitude;

        Eigen::Quaternionf qDot = Eigen::Quaternionf(
            0.5 * dT * (-rotateSpeed.x() * current.x() - rotateSpeed.y() * current.y() - rotateSpeed.z() * current.z()),
            0.5 * dT * (rotateSpeed.x() * current.w() + rotateSpeed.y() * current.z() - rotateSpeed.z() * current.y()),
            0.5 * dT * (-rotateSpeed.x() * current.z() + rotateSpeed.y() * current.w() + rotateSpeed.z() * current.x()),
            0.5 * dT * (rotateSpeed.x() * current.y() - rotateSpeed.y() * current.x() + rotateSpeed.z() * current.w()));
        current.coeffs() += qDot.coeffs();
        current.normalize();
        attitude = current;

        G = acceleration.norm();
        float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
        gain = std::clamp(gain, 0.f, 1.f);

        const Eigen::Vector3f normAcc = acceleration * (1.f / G);
        Eigen::Quaternionf worldFrameAcc = (current.conjugate() *
                                            Eigen::Quaternionf(0.f, normAcc.x(), normAcc.y(), normAcc.z())) *
                                           current;
        Eigen::Quaternionf accDeltaQ;
        accDeltaQ.w() = std::sqrt((worldFrameAcc.z() + 1) / 2);
        accDeltaQ.x() = -worldFrameAcc.y() / (2 * accDeltaQ.w());
        accDeltaQ.y() = worldFrameAcc.x() / (2 * accDeltaQ.w());
        accDeltaQ.z() = 0.0;
        accDeltaQ = adaptiveSLERP_I(accDeltaQ, gain);

        if (accDeltaQ.coeffs().allFinite())
            attitude = (current * accDeltaQ).normalized();

        update();
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
    }
}