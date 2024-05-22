#include "ahrs.hpp"
#include "atmosphere.hpp"
#include "Math/math.hpp"

#include "Common.hpp"
#include <algorithm>

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
    float pressure, temperature;
    Eigen::Vector3f linearAcceleration;

    Eigen::Matrix3f P{
        {100, 0, 0},
        {0, 100, 0},
        {0, 0, 10},
    };
    const Eigen::Matrix3f Q = Eigen::Matrix3f{
                                  {0.25, 0.5, 0.5},
                                  {0.5, 1., 1.},
                                  {0.5, 1., 1.},
                              } *
                              (1.0f / 100);
    Eigen::Vector3f x{0, 0, 0};

    void predictKalman(const float dt)
    {
        const Eigen::Vector3f newX{
            (x[2] * dt * dt) / 2 + x[1] * dt + x[0],
            x[1] + dt * x[2],
            x[2],
        };
        x = newX;

        const Eigen::Matrix3f newP{
            {
                P(0, 0) + P(1, 0) * dt + (dt * dt * ((P(2, 2) * dt * dt) / 2 + P(1, 2) * dt + P(0, 2))) / 2 + (P(2, 0) * dt * dt) / 2 + dt * ((P(2, 1) * dt * dt) / 2 + P(1, 1) * dt + P(0, 1)),
                P(0, 1) + P(1, 1) * dt + (P(2, 1) * dt * dt) / 2 + dt * ((P(2, 2) * dt * dt) / 2 + P(1, 2) * dt + P(0, 2)),
                (P(2, 2) * dt * dt) / 2 + P(1, 2) * dt + P(0, 2),
            },

            {
                P(1, 0) + P(2, 0) * dt + dt * (P(1, 1) + P(2, 1) * dt) + (dt * dt * (P(1, 2) + P(2, 2) * dt)) / 2,
                P(1, 1) + P(2, 1) * dt + dt * (P(1, 2) + P(2, 2) * dt),
                P(1, 2) + P(2, 2) * dt,
            },

            {
                (P(2, 2) * dt * dt) / 2 + P(2, 1) * dt + P(2, 0),
                P(2, 1) + P(2, 2) * dt,
                P(2, 2),
            },
        };
        P = newP + Q;
    }

    void correctPos(const float z,
                    const float R)
    {
        const float inv_sk = 1 / (P(0, 0) + R);
        const float error = x(0) - z;

        Eigen::Vector3f newX{
            x(0) - (P(0, 0) * error) * inv_sk,
            x(1) - (P(1, 0) * error) * inv_sk,
            x(2) - (P(2, 0) * error) * inv_sk,
        };
        x = newX;

        Eigen::Matrix3f newP{
            {-P(0, 0) * (P(0, 0) * inv_sk - 1), -P(0, 1) * (P(0, 0) * inv_sk - 1), -P(0, 2) * (P(0, 0) * inv_sk - 1)},
            {P(1, 0) - P(0, 0) * P(1, 0) * inv_sk, P(1, 1) - P(0, 1) * P(1, 0) * inv_sk, P(1, 2) - P(0, 2) * P(1, 0) * inv_sk},
            {P(2, 0) - P(0, 0) * P(2, 0) * inv_sk, P(2, 1) - P(0, 1) * P(2, 0) * inv_sk, P(2, 2) - P(0, 2) * P(2, 0) * inv_sk},
        };
        P = newP;
    }

    void correctAcc(const float z,
                    const float R)
    {
        const float inv_sk = 1 / (P(2, 2) + R);
        const float error = x(2) - z;

        Eigen::Vector3f newX{
            x(0) - (P(0, 2) * error) * inv_sk,
            x(1) - (P(1, 2) * error) * inv_sk,
            x(2) - (P(2, 2) * error) * inv_sk,
        };
        x = newX;

        Eigen::Matrix3f newP{
            {P(0, 0) - P(0, 2) * P(2, 0) * inv_sk, P(0, 1) - P(0, 2) * P(2, 1) * inv_sk, P(0, 2) - P(0, 2) * P(2, 2) * inv_sk},
            {P(1, 0) - P(1, 2) * P(2, 0) * inv_sk, P(1, 1) - P(1, 2) * P(2, 1) * inv_sk, P(1, 2) - P(1, 2) * P(2, 2) * inv_sk},
            {-P(2, 0) * (P(2, 2) * inv_sk - 1), -P(2, 1) * (P(2, 2) * inv_sk - 1), -P(2, 2) * (P(2, 2) * inv_sk - 1)},
        };
        P = newP;
    }

    void update()
    {
        static uint32_t maxClk = 0;
        uint32_t startTime = tick();

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

        maxClk = std::max(maxClk, tick() - startTime);
    }

    void updateByIMU(Eigen::Vector3f rSpeed, Eigen::Vector3f rAcc, float dT)
    {
        lastDT = dT;
        rawRSpeed = rSpeed;
        rawAcceleration = rAcc;

        rotateSpeed = rSpeed - Eigen::Vector3f{gyroscopeOffset};
        acceleration = rAcc - Eigen::Vector3f{accelerometerOffset};

        Eigen::Quaternionf current = attitude;
        current.coeffs() += omega(current, rotateSpeed).coeffs() * dT;
        current.normalize();
        attitude = current;

        G = acceleration.norm();
        float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
        gain = std::clamp(gain, 0.f, 1.f);

        Eigen::Quaternionf worldFrameAcc = (current.conjugate() *
                                            Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z())) *
                                           current;
        linearAcceleration = worldFrameAcc.vec() - Eigen::Vector3f{0, 0, 1};
        worldFrameAcc.coeffs() *= 1 / G;

        Eigen::Quaternionf accDeltaQ;
        accDeltaQ.w() = std::sqrt((worldFrameAcc.z() + 1) / 2);
        accDeltaQ.x() = -worldFrameAcc.y() / (2 * accDeltaQ.w());
        accDeltaQ.y() = worldFrameAcc.x() / (2 * accDeltaQ.w());
        accDeltaQ.z() = 0.0;
        accDeltaQ = adaptiveSLERP_I(accDeltaQ, gain);

        if (accDeltaQ.coeffs().allFinite())
            attitude = (current * accDeltaQ).normalized();

        update();
        correctAcc(linearAcceleration.z(), 0.35);
        predictKalman(dT);
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
    }

    void updateByPressure(float P)
    {
        pressure = P;
        correctPos(getAltitudeFromPressure(pressure, 101'325), 3.5);
    }

    void updateByTemperature(float T)
    {
        temperature = T;
    }

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

    float getPressure() { return pressure; }
    float getTemperature() { return temperature; }

    Eigen::Quaternionf getFRU_Attitude() { return attitude; }
    Eigen::Quaternionf getFRD_Attitude() { return Eigen::Quaternionf(attitude.w(), attitude.x(), attitude.y(), -attitude.z()); }
    Eulerf getEulerFRD() { return eulerAttitude; }

    Eigen::Vector3f getLinearAcceleration() { return linearAcceleration; }
}