#include "ahrs.hpp"
#include "atmosphere.hpp"
#include "Math/math.hpp"

#include "Common.hpp"
#include <algorithm>

namespace AHRS
{
    extern float gyroscopeOffset[3];
    extern float accelerometerOffset[3];

    extern float accelerationFilterGain;
    extern float accelerationRejection;

    extern float magnetometerFilterGain;
    extern float magnetometerMaximum[3], magnetometerMinimum[3];

    extern float baroSpeedCoef;
    extern float baroPosCoef;

    extern float accelerometerNoise;
    extern float barometerNoise;
    extern float mulka;

    Eigen::Vector3f rawRotateSpeed, rotateSpeed;
    Eigen::Vector3f rawAcceleration, acceleration;
    Eigen::Vector3f rawMagneticField;
    float pressure, temperature;

    float lastDt = 0;

    Eigen::Quaternionf attitude = Eigen::Quaternionf::Identity();
    Eulerf eulerAttitude;
    float G = 1;
    float tiltCos = 1;

    Eigen::Vector3f linearAcceleration;
    Eigen::Matrix3f P{
        {100, 0, 0},
        {0, 100, 0},
        {0, 0, 10},
    };
    extern Eigen::Matrix3f Q;
    Eigen::Vector3f x{0, 0, 0};

    void predictKalman(const float dt)
    {
        const Eigen::Vector3f newX{
            x[0] + (x[1] * dt) + (x[2] * dt * dt / 2),
            x[1] + (dt * x[2]),
            x[2],
        };
        x = newX;

        const Eigen::Matrix3f newP{
            {
                P(0, 0) + P(1, 0) * dt + ((dt * dt) * ((P(2, 2) * (dt * dt)) / 2 + P(1, 2) * dt + P(0, 2))) / 2 + (P(2, 0) * (dt * dt)) / 2 + dt * ((P(2, 1) * (dt * dt)) / 2 + P(1, 1) * dt + P(0, 1)),
                P(0, 1) + P(1, 1) * dt + (P(2, 1) * (dt * dt)) / 2 + dt * ((P(2, 2) * (dt * dt)) / 2 + P(1, 2) * dt + P(0, 2)),
                (P(2, 2) * (dt * dt)) / 2 + P(1, 2) * dt + P(0, 2),
            },

            {
                P(1, 0) + P(2, 0) * dt + dt * (P(1, 1) + P(2, 1) * dt) + ((dt * dt) * (P(1, 2) + P(2, 2) * dt)) / 2,
                P(1, 1) + P(2, 1) * dt + dt * (P(1, 2) + P(2, 2) * dt),
                P(1, 2) + P(2, 2) * dt,
            },

            {
                (P(2, 2) * (dt * dt)) / 2 + P(2, 1) * dt + P(2, 0),
                P(2, 1) + P(2, 2) * dt,
                P(2, 2),
            },
        };
        P = newP + (Q * (mulka * dt));
    }

    void correctPos(float z,
                    const float R)
    {
        const float S = P(0, 0) + R;
        const float y = x(0) - z;

        Eigen::Vector3f newX{
            x[0] - (P(0, 0) * y) / S,
            x[1] - (P(1, 0) * y) / S,
            x[2] - (P(2, 0) * y) / S,
        };
        x = newX;

        Eigen::Matrix3f newP{
            {
                -P(0, 0) * (P(0, 0) / S - 1),
                -P(0, 1) * (P(0, 0) / S - 1),
                -P(0, 2) * (P(0, 0) / S - 1),
            },
            {
                P(1, 0) - (P(0, 0) * P(1, 0)) / S,
                P(1, 1) - (P(0, 1) * P(1, 0)) / S,
                P(1, 2) - (P(0, 2) * P(1, 0)) / S,
            },
            {
                P(2, 0) - (P(0, 0) * P(2, 0)) / S,
                P(2, 1) - (P(0, 1) * P(2, 0)) / S,
                P(2, 2) - (P(0, 2) * P(2, 0)) / S,
            },
        };
        P = newP;
    }

    void correctAcc(const float z,
                    const float R)
    {
        const float S = P(2, 2) + R;
        const float y = x(2) - z;

        Eigen::Vector3f newX{
            x[0] - (P(0, 2) * y) / S,
            x[1] - (P(1, 2) * y) / S,
            x[2] - (P(2, 2) * y) / S,
        };
        x = newX;

        Eigen::Matrix3f newP{
            {
                P(0, 0) - (P(0, 2) * P(2, 0)) / S,
                P(0, 1) - (P(0, 2) * P(2, 1)) / S,
                P(0, 2) - (P(0, 2) * P(2, 2)) / S,
            },
            {
                P(1, 0) - (P(1, 2) * P(2, 0)) / S,
                P(1, 1) - (P(1, 2) * P(2, 1)) / S,
                P(1, 2) - (P(1, 2) * P(2, 2)) / S,
            },
            {
                -P(2, 0) * (P(2, 2) / S - 1),
                -P(2, 1) * (P(2, 2) / S - 1),
                -P(2, 2) * (P(2, 2) / S - 1),
            },
        };
        P = newP;
    }

    void updateByIMU(Eigen::Vector3f rSpeed, Eigen::Vector3f rAcc, float dT)
    {
        lastDt = dT;

        {
            rawRotateSpeed = rSpeed;
            rotateSpeed = rSpeed - Eigen::Vector3f{gyroscopeOffset};

            Eigen::Quaternionf current = attitude;
            current.coeffs() += omega(current, rotateSpeed).coeffs() * dT;
            current.normalize();
            attitude = current;
        }

        {
            rawAcceleration = rAcc;
            acceleration = rAcc - Eigen::Vector3f{accelerometerOffset};

            G = acceleration.norm();
            float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
            gain = std::clamp(gain, 0.f, 1.f);

            Eigen::Quaternionf worldFrameAcc = (attitude.conjugate() *
                                                Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z())) *
                                               attitude;
            linearAcceleration = worldFrameAcc.vec() - Eigen::Vector3f{0, 0, 1};
            linearAcceleration *= 9.81;

            worldFrameAcc.coeffs() *= 1 / G;
            Eigen::Quaternionf accDeltaQ;
            accDeltaQ.w() = std::sqrt((worldFrameAcc.z() + 1) / 2);
            accDeltaQ.x() = -worldFrameAcc.y() / (2 * accDeltaQ.w());
            accDeltaQ.y() = worldFrameAcc.x() / (2 * accDeltaQ.w());
            accDeltaQ.z() = 0.0;
            accDeltaQ = adaptiveSLERP_I(accDeltaQ, gain);

            if (accDeltaQ.coeffs().allFinite())
                attitude = (attitude * accDeltaQ).normalized();
        }

        {
            const float sinr_cosp = 2 * (attitude.w() * attitude.x() + attitude.y() * attitude.z());
            const float cosr_cosp = 1 - 2 * (attitude.x() * attitude.x() + attitude.y() * attitude.y());
            eulerAttitude.roll = std::atan2(sinr_cosp, cosr_cosp);

            float sinp = 2 * (attitude.w() * attitude.y() - attitude.x() * attitude.z());
            if (std::abs(sinp) >= 1)
                eulerAttitude.pitch = std::copysign<float>(M_PI / 2, sinp);
            else
                eulerAttitude.pitch = std::asin(sinp);

            const float siny_cosp = 2 * (attitude.w() * attitude.z() + attitude.x() * attitude.y());
            const float cosy_cosp = 1 - 2 * (attitude.y() * attitude.y() + attitude.z() * attitude.z());
            eulerAttitude.yaw = std::atan2(siny_cosp, cosy_cosp);
        }

        {
            tiltCos = 2 * (1 - (attitude.x() * attitude.x() + attitude.y() * attitude.y())) - 1;
            tiltCos = std::clamp(tiltCos, -1.f, 1.f);
        }

        correctAcc(linearAcceleration.z(),
                   accelerometerNoise * accelerometerNoise);
        predictKalman(dT);
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
        rawMagneticField = field;
        for (int i = 0; i < 3; i++)
        {
            const float radius = (magnetometerMaximum[i] - magnetometerMinimum[i]) / 2;
            field(i) = (field(i) - (magnetometerMinimum[i] + radius)) / radius;
        }

        const Eigen::Quaternionf current = attitude;
        Eigen::Quaternionf worldFrameMag = (current.conjugate() *
                                            Eigen::Quaternionf(0.f, field.x(), field.y(), field.z())) *
                                           current;

        const float gamma = worldFrameMag.x() * worldFrameMag.x() + worldFrameMag.y() * worldFrameMag.y();
        const float beta = std::sqrt(gamma + worldFrameMag.x() * std::sqrt(gamma));

        Eigen::Quaternionf magDeltaQ{
            beta / (std::sqrt(2.0 * gamma)),
            0.0,
            0.0,
            worldFrameMag.y() / (std::sqrt(2.0) * beta),
        };
        magDeltaQ = adaptiveSLERP_I(magDeltaQ, magnetometerFilterGain);
        if (magDeltaQ.coeffs().allFinite())
            attitude = (current * magDeltaQ).normalized();
    }

    void updateByPressure(float P)
    {
        pressure = P;
        correctPos(getAltitudeFromPressure(pressure, 101'325),
                   barometerNoise * barometerNoise);
    }

    void updateByTemperature(float T)
    {
        temperature = T;
    }

    Eigen::Vector3f getAcceleration() { return acceleration; }
    Eigen::Vector3f getRawAcceleration() { return rawAcceleration; }
    float getG() { return G; }

    Eigen::Vector3f getRotateAcceleration() { return Eigen::Vector3f(0, 0, 0); }

    Eigen::Vector3f getRawRotateSpeed() { return rawRotateSpeed; }
    Eigen::Vector3f getRotateSpeed() { return rotateSpeed; }
    Eigen::Vector3f getFRD_RotateSpeed() { return Eigen::Vector3f(rotateSpeed.x(), rotateSpeed.y(), -rotateSpeed.z()); }

    float getPressure() { return pressure; }
    float getTemperature() { return temperature; }

    Eigen::Vector3f getMagneticField() { return rawMagneticField; }

    Eigen::Quaternionf getFRU_Attitude() { return attitude; }
    Eigen::Quaternionf getFRD_Attitude() { return Eigen::Quaternionf(attitude.w(), attitude.x(), attitude.y(), -attitude.z()); }
    Eulerf getFRD_Euler() { return Eulerf{eulerAttitude.roll, eulerAttitude.pitch, -eulerAttitude.yaw}; }
    Eulerf getFRU_Euler() { return eulerAttitude; }
    float getTiltCos() { return tiltCos; }

    Eigen::Vector3f getFRD_LinearAcceleration()
    {
        return linearAcceleration;
    }

    float getLastDT() { return lastDt; }

    Eigen::Vector3f getZState() { return x; }
    Eigen::Vector3f getZVaraince() { return Eigen::Vector3f{P(0, 0), P(1, 1), P(2, 2)}; }
}