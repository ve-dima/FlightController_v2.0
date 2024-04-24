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
    Eigen::Vector3f rawRotate;
    Eigen::Vector3f rawAcceleration;

    Eigen::Quaternionf attitude = Eigen::Quaternionf(1, 0, 0, 0);
    Eigen::Quaternionf getAttitude() { return attitude; }

    Eigen::Vector3f getRawRotate() { return rawRotate; }
    Eigen::Vector3f getRawAcceleration() { return rawAcceleration; }

    float G = 1;

    void updateByIMU(Eigen::Vector3f rotate, Eigen::Vector3f acceleration, float dT)
    {
        rawRotate = rotate;
        rawAcceleration = acceleration;

        auto current = attitude;

//         rotate -= Eigen::Vector3f{gyroscopeOffset};
// #define qw current.w()
// #define qx current.x()
// #define qy current.y()
// #define qz current.z()
// #define wx rotate.x()
// #define wy rotate.y()
// #define wz rotate.z()
//         Eigen::Quaternionf qDot = Eigen::Quaternionf(
//             current.w() + dT * 0.5f * (-qx * wx - qy * wy - qz * wz),
//             current.x() + dT * 0.5f * (qw * wx + qy * wz - qz * wy),
//             current.y() + dT * 0.5f * (qw * wy - qx * wz + qz * wx),
//             current.z() + dT * 0.5f * (qw * wz + qx * wy - qy * wx));
// #undef qw
// #undef qx
// #undef qy
// #undef qz
// #undef wx
// #undef wy
// #undef wz
//         qDot.normalize();
//         current = qDot;
//         attitude = current;

        acceleration -= Eigen::Vector3f{accelerometerOffset};

        G = acceleration.norm();
        float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
        gain = std::clamp(gain, 0.f, 1.f);

        acceleration *= (1.f / G);
        Eigen::Quaternionf worldFrameAcc = (current.conjugate() *
                                            Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z())) *
                                           current;
        // auto worldFrameAcc = Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z());
        Eigen::Quaternionf accDeltaQ;
        accDeltaQ.w() = std::sqrt((worldFrameAcc.z() + 1.0) / 2.0);
        accDeltaQ.x() = -worldFrameAcc.y() / std::sqrt(2.0 * (worldFrameAcc.z() + 1));
        accDeltaQ.y() = worldFrameAcc.x() / std::sqrt(2.0 * (worldFrameAcc.z() + 1));
        accDeltaQ.z() = 0.0;
        // accDeltaQ = adaptiveSLERP_I(accDeltaQ, gain);

        // accDeltaQ = Eigen::Quaternionf(1, 0, 0, 0).slerp(gain, accDeltaQ);
        if (accDeltaQ.coeffs().allFinite())
            attitude = (current * accDeltaQ).normalized();
        // attitude = accDeltaQ.normalized();
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
    }

}