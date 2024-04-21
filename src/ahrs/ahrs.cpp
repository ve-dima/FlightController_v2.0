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
    Eigen::Quaternionf attitude = Eigen::Quaternionf(1, 0, 0, 0);
    Eigen::Quaternionf getAttitude() { return attitude; }

    float G = 1;

    void updateByIMU(Eigen::Vector3f rotate, Eigen::Vector3f acceleration, float dT)
    {
        auto current = attitude;

        rotate -= Eigen::Vector3f{gyroscopeOffset};
        rotate.x() = -rotate.x();
        rotate.y() = -rotate.y();

        current.w() += 0.5f * dT * (-rotate.x() * current.x() - rotate.y() * current.y() - rotate.z() * current.z());
        current.x() += 0.5f * dT * (rotate.x() * current.w() - rotate.y() * current.z() + rotate.z() * current.y());
        current.y() += 0.5f * dT * (rotate.x() * current.z() + rotate.y() * current.w() - rotate.z() * current.x());
        current.z() += 0.5f * dT * (-rotate.x() * current.y() + rotate.y() * current.x() + rotate.z() * current.w());
        current.normalize();
        attitude = current;

        acceleration -= Eigen::Vector3f{accelerometerOffset};
        
        G = acceleration.norm();
        float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
        gain = std::clamp(gain, 0.f, 1.f);

        acceleration *= (1.f / G);
        Eigen::Quaternionf worldFrameAcc = (current.conjugate() *
                                            Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z())) *
                                           current;

        Eigen::Quaternionf accDeltaQ;
        accDeltaQ.w() = std::sqrt((worldFrameAcc.z() + 1.f) * 0.5f);
        accDeltaQ.x() = -worldFrameAcc.y() / std::sqrt(2.0 * (worldFrameAcc.z() + 1.0));
        accDeltaQ.y() = worldFrameAcc.x() / std::sqrt(2.0 * (worldFrameAcc.z() + 1.0));
        accDeltaQ.z() = 0.f;

        accDeltaQ = adaptiveSLERP_I(accDeltaQ, gain);
        if (accDeltaQ.coeffs().allFinite())
        {
            current *= accDeltaQ;
            current.normalize();
            attitude = current;
        }
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
    }

}