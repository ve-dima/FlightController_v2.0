#include "ahrs.hpp"

Eigen::Quaternionf adaptiveSLERP_I(Eigen::Quaternionf q, float gain, const float threshold = 0.97)
{
    // optimize LERP and SLERP with qI(1,0,0,0)======================
    if (q.w() < threshold)
    {
        // Slerp (Spherical linear interpolation):
        float angle = std::acos(q.w());
        float A = std::sin(angle * (1.0 - gain)) / std::sin(angle);
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
    Eigen::Quaternionf attitude;
    Eigen::Quaternionf getAttitude() { return attitude; }

    float G = 1;

    void updateByIMU(Eigen::Vector3f rotate, Eigen::Vector3f acceleration, float dT)
    {
        auto current = attitude;

        current.w() += 0.5f * dT * (rotate.x() * current.x() + rotate.y() * current.y() + rotate.z() * current.z());
        current.x() += 0.5f * dT * (-rotate.x() * current.w() - rotate.y() * current.z() + rotate.z() * current.y());
        current.y() += 0.5f * dT * (rotate.x() * current.z() - rotate.y() * current.w() - rotate.z() * current.x());
        current.z() += 0.5f * dT * (-rotate.x() * current.y() + rotate.y() * current.x() - rotate.z() * current.w());
        current.normalize();

        G = acceleration.norm();
        float gain = accelerationFilterGain - std::abs(G - 1) * accelerationRejection;
        gain = std::clamp(gain, 0.f, 1.f);

        acceleration *= (1.f / G);
        Eigen::Quaternionf g = current *
                               Eigen::Quaternionf(0.f, acceleration.x(), acceleration.y(), acceleration.z()) *
                               current.conjugate();

        Eigen::Quaternionf accQ;
        accQ.w() = std::sqrt((g.z() + 1.f) * 0.5f);
        accQ.x() = -g.y() / (2.0f * accQ.w());
        accQ.y() = g.x() / (2.0f * accQ.w());
        accQ.z() = 0.f;

        Eigen::Quaternionf deltaQ = adaptiveSLERP_I(accQ, gain);
        if(deltaQ.coeffs().allFinite())
        {
            current *= deltaQ;
            current.normalize();
        }

        attitude = current;
    }

    void updateByMagnetometer(Eigen::Vector3f field)
    {
    }
}