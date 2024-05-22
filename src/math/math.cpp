#include "math.hpp"
#include <algorithm>

Eigen::Quaternionf omega(const Eigen::Quaternionf &quat, const Eigen::Vector3f &rot)
{
    return Eigen::Quaternionf(
        0.5 * (-rot.x() * quat.x() - rot.y() * quat.y() - rot.z() * quat.z()),
        0.5 * (rot.x() * quat.w() + rot.y() * quat.z() - rot.z() * quat.y()),
        0.5 * (-rot.x() * quat.z() + rot.y() * quat.w() + rot.z() * quat.x()),
        0.5 * (rot.x() * quat.y() - rot.y() * quat.x() + rot.z() * quat.w()));
}

float expo(float value, float exp)
{
    value = std::clamp<float>(value, -1, 1);
    exp = std::clamp<float>(exp, 0, 1);

    return (1 - exp) * value + exp * value * value * value;
}

Eigen::Vector3f dcm_z(const Eigen::Quaternionf &q)
{
    Eigen::Vector3f R_z;
    const float a = q.w();
    const float b = q.x();
    const float c = q.y();
    const float d = q.z();
    R_z[0] = 2 * (a * c + b * d);
    R_z[1] = 2 * (c * d - a * b);
    R_z[2] = a * a - b * b - c * c + d * d;
    return R_z;
}

Eigen::Quaternionf from2vec(const Eigen::Vector3f &u, const Eigen::Vector3f &v)
{
    Eigen::Quaternionf q;
    q.w() = u.dot(v) + std::sqrt(u.squaredNorm() * v.squaredNorm());
    q.vec() = u.cross(v);
    q.normalize();
    return q;
}