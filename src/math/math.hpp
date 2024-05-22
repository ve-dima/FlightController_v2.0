#pragma once
#include <Eigen/Geometry>

Eigen::Quaternionf omega(const Eigen::Quaternionf &quat, const Eigen::Vector3f &rot);
float expo(float value, float exp);
Eigen::Vector3f dcm_z(const Eigen::Quaternionf &q);
Eigen::Quaternionf from2vec(const Eigen::Vector3f &u, const Eigen::Vector3f &v);