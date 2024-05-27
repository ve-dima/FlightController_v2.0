#include "mavlink.hpp"
#include "mavlink/common/mavlink.h"
#include "Common.hpp"
#include "SRT/SRT.hpp"
#include "ahrs/ahrs.hpp"
#include "control/Control.hpp"
#include "motor/motor.hpp"
#include "rc/RC.hpp"
#include "ahrs/atmosphere.hpp"

MavLinkReporter::MavLinkReporter(mavlink_channel_t channel, MavLinkProfiles::Profile &profile) : channel(channel), profile(profile) {}

void mavlink_heartbeat_report(mavlink_channel_t ch)
{
    mavlink_msg_heartbeat_send(ch,
                               MAV_TYPE::MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC, 0, 0, 0);
}

void mavlink_quat_report(mavlink_channel_t ch)
{
    const Eigen::Quaternionf attitude = AHRS::getFRD_Attitude();
    const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

    mavlink_msg_attitude_quaternion_send(ch, millis(),
                                         attitude.w(), attitude.x(), attitude.y(), attitude.z(),
                                         rotateRate.x(), rotateRate.y(), rotateRate.z(), nullptr);
}

void mavlink_euler_report(mavlink_channel_t ch)
{
    const AHRS::Eulerf attitude = AHRS::getFRD_Euler();
    const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

    mavlink_msg_attitude_send(ch, millis(),
                              attitude.roll, attitude.pitch, attitude.yaw,
                              rotateRate.x(), rotateRate.y(), rotateRate.z());
}

void mavlink_attitude_target_report(mavlink_channel_t ch)
{
    Eigen::Quaternionf targetAttitude = Control::getTargetAttitude();
    std::swap(targetAttitude.w(), targetAttitude.z()); // eigen store coeffs in x y z w order, need w first

    const Eigen::Vector3f targetRate = Control::getTargetRate();
    const float targetTrust = Control::getTargetThrust();

    mavlink_msg_attitude_target_send(MAVLINK_COMM_0, millis(),
                                     0, targetAttitude.coeffs().data(),
                                     targetRate.x(), targetRate.y(), targetRate.z(),
                                     targetTrust);
}

void mavlink_local_position_report(mavlink_channel_t ch)
{
    const Eigen::Vector3f verticalState = AHRS::getZState();
    mavlink_msg_local_position_ned_send(ch, millis(),
                                        NAN, NAN, verticalState(0),
                                        NAN, NAN, verticalState(1));
}

void mavlink_pressure_report(mavlink_channel_t ch)
{
    const float hPa = AHRS::getPressure() * 1e-2;
    int16_t cDeg = AHRS::getTemperature() * 100;
    mavlink_msg_scaled_pressure_send(ch, millis(),
                                     hPa, NAN,
                                     cDeg, 0);
}

void mavlink_state_report(mavlink_channel_t ch)
{
    const Eigen::Vector3f linearAcc = AHRS::getFRD_LinearAcceleration();
    const Eigen::Vector3f verticalState = AHRS::getZState();
    const Eigen::Vector3f verticalVariance = AHRS::getZVaraince();

    const float velVariance[3] = {NAN, NAN, verticalVariance(1)};
    const float posVariance[3] = {NAN, NAN, verticalVariance(0)};

    Eigen::Quaternionf attitude = AHRS::getFRD_Attitude();
    std::swap(attitude.w(), attitude.z()); // eigen store coeffs in x y z w order, need w first
    const Eigen::Vector3f rotateRate = AHRS::getFRD_RotateSpeed();

    mavlink_msg_control_system_state_send(ch, millis() * 1000,
                                          linearAcc.x(), linearAcc.y(), linearAcc.z(),
                                          NAN, NAN, verticalState(1),
                                          NAN, NAN, verticalState(0),
                                          NAN,
                                          velVariance, posVariance,
                                          attitude.coeffs().data(),
                                          rotateRate.x(), rotateRate.y(), rotateRate.z());
}

void mavlink_actuator_report(mavlink_channel_t ch)
{
    float powers[8];
    std::copy(Motor::getPower(), Motor::getPower() + Motor::maxCount, powers);

    mavlink_msg_actuator_control_target_send(MAVLINK_COMM_0, millis() * 1000,
                                             0, powers);
}

void mavlink_rc_report(mavlink_channel_t ch)
{
    mavlink_msg_rc_channels_send(MAVLINK_COMM_0, millis(), RC::channelCount(),
                                 RC::rawChannel(1),
                                 RC::rawChannel(2),
                                 RC::rawChannel(3),
                                 RC::rawChannel(4),
                                 RC::rawChannel(5),
                                 RC::rawChannel(6),
                                 RC::rawChannel(7),
                                 RC::rawChannel(8),
                                 RC::rawChannel(9),
                                 RC::rawChannel(10),
                                 RC::rawChannel(11),
                                 RC::rawChannel(12),
                                 RC::rawChannel(13),
                                 RC::rawChannel(14),
                                 RC::rawChannel(15),
                                 RC::rawChannel(16),
                                 RC::rawChannel(17),
                                 RC::rawChannel(18),
                                 RC::rssi());
}
