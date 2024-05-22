#include "mavlink.hpp"
#include "mavlink/common/mavlink.h"
#include "Common.hpp"
#include "SRT/SRT.hpp"
#include "ahrs/ahrs.hpp"
#include "control/Control.hpp"
#include "motor/motor.hpp"
#include "rc/RC.hpp"
#include "ahrs/atmosphere.hpp"

extern Eigen::Vector3f dcm_z(const Eigen::Quaternionf &q);
extern Eigen::Quaternionf from2vec(const Eigen::Vector3f &u, const Eigen::Vector3f &v);

namespace mavlink
{
    void init() {}

    void enable() {}

    void handler()
    {
        for (static uint32_t hearBeatTimer = 0; millis() - hearBeatTimer > 500; hearBeatTimer = millis())
            mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE::MAV_TYPE_QUADROTOR, MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC, 0, 0, 0);

        for (static uint32_t attTimer = 0; millis() - attTimer > 50; attTimer = millis())
        {
            // auto attitude = AHRS::getFRD_Attitude();
            // const AHRS::Eulerf eulerAttitude = AHRS::getEulerFRD();
            const auto rotateRate = AHRS::getRSpeed();
            // const auto targetRate = Control::getTargetRate();
            const auto targetAttitude = Control::getTargetAttitude();
            // const auto targetThrust = Control::getTargetThrust();
            // const float mavTAtt[4] = {targetAttitude.w(), targetAttitude.x(), targetAttitude.y(), targetAttitude.z()};

            mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, millis(),
                                                 targetAttitude.w(), targetAttitude.x(), targetAttitude.y(), targetAttitude.z(),
                                                 rotateRate.x(), rotateRate.y(), -rotateRate.z(), nullptr);

            // mavlink_msg_attitude_send(MAVLINK_COMM_0, millis(),
            //                           eulerAttitude.roll, eulerAttitude.pitch, eulerAttitude.yaw,
            //                           rotateRate.x, rotateRate.y, -rotateRate.z);

            // mavlink_msg_attitude_target_send(MAVLINK_COMM_0, millis(),
            //                                  0, mavTAtt,
            //                                  targetRate.x(), targetRate.y(), -targetRate.z(),
            //                                  targetThrust);

            // const Eigen::Vector3f t = Control::getTargetThrustVector();
            // float powers[8];
            // std::copy(Motor::getPower(), Motor::getPower() + 4, powers);
            // powers[4] = t.x();
            // powers[5] = t.y();
            // powers[6] = t.z();

            // mavlink_msg_actuator_control_target_send(MAVLINK_COMM_0, millis(),
            //                                          0, powers);

            // mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, millis(),
            //                                  AHRS::getPressure(), 0,
            //                                  AHRS::getTemperature() * 100, 0);

            // const Eigen::Vector3f acc = AHRS::getRawAcceleration(),
            //                       gyr = AHRS::getRawRSpeed();
            // const float pres = AHRS::getPressure();
            // const float heightByPressure = getAltitudeFromPressure(pres, 101'325) * 100 + 2'000;
            // mavlink_msg_highres_imu_send(MAVLINK_COMM_0, millis(),
            //                              acc.x(), acc.y(), acc.z(),
            //                              gyr.x(), gyr.y(), gyr.z(),
            //                              0, 0, 0,
            //                              pres, heightByPressure, 0, 0, 0, 0);

            // mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, millis(),
            //                                     AHRS::x(0) * 100 + 2'000, AHRS::x(1), heightByPressure,
            //                                     AHRS::P(0, 0), AHRS::P(1, 1), AHRS::P(2, 2));
        }

        // for (static uint32_t timer = 0; millis() - timer > 100; timer = millis())
        // {
        //     const Eigen::Vector3f acc = AHRS::getAcceleration();
        //     const Eigen::Vector3f lAcc = AHRS::getLinearAcceleration();
        //     mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, millis(),
        //                                         acc.x, acc.y, acc.z,
        //                                         lAcc.x, lAcc.y, lAcc.z);
        // }

        // for (static uint32_t rcTimer = 0; millis() - rcTimer > (1000 / 50); rcTimer = millis())
        //     mavlink_msg_rc_channels_send(MAVLINK_COMM_0, millis(), RC::channelCount(),
        //                                  RC::rawChannel(0),
        //                                  RC::rawChannel(1),
        //                                  RC::rawChannel(2),
        //                                  RC::rawChannel(3),
        //                                  RC::rawChannel(4),
        //                                  RC::rawChannel(5),
        //                                  RC::rawChannel(6),
        //                                  RC::rawChannel(7),
        //                                  RC::rawChannel(8),
        //                                  RC::rawChannel(9),
        //                                  RC::rawChannel(10),
        //                                  RC::rawChannel(11),
        //                                  RC::rawChannel(12),
        //                                  RC::rawChannel(13),
        //                                  RC::rawChannel(14),
        //                                  RC::rawChannel(15),
        //                                  RC::rawChannel(16),
        //                                  RC::rawChannel(17),
        //                                  RC::rssi());

        // for (static uint32_t rcTimer = 0; millis() - rcTimer > (1000 / 5); rcTimer = millis())
        //     mavlink_msg_rc_channels_send(MAVLINK_COMM_0, millis(), RC::channelCount(),
        //                                  RC::channel(1) * 1000 + 1000,
        //                                  RC::channel(2) * 1000 + 1000,
        //                                  RC::channel(3) * 1000 + 1000,
        //                                  RC::channel(4) * 1000 + 1000,
        //                                  RC::channel(5) * 1000 + 1000,
        //                                  RC::channel(6) * 1000 + 1000,
        //                                  RC::channel(7) * 1000 + 1000,
        //                                  RC::channel(8) * 1000 + 1000,
        //                                  RC::channel(9) * 1000 + 1000,
        //                                  RC::channel(10) * 1000 + 1000,
        //                                  RC::channel(11) * 1000 + 1000,
        //                                  RC::channel(12) * 1000 + 1000,
        //                                  RC::channel(13) * 1000 + 1000,
        //                                  RC::channel(14) * 1000 + 1000,
        //                                  RC::channel(15) * 1000 + 1000,
        //                                  RC::channel(16) * 1000 + 1000,
        //                                  RC::channel(17) * 1000 + 1000,
        //                                  RC::channel(18) * 1000 + 1000,
        //                                  RC::rssi());
    }
    REGISTER_SRT_MODULE(mavlink, init, enable, handler);
}