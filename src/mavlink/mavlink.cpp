#include "mavlink.hpp"
#include "mavlink/common/mavlink.h"
#include "Common.hpp"
#include "SRT/SRT.hpp"
#include "ahrs/ahrs.hpp"

namespace mavlink
{
    void init() {}

    void enable() {}

    void handler()
    {
        for (static uint32_t hearBeatTimer = 0; millis() - hearBeatTimer > 500; hearBeatTimer = millis())
            mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE::MAV_TYPE_IMU, MAV_AUTOPILOT::MAV_AUTOPILOT_INVALID, 0, 0, 0);

        for (static uint32_t attTimer = 0; millis() - attTimer > 50; attTimer = millis())
        {
            const auto attitude = AHRS::getAttitude();
            const auto accel = AHRS::getRawAcceleration();
            const auto gyro = AHRS::getRawRotate();

            mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, millis(),
                                                 attitude.w(), attitude.y(), attitude.x(), -attitude.z(),
                                                 NAN, NAN, NAN, nullptr);
#define qw attitude.w()
#define qx attitude.x()
#define qy attitude.y()
#define qz attitude.z()
            // mavlink_msg_attitude_send(MAVLINK_COMM_0, millis(),
            //                           std::atan2(2 * qw * qx + 2 * qy * qz, -2 * qx * qx - 2 * qy * qy + 1),
            //                           std::asin(2 * qw * qy - 2 * qx * qz),
            //                           std::atan2(2 * qw * qz + 2 * qx * qy, -2 * qy * qy - 2 * qz * qz + 1),
            //                           NAN, NAN, NAN);
#undef qw
#undef qx
#undef qy
#undef qz
            mavlink_msg_highres_imu_send(MAVLINK_COMM_0, millis(),
                                         accel.x(), accel.y(), accel.z(),
                                         gyro.x(), gyro.y(), gyro.z(),
                                         NAN, NAN, NAN, NAN, NAN, NAN, NAN,
                                         HIGHRES_IMU_UPDATED_FLAGS::HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_FLAGS::HIGHRES_IMU_UPDATED_YACC | HIGHRES_IMU_UPDATED_FLAGS::HIGHRES_IMU_UPDATED_ZACC |
                                             HIGHRES_IMU_UPDATED_FLAGS::HIGHRES_IMU_UPDATED_XGYRO | HIGHRES_IMU_UPDATED_FLAGS::HIGHRES_IMU_UPDATED_YGYRO | HIGHRES_IMU_UPDATED_FLAGS::HIGHRES_IMU_UPDATED_ZGYRO,
                                         0);
        }
    }
    // REGISTER_SRT_MODULE(mavlink, init, enable, handler);
}