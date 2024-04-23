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
            mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, millis(),
                                                 attitude.w(), attitude.y(), attitude.x(), -attitude.z(),
                                                 NAN, NAN, NAN, nullptr);
        }

        
    }
    REGISTER_SRT_MODULE(mavlink, init, enable, handler);
}